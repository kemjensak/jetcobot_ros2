import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
from action_msgs.msg import GoalStatusArray
import time
import math
import pymycobot
from packaging import version
from pymycobot.mycobot280 import MyCobot280
    
class Joint_controller(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            # "joint_commands",
            "joint_state_broadcaster/joint_states",
            self.listener_callback,
            1
        )
        self.sub_grippper = self.create_subscription(
            Int32,
            "gripper_command",
            self.gripper_callback,
            1
        )
        self.sub_get_angles_cmd = self.create_subscription(
            Bool,
            "get_angles_cmd",
            self.get_radians_cmd_callback,
            1
        )

        self.move_action_status_sub = self.create_subscription(
            GoalStatusArray,
            '/arm_group_controller/follow_joint_trajectory/_action/status',
            self.move_action_status_callback,
            10
        )

        self.pub = self.create_publisher(JointState, "real_joint_states", 1)
        self.gripper_state_pub = self.create_publisher(Int32, "gripper_state", 1)
        self.gripper_command = None
        
        # move_action 상태 추적
        self.move_action_status = None
        self.should_stop_movement = True
        
        # get_radians_cmd를 자동으로 실행하기 위한 타이머
        self.get_radians_timer = self.create_timer(1.0, self.auto_get_radians_callback)

        self.mc = MyCobot280("/dev/ttyJETCOBOT", 1000000, thread_lock=False)
        time.sleep(0.4)
        self.mc.set_fresh_mode(1)
        time.sleep(0.4)

    def listener_callback(self, msg):
        # follow_joint_trajectory status가 4(SUCCEEDED)면 send_angles 중단
        if self.should_stop_movement:
            return
            
        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)

        self.mc.send_angles(data_list, 100, _async=True)
        
    def gripper_callback(self, msg): ## Int 메세지 수신 # gripper_command는 0~100 범위
        self.gripper_command = int(msg.data) ## close(true) or open(false) -> Int
        for attempt in range(5):
            # time.sleep(0.1)
            self.mc.set_gripper_value(self.gripper_command, 100) # set_gripper_state 대신 set_gripper_value 사용  # gripper_value, speed
            result = self.mc.get_gripper_value(1)
            if result != -1:
                # result를 Int32 메시지로 publish
                gripper_state_msg = Int32()
                gripper_state_msg.data = int(result)
                self.gripper_state_pub.publish(gripper_state_msg)
                
                self.get_logger().info(f"Set gripper command: {self.gripper_command}, Gripper state: {result}")
                # self.get_logger().info(f"Gripper state successfully set to {result} on attempt {attempt + 1}")'
                break
            else:
                self.get_logger().warn(f"Failed to set gripper state, attempt {attempt + 1}/5")
                if attempt == 4:
                    self.get_logger().error("Failed to set gripper state after 5 attempts")

    def move_action_status_callback(self, msg):
        """move_action status 토픽 콜백"""
        if not msg.status_list:
            return
        
        # 가장 최근 status 확인
        latest_status = msg.status_list[-1]
        current_status = latest_status.status
        
        # status가 2(EXECUTING)가 아니면 movement 중단
        if current_status != 2:
            if not self.should_stop_movement:
                self.get_logger().info("follow_joint_trajectory SUCCEEDED (status 4) - stopping send_angles")
                self.should_stop_movement = True
        else:
            # 다른 상태일 때는 movement 재개
            if self.should_stop_movement:
                self.get_logger().warn(f"follow_joint_trajectory status changed to {current_status} - resuming send_angles")
                self.should_stop_movement = False
        
        self.move_action_status = current_status

    def auto_get_radians_callback(self):
        """move_action status가 2(EXECUTING)가 아닐 때 자동으로 get_radians 실행"""
        # move_action_status가 None이거나 2(EXECUTING)가 아닐 때만 실행
        if self.move_action_status is None or self.move_action_status != 2:
            self.get_radians_cmd_callback(None)

    def get_radians_cmd_callback(self,_):
        joint_state = JointState()
        
        # 최대 5번 재시도
        for attempt in range(5):
            # time.sleep(0.2)
            angles = self.mc.get_angles()
            # angles가 존재하고 리스트이며 6개의 값이 모두 있는지 확인
            if angles != -1:
                # 각도를 라디안으로 변환
                joint_state.position = [math.radians(angle) for angle in angles]
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = [
                    "1_Joint",
                    "2_Joint",
                    "3_Joint",
                    "4_Joint",
                    "5_Joint",
                    "6_Joint",
                ]
                joint_state.velocity = [0.0] * len(joint_state.name)
                joint_state.effort = [0.0] * len(joint_state.name)
                self.pub.publish(joint_state)
                break
            else:
                self.get_logger().warn(f"Failed to get 6 joint angles, attempt {attempt + 1}/5. Got: {angles}")
                if attempt == 4:  # 마지막 시도
                    self.get_logger().error("Failed to get valid joint angles after 5 attempts")
                    
def main(args=None):
    rclpy.init(args=args)
    joint_controller = Joint_controller()

    rclpy.spin(joint_controller)
    
    joint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
