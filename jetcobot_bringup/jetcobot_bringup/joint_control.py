import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray
import time
import math
import pymycobot
from packaging import version

# MAX_REQUIRE_VERSION = '3.5.3'
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
            Bool,
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
            '/arm_controller/follow_joint_trajectory/_action/status',
            self.move_action_status_callback,
            10
        )

        self.pub = self.create_publisher(JointState, "real_joint_states", 1)

        self.last_gripper_command = None
        self.gripper_command = None
        
        # move_action 상태 추적
        self.move_action_status = None
        self.should_stop_movement = False

        self.mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
        time.sleep(0.3)
        self.mc.set_fresh_mode(1)
        time.sleep(0.3)

    def listener_callback(self, msg):
        # follow_joint_trajectory status가 4(SUCCEEDED)면 send_angles 중단
        if self.should_stop_movement:
            return
            
        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        self.mc.send_angles(data_list, 100, _async=True)
        
    def gripper_callback(self, msg):
        self.gripper_command = int(msg.data)
        time.sleep(0.2)
        for i in range(5):
           self.mc.set_gripper_state(int(msg.data), 30)
           time.sleep(0.05)
        self.get_logger().info(f"Gripper command: {int(msg.data)}")
        time.sleep(0.2)
        
    def move_action_status_callback(self, msg):
        """move_action status 토픽 콜백"""
        if not msg.status_list:
            return
        
        # 가장 최근 status 확인
        latest_status = msg.status_list[-1]
        current_status = latest_status.status
        
        # status가 4(SUCCEEDED)일 때 movement 중단
        if current_status == 4:
            if not self.should_stop_movement:
                self.get_logger().info("follow_joint_trajectory SUCCEEDED (status 4) - stopping send_angles")
                self.should_stop_movement = True
        else:
            # 다른 상태일 때는 movement 재개
            if self.should_stop_movement:
                self.get_logger().warn(f"follow_joint_trajectory status changed to {current_status} - resuming send_angles")
                self.should_stop_movement = False
        
        self.move_action_status = current_status

    def get_radians_cmd_callback(self,_):
        joint_state = JointState()
        
        # 최대 5번 재시도
        for attempt in range(5):
            time.sleep(0.2)
            angles = self.mc.get_angles()
            
            # angles가 존재하고 리스트이며 6개의 값이 모두 있는지 확인
            if angles != -1:
                # 각도를 라디안으로 변환
                joint_state.position = [math.radians(angle) for angle in angles]
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = [
                    "link2_to_link1",
                    "link3_to_link2",
                    "link4_to_link3",
                    "link5_to_link4",
                    "link6_to_link5",
                    "link6output_to_link6",
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
