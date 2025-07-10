import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import math
import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.5.3'

from pymycobot import MyCobot
    

class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            # "joint_commands",
            "joint_states",
            self.listener_callback,
            1
        )
        self.sub_grippper = self.create_subscription(
            Bool,
            "gripper_command",
            self.gripper_callback,
            1
        )

        self.pub = self.create_publisher(JointState, "real_joint_states", 1)

        self.joint_state = JointState()

        self.last_gripper_command = None
        self.gripper_command = None

        # Create timer to publish joint states at 30Hz
        # self.timer = self.create_timer(1.0/5.0, self.publish_joint_states)

        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)



    def listener_callback(self, msg):

        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
            
        # print('data_list: {}'.format(data_list))

        self.mc.send_angles(data_list, 100)
        # angles = self.mc.get_angles()
        # if angles is not None:
        #     self.joint_state.position = angles
        
        # if self.gripper_command is not None:
        #     if self.last_gripper_command != self.gripper_command:
        #         #sleep for gripper command to take effect
        #         time.sleep(1.0)
        #         if self.gripper_command == 0:
        #             self.mc.set_gripper_state(0, 30,3)
        #             self.get_logger().info("Gripper command: Open")
        #         elif self.gripper_command == 1:
        #             self.mc.set_gripper_state(1, 30,3)
        #             self.get_logger().info("Gripper command: Close")
        #         self.last_gripper_command = self.gripper_command
        #         time.sleep(1.0)

        
    
    def gripper_callback(self, msg):
        self.gripper_command = int(msg.data)
        time.sleep(0.1)
        for i in range(5):
           self.mc.set_gripper_state(int(msg.data), 30)
           time.sleep(0.05)
        self.get_logger().info(f"Gripper command: {int(msg.data)}")
        


    def publish_joint_states(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.name = [
            "link2_to_link1",
            "link3_to_link2",
            "link4_to_link3",
            "link5_to_link4",
            "link6_to_link5",
            "link6output_to_link6",
        ]
        self.joint_state.velocity = [0.0] * len(self.joint_state.name)
        self.joint_state.effort = [0.0] * len(self.joint_state.name)
        self.pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()

    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
