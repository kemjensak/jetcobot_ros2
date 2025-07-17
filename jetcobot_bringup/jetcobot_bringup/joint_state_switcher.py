#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
import time


class JointStateSwitcher(Node):
    def __init__(self):
        super().__init__('joint_state_switcher')
        
        # 구독자 설정
        self.fake_joint_states_sub = self.create_subscription(
            JointState,
            'joint_state_broadcaster/joint_states',
            self.fake_joint_states_callback,
            10
        )
        
        self.real_joint_states_sub = self.create_subscription(
            JointState,
            'real_joint_states',
            self.real_joint_states_callback,
            10
        )
        
        self.move_action_status_sub = self.create_subscription(
            GoalStatusArray,
            '/arm_controller/follow_joint_trajectory/_action/status',
            self.move_action_status_callback,
            10
        )
        
        # 발행자 설정
        self.joint_states_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.get_angles_cmd_pub = self.create_publisher(
            Bool,
            'get_angles_cmd',
            10
        )
        
        # 상태 변수들
        self.latest_fake_joint_states = None
        self.latest_real_joint_states = None
        self.current_status = None
        self.previous_status = None
        self.using_real_states = False
        
        # 40Hz 타이머 (0.025초마다 실행)
        self.timer = self.create_timer(0.025, self.timer_callback)
        
        self.get_logger().info('Joint State Switcher node initialized')
    
    def fake_joint_states_callback(self, msg):
        """fake_joint_states 토픽 콜백"""
        self.latest_fake_joint_states = msg
    
    def real_joint_states_callback(self, msg):
        """real_joint_states 토픽 콜백"""
        self.get_logger().warn(f'Received real joint states at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        self.latest_real_joint_states = msg
    
    def timer_callback(self):
        """40Hz로 joint_states 발행"""
        if self.using_real_states and self.latest_real_joint_states is not None:
            # real states 모드일 때 - timestamp 갱신
            updated_msg = JointState()
            updated_msg.header = self.latest_real_joint_states.header
            updated_msg.header.stamp = self.get_clock().now().to_msg()
            updated_msg.name = self.latest_real_joint_states.name
            updated_msg.position = self.latest_real_joint_states.position
            updated_msg.velocity = self.latest_real_joint_states.velocity
            updated_msg.effort = self.latest_real_joint_states.effort
            # self.get_logger().warn(f'Publishing real joint states at {updated_msg.header.stamp.sec}.{updated_msg.header.stamp.nanosec}')
            self.joint_states_pub.publish(updated_msg)
        elif self.latest_fake_joint_states is not None:
            # fake states 모드일 때 (status가 2)
            self.joint_states_pub.publish(self.latest_fake_joint_states)
    
    def move_action_status_callback(self, msg):
        """move_action status 토픽 콜백"""
        if not msg.status_list:
            return
        
        # 이전 상태 저장
        self.previous_status = self.current_status
        
        # 현재 상태 업데이트 (가장 최신 status 사용)
        self.current_status = msg.status_list[-1].status
        
        # self.get_logger().info(f'Status changed: {self.previous_status} -> {self.current_status}')
        
        # 상태 변화 감지: 2에서 4로 변경
        if self.previous_status == 2 and self.current_status == 4:
            # self.get_logger().info('Status changed from 2 to 4, switching to real joint states')
            
            # get_angles_cmd 토픽에 아무 값이나 발행
            # time.sleep(0.2)  # 잠시 대기
            cmd_msg = Bool()
            self.get_angles_cmd_pub.publish(cmd_msg)
            # self.get_logger().info('Published get_angles_cmd')
            
            # real_joint_states 사용 모드로 전환
            self.using_real_states = True
        
        # 상태가 2로 돌아왔을 때
        else:
            self.latest_real_joint_states = None  # real joint states 초기화
            # self.get_logger().info('Status is 2, switching to fake joint states')
            self.using_real_states = False


def main(args=None):
    rclpy.init(args=args)
    
    joint_state_switcher = JointStateSwitcher()
    
    try:
        rclpy.spin(joint_state_switcher)
    except KeyboardInterrupt:
        joint_state_switcher.get_logger().info('Shutting down Joint State Switcher')
    finally:
        joint_state_switcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
