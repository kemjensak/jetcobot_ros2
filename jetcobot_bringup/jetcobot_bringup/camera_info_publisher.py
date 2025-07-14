#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_name', 'jetcocam')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('publish_rate', 30.0)
        
        # Get parameters
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Create publisher
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        # Load camera info
        self.camera_info_msg = self.load_camera_info(camera_info_url)
        
        if self.camera_info_msg is None:
            self.get_logger().error('Failed to load camera info. Exiting.')
            raise RuntimeError('Camera info loading failed')
        
        # Create timer
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_camera_info)
        
        self.get_logger().info(f'Camera info publisher started for {self.camera_name}')
        self.get_logger().info(f'Publishing camera_info at {publish_rate} Hz')
    
    def load_camera_info(self, camera_info_url):
        """Load camera info from YAML file"""
        camera_info = CameraInfo()
        
        if not camera_info_url:
            self.get_logger().error('No camera_info_url provided')
            return None
        
        # Handle package:// URLs
        if camera_info_url.startswith('package://'):
            # Extract package name and relative path
            parts = camera_info_url[10:].split('/', 1)
            if len(parts) != 2:
                self.get_logger().error(f'Invalid package URL: {camera_info_url}')
                return None
            
            package_name, relative_path = parts
            
            try:
                # Use ament_index_python to find package share directory
                package_share_dir = get_package_share_directory(package_name)
                file_path = os.path.join(package_share_dir, relative_path)
            except Exception as e:
                self.get_logger().error(f'Failed to find package {package_name}: {str(e)}')
                return None
        else:
            # Direct file path
            file_path = camera_info_url
        
        if not file_path or not os.path.exists(file_path):
            self.get_logger().error(f'Camera info file not found: {camera_info_url}')
            return None
        
        try:
            with open(file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            camera_info.header.frame_id = self.frame_id
            camera_info.width = calib_data.get('image_width', 640)
            camera_info.height = calib_data.get('image_height', 480)
            
            # Camera matrix (K)
            if 'camera_matrix' in calib_data:
                camera_info.k = calib_data['camera_matrix']['data']
            
            # Distortion coefficients (D)
            if 'distortion_coefficients' in calib_data:
                camera_info.d = calib_data['distortion_coefficients']['data']
            
            # Rectification matrix (R)
            if 'rectification_matrix' in calib_data:
                camera_info.r = calib_data['rectification_matrix']['data']
            
            # Projection matrix (P)
            if 'projection_matrix' in calib_data:
                camera_info.p = calib_data['projection_matrix']['data']
            
            # Distortion model
            camera_info.distortion_model = 'plumb_bob'
            
            self.get_logger().info(f'Successfully loaded camera info from {file_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load camera info: {str(e)}')
            return None
        
        return camera_info
    
    
    def publish_camera_info(self):
        """Publish camera info message"""
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
            self.camera_info_pub.publish(self.camera_info_msg)
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraInfoPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
