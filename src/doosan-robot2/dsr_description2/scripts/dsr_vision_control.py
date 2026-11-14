#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
import math

class DsrVisionControlNode(Node):
    def __init__(self):
        super().__init__('dsr_vision_control_node')
        
        # 1. Subscribe to virtual camera image topic with Best Effort QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/dsr01/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # 2. Publisher for robot joint states control
        self.joint_pub = self.create_publisher(
            JointState,
            '/dsr01/joint_states',
            10
        )
        
        # Initialize control variables
        self.img_received_count = 0
        self.is_burger_detected = False
        self.time_step = 0.0
        
        # Setup control loop timer (30Hz = 30 times per second)
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)
        
        self.get_logger().info('Dsr Vision Control Node has been started')
        self.get_logger().info('Waiting for hamburger images from virtual camera')

    def image_callback(self, msg):
        # Trigger robot motion if image data is received continuously
        if not self.is_burger_detected:
            self.img_received_count += 1
            if self.img_received_count > 5:
                self.is_burger_detected = True
                self.get_logger().info('Hamburger detected by camera Starting Doosan Robot')

    def control_loop(self):
        # Wait until camera detects the trigger image
        if not self.is_burger_detected:
            return
            
        # Generate sine wave motion for robot joints
        self.time_step += 0.05
        joint_angle_1 = 0.5 * math.sin(self.time_step) # Wave motion for joint_1
        joint_angle_2 = 0.3 * math.sin(self.time_step) # Wave motion for joint_2
        
        # Define Doosan Robot e0509 joint names (Matching URDF specification)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'joint_1', 
            'joint_2', 
            'joint_3', 
            'joint_4', 
            'joint_5', 
            'joint_6'
        ]
        
        # Target positions in radians
        msg.position = [
            joint_angle_1,  # Base rotation
            joint_angle_2,  # Shoulder rotation
            0.0,            # Elbow fixed
            0.0, 
            0.0, 
            0.0
        ]
        
        # Publish to update robot model in RViz2
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DsrVisionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
