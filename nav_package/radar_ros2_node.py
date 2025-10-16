#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np

from rd03d_jetson import RD03D, Target


class RadarNode(Node):
    def __init__(self):
        super().__init__('rd03d_radar_node')
        
        # Declare parameters
        self.declare_parameter('uart_port', '/dev/ttyTHS0')
        self.declare_parameter('baudrate', 256000)
        self.declare_parameter('multi_mode', True)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('frame_id', 'radar_link')
        
        # Get parameters
        uart_port = self.get_parameter('uart_port').value
        baudrate = self.get_parameter('baudrate').value
        multi_mode = self.get_parameter('multi_mode').value
        publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize radar
        try:
            self.radar = RD03D(uart_port=uart_port, baudrate=baudrate, multi_mode=multi_mode)
            self.get_logger().info(f'Radar initialized on {uart_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize radar: {e}')
            raise
        
        # Create publisher for PointCloud2
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            'radar/pointcloud',
            10
        )
        
        # Create timer for periodic publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('RD-03D Radar Node started')
        self.get_logger().info(f'Publishing at {publish_rate} Hz on topic: radar/pointcloud')
    
    def timer_callback(self):
        """Periodic callback to read radar data and publish"""
        if self.radar.update():
            targets = self.radar.get_all_targets()
            
            if targets:
                # Create and publish PointCloud2 message
                pointcloud_msg = self.create_pointcloud2(targets)
                self.pointcloud_pub.publish(pointcloud_msg)
    
    def create_pointcloud2(self, targets):
        """
        Convert radar targets to PointCloud2 message
        
        Format: Each point contains [x, y, z, speed, distance, angle]
        - x, y: Cartesian coordinates in mm (converted to meters)
        - z: Always 0 (2D radar)
        - speed: Radial speed in cm/s (converted to m/s)
        - distance: Distance from sensor in mm (converted to meters)
        - angle: Angle in degrees
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        
        # Define point fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='speed', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='angle', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Convert targets to point cloud data
        points = []
        for target in targets:
            if target.distance > 0:  # Filter out invalid targets
                # Convert mm to meters, cm/s to m/s
                x = target.x / 1000.0  # mm to m
                y = target.y / 1000.0  # mm to m
                z = 0.0  # 2D radar
                speed = target.speed / 100.0  # cm/s to m/s
                distance = target.distance / 1000.0  # mm to m
                angle = target.angle
                
                points.append([x, y, z, speed, distance, angle])
        
        # Pack data into binary format
        point_step = 24  # 6 float32 values * 4 bytes
        data = b''
        for point in points:
            data += struct.pack('6f', *point)
        
        # Create PointCloud2 message
        pointcloud = PointCloud2()
        pointcloud.header = header
        pointcloud.height = 1
        pointcloud.width = len(points)
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = point_step
        pointcloud.row_step = point_step * len(points)
        pointcloud.data = data
        pointcloud.is_dense = True
        
        return pointcloud
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.radar.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RadarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()