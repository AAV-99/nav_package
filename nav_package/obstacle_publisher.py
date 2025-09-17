#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_pub')

        # Publicador de nubes de puntos
        self.pub = self.create_publisher(PointCloud2, '/extra_obstacles', 10)

        # Timer para publicar cada 0.5s
        self.timer = self.create_timer(0.5, self.publish_obstacle)

    def publish_obstacle(self):
        # 🚧 Obstáculo relativo al robot (en base_link)
        # Ejemplo: 1.0 m al frente, 0.2 m a la derecha
        points = [(1.0, -0.2, 0.0)]

        # Crear header con frame_id en base_link
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"

        # Crear nube de puntos XYZ
        cloud = pc2.create_cloud_xyz32(header, points)

        # Publicar
        self.pub.publish(cloud)
        self.get_logger().info("📡 Publicado obstáculo en base_link: (1.0, -0.2, 0.0)")


def main():
    rclpy.init()
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
