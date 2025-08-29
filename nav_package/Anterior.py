#!/usr/bin/env python3
"""
Nodo de ROS 2 que envía al robot:
1. Una pose inicial al tópico /initialpose (para AMCL).
2. Puntos de destino (goals) cargados desde un archivo YAML al tópico /goal_pose/goal para que Nav2 navegue hacia ellos.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_package.posestamped_msg_generator import PoseStampedGenerator
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import time


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # --- Publicadores ---
        self.publisher_goal = self.create_publisher(PoseStamped, '/goal_pose/goal', 10)
        self.publisher_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info('Goal Publisher node has been initialized')

        # Generador de mensajes PoseStamped
        self.pose_generator = PoseStampedGenerator('pose_generator')

        # --- Cargar ubicaciones ---
        self.locations = self.load_locations()

        # --- Publicar pose inicial automáticamente al iniciar ---
        self.publish_initial_pose(
            x=-0.29221153259277344,
            y=0.003913849592208862,
            qx=0.0,
            qy=0.0,
            qz=-0.9999803487784081,
            qw=0.006269135268389216
        )

    def load_locations(self):
        """
        Lee las coordenadas de destinos desde un archivo YAML usando el path del paquete.
        """
        try:
            package_share_dir = get_package_share_directory('nav_package')
            yaml_path = os.path.join(package_share_dir, 'config', 'locations.yaml')

            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            self.get_logger().info(f"Loaded locations from {yaml_path}")
            return data.get('locations', {})
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file: {e}")
            return {}

    def publish_initial_pose(self, x, y, qx, qy, qz, qw):
        """
        Publica la estimación inicial de la posición del robot en el mapa para AMCL.
        """
        msg = PoseWithCovarianceStamped()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Covarianza estándar para initialpose en AMCL (x, y, yaw)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        # Publicar varias veces para asegurar que AMCL lo reciba
        for i in range(3):
            self.publisher_initial_pose.publish(msg)
            self.get_logger().info(f'Initial pose published! (Attempt {i+1}/3)')
            time.sleep(0.1)  # Pausa pequeña entre envíos


    def publish_goal(self, table_number):
        """
        Publica un objetivo de navegación (goal) usando la ubicación indicada.
        """
        coords = self.locations.get(table_number)
        if not coords:
            self.get_logger().error(f"No coordinates found for {table_number}")
            return

        pose_msg = self.pose_generator.create_pose_stamped(
            x=coords['x'],
            y=coords['y'],
            z=0.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            frame_id='map'
        )

        self.publisher_goal.publish(pose_msg)
        self.get_logger().info(f'Goal pose published for location {table_number}')

    def run_interface(self):
        """
        Muestra un menú interactivo para seleccionar y publicar destinos.
        """
        while True:
            print("\nAVAILABLE LOCATIONS:")
            for loc in sorted(self.locations.keys()):
                print(f"Location {loc}")

            user_input = input('\nEnter location number or "q" to quit: ').strip()

            if user_input.lower() == 'q':
                break

            if user_input in self.locations:
                self.publish_goal(user_input)
            else:
                print("Invalid number! Please enter a valid location key.")


def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()

    try:
        goal_publisher.run_interface()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        goal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

