#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import TaskResult

# Importar el wrapper de turtlebot4
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class NavToPosesNode(Node):
    def __init__(self):
        super().__init__('nav_to_poses')

        # Suscriptor al tópico que recibe la meta como string
        self.create_subscription(String, '/orchestrator/goal', self.goal_callback, 10)

        # Publicador del estado de navegación
        self.status_pub = self.create_publisher(String, '/goal_status', 10)

        # Crear navegador
        self.navigator = TurtleBot4Navigator()

        # Cargar archivo YAML con ubicaciones
        pkg_path = get_package_share_directory('nav_package')
        yaml_path = os.path.join(pkg_path, 'config', 'locations.yaml')
        with open(yaml_path, 'r') as f:
            self.locations = yaml.safe_load(f)['locations']
        
        self.last_goal_id = None
        self.get_logger().info("✅ Nodo nav_to_poses listo y esperando metas...")

    def goal_callback(self, msg: String):
        """Callback que recibe el ID de la meta (ej: '1') y arranca navegación"""
        goal_id = msg.data.strip()

        if goal_id not in self.locations:
            self.get_logger().error(f"❌ Meta '{goal_id}' no encontrada en locations.yaml")
            return

        loc = self.locations[goal_id]

        # Crear el PoseStamped
        goal_pose = self.navigator.getPoseStamped(
            [loc['x'], loc['y']],
            TurtleBot4Directions.EAST
        )

        if goal_id != self.last_goal_id:
            loc = self.locations[goal_id]
            self.get_logger().info(f"📍 Nueva meta recibida: {goal_id} -> {loc['x']}, {loc['y']}")
            self.last_goal_id = goal_id

        # Enviar al robot hacia la meta
        self.navigator.goToPose(goal_pose)

        # Esperar hasta que termine la navegación
        if self.navigator.isTaskComplete():
            # Revisar el resultado
            result = str(self.navigator.getResult())
            self.get_logger().info(result)
            status_msg = String()

            if "SUCCEEDED" in result:
                status_msg.data = "SUCCEEDED"
                self.get_logger().info("✅ Meta alcanzada.")
                self.status_pub.publish(status_msg)
            elif "CANCELED" in result:
                status_msg.data = "CANCELED"
                self.get_logger().warn("⚠️ Meta cancelada.")
                self.status_pub.publish(status_msg)
            elif "FAILED" in result:
                status_msg.data = "FAILED"
                self.get_logger().error("❌ Meta fallida.")
                self.status_pub.publish(status_msg)

        

def main():
    rclpy.init()
    node = NavToPosesNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
