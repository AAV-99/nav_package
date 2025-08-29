#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Servicio estándar: Request vacío, Response con success + message

# Importar el wrapper del TurtleBot4
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


class DockUndockNode(Node):
    def __init__(self):
        super().__init__('dock_undock_node')

        # Crear servidores de servicio
        self.dock_srv = self.create_service(Trigger, 'dock', self.dock_callback)
        self.undock_srv = self.create_service(Trigger, 'undock', self.undock_callback)

        # Crear instancia del navegador
        self.navigator = TurtleBot4Navigator()

        self.get_logger().info("Nodo de dock/undock inicializado. Esperando llamadas a los servicios 'dock' y 'undock'...")

    def dock_callback(self, request, response):
        """
        Se ejecuta cuando alguien llama al servicio /dock
        """
        self.get_logger().info("Solicitud recibida: dock")

        # Esperar a que Nav2 esté activo
        self.navigator.waitUntilNav2Active()

        if not self.navigator.getDockedStatus():
            self.get_logger().info("Robot no está en el dock. Ejecutando docking...")
            self.navigator.dock()
            self.get_logger().info("✅ Robot acoplado.")
            response.success = True
            response.message = "Robot acoplado correctamente."
        else:
            self.get_logger().info("El robot ya estaba acoplado.")
            response.success = False
            response.message = "El robot ya estaba en el dock."

        return response

    def undock_callback(self, request, response):
        """
        Se ejecuta cuando alguien llama al servicio /undock
        """
        self.get_logger().info("Solicitud recibida: undock")

        # Esperar a que Nav2 esté activo
        self.navigator.waitUntilNav2Active()

        if self.navigator.getDockedStatus():
            self.get_logger().info("Robot acoplado. Ejecutando undocking...")
            self.navigator.undock()
            self.get_logger().info("✅ Robot desacoplado.")
            response.success = True
            response.message = "Robot desacoplado correctamente."
        else:
            self.get_logger().info("El robot ya estaba fuera del dock.")
            response.success = False
            response.message = "El robot no estaba acoplado."

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DockUndockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
