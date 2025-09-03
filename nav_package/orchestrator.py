#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from enum import Enum


class State(Enum):
    IDLE = 0
    UNDOCK = 1
    NAVIGATE = 2
    DOCK = 3
    DONE = 4


class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')

        # Estado inicial
        self.state = State.IDLE
        self.current_goal = 1
        self.max_goals = 4
        self.retry_count = 0
        self.max_retries = 25

        # Pub/Sub
        self.goal_pub = self.create_publisher(String, '/orchestrator/goal', 10)
        self.result_sub = self.create_subscription(String, '/goal_status', self.result_callback, 10)

        # Clientes de servicio
        self.dock_client = self.create_client(Trigger, 'dock')
        self.undock_client = self.create_client(Trigger, 'undock')

        # Timer FSM
        self.timer = self.create_timer(1.0, self.fsm_step)

        # Futuro activo
        self.pending_future = None
        self.pending_action = None

    def call_service_async(self, client, label, on_success, on_fail):
        """Helper no bloqueante para servicios"""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"⏱️ Servicio {label} no disponible.")
            return

        req = Trigger.Request()
        future = client.call_async(req)
        self.pending_future = future
        self.pending_action = (label, on_success, on_fail)
        future.add_done_callback(self.service_done)

    def service_done(self, future):
        label, on_success, on_fail = self.pending_action
        self.pending_future = None
        self.pending_action = None

        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"✅ {label} completado: {result.message}")
                if on_success:
                    on_success()
            else:
                self.get_logger().error(f"❌ {label} falló")
                if on_fail:
                    on_fail()
        except Exception as e:
            self.get_logger().error(f"⚠️ Error en {label}: {e}")
            if on_fail:
                on_fail()

    def fsm_step(self):
        if self.pending_future:
            return  # esperar respuesta del servicio antes de avanzar

        if self.state == State.IDLE:
            if self.current_goal == 1:
                self.get_logger().info("➡️ Pasando a UNDOCK")
                self.state = State.UNDOCK
            elif self.current_goal <= self.max_goals:
                self.get_logger().info(f"➡️ Pasando a NAVIGATE con goal {self.current_goal}")
                self.state = State.NAVIGATE
            else:
                self.get_logger().info("➡️ Pasando a DOCK")
                self.state = State.DOCK

        elif self.state == State.UNDOCK:
            self.get_logger().info("🚀 Ejecutando undock...")
            self.call_service_async(
                self.undock_client,
                "undock",
                on_success=lambda: self._undock_success(),
                on_fail=lambda: self._undock_fail()
            )
            # se queda esperando callback

        elif self.state == State.NAVIGATE:
            msg = String()
            msg.data = str(self.current_goal)
            #self.get_logger().info(f"📍 Enviando meta {msg.data}")
            self.goal_pub.publish(msg)
            # quedamos en NAVIGATE hasta result_callback

        elif self.state == State.DOCK:
            self.get_logger().info("⚡ Ejecutando dock...")
            self.call_service_async(
                self.dock_client,
                "dock",
                on_success=lambda: self._dock_success(),
                on_fail=lambda: self._dock_fail()
            )

        elif self.state == State.DONE:
            self.get_logger().info("✅ Misión completa, FSM detenida.")
            self.timer.cancel()

    # Callbacks de transición
    def _undock_success(self):
        self.current_goal = 1
        self.get_logger().info("✅ Undock completado, pasando a NAVIGATE")
        self.state = State.NAVIGATE

    def _undock_fail(self):
        self.get_logger().error("🚨 No se pudo hacer undock, pasando a DOCK.")
        self.state = State.DOCK

    def _dock_success(self):
        self.state = State.DONE

    def _dock_fail(self):
        self.get_logger().error("❌ No se pudo hacer dock, misión abortada.")
        self.state = State.DONE

    def result_callback(self, msg: String):
        result = msg.data
        self.get_logger().info(f"📩 Resultado recibido: {result}")

        if self.state == State.NAVIGATE:
            if "SUCCEEDED" in result:
                self.get_logger().info(f"🎯 Goal {self.current_goal} alcanzado.")
                self.retry_count = 0
                self.current_goal += 1
                self.state = State.IDLE
            else:
                self.retry_count += 1
                if self.retry_count <= self.max_retries:
                    self.get_logger().warn(f"❌ Fallo en navegación, reintentando {self.retry_count}/{self.max_retries}...")
                    self.state = State.NAVIGATE
                else:
                    self.get_logger().error("🚨 Navegación fallida tras 3 intentos. Pasando a DOCK.")
                    self.state = State.DOCK


def main():
    rclpy.init()
    node = Orchestrator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
