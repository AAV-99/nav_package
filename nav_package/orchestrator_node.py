#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor

# Servicio custom
from nav_package.srv import SetMode

# Para lanzar/terminar stacks por launch
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# ------------------ Estados válidos ------------------
IDLE = "IDLE"
MAPPING = "MAPPING"
NAVIGATION = "NAVIGATION"
DOCKING = "DOCKING"
UNDOCKING = "UNDOCKING"

VALID_MODES = {IDLE, MAPPING, NAVIGATION, DOCKING, UNDOCKING}


class Orchestrator(Node):
    """
    Orquestador con FSM:
      - Controla bring-up/shutdown de stacks Mapping y Navigation
      - Coordina acciones de Dock/Undock
      - Expone /set_mode y publica /orchestrator/mode y /orchestrator/navigation_status
    """
    def __init__(self):
        super().__init__('orchestrator')

        # Publicadores de estado
        self.mode_pub = self.create_publisher(String, '/orchestrator/mode', 10)
        self.nav_status_pub = self.create_publisher(String, '/orchestrator/navigation_status', 10)

        # Servicios
        self.srv_set_mode = self.create_service(SetMode, '/set_mode', self.on_set_mode)

        # Clientes a servicios de Dock/Undock (tu nodo combinado)
        self.cli_dock = self.create_client(Trigger, '/dock')
        self.cli_undock = self.create_client(Trigger, '/undock')

        # Ubicación de los launch del propio paquete
        self.pkg_share = get_package_share_directory('nav_package')
        self.launch_dir = os.path.join(self.pkg_share, 'launch')

        # LaunchService por stack
        self.mapping_ls = None
        self.navigation_ls = None

        # Estado actual
        self.current_mode = IDLE
        self._publish_mode(self.current_mode)
        self._publish_nav_status("idle")

        self.get_logger().info("Orchestrator listo ✅")

    # --------------- Servicios ----------------
    def on_set_mode(self, req, resp):
        target = (req.mode or "").upper().strip()
        if target not in VALID_MODES:
            resp.success = False
            resp.message = f"Modo inválido: {req.mode}. Válidos: {sorted(list(VALID_MODES))}"
            return resp

        try:
            self._transition_to(target)
            resp.success = True
            resp.message = f"Transición completada → {self.current_mode}"
        except Exception as e:
            self.get_logger().error(f"Error en transición: {e}")
            resp.success = False
            resp.message = f"Error: {e}"
        return resp

    # --------------- FSM: Transiciones ----------------
    def _transition_to(self, target_mode: str):
        """
        Reglas simples:
          - MAPPING y NAVIGATION son mutuamente excluyentes.
          - DOCKING/UNDOCKING se tratan como acciones; al finalizar vuelven a IDLE.
        """
        self.get_logger().info(f"Solicitud de transición: {self.current_mode} → {target_mode}")

        if target_mode == IDLE:
            self._stop_mapping()
            self._stop_navigation()
            self._set_mode(IDLE)
            return

        if target_mode == MAPPING:
            # Apagar NAV si estuviera
            self._stop_navigation()
            # Encender Mapping
            self._start_mapping()
            self._set_mode(MAPPING)
            # Estado de navegación (no navegamos en mapping)
            self._publish_nav_status("mapping_active")
            return

        if target_mode == NAVIGATION:
            # Apagar Mapping si estuviera
            self._stop_mapping()
            # Encender Navigation
            self._start_navigation()
            self._set_mode(NAVIGATION)
            self._publish_nav_status("standby")
            return

        if target_mode == DOCKING:
            # Normalmente hacer dock con Nav2 activo (para localización/BT docking),
            # pero puedes permitirlo en cualquiera. Aquí no forzamos.
            self._set_mode(DOCKING)
            self._publish_nav_status("docking")
            self._do_dock()
            # Tras docking, apagamos stacks y vamos a IDLE
            self._stop_mapping()
            self._stop_navigation()
            self._set_mode(IDLE)
            self._publish_nav_status("docked")
            return

        if target_mode == UNDOCKING:
            self._set_mode(UNDOCKING)
            self._publish_nav_status("undocking")
            self._do_undock()
            # Tras undocking quedamos en IDLE por seguridad
            self._set_mode(IDLE)
            self._publish_nav_status("idle")
            return

    # --------------- Acciones Dock/Undock ----------------
    def _do_dock(self):
        if not self.cli_dock.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("Servicio /dock no disponible")
        future = self.cli_dock.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if not resp or not resp.success:
            raise RuntimeError(f"Dock falló: {None if not resp else resp.message}")
        self.get_logger().info(f"Dock OK: {resp.message}")

    def _do_undock(self):
        if not self.cli_undock.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("Servicio /undock no disponible")
        future = self.cli_undock.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if not resp or not resp.success:
            raise RuntimeError(f"Undock falló: {None if not resp else resp.message}")
        self.get_logger().info(f"Undock OK: {resp.message}")

    # --------------- Manejo de stacks ----------------
    def _start_mapping(self):
        if self.mapping_ls:
            self.get_logger().warn("Mapping ya estaba activo")
            return
        self.get_logger().info("Iniciando stack MAPPING...")
        self.mapping_ls = LaunchService()
        src = PythonLaunchDescriptionSource(os.path.join(self.launch_dir, 'mapping.launch.py'))
        action = IncludeLaunchDescription(src)
        ld = LaunchDescription([action])
        self.mapping_ls.include_launch_description(ld)
        self.mapping_ls_task = self.mapping_ls.run_async()
        time.sleep(0.5)  # pequeño respiro

    def _stop_mapping(self):
        if self.mapping_ls:
            self.get_logger().info("Deteniendo stack MAPPING...")
            self.mapping_ls.shutdown()
            self.mapping_ls = None
            time.sleep(0.5)

    def _start_navigation(self):
        if self.navigation_ls:
            self.get_logger().warn("Navigation ya estaba activo")
            return
        self.get_logger().info("Iniciando stack NAVIGATION...")
        self.navigation_ls = LaunchService()
        # Si tu nav necesita argumento 'map', ajusta aquí con IncludeLaunchDescription(..., launch_arguments=...)
        src = PythonLaunchDescriptionSource(os.path.join(self.launch_dir, 'nav.launch.py'))
        action = IncludeLaunchDescription(src)
        ld = LaunchDescription([action])
        self.navigation_ls.include_launch_description(ld)
        self.navigation_ls_task = self.navigation_ls.run_async()
        time.sleep(0.5)

    def _stop_navigation(self):
        if self.navigation_ls:
            self.get_logger().info("Deteniendo stack NAVIGATION...")
            self.navigation_ls.shutdown()
            self.navigation_ls = None
            time.sleep(0.5)

    # --------------- Helpers ----------------
    def _set_mode(self, new_mode: str):
        self.current_mode = new_mode
        self._publish_mode(new_mode)
        self.get_logger().info(f"Modo actual: {self.current_mode}")

    def _publish_mode(self, mode: str):
        msg = String(); msg.data = mode
        self.mode_pub.publish(msg)

    def _publish_nav_status(self, status: str):
        msg = String(); msg.data = status
        self.nav_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        # MultiThreaded por si en el futuro agregas timers/acciones
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
