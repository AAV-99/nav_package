#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import time

# Importar el wrapper de turtlebot4
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    # Crear navegador del TurtleBot4
    navigator = TurtleBot4Navigator()

    # 🔹 Chequear si el robot está dockeado y hacer undock antes de arrancar
    if navigator.getDockedStatus():
        navigator.info("El robot está en el dock, realizando undock...")
        navigator.undock()
    else:
        navigator.info("El robot ya está fuera del dock.")
        
     # Crear pose inicial con helper
    #initial_pose = navigator.getPoseStamped([-0.29221153259277344, 0.003913849592208862],
    #                                    TurtleBot4Directions.SOUTH)

    # Establecerla
    #navigator.setInitialPose(initial_pose)


    # Esperar que Nav2 esté activo
    #navigator.waitUntilNav2Active() 


    # Cargar archivo YAML con ubicaciones
    pkg_path = get_package_share_directory('nav_package')
    yaml_path = os.path.join(pkg_path, 'config', 'locations.yaml')

    with open(yaml_path, 'r') as f:
        locations = yaml.safe_load(f)['locations']

    # Loop interactivo
    while True:
        print("\n=== Coordenadas disponibles ===")
        for key, val in locations.items():
            print(f"{key}: x={val['x']}, y={val['y']}")
        print("q: salir")

        choice = input("\nSelecciona el punto al que quieres ir: ").strip()

        if choice.lower() == 'q':
            print("Saliendo...")
            break

        if choice not in locations:
            print("Punto inválido, intenta de nuevo.")
            continue

        # Crear pose objetivo orientada al ESTE
        goal_pose = navigator.getPoseStamped(
            [locations[choice]['x'], locations[choice]['y']],
            TurtleBot4Directions.EAST
        )

        print(f"\n🚀 Navegando hacia {choice}...")

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    "ETA: {:.0f} segundos.".format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    )
                )
            i += 1

        result = navigator.getResult()
        if result == navigator.TaskResult.SUCCEEDED:
            print(f"✅ Meta {choice} alcanzada.")
        elif result == navigator.TaskResult.CANCELED:
            print(f"⚠️ Meta {choice} cancelada.")
        elif result == navigator.TaskResult.FAILED:
            print(f"❌ Meta {choice} fallida.")
            navigator.warn("Reintentando con pose inicial...")
            navigator.setInitialPose(initial_pose)
            wait_for_nav2(navigator, timeout=MAX_WAIT)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
