#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import paho.mqtt.client as mqtt
import importlib

from threading import Thread

class MQTTBridge(Node):
    def __init__(self, config_file):
        super().__init__('mqtt_bridge')

        # Cargar configuración YAML
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        # Configuración broker
        broker_cfg = self.config['mqtt']['broker']
        self.mqtt_client = mqtt.Client()
        if broker_cfg.get("username"):
            self.mqtt_client.username_pw_set(broker_cfg["username"], broker_cfg["password"])
        self.mqtt_client.connect(broker_cfg["host"], broker_cfg["port"], 60)

        # Diccionarios para ros↔mqtt
        self.ros_to_mqtt = {}
        self.mqtt_to_ros = {}

        # Crear publishers/subscribers ROS ↔ MQTT
        self._setup_ros_to_mqtt()
        self._setup_mqtt_to_ros()

        # Lanzar cliente MQTT en hilo aparte
        Thread(target=self.mqtt_client.loop_forever, daemon=True).start()

    # =========================
    # ROS → MQTT
    # =========================
    def _setup_ros_to_mqtt(self):
        for bridge in self.config['mqtt']['ros_to_mqtt']:
            ros_topic = bridge['topic']
            msg_type = self._import_msg_type(bridge['type'])
            mqtt_topic = bridge['mqtt_topic']

            self.get_logger().info(f"ROS→MQTT: {ros_topic} -> {mqtt_topic}")

            sub = self.create_subscription(
                msg_type,
                ros_topic,
                lambda msg, mt=mqtt_topic: self._ros_to_mqtt_callback(msg, mt),
                10
            )
            self.ros_to_mqtt[ros_topic] = (sub, mqtt_topic)

    def _ros_to_mqtt_callback(self, msg, mqtt_topic):
        try:
            # Convertir a dict serializable
            data = self._ros_msg_to_dict(msg)
            self.mqtt_client.publish(mqtt_topic, yaml.dump(data))
        except Exception as e:
            self.get_logger().error(f"Error en ROS→MQTT ({mqtt_topic}): {e}")

    # =========================
    # MQTT → ROS
    # =========================
    def _setup_mqtt_to_ros(self):
        for bridge in self.config['mqtt']['mqtt_to_ros']:
            mqtt_topic = bridge['mqtt_topic']
            ros_topic = bridge['ros_topic']
            msg_type = self._import_msg_type(bridge['type'])

            self.get_logger().info(f"MQTT→ROS: {mqtt_topic} -> {ros_topic}")

            pub = self.create_publisher(msg_type, ros_topic, 10)
            self.mqtt_to_ros[mqtt_topic] = (pub, msg_type)

            # Subscribirse al tópico MQTT
            self.mqtt_client.subscribe(mqtt_topic)
            self.mqtt_client.message_callback_add(
                mqtt_topic,
                lambda client, userdata, msg, mt=mqtt_topic: self._mqtt_to_ros_callback(msg, mt)
            )

    def _mqtt_to_ros_callback(self, msg, mqtt_topic):
        try:
            pub, msg_type = self.mqtt_to_ros[mqtt_topic]
            data = yaml.safe_load(msg.payload.decode("utf-8"))
            ros_msg = self._dict_to_ros_msg(data, msg_type)
            pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().error(f"Error en MQTT→ROS ({mqtt_topic}): {e}")

    # =========================
    # Utils
    # =========================
    def _import_msg_type(self, type_str):
        module_name, msg_name = type_str.split('/msg/')
        module = importlib.import_module(module_name + '.msg')
        return getattr(module, msg_name)

    def _ros_msg_to_dict(self, msg):
        # Convierte un ROS2 msg en dict (recursivo)
        if hasattr(msg, '__slots__'):
            return {slot: self._ros_msg_to_dict(getattr(msg, slot)) for slot in msg.__slots__}
        elif isinstance(msg, (list, tuple)):
            return [self._ros_msg_to_dict(x) for x in msg]
        else:
            return msg

    def _dict_to_ros_msg(self, data, msg_type):
        msg = msg_type()
        for field in data:
            if hasattr(msg, field):
                setattr(msg, field, data[field])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridge("install/nav_package/share/nav_package/config/mqtt_topics.yaml")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
