"""
Простой пример ROS2 программы для тестирования.

Создаёт узел, который публикует случайные значения в несколько топиков вида:
  /sensor/<sensor_name>

Набор сенсоров формируется случайно при запуске из доступного списка.
Используется для проверки подписчиков (например, SensorMonitor)
"""

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SensorPublisher(Node):
    """
    Узел ROS2, публикующий случайные значения для случайного набора сенсоров
    """

    def __init__(self):
        super().__init__('sensor_publisher')

        # Доступные типы сенсоров — список можно расширить при необходимости
        available_sensors = [
            "light_sensor",
            "smoke_sensor",
            "temperature_sensor",
            "humidity_sensor",
            "pressure_sensor",
            "motion_sensor",
            "sound_sensor",
        ]

        # Случайно выбираем от 2 до 5 сенсоров из списка
        selected = random.sample(available_sensors, random.randint(2, 5))

        self.get_logger().info(f"Активные сенсоры: {', '.join(selected)}")

        # Создаём паблишеры для выбранных топиков
        self.sensors = {
            name: self.create_publisher(Float64, f'/sensor/{name}', 10)
            for name in selected
        }

        # Таймер вызывает publish_values каждые 0.2 секунды
        self.timer = self.create_timer(0.2, self.publish_values)

    def publish_values(self):
        """
        Генерирует случайные значения и публикует их в топики
        """
        for name, pub in self.sensors.items():
            value = random.uniform(-1500, 1500)
            msg = Float64()
            msg.data = value
            pub.publish(msg)
            self.get_logger().info(f'Published: {name}={value:.3f}')


def main():
    """
    Точка входа: инициализирует ROS2 и запускает узел
    """
    rclpy.init()
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
