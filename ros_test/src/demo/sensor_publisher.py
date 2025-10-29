"""
Простой пример ROS2 программы для тестирования.

Создаёт узел, который публикует случайные значения в два топика:
  - /sensor/light_sensor
  - /sensor/smoke_sensor

Используется для проверки подписчиков (например, SensorMonitor).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class SensorPublisher(Node):
    """
    Узел ROS2, публикующий случайные значения двух сенсоров
    """

    def __init__(self):
        # Инициализация узла с именем 'sensor_publisher'
        super().__init__('sensor_publisher')

        # Создаём два паблишера для топиков /sensor/light_sensor и /sensor/smoke_sensor
        self.sensors = {
            "light_sensor": self.create_publisher(Float64, '/sensor/light_sensor', 10),
            "smoke_sensor": self.create_publisher(Float64, '/sensor/smoke_sensor', 10)
        }

        # Таймер вызывает метод publish_values каждые 0.1 секунды
        self.timer = self.create_timer(0.1, self.publish_values)

    def publish_values(self):
        """Генерирует случайные значения и публикует их в топики."""
        for name, pub in self.sensors.items():
            value = random.uniform(-1500, 1500)  # Случайное значение от -1500 до 1500
            msg = Float64()
            msg.data = value
            pub.publish(msg)
            self.get_logger().info(f'Published: {name}={value:.3f}')


def main():
    """Точка входа: инициализация ROS2 и запуск узла."""
    rclpy.init()
    node = SensorPublisher()
    try:
        # Запуск узла до прерывания (Ctrl+C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Корректное завершение работы узла и ROS2 контекста
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
