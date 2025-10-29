"""
ROS2-узел, который:
1. Автоматически определяет все топики формата /sensor/<sensor_name>
2. Подписывается на каждый из них
3. Считывает одно значение от каждого сенсора
4. Проверяет диапазон (-1000 <= value <= 1000) и выводит результат
"""

import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SensorMonitor(Node):
    """
    ROS2-узел для проверки корректности показаний сенсоров
    """

    def __init__(self):
        """
        Инициализация узла, поиск сенсоров и установка подписок
        """
        super().__init__('sensor_monitor')

        # Получаем все топики и фильтруем по шаблону /sensor/<name>
        topics = self.get_topic_names_and_types()
        pattern = re.compile(r'^/sensor/([a-zA-Z0-9_]+)$')

        self.sensors = [
            match.group(1)
            for name, types in topics
            if pattern.match(name) and 'std_msgs/msg/Float64' in types
            for match in [pattern.match(name)]
        ]

        # Если сенсоры не найдены — завершаем работу
        if not self.sensors:
            self.done = True
            return

        self.results = {}
        self.done = False

        # Подписка на каждый найденный сенсор
        for sensor in self.sensors:
            topic = f'/sensor/{sensor}'
            self.create_subscription(
                Float64,
                topic,
                lambda msg, n=sensor: self.callback(msg, n),
                10
            )

    def callback(self, msg: Float64, sensor_name: str):
        """
        Получает данные от сенсора, проверяет диапазон и выводит результат
        """
        if sensor_name in self.results:
            return  # пропускаем повторные значения

        value = msg.data

        # Вывод строго по формату задачи
        if value < -1000 or value > 1000:
            print(f"{sensor_name} ERROR")
        else:
            print(f"{sensor_name} {value}")

        # Запоминаем, что сенсор обработан
        self.results[sensor_name] = value

        # Завершаем, если получили данные от всех сенсоров
        if len(self.results) == len(self.sensors):
            self.done = True


def main():
    """
    Точка входа в программу
    """
    rclpy.init()
    node = SensorMonitor()

    try:
        # Основной цикл обработки сообщений
        while rclpy.ok() and not getattr(node, "done", False):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
