"""
ROS2-узел, который:
1. Автоматически определяет все топики формата /sensor/<sensor_name>
2. Подписывается на каждый из них
3. Считывает одно значение от каждого сенсора
4. Проверяет диапазон (-1000 <= value <= 1000) и выводит результат
"""

import time
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

        # Поиск сенсорных топиков с ожиданием
        self.sensors = self._find_sensor_topics(timeout=5.0)

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

    def _find_sensor_topics(self, timeout: float = 5.0) -> list[str]:
        """
        Ищет все топики формата /sensor/<sensor_name> с типом std_msgs/msg/Float64.
        Если топики отсутствуют, ожидает их появления в течение заданного времени.
        """
        pattern = re.compile(r'^/sensor/([a-zA-Z0-9_]+)$')
        start = time.time()
        found = []
        time.sleep(0.5)  # небольшая задержка перед первым запросом

        while not found and time.time() - start < timeout:
            topics = self.get_topic_names_and_types()
            found = [
                match.group(1)
                for name, types in topics
                if pattern.match(name) and 'std_msgs/msg/Float64' in types
                for match in [pattern.match(name)]
            ]
            if not found:
                time.sleep(0.1)

        return found


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
