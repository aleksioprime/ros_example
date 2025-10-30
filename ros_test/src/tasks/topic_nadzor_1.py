"""
ROS2-узел, который подписывается на несколько сенсорных топиков
и выводит полученные значения с проверкой диапазона.
Завершает работу после получения первого сообщения от каждого сенсора
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SensorMonitor(Node):
    """
    ROS2-узел, который подписывается на несколько сенсорных топиков
    и выводит полученные значения с проверкой диапазона.
    Завершает работу после получения первого сообщения от каждого сенсора
    """

    def __init__(self, sensors: list[str]):
        """
        Создаёт подписки для всех указанных сенсоров
        """
        super().__init__('sensor_monitor')
        self.sensors = sensors
        self.results = {}
        self.done = False

        for name in sensors:
            topic = f'/sensor/{name}'
            self.create_subscription(
                Float64,
                topic,
                lambda msg, n=name: self.callback(msg, n),
                10
            )

    def callback(self, msg: Float64, sensor_name: str) -> None:
        """
        Обрабатывает первое полученное сообщение от сенсора
        """
        if sensor_name in self.results:
            return

        value = msg.data
        if value < -1000 or value > 1000:
            print(f'{sensor_name} ERROR')
        else:
            print(f'{sensor_name} {value}')

        self.results[sensor_name] = value
        if len(self.results) == len(self.sensors):
            self.done = True


def main() -> None:
    """
    Точка входа: инициализирует ROS2, запускает узел и обрабатывает события.
    """
    rclpy.init()
    sensors = ["light_sensor", "smoke_sensor"]
    node = SensorMonitor(sensors)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
