import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SensorMonitor(Node):
    def __init__(self, sensors):
        super().__init__('sensor_monitor')
        self.sensors = sensors
        self.results = {}
        self.subscribers = []

        for name in sensors:
            topic = f'/sensor/{name}'
            sub = self.create_subscription(
                Float64,
                topic,
                lambda msg, n=name: self.callback(msg, n),
                10
            )
            self.subscribers.append(sub)

    def callback(self, msg, sensor_name):
        # чтобы не принимать повторные значения
        if sensor_name in self.results:
            return

        value = msg.data
        if value < -1000 or value > 1000:
            print(f'{sensor_name} ERROR')
        else:
            print(f'{sensor_name} {value}')

        self.results[sensor_name] = value

        # если получили данные со всех сенсоров — завершаем
        if len(self.results) == len(self.sensors):
            rclpy.shutdown()


def main():
    rclpy.init()

    sensors = ["light_sensor", "smoke_sensor"]
    node = SensorMonitor(sensors)

    # крутим до тех пор, пока не получим все значения
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
