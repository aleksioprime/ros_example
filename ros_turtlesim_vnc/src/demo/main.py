#!/usr/bin/env python3
"""
Демонстрационная программа ROS2 для управления черепахой в turtlesim
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Turtle Controller запущен!')

        # Таймер для управления движением
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.start_time = time.time()

    def move_turtle(self):
        msg = Twist()
        current_time = time.time() - self.start_time

        # Создаем круговое движение
        msg.linear.x = 2.0
        msg.angular.z = 1.0 * math.sin(current_time * 0.5)

        self.publisher_.publish(msg)

        # Логируем каждые 2 секунды
        if int(current_time) % 2 == 0 and current_time - int(current_time) < 0.1:
            self.get_logger().info(f'Черепаха движется: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()

    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        turtle_controller.get_logger().info('Программа остановлена')
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()