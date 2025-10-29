#!/usr/bin/env python3
"""
Демонстрационная программа ROS2 для headless режима
Показывает основные возможности ROS2 без GUI
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Point
import math
import time
import json


class HeadlessDemo(Node):
    def __init__(self):
        super().__init__('headless_demo')

        # Publishers
        self.status_publisher = self.create_publisher(String, '/demo/status', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/demo/velocity', 10)
        self.position_publisher = self.create_publisher(Point, '/demo/position', 10)
        self.metrics_publisher = self.create_publisher(String, '/demo/metrics', 10)

        # Subscribers
        self.velocity_subscriber = self.create_subscription(
            Twist, '/demo/velocity', self.velocity_callback, 10)

        # Внутренние переменры
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_time = time.time()
        self.message_count = 0

        # Таймеры
        self.move_timer = self.create_timer(0.1, self.update_movement)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.metrics_timer = self.create_timer(5.0, self.publish_metrics)

        self.get_logger().info('🚀 Headless Demo запущен!')
        self.get_logger().info('📊 Будут публиковаться сообщения на топики:')
        self.get_logger().info('   - /demo/status (статус каждую секунду)')
        self.get_logger().info('   - /demo/velocity (команды движения)')
        self.get_logger().info('   - /demo/position (позиция)')
        self.get_logger().info('   - /demo/metrics (метрики каждые 5 секунд)')

    def update_movement(self):
        """Обновляет движение виртуального объекта"""
        current_time = time.time() - self.start_time

        # Создаем паттерн движения
        velocity = Twist()
        velocity.linear.x = 2.0 * math.sin(current_time * 0.5)
        velocity.angular.z = 1.0 * math.cos(current_time * 0.3)

        # Обновляем позицию
        dt = 0.1
        self.x += velocity.linear.x * dt * math.cos(self.theta)
        self.y += velocity.linear.x * dt * math.sin(self.theta)
        self.theta += velocity.angular.z * dt

        # Публикуем скорость и позицию
        self.velocity_publisher.publish(velocity)

        position = Point()
        position.x = self.x
        position.y = self.y
        position.z = self.theta
        self.position_publisher.publish(position)

        self.message_count += 1

    def publish_status(self):
        """Публикует статус системы"""
        current_time = time.time() - self.start_time

        status_data = {
            'uptime_seconds': round(current_time, 1),
            'position': {'x': round(self.x, 2), 'y': round(self.y, 2)},
            'orientation': round(self.theta, 2),
            'messages_sent': self.message_count,
            'status': 'running'
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_publisher.publish(status_msg)

        # Логируем в консоль
        self.get_logger().info(
            f'📍 Позиция: ({self.x:.2f}, {self.y:.2f}), '
            f'Угол: {self.theta:.2f}, '
            f'Время работы: {current_time:.1f}с'
        )

    def publish_metrics(self):
        """Публикует метрики производительности"""
        current_time = time.time() - self.start_time

        metrics = {
            'total_runtime': round(current_time, 1),
            'messages_per_second': round(self.message_count / current_time, 2),
            'distance_traveled': round(math.sqrt(self.x**2 + self.y**2), 2),
            'total_messages': self.message_count
        }

        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics)
        self.metrics_publisher.publish(metrics_msg)

        self.get_logger().info(
            f'📊 МЕТРИКИ: '
            f'Сообщений/сек: {metrics["messages_per_second"]}, '
            f'Пройдено: {metrics["distance_traveled"]:.2f}м'
        )

    def velocity_callback(self, msg):
        """Обработчик полученных команд скорости"""
        # Это показывает как обрабатывать входящие сообщения
        pass


def main(args=None):
    print("🎯 Инициализация ROS2 Headless Demo...")
    rclpy.init(args=args)
    demo_node = HeadlessDemo()

    try:
        print("✅ Демонстрация запущена! Используйте Ctrl+C для остановки")
        print("📡 Мониторинг топиков:")
        print("   ros2 topic echo /demo/status")
        print("   ros2 topic echo /demo/metrics")
        print("")
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        demo_node.get_logger().info('🛑 Программа остановлена пользователем')
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()
        print("👋 ROS2 Headless Demo завершен")


if __name__ == '__main__':
    main()
