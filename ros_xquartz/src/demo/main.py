#!/usr/bin/env python3
"""
Демонстрационная программа ROS2 для XQuartz (X11 forwarding)
Управляет черепахой в TurtleSim через X11 forwarding
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Empty
import math
import time
import random


class XQuartzTurtleDemo(Node):
    def __init__(self):
        super().__init__('xquartz_turtle_demo')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Service clients
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.clear_client = self.create_client(Empty, '/clear')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Текущая поза черепахи
        self.current_pose = None
        self.demo_phase = 0
        self.start_time = time.time()
        self.phase_start_time = time.time()

        # Таймер для демонстрации
        self.demo_timer = self.create_timer(0.1, self.run_demo)

        self.get_logger().info('🎨 XQuartz Turtle Demo запущен!')
        self.get_logger().info('🖥️  Окно TurtleSim должно отобразиться через XQuartz')

        # Ждем готовности сервисов
        self.wait_for_services()

        # Инициализация демонстрации
        self.setup_demo()

    def wait_for_services(self):
        """Ждет готовности ROS2 сервисов"""
        services = [
            (self.clear_client, '/clear'),
            (self.set_pen_client, '/turtle1/set_pen'),
            (self.spawn_client, '/spawn')
        ]

        for client, service_name in services:
            self.get_logger().info(f'⏳ Ожидание сервиса {service_name}...')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'⏳ Сервис {service_name} пока недоступен...')

        self.get_logger().info('✅ Все сервисы готовы!')

    def setup_demo(self):
        """Настройка демонстрации"""
        # Очищаем экран
        self.call_clear_service()

        # Настраиваем перо
        self.set_pen_color(255, 0, 0, 3, True)  # Красное перо

    def call_clear_service(self):
        """Очищает экран TurtleSim"""
        request = Empty.Request()
        future = self.clear_client.call_async(request)

    def set_pen_color(self, r, g, b, width, on):
        """Устанавливает цвет и параметры пера"""
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = not on

        future = self.set_pen_client.call_async(request)

    def pose_callback(self, msg):
        """Обработчик позиции черепахи"""
        self.current_pose = msg

    def run_demo(self):
        """Основной цикл демонстрации"""
        if self.current_pose is None:
            return

        current_time = time.time()
        phase_duration = current_time - self.phase_start_time
        total_time = current_time - self.start_time

        cmd = Twist()

        # Фаза 0: Рисуем круг (20 секунд)
        if self.demo_phase == 0:
            if phase_duration < 20.0:
                cmd.linear.x = 2.0
                cmd.angular.z = 1.0
                if int(phase_duration) % 5 == 0 and phase_duration - int(phase_duration) < 0.1:
                    self.get_logger().info(f'🔴 Рисуем красный круг... ({phase_duration:.1f}s)')
            else:
                self.demo_phase = 1
                self.phase_start_time = current_time
                self.set_pen_color(0, 255, 0, 5, True)  # Зеленое перо
                self.get_logger().info('🟢 Переходим к зеленому квадрату!')

        # Фаза 1: Рисуем квадрат (16 секунд)
        elif self.demo_phase == 1:
            if phase_duration < 16.0:
                cycle_time = phase_duration % 4.0
                if cycle_time < 2.0:
                    cmd.linear.x = 2.0  # Движение вперед
                    cmd.angular.z = 0.0
                elif cycle_time < 2.5:
                    cmd.linear.x = 0.0  # Поворот
                    cmd.angular.z = math.pi / 2.0 / 0.5  # 90 градусов за 0.5 секунды
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                if int(phase_duration) % 4 == 0 and phase_duration - int(phase_duration) < 0.1:
                    self.get_logger().info(f'🟢 Рисуем зеленый квадрат... ({phase_duration:.1f}s)')
            else:
                self.demo_phase = 2
                self.phase_start_time = current_time
                self.set_pen_color(0, 0, 255, 2, True)  # Синее перо
                self.get_logger().info('🔵 Переходим к синим волнам!')

        # Фаза 2: Волнообразное движение (15 секунд)
        elif self.demo_phase == 2:
            if phase_duration < 15.0:
                cmd.linear.x = 1.5
                cmd.angular.z = 2.0 * math.sin(phase_duration * 2.0)
                if int(phase_duration) % 3 == 0 and phase_duration - int(phase_duration) < 0.1:
                    self.get_logger().info(f'🔵 Рисуем синие волны... ({phase_duration:.1f}s)')
            else:
                self.demo_phase = 3
                self.phase_start_time = current_time
                self.set_pen_color(255, 255, 0, 4, True)  # Желтое перо
                self.get_logger().info('🟡 Переходим к желтой спирали!')

        # Фаза 3: Спираль (10 секунд)
        elif self.demo_phase == 3:
            if phase_duration < 10.0:
                cmd.linear.x = 0.5 + phase_duration * 0.2
                cmd.angular.z = 3.0
                if int(phase_duration) % 2 == 0 and phase_duration - int(phase_duration) < 0.1:
                    self.get_logger().info(f'🟡 Рисуем желтую спираль... ({phase_duration:.1f}s)')
            else:
                self.demo_phase = 4
                self.phase_start_time = current_time
                self.get_logger().info('🎉 Демонстрация завершена! Повторяем...')
                self.call_clear_service()

        # Фаза 4: Пауза и перезапуск
        elif self.demo_phase == 4:
            if phase_duration < 3.0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                self.demo_phase = 0
                self.phase_start_time = current_time
                self.set_pen_color(255, 0, 0, 3, True)  # Возвращаемся к красному
                self.get_logger().info('🔄 Начинаем заново с красного круга!')

        # Публикуем команду движения
        self.cmd_vel_pub.publish(cmd)

        # Логируем общее время каждые 10 секунд
        if int(total_time) % 10 == 0 and total_time - int(total_time) < 0.1:
            self.get_logger().info(f'⏱️  Общее время работы: {total_time:.1f} секунд')


def main(args=None):
    print("🎨 Инициализация XQuartz Turtle Demo...")
    rclpy.init(args=args)

    demo_node = XQuartzTurtleDemo()

    try:
        print("✅ Демонстрация запущена!")
        print("🖥️  Следите за окном TurtleSim в XQuartz")
        print("🎨 Будут нарисованы: круг, квадрат, волны и спираль")
        print("🔄 Демонстрация повторяется циклически")
        print("⏹️  Используйте Ctrl+C для остановки")
        print("")
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        demo_node.get_logger().info('🛑 Демонстрация остановлена пользователем')
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()
        print("👋 XQuartz Turtle Demo завершен")


if __name__ == '__main__':
    main()