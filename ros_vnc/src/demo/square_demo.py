#!/usr/bin/env python3
"""
Демонстрационная программа ROS2 - рисование квадрата
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
import math
import time


class SquareDrawer(Node):
    def __init__(self):
        super().__init__('square_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Сервисы
        self.clear_client = self.create_client(Empty, '/clear')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.get_logger().info('🟦 Square Drawer запущен!')

        # Состояние
        self.side = 0  # Какая сторона квадрата (0-3)
        self.start_time = time.time()
        self.phase = 'moving'  # 'moving' или 'turning'
        self.phase_start = time.time()

        # Настройки
        self.side_length_time = 2.0  # секунд на сторону
        self.turn_time = 0.5  # секунд на поворот
        self.linear_speed = 2.0
        self.angular_speed = math.pi / 2 / self.turn_time  # 90 градусов за turn_time секунд

        # Таймер
        self.timer = self.create_timer(0.1, self.draw_square)

        # Инициализация
        self.setup_drawing()

    def setup_drawing(self):
        """Настройка рисования"""
        # Ждем сервисы
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Ожидание сервиса /clear...')

        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Ожидание сервиса /turtle1/set_pen...')

        # Очищаем экран
        self.clear_client.call_async(Empty.Request())

        # Настраиваем перо - зеленый цвет
        pen_request = SetPen.Request()
        pen_request.r = 0
        pen_request.g = 255
        pen_request.b = 0
        pen_request.width = 3
        pen_request.off = False
        self.set_pen_client.call_async(pen_request)

        self.get_logger().info('🎨 Начинаю рисовать зеленый квадрат!')

    def draw_square(self):
        """Основной цикл рисования квадрата"""
        current_time = time.time()
        phase_duration = current_time - self.phase_start
        total_time = current_time - self.start_time

        msg = Twist()

        if self.side >= 4:
            # Квадрат завершен
            if total_time < 15.0:  # Показываем результат 15 секунд
                msg.linear.x = 0.0
                msg.angular.z = 0.0

                if int(total_time) % 3 == 0 and total_time - int(total_time) < 0.1:
                    self.get_logger().info('🎉 Квадрат завершен! Любуемся результатом...')
            else:
                # Начинаем заново
                self.get_logger().info('🔄 Начинаем новый квадрат!')
                self.side = 0
                self.phase = 'moving'
                self.phase_start = current_time
                self.start_time = current_time

                # Меняем цвет
                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
                color = colors[int(total_time / 15) % len(colors)]

                pen_request = SetPen.Request()
                pen_request.r = color[0]
                pen_request.g = color[1]
                pen_request.b = color[2]
                pen_request.width = 3
                pen_request.off = False
                self.set_pen_client.call_async(pen_request)
        else:
            if self.phase == 'moving':
                # Движение по стороне квадрата
                if phase_duration < self.side_length_time:
                    msg.linear.x = self.linear_speed
                    msg.angular.z = 0.0

                    if int(phase_duration * 2) % 2 == 0 and phase_duration - int(phase_duration * 2) / 2 < 0.05:
                        self.get_logger().info(f'🟦 Рисую сторону {self.side + 1}/4...')
                else:
                    # Переходим к повороту
                    self.phase = 'turning'
                    self.phase_start = current_time
                    self.get_logger().info(f'🔄 Поворачиваю после стороны {self.side + 1}')

            elif self.phase == 'turning':
                # Поворот на 90 градусов
                if phase_duration < self.turn_time:
                    msg.linear.x = 0.0
                    msg.angular.z = self.angular_speed
                else:
                    # Переходим к следующей стороне
                    self.side += 1
                    self.phase = 'moving'
                    self.phase_start = current_time

                    if self.side < 4:
                        self.get_logger().info(f'✅ Поворот завершен, начинаю сторону {self.side + 1}')

        # Публикуем команду
        self.publisher_.publish(msg)


def main(args=None):
    print("🟦 Инициализация Square Drawer...")
    rclpy.init(args=args)

    square_drawer = SquareDrawer()

    try:
        print("✅ Рисование квадрата запущено!")
        print("🎨 Черепаха будет рисовать квадраты разными цветами")
        print("⏹️  Используйте Ctrl+C для остановки")
        rclpy.spin(square_drawer)
    except KeyboardInterrupt:
        square_drawer.get_logger().info('🛑 Рисование остановлено')
    finally:
        square_drawer.destroy_node()
        rclpy.shutdown()
        print("👋 Square Drawer завершен")


if __name__ == '__main__':
    main()