#!/usr/bin/env python3
"""
Демонстрационная программа ROS2 - спиральное движение
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
import math
import time


class SpiralDrawer(Node):
    def __init__(self):
        super().__init__('spiral_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Сервисы
        self.clear_client = self.create_client(Empty, '/clear')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.get_logger().info('🌀 Spiral Drawer запущен!')

        # Состояние
        self.start_time = time.time()
        self.cycle_start = time.time()
        self.cycle_duration = 20.0  # секунд на один цикл спирали

        # Таймер
        self.timer = self.create_timer(0.1, self.draw_spiral)

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

        # Настраиваем перо - синий цвет
        self.set_pen_color(0, 0, 255, 2)

        self.get_logger().info('🎨 Начинаю рисовать синюю спираль!')

    def set_pen_color(self, r, g, b, width):
        """Установить цвет пера"""
        pen_request = SetPen.Request()
        pen_request.r = r
        pen_request.g = g
        pen_request.b = b
        pen_request.width = width
        pen_request.off = False
        self.set_pen_client.call_async(pen_request)

    def draw_spiral(self):
        """Основной цикл рисования спирали"""
        current_time = time.time()
        total_time = current_time - self.start_time
        cycle_time = current_time - self.cycle_start

        msg = Twist()

        if cycle_time >= self.cycle_duration:
            # Начинаем новый цикл с новым цветом
            self.cycle_start = current_time
            cycle_time = 0

            # Меняем цвет
            colors = [
                (255, 0, 0, "красную"),    # Красный
                (0, 255, 0, "зеленую"),    # Зеленый
                (0, 0, 255, "синюю"),      # Синий
                (255, 255, 0, "желтую"),   # Желтый
                (255, 0, 255, "фиолетовую"), # Фиолетовый
                (0, 255, 255, "голубую")   # Голубой
            ]

            color_index = int(total_time / self.cycle_duration) % len(colors)
            color = colors[color_index]

            self.set_pen_color(color[0], color[1], color[2], 2)
            self.get_logger().info(f'🎨 Начинаю рисовать {color[3]} спираль!')

            # Очищаем экран для новой спирали
            self.clear_client.call_async(Empty.Request())

        # Параметры спирали
        progress = cycle_time / self.cycle_duration  # 0.0 - 1.0

        # Увеличивающаяся скорость вращения
        base_angular = 2.0
        angular_speed = base_angular + progress * 3.0

        # Изменяющаяся линейная скорость для создания спирали
        base_linear = 0.5
        linear_speed = base_linear + progress * 2.0

        # Добавляем волнообразность
        wave_factor = math.sin(cycle_time * 4.0) * 0.3

        msg.linear.x = linear_speed + wave_factor
        msg.angular.z = angular_speed

        # Публикуем команду
        self.publisher_.publish(msg)

        # Логируем прогресс
        if int(cycle_time * 2) % 4 == 0 and cycle_time - int(cycle_time * 2) / 2 < 0.05:
            progress_percent = int(progress * 100)
            self.get_logger().info(f'🌀 Спираль: {progress_percent}% завершено')


def main(args=None):
    print("🌀 Инициализация Spiral Drawer...")
    rclpy.init(args=args)

    spiral_drawer = SpiralDrawer()

    try:
        print("✅ Рисование спирали запущено!")
        print("🎨 Черепаха будет рисовать цветные спирали")
        print("🌈 Каждые 20 секунд - новый цвет и новая спираль")
        print("⏹️  Используйте Ctrl+C для остановки")
        rclpy.spin(spiral_drawer)
    except KeyboardInterrupt:
        spiral_drawer.get_logger().info('🛑 Рисование остановлено')
    finally:
        spiral_drawer.destroy_node()
        rclpy.shutdown()
        print("👋 Spiral Drawer завершен")


if __name__ == '__main__':
    main()