#!/usr/bin/env python3
"""
Демонстрационная программа ROS2 - цветное художественное рисование
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
import math
import time
import random


class ColorfulArtist(Node):
    def __init__(self):
        super().__init__('colorful_artist')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Сервисы
        self.clear_client = self.create_client(Empty, '/clear')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.get_logger().info('🎨 Colorful Artist запущен!')

        # Состояние
        self.start_time = time.time()
        self.last_color_change = time.time()
        self.pattern_start = time.time()
        self.current_pattern = 0

        # Паттерны рисования
        self.patterns = [
            {"name": "Цветочек", "duration": 15, "func": self.draw_flower},
            {"name": "Звезда", "duration": 12, "func": self.draw_star},
            {"name": "Волны", "duration": 10, "func": self.draw_waves},
            {"name": "Спираль-радуга", "duration": 18, "func": self.draw_rainbow_spiral}
        ]

        # Таймер
        self.timer = self.create_timer(0.1, self.create_art)

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

        # Начинаем с яркого цвета
        self.set_random_pen_color()

        pattern_name = self.patterns[self.current_pattern]["name"]
        self.get_logger().info(f'🎨 Начинаю рисовать: {pattern_name}!')

    def set_random_pen_color(self):
        """Установить случайный яркий цвет пера"""
        colors = [
            (255, 0, 0),    # Красный
            (0, 255, 0),    # Зеленый
            (0, 0, 255),    # Синий
            (255, 255, 0),  # Желтый
            (255, 0, 255),  # Фиолетовый
            (0, 255, 255),  # Голубой
            (255, 128, 0),  # Оранжевый
            (128, 0, 255),  # Фиолетовый
            (255, 0, 128),  # Розовый
            (128, 255, 0),  # Лайм
        ]

        color = random.choice(colors)
        width = random.randint(2, 5)

        pen_request = SetPen.Request()
        pen_request.r = color[0]
        pen_request.g = color[1]
        pen_request.b = color[2]
        pen_request.width = width
        pen_request.off = False
        self.set_pen_client.call_async(pen_request)

    def draw_flower(self, pattern_time):
        """Рисование цветочка"""
        msg = Twist()

        # Создаем лепестки
        petal_freq = 6.0  # Частота лепестков
        base_speed = 1.5

        msg.linear.x = base_speed + math.sin(pattern_time * petal_freq) * 1.0
        msg.angular.z = 2.0 + math.cos(pattern_time * petal_freq * 0.7) * 1.5

        return msg

    def draw_star(self, pattern_time):
        """Рисование звезды"""
        msg = Twist()

        # Создаем угловатую звезду
        star_freq = 5.0
        angle_pattern = math.sin(pattern_time * star_freq)

        if abs(angle_pattern) > 0.7:
            msg.linear.x = 2.5
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.5
            msg.angular.z = 4.0 * (1 if angle_pattern > 0 else -1)

        return msg

    def draw_waves(self, pattern_time):
        """Рисование волн"""
        msg = Twist()

        wave1 = math.sin(pattern_time * 3.0)
        wave2 = math.cos(pattern_time * 2.0)

        msg.linear.x = 1.8 + wave2 * 0.5
        msg.angular.z = wave1 * 2.5

        return msg

    def draw_rainbow_spiral(self, pattern_time):
        """Рисование радужной спирали"""
        msg = Twist()

        # Увеличивающаяся спираль
        radius_growth = pattern_time * 0.1
        spiral_speed = 3.0 + radius_growth

        msg.linear.x = 0.8 + radius_growth
        msg.angular.z = spiral_speed

        return msg

    def create_art(self):
        """Основной цикл создания искусства"""
        current_time = time.time()
        total_time = current_time - self.start_time
        pattern_time = current_time - self.pattern_start

        # Текущий паттерн
        current_pattern_info = self.patterns[self.current_pattern]
        pattern_duration = current_pattern_info["duration"]
        pattern_func = current_pattern_info["func"]

        # Смена цвета каждые 2-3 секунды
        if current_time - self.last_color_change > random.uniform(2.0, 3.5):
            self.set_random_pen_color()
            self.last_color_change = current_time

        # Смена паттерна
        if pattern_time >= pattern_duration:
            self.current_pattern = (self.current_pattern + 1) % len(self.patterns)
            self.pattern_start = current_time
            pattern_time = 0

            # Очищаем экран для нового паттерна
            self.clear_client.call_async(Empty.Request())

            pattern_name = self.patterns[self.current_pattern]["name"]
            self.get_logger().info(f'🎨 Переключаемся на: {pattern_name}!')

            # Пауза перед новым паттерном
            time.sleep(0.5)

        # Выполняем текущий паттерн
        msg = pattern_func(pattern_time)

        # Публикуем команду
        self.publisher_.publish(msg)

        # Логируем прогресс
        if int(pattern_time * 2) % 4 == 0 and pattern_time - int(pattern_time * 2) / 2 < 0.05:
            progress = int((pattern_time / pattern_duration) * 100)
            pattern_name = current_pattern_info["name"]
            self.get_logger().info(f'🎨 {pattern_name}: {progress}% выполнено')


def main(args=None):
    print("🎨 Инициализация Colorful Artist...")
    rclpy.init(args=args)

    artist = ColorfulArtist()

    try:
        print("✅ Художественное рисование запущено!")
        print("🌈 Черепаха будет рисовать разноцветные узоры:")
        print("   - Цветочки")
        print("   - Звезды")
        print("   - Волны")
        print("   - Радужные спирали")
        print("🎭 Цвета и толщина линий меняются автоматически")
        print("⏹️  Используйте Ctrl+C для остановки")
        rclpy.spin(artist)
    except KeyboardInterrupt:
        artist.get_logger().info('🛑 Рисование остановлено')
    finally:
        artist.destroy_node()
        rclpy.shutdown()
        print("👋 Colorful Artist завершен")


if __name__ == '__main__':
    main()