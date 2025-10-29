#!/usr/bin/env python3
"""
Демонстрационная программа ROS2 - управление клавишами
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import sys
import select
import termios
import tty
import time
import os


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.get_logger().info('⌨️  Keyboard Controller запущен!')

        # Настройки скорости
        self.linear_speed = 2.0
        self.angular_speed = 2.0

        # Состояние клавиш
        self.keys_pressed = set()

        # Настройка терминала
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Таймер для обработки движения
        self.timer = self.create_timer(0.1, self.process_movement)

        # Ждем сервис пера
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Ожидание сервиса /turtle1/set_pen...')

        # Включаем перо
        self.set_pen_state(True, 255, 0, 0, 2)  # Красное перо

        self.show_instructions()

    def show_instructions(self):
        """Показать инструкции управления"""
        print("\n" + "="*50)
        print("🐢 УПРАВЛЕНИЕ ЧЕРЕПАХОЙ КЛАВИШАМИ")
        print("="*50)
        print("Движение:")
        print("  W/↑ - Вперед")
        print("  S/↓ - Назад")
        print("  A/← - Поворот влево")
        print("  D/→ - Поворот вправо")
        print()
        print("Перо:")
        print("  1 - Красное перо")
        print("  2 - Зеленое перо")
        print("  3 - Синее перо")
        print("  4 - Желтое перо")
        print("  0 - Выключить перо")
        print("  9 - Включить перо")
        print()
        print("Управление:")
        print("  + - Увеличить скорость")
        print("  - - Уменьшить скорость")
        print("  SPACE - Стоп")
        print("  Q - Выход")
        print("="*50)
        print("💡 Можно зажимать несколько клавиш одновременно!")
        print("Текущая скорость: {:.1f}".format(self.linear_speed))
        print()

    def set_pen_state(self, on, r=255, g=0, b=0, width=2):
        """Установить состояние пера"""
        pen_request = SetPen.Request()
        pen_request.r = r
        pen_request.g = g
        pen_request.b = b
        pen_request.width = width
        pen_request.off = not on
        self.set_pen_client.call_async(pen_request)

    def get_key(self):
        """Неблокирующее чтение клавиши"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            # Обработка специальных клавиш (стрелки)
            if ord(key) == 27:  # ESC sequence
                next1 = sys.stdin.read(1)
                next2 = sys.stdin.read(1)
                if next1 == '[':
                    if next2 == 'A':
                        return 'W'  # Up arrow
                    elif next2 == 'B':
                        return 'S'  # Down arrow
                    elif next2 == 'C':
                        return 'D'  # Right arrow
                    elif next2 == 'D':
                        return 'A'  # Left arrow
            return key
        return None

    def process_movement(self):
        """Обработка движения на основе нажатых клавиш"""
        # Читаем новые клавиши
        key = self.get_key()
        if key:
            key = key.upper()

            # Обработка специальных команд
            if key == 'Q':
                self.get_logger().info('👋 Выход по запросу пользователя')
                self.cleanup()
                rclpy.shutdown()
                return
            elif key == ' ':
                self.keys_pressed.clear()
                self.get_logger().info('⏹️  СТОП')
            elif key == '+' or key == '=':
                self.linear_speed = min(5.0, self.linear_speed + 0.5)
                self.angular_speed = min(5.0, self.angular_speed + 0.5)
                print(f"⚡ Скорость увеличена до {self.linear_speed:.1f}")
            elif key == '-':
                self.linear_speed = max(0.5, self.linear_speed - 0.5)
                self.angular_speed = max(0.5, self.angular_speed - 0.5)
                print(f"🐌 Скорость уменьшена до {self.linear_speed:.1f}")
            elif key in ['1', '2', '3', '4']:
                colors = {
                    '1': (255, 0, 0, "красное"),
                    '2': (0, 255, 0, "зеленое"),
                    '3': (0, 0, 255, "синее"),
                    '4': (255, 255, 0, "желтое")
                }
                color = colors[key]
                self.set_pen_state(True, color[0], color[1], color[2], 3)
                print(f"🎨 Установлено {color[3]} перо")
            elif key == '0':
                self.set_pen_state(False)
                print("✏️  Перо выключено")
            elif key == '9':
                self.set_pen_state(True, 255, 0, 0, 2)
                print("✏️  Перо включено (красное)")
            elif key in ['W', 'S', 'A', 'D']:
                if key in self.keys_pressed:
                    self.keys_pressed.remove(key)
                else:
                    self.keys_pressed.add(key)

        # Создаем команду движения на основе нажатых клавиш
        msg = Twist()

        if 'W' in self.keys_pressed:
            msg.linear.x += self.linear_speed
        if 'S' in self.keys_pressed:
            msg.linear.x -= self.linear_speed
        if 'A' in self.keys_pressed:
            msg.angular.z += self.angular_speed
        if 'D' in self.keys_pressed:
            msg.angular.z -= self.angular_speed

        # Публикуем команду
        self.publisher_.publish(msg)

    def cleanup(self):
        """Очистка ресурсов"""
        # Восстанавливаем настройки терминала
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        # Останавливаем черепаху
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)


def main(args=None):
    print("⌨️  Инициализация Keyboard Controller...")
    rclpy.init(args=args)

    controller = None
    try:
        controller = KeyboardController()
        print("✅ Управление клавишами активировано!")
        print("🎮 Используйте WASD для управления черепахой")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        if controller:
            controller.get_logger().info('🛑 Управление остановлено')
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        if controller:
            controller.cleanup()
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("👋 Keyboard Controller завершен")


if __name__ == '__main__':
    main()