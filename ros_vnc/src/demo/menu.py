#!/usr/bin/env python3
"""
Интерактивное меню для запуска различных демонстрационных программ ROS2
"""
import os
import sys
import subprocess
import signal
import time
import threading
from pathlib import Path

class DemoMenu:
    def __init__(self):
        self.current_process = None
        self.base_path = Path("/ros_workspace/src/demo")
        self.programs = {
            "1": {"name": "Базовое круговое движение", "file": "main.py"},
            "2": {"name": "Рисование квадрата", "file": "square_demo.py"},
            "3": {"name": "Спиральное движение", "file": "spiral_demo.py"},
            "4": {"name": "Управление клавишами", "file": "keyboard_demo.py"},
            "5": {"name": "Рисование цветами", "file": "colorful_demo.py"},
        }

    def show_menu(self):
        """Показать главное меню"""
        os.system('clear')
        print("🐢 ROS2 TurtleSim Демонстрационное меню")
        print("=" * 50)
        print()

        for key, program in self.programs.items():
            status = "✅ Доступно" if (self.base_path / program["file"]).exists() else "❌ Не найдено"
            print(f"{key}. {program['name']} - {status}")

        print()
        print("Управление:")
        print("s. Остановить текущую программу")
        print("r. Перезапустить TurtleSim")
        print("c. Очистить экран TurtleSim")
        print("t. Открыть терминал ROS2")
        print("q. Выход")
        print()

        if self.current_process:
            print(f"🟢 Сейчас запущена программа (PID: {self.current_process.pid})")
        else:
            print("⚪ Нет запущенных программ")
        print()

    def stop_current_program(self):
        """Остановить текущую программу"""
        if self.current_process and self.current_process.poll() is None:
            print("🛑 Остановка текущей программы...")
            self.current_process.terminate()
            try:
                self.current_process.wait(timeout=5)
                print("✅ Программа остановлена")
            except subprocess.TimeoutExpired:
                print("⚠️  Принудительная остановка...")
                self.current_process.kill()
                self.current_process.wait()
            self.current_process = None
        else:
            print("ℹ️  Нет запущенных программ")

    def run_program(self, program_file):
        """Запустить программу"""
        self.stop_current_program()

        program_path = self.base_path / program_file
        if not program_path.exists():
            print(f"❌ Файл {program_file} не найден!")
            return

        print(f"🚀 Запуск {program_file}...")
        print("💡 Для остановки выберите 's' в меню или используйте Ctrl+C")

        env = os.environ.copy()
        env['DISPLAY'] = ':1'
        env['PYTHONPATH'] = '/opt/ros/jazzy/lib/python3.12/site-packages'

        try:
            self.current_process = subprocess.Popen([
                'python3', str(program_path)
            ], env=env, cwd=str(self.base_path.parent.parent))

            # Даем программе время на запуск
            time.sleep(1)

            if self.current_process.poll() is None:
                print("✅ Программа запущена успешно")
            else:
                print("❌ Программа завершилась с ошибкой")
                self.current_process = None

        except Exception as e:
            print(f"❌ Ошибка запуска: {e}")
            self.current_process = None

    def restart_turtlesim(self):
        """Перезапустить TurtleSim"""
        print("🔄 Перезапуск TurtleSim...")

        # Остановим все процессы turtlesim
        os.system("pkill -f turtlesim_node")
        time.sleep(2)

        # Запустим заново
        env = os.environ.copy()
        env['DISPLAY'] = ':1'

        subprocess.Popen([
            'ros2', 'run', 'turtlesim', 'turtlesim_node'
        ], env=env)

        print("✅ TurtleSim перезапущен")
        time.sleep(2)

    def clear_turtlesim(self):
        """Очистить экран TurtleSim"""
        print("🧹 Очистка экрана TurtleSim...")
        try:
            env = os.environ.copy()
            env['DISPLAY'] = ':1'

            result = subprocess.run([
                'ros2', 'service', 'call', '/clear', 'std_srvs/srv/Empty'
            ], env=env, capture_output=True, text=True, timeout=5)

            if result.returncode == 0:
                print("✅ Экран очищен")
            else:
                print("❌ Не удалось очистить экран")

        except subprocess.TimeoutExpired:
            print("⚠️  Тайм-аут при очистке экрана")
        except Exception as e:
            print(f"❌ Ошибка: {e}")

    def open_ros_terminal(self):
        """Открыть терминал с ROS2"""
        print("🖥️  Открытие ROS2 терминала...")
        print("Доступные команды:")
        print("  ros2 topic list")
        print("  ros2 topic echo /turtle1/cmd_vel")
        print("  ros2 topic echo /turtle1/pose")
        print("  ros2 service list")
        print("  ros2 run turtlesim turtle_teleop_key")
        print()
        print("Для возврата в меню введите 'exit'")
        print()

        # Настройка окружения
        os.system("source /opt/ros/jazzy/setup.bash && bash")

    def run(self):
        """Главный цикл меню"""
        print("🚀 Запуск демонстрационного меню...")
        time.sleep(1)

        # Обработчик сигналов для корректного завершения
        def signal_handler(sig, frame):
            print("\n🛑 Получен сигнал завершения...")
            self.stop_current_program()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        try:
            while True:
                self.show_menu()
                choice = input("Выберите опцию: ").strip().lower()

                if choice == 'q':
                    self.stop_current_program()
                    print("👋 До свидания!")
                    break
                elif choice == 's':
                    self.stop_current_program()
                    input("Нажмите Enter для продолжения...")
                elif choice == 'r':
                    self.restart_turtlesim()
                    input("Нажмите Enter для продолжения...")
                elif choice == 'c':
                    self.clear_turtlesim()
                    input("Нажмите Enter для продолжения...")
                elif choice == 't':
                    self.open_ros_terminal()
                elif choice in self.programs:
                    program = self.programs[choice]
                    self.run_program(program["file"])
                    input("Нажмите Enter для продолжения...")
                else:
                    print("❌ Неверный выбор. Попробуйте снова.")
                    time.sleep(1)

        except KeyboardInterrupt:
            print("\n🛑 Принудительное завершение...")
            self.stop_current_program()
        except Exception as e:
            print(f"❌ Неожиданная ошибка: {e}")
            self.stop_current_program()


if __name__ == '__main__':
    menu = DemoMenu()
    menu.run()