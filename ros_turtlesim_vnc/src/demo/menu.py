#!/usr/bin/env python3
"""
Простое меню для запуска демонстрационных программ ROS2
"""
import os
import sys
import subprocess
from pathlib import Path

class DemoMenu:
    def __init__(self):
        self.base_path = Path("/home/ros/ros_workspace/src/demo")
        self.programs = {
            "1": {"name": "Базовое движение", "file": "main.py"},
            "2": {"name": "Квадрат", "file": "examples/square.py"},
            "3": {"name": "Спираль", "file": "examples/spiral.py"},
            "4": {"name": "Клавиши", "file": "keyboard.py"},
            "5": {"name": "Цветное рисование", "file": "examples/colorful.py"},
        }

    def show_menu(self):
        """Показать меню"""
        print("\n🐢 TurtleSim Демо")
        print("-" * 20)
        for key, program in self.programs.items():
            print(f"{key}. {program['name']}")
        print("q. Выход")

    def run_program(self, program_file):
        """Запустить программу"""
        program_path = self.base_path / program_file
        if not program_path.exists():
            print(f"Файл {program_file} не найден!")
            return

        env = os.environ.copy()
        env['DISPLAY'] = ':1'
        subprocess.run(['python3', str(program_path)], env=env)

    def run(self):
        """Главный цикл меню"""
        while True:
            self.show_menu()
            choice = input("Выбор: ").strip()

            if choice == 'q':
                break
            elif choice in self.programs:
                program = self.programs[choice]
                self.run_program(program["file"])
            else:
                print("Неверный выбор")


if __name__ == '__main__':
    menu = DemoMenu()
    menu.run()