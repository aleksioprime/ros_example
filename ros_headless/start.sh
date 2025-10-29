#!/bin/bash

# Подгружаем окружение ROS 2
source /opt/ros/jazzy/setup.bash

echo "=== ROS2 Headless Demo ==="
echo "Запуск демонстрационной программы без GUI..."
echo "Программа будет публиковать сообщения и выводить информацию в консоль"
echo "Нажмите Ctrl+C для остановки"
echo ""

# Переходим в рабочую директорию
cd /ros_workspace

# Запускаем демонстрационную программу
python3 src/demo/main.py