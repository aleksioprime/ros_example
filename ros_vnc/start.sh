#!/bin/bash

# Подгружаем окружение ROS 2
source /opt/ros/jazzy/setup.bash

# Настройка VNC
mkdir -p ~/.vnc
echo "password" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Запуск VNC-сервера
vncserver :1 -geometry 1280x800 -depth 24 -localhost no

# Экспортируем виртуальный дисплей
export DISPLAY=:1

# Запуск noVNC для браузерного доступа
websockify --web=/usr/share/novnc/ 6080 localhost:5901 &

echo "VNC сервер запущен на :1 (порт 5901)"
echo "Web-интерфейс доступен на http://localhost:6080"
echo "Пароль VNC: password"

# Запускаем оконный менеджер
DISPLAY=:1 fluxbox &

# Ждем запуска оконного менеджера
sleep 3

# Запускаем turtlesim
echo "Запуск turtlesim..."
DISPLAY=:1 ros2 run turtlesim turtlesim_node &

# Ждем запуска turtlesim
sleep 5
