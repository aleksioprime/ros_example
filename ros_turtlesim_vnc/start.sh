#!/bin/bash

# Подгружаем окружение ROS 2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

# Настройка VNC
mkdir -p /home/ros/.vnc
echo "password" | vncpasswd -f > /home/ros/.vnc/passwd
chmod 600 /home/ros/.vnc/passwd

# Очистка старых процессов VNC
vncserver -kill :1 2>/dev/null || true

# Запуск VNC-сервера
vncserver :1 -geometry 1280x800 -depth 24 -localhost no

# Экспортируем виртуальный дисплей
export DISPLAY=:1

# Запуск noVNC для браузерного доступа
websockify --web=/usr/share/novnc/ 6080 localhost:5901 &

echo "VNC сервер запущен на :1 (порт 5901)"
echo "Web-интерфейс доступен на http://localhost:6080"
echo "Пароль VNC: password"

# Создаем базовую конфигурацию для fluxbox
mkdir -p /home/ros/.fluxbox
echo "session.screen0.workspaces: 1" > /home/ros/.fluxbox/init

# Запускаем оконный менеджер
DISPLAY=:1 fluxbox &

# Ждем запуска оконного менеджера
sleep 3

# Запускаем turtlesim
echo "Запуск turtlesim..."
DISPLAY=:1 ros2 run turtlesim turtlesim_node &

# Ждем запуска turtlesim
sleep 5

# Держим контейнер активным
echo "Готово! VNC: localhost:5901, Web: http://localhost:6080"
while true; do
    sleep 60
done
