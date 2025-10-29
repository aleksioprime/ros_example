# Базовый образ с ROS2 Jazzy и поддержкой GUI
FROM osrf/ros:jazzy-desktop

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlesim \
    tigervnc-standalone-server \
    novnc websockify \
    fluxbox \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Создание рабочей директории
WORKDIR /ros_workspace

# Копируем демонстрационные файлы
COPY src/ ./src/

# Копируем стартовый скрипт
COPY --chmod=755 start.sh /start.sh

# Открываем порты для VNC и web-интерфейса
EXPOSE 5901 6080

# Команда по умолчанию
CMD ["/start.sh"]
