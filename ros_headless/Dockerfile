# Базовый образ с ROS2 Jazzy (без GUI)
FROM ros:jazzy-ros-base

# Установка необходимых пакетов
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Создание рабочей директории
WORKDIR /ros_workspace

# Копируем демонстрационные файлы
COPY src/ ./src/

# Копируем стартовый скрипт
COPY --chmod=755 start.sh /start.sh

# Команда по умолчанию
CMD ["/start.sh"]