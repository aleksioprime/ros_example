# Базовый образ с ROS2 Jazzy и поддержкой GUI
FROM ros:jazzy-desktop

# Установка зависимостей для X11
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlesim \
    python3-pip \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Создание рабочей директории
WORKDIR /ros_workspace

# Копируем демонстрационные файлы
COPY src/ ./src/

# Копируем стартовый скрипт
COPY --chmod=755 start.sh /start.sh

# Команда по умолчанию
CMD ["/start.sh"]