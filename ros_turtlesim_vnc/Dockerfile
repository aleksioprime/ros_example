# Базовый образ с ROS2 Jazzy и поддержкой GUI
FROM osrf/ros:jazzy-desktop

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlesim \
    tigervnc-standalone-server \
    novnc websockify \
    fluxbox \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Создание пользователя ros
RUN useradd -m -s /bin/bash ros && \
    echo "ros:password" | chpasswd && \
    usermod -aG sudo ros

# Переключение на пользователя ros
USER ros
WORKDIR /home/ros

# Создание рабочей директории
RUN mkdir -p /home/ros/ros_workspace
WORKDIR /home/ros/ros_workspace

# Копируем демонстрационные файлы
COPY --chown=ros:ros src/ ./src/

# Делаем скрипты исполняемыми
RUN chmod +x ./src/demo/*.py ./src/demo/examples/*.py

# Копируем стартовый скрипт
COPY --chown=ros:ros --chmod=755 start.sh /home/ros/start.sh

# Создание базовых ROS2 файлов настройки
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc && \
    echo "export DISPLAY=:1" >> /home/ros/.bashrc

# Открываем порты для VNC и web-интерфейса
EXPOSE 5901 6080

# Команда по умолчанию
CMD ["/home/ros/start.sh"]
