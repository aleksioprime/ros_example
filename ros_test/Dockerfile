# Используем официальный базовый образ ROS 2 Jazzy (вариант "ros-base" без GUI)
# Он уже содержит rclpy, colcon, ros2cli и стандартные пакеты
FROM ros:jazzy-ros-base

# Добавляем команду, которая при каждом запуске bash будет автоматически
# подгружать окружение ROS 2 (setup.bash).
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Устанавливаем рабочую директорию внутри контейнера,
# где будут храниться исходники и запускаться команды.
WORKDIR /ros_workspace