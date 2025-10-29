# ROS2 TurtleSim VNC

Простой ROS2 TurtleSim с VNC доступом через браузер.

## Запуск

```bash
docker compose up -d --build
```

## Доступ

- **Браузер**: http://localhost:6080 (пароль: password)
- **VNC**: localhost:5901 (пароль: password)

## Демо программы

```bash
# Запуск меню
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && python3 /home/ros/ros_workspace/src/demo/menu.py"
```