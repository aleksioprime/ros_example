# Тестирование программ на ROS

## Подготовка:

Перейдите в папку проекта и запустите контейнер:
```sh
cd ros_test
docker compose -p ros_test up -d --build
```

Войдите внутрь контейнера:
```sh
docker exec -it ros_test bash
```

Проверьте топики:
```sh
ros2 topic list
```

Выход из контейнера:
```sh
exit
```

## Запуск программ:

Запустите программму для публикации значения с сенсоров в топики в отдельном терминале:
```sh
docker exec -it ros_test bash
python3 src/demo/sensor_publisher.py
```

Проверьте публикацию данных в топики:
```sh
ros2 topic echo --once /sensor/light_sensor
ros2 topic echo --once /sensor/smoke_sensor
```

Запустите программму для получния значения с сенсоров из топиков в отдельном терминале:
```sh
docker exec -it ros_test bash
docker exec -it ros_test python3 src/tasks/topic_nadzor.py
```