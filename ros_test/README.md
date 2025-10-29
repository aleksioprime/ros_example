# Тестирование программ на ROS 2 (Jazzy)

## Подготовка окружения

Склонируйте репозиторий проекта (если ещё не сделали):
```sh
git clone https://github.com/aleksioprime/ros_example.git
cd ros_test
```

Соберите и запустите контейнер:
```sh
docker compose -p ros_test up -d --build
```

Войдите внутрь контейнера:
```sh
docker exec -it ros_test bash
```

Проверьте, что окружение ROS2 активировано и топики доступны:
```sh
ros2 topic list
```

Если всё работает, вы должны увидеть базовые топики:
```bash
/parameter_events
/rosout
```

Выход из контейнера:
```sh
exit
```

## Запуск программ

### Публикация данных сенсоров

Откройте новый терминал и запустите программу, которая публикует случайные значения сенсоров в топики:
```sh
docker exec -it ros_test bash

python3 src/demo/sensor_publisher.py
```

### Подписка на данные сенсоров

Откройте другой терминал и проверьте, что данные действительно публикуются:
```sh
docker exec -it ros_test bash

ros2 topic echo --once /sensor/light_sensor
ros2 topic echo --once /sensor/smoke_sensor
```

Запустите программму, которая считывает данные из топиков:
```sh
docker exec -it ros_test python3 src/tasks/topic_nadzor.py
```

После тестирования можно остановить контейнер:
```sh
docker compose -p ros_test down
```