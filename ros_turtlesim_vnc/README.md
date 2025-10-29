# ROS2 TurtleSim с VNC и Web-интерфейсом

Этот проект предоставляет ROS2 TurtleSim с возможностью просмотра как через VNC-клиент, так и через веб-браузер.

## Особенности

- ROS2 Jazzy с TurtleSim
- VNC сервер для удаленного доступа
- NoVNC для браузерного доступа
- Демонстрационная программа управления черепахой
- Оконный менеджер Fluxbox
- Volume mount для `src/` - мгновенные изменения кода без пересборки
- Документированные зависимости в `requirements.txt`

## Сборка и запуск

### Рекомендуемый способ: Docker Compose

```bash
# Первый запуск (сборка + запуск)
docker compose up -d --build

# Просмотр логов
docker compose logs -f ros_turtlesim_vnc

# Остановка
docker compose down

# Перезапуск (например, после изменения файлов в src/)
docker compose restart ros_turtlesim_vnc
```

### Альтернативный способ: обычный Docker

```bash
# Сборка образа
docker build -t ros2-jazzy-turtlesim-vnc .

# Запуск контейнера
docker run -p 5901:5901 -p 6080:6080 ros2-jazzy-turtlesim-vnc
```

## Доступ к интерфейсу

### Через VNC клиент
- **Адрес**: `localhost:5901`
- **Пароль**: `password`

### Через веб-браузер
- **URL**: `http://localhost:6080`
- **Пароль**: `password`

## Что происходит при запуске

1. Запускается VNC сервер
2. Настраивается noVNC для браузерного доступа
3. Запускается оконный менеджер Fluxbox
4. Автоматически запускается TurtleSim
5. Запускается демонстрационная программа управления черепахой

## Демонстрационные программы

При запуске контейнера открывается интерактивное меню с выбором программ:

### 🎮 Доступные демонстрации:
1. **Базовое круговое движение** (`main.py`) - простое движение по кругу
2. **Рисование квадрата** (`square_demo.py`) - цветные квадраты
3. **Спиральное движение** (`spiral_demo.py`) - разноцветные спирали
4. **Управление клавишами** (`keyboard_demo.py`) - интерактивное управление WASD
5. **Цветное рисование** (`colorful_demo.py`) - художественные узоры

### 🎛️ Управление программами:
- **s** - Остановить текущую программу
- **r** - Перезапустить TurtleSim
- **c** - Очистить экран TurtleSim
- **t** - Открыть терминал ROS2
- **q** - Выход

### 📝 Добавление новых программ:

#### Для демонстрационных программ:
1. Создайте новый `.py` файл в `src/demo/`
2. Используйте структуру как в существующих примерах
3. Добавьте программу в меню, отредактировав `menu.py`

#### Для рабочих программ (src/tasks/):
1. Создайте новый `.py` файл в `src/tasks/` (например, `topic_nadzor.py`)
2. При использовании docker-compose файлы автоматически синхронизируются с контейнером
3. **❗ Важно**: ROS2 программы требуют загрузки окружения перед запуском

   **Правильный способ запуска:**
   ```bash
   # С настройкой ROS2 окружения
   docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && python3 /ros_workspace/src/tasks/topic_nadzor.py"

   # Интерактивный режим
   docker compose exec ros_turtlesim_vnc bash
   # Внутри контейнера:
   source /opt/ros/jazzy/setup.bash
   python3 /ros_workspace/src/tasks/topic_nadzor.py
   ```

   **❌ Неправильно (вызовет ошибку `ImportError: No module named 'rclpy'`):**
   ```bash
   docker compose exec ros_turtlesim_vnc python3 /ros_workspace/src/tasks/topic_nadzor.py
   ```

4. **Преимущество**: изменения кода применяются без пересборки образа - достаточно перезапуска контейнера

### ✅ Готовые примеры для тестирования:
- `src/tasks/topic_nadzor.py` - основная программа мониторинга сенсоров
- `src/tasks/sensor_publisher.py` - генератор тестовых данных

## Управление контейнером

### Docker Compose (рекомендуется)
```bash
# Остановка
docker compose down

# Перезапуск (сохраняет изменения в src/)
docker compose restart ros_turtlesim_vnc

# Просмотр статуса
docker compose ps

# Просмотр логов
docker compose logs -f ros_turtlesim_vnc
```

### Обычный Docker
```bash
# Остановка (используйте Ctrl+C или)
docker stop <container_id>
```

## Разработка и отладка

### Монтирование volume для src/
При использовании docker-compose папка `src/` монтируется в контейнер, что означает:
- ✅ Изменения файлов применяются мгновенно
- ✅ Не нужно пересобирать образ при изменении кода
- ✅ Можно разрабатывать программы в `src/tasks/` и сразу тестировать

### Полезные команды для разработки

#### 🚀 Запуск ROS2 программ
```bash
# Правильный способ с настройкой окружения
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && python3 /ros_workspace/src/tasks/topic_nadzor.py"

# Интерактивный режим
docker compose exec ros_turtlesim_vnc bash
# Внутри контейнера не забудьте: source /opt/ros/jazzy/setup.bash
```

#### 🔍 Отладка и мониторинг ROS2
```bash
# Посмотреть запущенные ROS2 узлы
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list"

# Мониторинг топиков
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /turtle1/pose"

# Информация о топике
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic info /sensor/light_sensor"

# Отправить тестовое сообщение в топик
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub /sensor/light_sensor std_msgs/Float64 'data: 123.45'"
```

#### ⚠️ Частые ошибки
- **`ImportError: No module named 'rclpy'`** → Забыли выполнить `source /opt/ros/jazzy/setup.bash`
- **Топики не найдены** → Проверьте, что узлы-издатели запущены
- **Программа не получает данные** → Убедитесь в правильности имен топиков
- **IDE показывает ошибки импорта ROS2** → Это нормально, ROS2 пакеты доступны только в контейнере

#### 🧪 Тестирование программ
```bash
# Запустить генератор тестовых данных (в фоне)
docker compose exec -d ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && python3 /ros_workspace/src/tasks/sensor_publisher.py"

# Затем запустить основную программу
docker compose exec ros_turtlesim_vnc bash -c "source /opt/ros/jazzy/setup.bash && python3 /ros_workspace/src/tasks/topic_nadzor.py"

# Остановить фоновые процессы
docker compose exec ros_turtlesim_vnc pkill -f sensor_publisher
```