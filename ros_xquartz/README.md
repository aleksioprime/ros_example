# ROS2 TurtleSim с XQuartz (X11 Forwarding)

Этот проект демонстрирует запуск ROS2 TurtleSim в Docker-контейнере с отображением через XQuartz на macOS.

## Особенности

- ROS2 Jazzy с полной поддержкой GUI
- TurtleSim с X11 forwarding через XQuartz
- Продвинутая демонстрационная программа с разноцветным рисованием
- Автоматическое тестирование X11 соединения
- Циклическая демонстрация различных паттернов движения

## Требования

### macOS с XQuartz

1. **Установите XQuartz:**
   ```bash
   brew install --cask xquartz
   ```

2. **Запустите XQuartz** и включите сетевые подключения:
   - Откройте XQuartz
   - Перейдите в XQuartz → Settings → Security
   - Включите "Allow connections from network clients"
   - Перезапустите XQuartz

3. **Разрешите подключения:**
   ```bash
   xhost +localhost
   ```

## Сборка и запуск

### 1. Сборка Docker-образа

```bash
docker build -t ros2-xquartz-demo .
```

### 2. Запуск контейнера

```bash
docker run -it \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-xquartz-demo
```

### Альтернативный запуск (если первый не работает)

```bash
# Узнайте IP адрес хоста
export HOST_IP=$(ifconfig en0 | grep inet | grep -v inet6 | awk '{print $2}')

# Разрешите подключения с этого IP
xhost +$HOST_IP

# Запустите контейнер
docker run -it \
  -e DISPLAY=$HOST_IP:0 \
  ros2-xquartz-demo
```

## Что происходит при запуске

1. **Проверка X11**: Тестируется соединение с XQuartz
2. **Запуск TurtleSim**: Автоматически открывается окно TurtleSim
3. **Демонстрация**: Запускается циклическая программа рисования:
   - 🔴 **Красный круг** (20 секунд)
   - 🟢 **Зеленый квадрат** (16 секунд)
   - 🔵 **Синие волны** (15 секунд)
   - 🟡 **Желтая спираль** (10 секунд)
   - 🔄 **Повтор цикла**

## Демонстрационная программа

Файл `src/demo/main.py` включает:
- Управление движением черепахи
- Изменение цветов и толщины пера
- Различные геометрические паттерны
- ROS2 сервисы для управления TurtleSim
- Подробное логирование

## Устранение неполадок

### Окно не отображается
```bash
# Проверьте XQuartz
ps aux | grep XQuartz

# Проверьте разрешения
xhost

# Перезапустите XQuartz
sudo pkill XQuartz
open -a XQuartz
```

### Ошибки подключения
```bash
# Проверьте переменную DISPLAY
echo $DISPLAY

# Проверьте сетевые настройки XQuartz
# Settings → Security → "Allow connections from network clients"
```

### Проблемы с Docker
```bash
# Проверьте режим Docker Desktop
# Docker Desktop → Settings → Use Docker Compose V2
```

## Интерактивное использование

Для ручного управления подключитесь к контейнеру:

```bash
docker run -it \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-xquartz-demo bash
```

Затем:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node &
ros2 run turtlesim turtle_teleop_key
```