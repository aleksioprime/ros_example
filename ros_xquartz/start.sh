#!/bin/bash

# Подгружаем окружение ROS 2
source /opt/ros/jazzy/setup.bash

echo "=== ROS2 XQuartz Demo ==="
echo "Для корректной работы убедитесь что:"
echo "1. XQuartz запущен на хост-системе"
echo "2. Разрешен доступ для сетевых клиентов"
echo "3. Контейнер запущен с правильными параметрами X11"
echo ""

# Проверяем доступность дисплея
if [ -z "$DISPLAY" ]; then
    echo "❌ Переменная DISPLAY не установлена!"
    echo "Убедитесь, что контейнер запущен с параметром -e DISPLAY=\$DISPLAY"
    exit 1
fi

echo "✅ DISPLAY: $DISPLAY"

# Тест X11 соединения
echo "🔧 Тестирование X11 соединения..."
if command -v xeyes > /dev/null; then
    timeout 3 xeyes &
    X11_PID=$!
    sleep 1
    if kill -0 $X11_PID 2>/dev/null; then
        echo "✅ X11 соединение работает"
        kill $X11_PID
    else
        echo "❌ Проблемы с X11 соединением"
    fi
else
    echo "⚠️  xeyes не найден, пропускаем тест"
fi

# Запускаем turtlesim
echo "🐢 Запуск TurtleSim..."
ros2 run turtlesim turtlesim_node &
TURTLE_PID=$!

# Даем время на запуск
sleep 3

# Проверяем, что turtlesim запустился
if kill -0 $TURTLE_PID 2>/dev/null; then
    echo "✅ TurtleSim запущен (PID: $TURTLE_PID)"
else
    echo "❌ Не удалось запустить TurtleSim"
    exit 1
fi

# Переходим в рабочую директорию и запускаем демо
cd /ros_workspace
echo "🎮 Запуск демонстрационной программы управления..."
python3 src/demo/main.py