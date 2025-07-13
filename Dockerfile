# Используем базовый образ ROS Noetic
FROM ros:noetic-ros-base

# Устанавливаем переменную окружения для неинтерактивного режима apt
ENV DEBIAN_FRONTEND=noninteractive

# Устанавливаем необходимые системные зависимости и Python-пакеты
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    # RTAB-Map - мощная SLAM-система, которая поддерживает IMU
    ros-noetic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем Python-библиотеку для работы с WebSocket
RUN pip3 install websocket-client

# Создаем рабочее пространство Catkin
WORKDIR /
RUN mkdir -p ros_ws/src
WORKDIR /ros_ws

# Собираем пустое пространство, чтобы можно было использовать source
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

# *** ИЗМЕНЕНИЯ ЗДЕСЬ ***

# Устанавливаем точку входа, которая будет просто запускать /bin/bash
ENTRYPOINT ["/bin/bash"]

# CMD по умолчанию, чтобы контейнер не завершался сразу после запуска.
# Используем "-c", чтобы /bin/bash выполнил команду, включающую source и tail.
# В этой форме команда передается как единая строка, что позволяет bash
# правильно ее разобрать и выполнить.
CMD ["-c", "source /ros_ws/devel/setup.bash && tail -f /dev/null"]