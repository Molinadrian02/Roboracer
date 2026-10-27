# websim como contenedor independiente (no hace falta X11, noVNC ni RViz).
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ackermann-msgs \
        ros-humble-tf2-ros \
        python3-numpy python3-scipy python3-yaml python3-pil \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /websim_ws
COPY . src/f1tenth_websim
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select f1tenth_websim

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "f1tenth_websim", "websim_launch.py"]
