FROM osrf/ros:humble-desktop

# Evita prompts interactivos durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# Dependencias del sistema y herramientas útiles
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ackermann-msgs \
    nano \
    vim \
    git \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Inicializa rosdep (si no está ya inicializado en la imagen base)
RUN rosdep update || true

# --- Usuario no-root con el mismo UID/GID que el usuario del host ---
# Esto evita que los archivos creados dentro del contenedor (sobre el
# volumen montado) queden con dueño "root" y no puedas editarlos desde fuera.
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Workspace de trabajo
WORKDIR /f1tenth_ws
RUN chown $USER_UID:$USER_GID /f1tenth_ws

# Fuente ROS 2 automáticamente en cada shell interactiva (para el nuevo usuario)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f /f1tenth_ws/install/setup.bash ]; then source /f1tenth_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Prompt personalizado para diferenciar el contenedor de la terminal del host
# Formato: 🏎️ f1tenth /ruta/actual #  (en cian y verde)
RUN echo "export PS1='\[\033[1;36m\]🏎️  f1tenth\[\033[0m\] \[\033[1;32m\]\w\[\033[0m\] # '" >> /home/$USERNAME/.bashrc

RUN chown $USER_UID:$USER_GID /home/$USERNAME/.bashrc

USER $USERNAME

CMD ["/bin/bash"]