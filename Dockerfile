FROM osrf/ros:humble-desktop

# Evita prompts interactivos durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# --- Dependencias base (Lab 1) + simulador f1tenth_gym_ros + simulador web f1tenth_websim ---
# libeigen3-dev, tmux, ros-humble-rviz2 vienen del Dockerfile oficial del simulador
# (que usa ros-foxy-rviz2; aqui usamos la version Humble equivalente).
# numpy / scipy / yaml / pillow son las dependencias Python de f1tenth_websim
# (rosdep tambien las resolveria desde su package.xml; se declaran aqui de forma
# explicita para que la imagen funcione aunque rosdep falle sin red).
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ackermann-msgs \
    ros-humble-rviz2 \
    libeigen3-dev \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    python3-pil \
    nano \
    vim \
    git \
    tmux \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# transforms3d: dependencia Python del simulador (paquete puro, no depende de la distro de ROS)
RUN pip3 install transforms3d

# f1tenth_gym: el simulador en si, es un paquete Python puro instalado con pip -e .
# NOTA: f1tenth_gym declara como dependencia gym==0.19.0, que tiene metadatos rotos
# e incompatibles con versiones de pip >= 24.1 (ver: github.com/f1tenth/f1tenth_gym/issues/138).
# Por eso fijamos una version de pip anterior antes de instalar el simulador.
RUN pip3 install "pip<24.1"
RUN git clone https://github.com/f1tenth/f1tenth_gym /opt/f1tenth_gym && \
    cd /opt/f1tenth_gym && \
    pip3 install -e .

# Inicializa rosdep (si no está ya inicializado en la imagen base)
RUN rosdep update || true

# Instala las dependencias declaradas por los paquetes del workspace (package.xml),
# incluyendo las de f1tenth_gym_ros (ej. nav2_lifecycle_manager, joint_state_publisher, etc.)
# Esto se hornea en la imagen para que sobreviva a futuros rebuilds, sin depender
# de ejecutar rosdep manualmente cada vez dentro del contenedor.
COPY src /tmp/src_for_rosdep
RUN apt-get update && \
    rosdep install -i --from-path /tmp/src_for_rosdep --rosdistro humble -y && \
    rm -rf /tmp/src_for_rosdep && \
    rm -rf /var/lib/apt/lists/*

# --- Usuario no-root con el mismo UID/GID que el usuario del host ---
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Workspace de trabajo
WORKDIR /f1tenth_ws
RUN chown $USER_UID:$USER_GID /f1tenth_ws

# También le damos permiso al usuario sobre el clon de f1tenth_gym,
# por si en algún momento necesita reinstalarlo en modo editable (-e)
RUN chown -R $USER_UID:$USER_GID /opt/f1tenth_gym

# Fuente ROS 2 automáticamente en cada shell interactiva (para el nuevo usuario)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f /f1tenth_ws/install/setup.bash ]; then source /f1tenth_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Prompt personalizado para diferenciar el contenedor de la terminal del host
RUN echo "export PS1='\[\033[1;36m\]🏎️  f1tenth\[\033[0m\] \[\033[1;32m\]\w\[\033[0m\] # '" >> /home/$USERNAME/.bashrc

RUN chown $USER_UID:$USER_GID /home/$USERNAME/.bashrc

USER $USERNAME

CMD ["/bin/bash"]