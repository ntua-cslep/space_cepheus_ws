# ============================================================
# Base image
# ============================================================
FROM ros:noetic-ros-base

# Non-interactive installs, locale, QT shared memory workaround
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    QT_X11_NO_MITSHM=1 \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ============================================================
# User setup (match host UID/GID so mounted files are not root-owned)
# ============================================================
ARG UID=1000
ARG GID=1000
ARG USERNAME=pilot

RUN groupadd -g "${GID}" "${USERNAME}" && \
    useradd -m -u "${UID}" -g "${GID}" -s /bin/bash "${USERNAME}"

# ============================================================
# Base tools + ROS GUI / dev packages
# ============================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        curl \
        wget \
        iputils-ping \
        net-tools \
        locales \
        sudo \
        micro \
        # ROS core dev
        ros-noetic-catkin \
        # ROS GUI / tools
        ros-noetic-rviz \
        ros-noetic-rqt \
        ros-noetic-rqt-graph \
        ros-noetic-rqt-plot \
        ros-noetic-plotjuggler \
        ros-noetic-plotjuggler-ros \
        # Core message packages
        ros-noetic-common-msgs \
        ros-noetic-control-msgs \
        ros-noetic-nav-msgs \
        ros-noetic-gazebo-msgs \
        ros-noetic-kdl-parser \
        # IMU/Xsens-related deps
        ros-noetic-nmea-msgs \
        ros-noetic-mavros-msgs \
        # Diagnostics (needed by vicon and others)
        ros-noetic-diagnostic-updater \
        # Ft Sensor related
        ros-noetic-xacro \
        # ROS control stack (hardware interface, controllers, etc.)
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        # Math / GSL
        libgsl-dev \
        # Shell
        zsh \
        ca-certificates \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Create catkin workspace skeleton
# (will be overlaid by host volume, but avoids weird empty home)
# ============================================================
RUN mkdir -p /home/${USERNAME}/space_cepheus_ws/src && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Bash gets ROS by default
RUN echo 'source /opt/ros/noetic/setup.bash 2>/dev/null || true' >> /home/${USERNAME}/.bashrc

# ============================================================
# Shell tooling: starship (oh-my-zsh is mounted from host)
# ============================================================

# Install starship prompt (global)
RUN curl -fsSL https://starship.rs/install.sh | sh -s -- -y

# Fix ownership, THEN switch to user and configure shell
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}
SHELL ["/bin/bash","-lc"]

# ROS env + workspace overlay in zsh
RUN echo 'if [ -f /opt/ros/noetic/setup.zsh ]; then' >> /home/${USERNAME}/.zshrc && \
    echo '  source /opt/ros/noetic/setup.zsh' >> /home/${USERNAME}/.zshrc && \
    echo 'fi' >> /home/${USERNAME}/.zshrc && \
    echo '[ -f "$HOME/space_cepheus_ws/devel/setup.zsh" ] && source "$HOME/space_cepheus_ws/devel/setup.zsh"' >> /home/${USERNAME}/.zshrc


# Enable starship in zsh (config file is mounted from host)
RUN mkdir -p /home/${USERNAME}/.config && \
    echo 'eval "$(starship init zsh)"' >> /home/${USERNAME}/.zshrc

# ============================================================
# Default command
# ============================================================
CMD ["/usr/bin/zsh"]

