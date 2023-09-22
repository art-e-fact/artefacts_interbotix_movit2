FROM public.ecr.aws/artefacts/ros2:humble-turtlesim

RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 && \
    mkdir -p /etc/apt/sources.list.d && \
    echo deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install --yes --quiet libgazebo-dev libgazebo11 gazebo && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY src src
RUN source /opt/ros/humble/setup.bash && \
    colcon build && \
    source /ws/install/setup.bash && \
    source /usr/share/gazebo/setup.bash

COPY artefacts.yaml .
CMD ["artefacts", "run", "$ARTEFACTS_JOB_NAME"]
