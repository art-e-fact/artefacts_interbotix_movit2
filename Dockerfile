FROM public.ecr.aws/artefacts/ros2:humble-turtlesim

RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 && \
    mkdir -p /etc/apt/sources.list.d && \
    echo deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install --yes --quiet libgazebo-dev libgazebo11 gazebo ros-humble-hardware-interface ros-humble-moveit-common ros-humble-dynamixel-sdk
RUN apt-get install --yes --quiet ros-humble-graph-msgs
RUN apt-get install --yes --quiet ros-humble-moveit-core
RUN apt-get install --yes --quiet ros-humble-moveit-ros-planning
RUN apt-get install --yes --quiet ros-humble-rviz-visual-tools
RUN apt-get install --yes --quiet ros-humble-ros2-control-test-assets
RUN apt-get install --yes --quiet ros-humble-robot-state-publisher
RUN apt-get install --yes --quiet ros-humble-xacro
RUN apt-get install --yes --quiet ros-humble-moveit-ros-move-group
RUN apt-get install --yes --quiet ros-humble-gazebo-ros
RUN apt-get install --yes --quiet ros-humble-ompl
RUN apt-get install --yes --quiet ros-humble-moveit-planners-ompl
RUN apt-get install --yes --quiet ros-humble-moveit-ros-control-interface
RUN apt-get install --yes --quiet ros-humble-controller-manager
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY src src
RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src src --default-yes --rosdistro humble && \
    source /usr/share/gazebo/setup.bash && \
    colcon build

COPY artefacts.yaml .
CMD source /opt/ros/humble/setup.bash && source /usr/share/gazebo/setup.bash && source /ws/install/setup.bash && artefacts run $ARTEFACTS_JOB_NAME
