FROM osrf/subt-virtual-testbed:subt_solution_latest

# RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack pyserial "opencv-python>3.4.2,<4" pyzmq
RUN git clone https://github.com/robotika/osgar.git osgar \
  && cd osgar \
  && git checkout feature/virtual-ros

COPY run_solution.bash ./

COPY subt_seed_node.cc ./src/subt_seed/src/subt_seed_node.cc
COPY CMakeLists.txt ./src/subt_seed/CMakeLists.txt

ADD ros ./src/

RUN sudo apt install -y ros-melodic-image-geometry
RUN sudo apt install -y ros-melodic-pcl-ros
RUN sudo apt install -y ros-melodic-costmap-2d
RUN sudo apt install -y libdxflib-dev
RUN sudo apt install -y ros-melodic-rtabmap-ros
RUN sudo apt install -y ros-melodic-teleop-twist-keyboard
RUN sudo apt install -y ros-melodic-move-base
RUN sudo apt install -y ros-melodic-dwa-local-planner
RUN sudo apt install -y ros-melodic-robot-localization
RUN sudo apt install -y mc

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'


ENTRYPOINT ["./run_solution.bash"]

