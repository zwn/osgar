FROM osrf/subt-virtual-testbed:subt_solution_latest

RUN sudo apt update
RUN /usr/bin/python3 -m pip install --user --no-cache-dir msgpack pyserial "opencv-python>3.4.2,<4" pyzmq
RUN git clone https://github.com/robotika/osgar.git osgar \
  && cd osgar \
  && git checkout feature/virtual-ros-md

COPY run_solution.bash ./

#COPY subt_seed_node.cc ./src/subt_seed/src/subt_seed_node.cc
#COPY CMakeLists.txt ./src/subt_seed/CMakeLists.txt

ADD ros ./src

RUN sudo apt install -y ros-melodic-image-geometry \
			ros-melodic-pcl-ros \
			ros-melodic-costmap-2d \
			libdxflib-dev \
			ros-melodic-rtabmap-ros \
			ros-melodic-teleop-twist-keyboard \
			ros-melodic-move-base \
			ros-melodic-dwa-local-planner \
			ros-melodic-robot-localization \
			mc

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_make install'


ENTRYPOINT ["./run_solution.bash"]

