FROM duckietown/rpi-ros-kinetic-base:master18

RUN [ "cross-build-start" ]

ENV READTHEDOCS True
RUN /bin/bash -c "ls /"
COPY src/duckietown-intnav/docker/requirements.txt /requirements.txt
COPY src/duckietown-intnav/docker/misc/environment.sh /home/software/environment.sh
COPY src/duckietown-intnav/docker/misc/duckie.art /home/software/misc/duckie.art
COPY src/duckietown-intnav/docker/misc/init_config_defaults.sh /home/software/docker/init_config_defaults.sh

RUN pip install -r /requirements.txt

RUN mkdir -p /home/software/catkin_ws

COPY . /home/software/catkin_ws

ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
RUN /bin/bash -c "cd /home/software/ && source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"
RUN /bin/bash -c "cd /home/software/catkin_ws && source devel/setup.bash"

COPY src/duckietown-intnav/docker/misc/env.sh /home/software/docker/env.sh
RUN echo "source /home/software/docker/env.sh" >> ~/.bashrc

RUN [ "cross-build-end" ]

WORKDIR /home/software/catkin_ws

CMD [ "/bin/bash" ]

ENV DISABLE_CONTRACTS=1

LABEL maintainer="Simon Schaefer sischaef@student.ethz.ch"