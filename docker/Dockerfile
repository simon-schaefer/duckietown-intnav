# Note: this container will have the name duckietown/rpi-duckiebot-base
FROM duckietown/rpi-ros-kinetic-base:master18

RUN [ "cross-build-start" ]

ENV READTHEDOCS True
RUN /bin/bash -c "ls /"
COPY src/duckietown-intnav/lib-intnav/requirements.txt /requirements.txt

RUN pip install -r /requirements.txt

RUN mkdir -p /home/software/intersection_ws

COPY . /home/software/intersection_ws

ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
RUN /bin/bash -c "cd /home/software/ && source /opt/ros/kinetic/setup.bash && catkin_make -j -C intersection_ws/"
RUN /bin/bash -c "cd /home/software/intersection_ws && source devel/setup.bash"

RUN mkdir /home/software/docker
COPY src/duckietown-intnav/docker/env.sh /home/software/docker/env.sh
RUN echo "source /home/software/docker/env.sh" >> ~/.bashrc

RUN [ "cross-build-end" ]

WORKDIR /home/software/intersection_ws

CMD [ "/bin/bash" ]

ENV DISABLE_CONTRACTS=1

LABEL maintainer="Simon Schaefer sischaef@student.ethz.ch"
