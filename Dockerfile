FROM ros:kinetic-ros-base-xenial

MAINTAINER viniciusbaltoe@gmail.com

RUN apt-get update && apt-get install -y vim \
    && apt-get install -y python3 \
    && apt-get install -y python3-pip \
    && pip3 install opencv-contrib-python \ #Este nao funciona na rasp.
    && apt-get clean

# Config de acesso ao Display
ENV QT_X11_NO_MITSHM 1

# Config de funcionamento do roscore (para poder dar ping no pr'oprio pc)
ENV ROS_MASTER_URI http://localhost:11311
ENV ROS_HOSTNAME localhost

# Pacote para o funcionamento do turtlesim no ROS
RUN apt install -y ros-kinetic-turtlesim 

# Pacote para o funcionamento do OpenCV no ROS
RUN apt install  -y ros-kinetic-cv-bridge 
RUN apt install  -y ros-kinetic-image-transport 
RUN apt-get install -y ros-kinetic-usb-cam
# RUN apt install  -y ros-kinetic-image-geometry

