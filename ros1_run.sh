#!/bin/bash
docker run -it --rm \
  --gpus all \
  --name ros1 \
  --cpus=0.0 \
  --shm-size=4G \
  --net host \
  --ipc=host \
  -v ${HOME}:/mnt:rw \
  -v /media:/media:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${HOME}/.Xauthority:/root/.Xauthority:rw \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /dev/video0:/dev/video0 \
  -v /dev/video1:/dev/video1 \
  -v /dev/video2:/dev/video2 \
  -v /dev/video3:/dev/video3 \
  -v /dev/video4:/dev/video4 \
  -v /dev/video5:/dev/video5 \
  -v /dev/video6:/dev/video6 \
  -v /dev/video7:/dev/video7 \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --privileged \
  tiryoh/ros-desktop-vnc:melodic
