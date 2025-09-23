#!/bin/bash

UNIX=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

CONTAINER_NAME="sofa"
IMAGE_NAME="fernand0labra/sofa:v24.12"

WORKSPACE="/home/YOUR_USER/rai-sofa-assignment"

###

xhost +

# docker build -t fernand0labra/sofa:v24.12 . 
docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME
docker run -itd --gpus all --cpus 28 \
    --privileged \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env XAUTHORITY=$XAUTH \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --volume="$XAUTH:$XAUTH" \
    --volume="$UNIX:$UNIX:rw" \
    --mount type=bind,source=$WORKSPACE,target=/rai-sofa-assignment \
    --mount type=bind,source="/usr/share/vulkan/icd.d",target="/usr/share/vulkan/icd.d" \
    --name $CONTAINER_NAME \
    $IMAGE_NAME
    
