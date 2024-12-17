#!/bin/bash
docker container run -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-desktop-full \
    bash -c "
        echo 'Starting Action Server...'
        python3 /home/codes/raspberry_pi/upi_python/upi_info_display/action_node.py
    "&