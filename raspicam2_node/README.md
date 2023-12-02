# raspicam_node

ROS2 node for the Raspberry Pi Camera Module. Works with both the V1.x and V2.x versions of the module. We recommend using the v2.x cameras as they have better auto gain, and the general image quality is better.

## Build Intructions
1. On Ubuntu: Install the VideoCore libraries. Since these conflict with packages `libgles2-mesa-dev` and `mesa-common-dev`, you have to remove those one first.
```
sudo apt autoremove --purge libgles2-mesa-dev mesa-common-dev
sudo add-apt-repository ppa:ubuntu-pi-flavour-makers/ppa
sudo apt install libraspberrypi-bin libraspberrypi-dev
```
2. Change to the workspace and build:
```
colcon build
```

## Running the Node
1. Activate the camera via `raspi-config` (`sudo apt install raspi-config`)
2. source the workspace and start the node executable
```
source install/setup.bash
ros2 run raspicam2 raspicam2_node --ros-args --params-file `ros2 pkg prefix raspicam2`/share/raspicam2/cfg/params.yaml
```

## Troubleshooting
1. Make sure that your user is in the `video` group by running `groups|grep video`.

2. If you get an error saying: `Failed to create camera component`,
make sure that the camera cable is properly seated on both ends, and that the cable is not missing any pins. If this doesn't work update your firmware with `rpi-update`.

3. If the publish rate of the image over the network is lower than expected, consider using a lower resolution to reduce the amount of bandwidth required.

## Node Information
TODO
