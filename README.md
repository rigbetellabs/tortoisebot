# Tortoisebot ROS2 Humble Ignition Release

# ![TortoiseBot Banner](https://github.com/rigbetellabs/tortoisebot_docs/raw/master/imgs/packaging/pack_front.png)

![stars](https://img.shields.io/github/stars/rigbetellabs/tortoisebot?style=for-the-badge)
![forks](https://img.shields.io/github/forks/rigbetellabs/tortoisebot?style=for-the-badge)
![watchers](https://img.shields.io/github/watchers/rigbetellabs/tortoisebot?style=for-the-badge)
![repo-size](https://img.shields.io/github/repo-size/rigbetellabs/tortoisebot?style=for-the-badge)
![contributors](https://img.shields.io/github/contributors/rigbetellabs/tortoisebot?style=for-the-badge)

---
<p align="center"><a href="#connect-with-us-">Connect with Us</a> • <a href="#1-installation">Installation</a> • <a href="#2-setup">Setup</a> • <a href="#3-demos">Demos</a>

<h1 align="center"> TortoiseBot </h1>

# Connect with us ![some-changes](https://img.shields.io/badge/some_changes-yellow)

<a href="https://rigbetellabs.com/">![Website](https://img.shields.io/website?down_color=lightgrey&down_message=offline&label=Rigbetellabs%20Website&style=for-the-badge&up_color=green&up_message=online&url=https%3A%2F%2Frigbetellabs.com%2F)</a>
<a href="https://rigbetellabs.com/discord">![Discord Channel](https://img.shields.io/discord/890669104330063903?logo=Discord&style=for-the-badge)</a>
<a href="https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA">![Youtube Subscribers](https://img.shields.io/youtube/channel/subscribers/UCfIX89y8OvDIbEFZAAciHEA?label=YT%20Subscribers&style=for-the-badge)</a>
<a href="https://www.instagram.com/rigbetellabs/">![Instagram](https://img.shields.io/badge/Follow_on-Instagram-pink?style=for-the-badge&logo=appveyor?label=Instagram)</a>
# 1. Installation
## 1.1 Required Dependencies:
```bash
sudo apt install \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy \
  ros-humble-xacro \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-urdf \
  ros-humble-robot-localization \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-interfaces
```

## 1.2 Clone this repo
Make sure you clone the repo in your robot and your remote PC
```bash
mkdir -p ~/tb_ws/src && cd ~/tb_ws/src
git clone -b ros2-humble https://github.com/rigbetellabs/tortoisebot.git
```
```bash
cd ~/tb_ws
colcon build
source install/setup.bash
```
# 2. Setup

- Use `autobringup.launch.py` as the single entry point for all modes (simulation and real robot)
- Set `use_sim_time:=True` for Ignition Gazebo simulation, `use_sim_time:=False` for real robot
- Set `exploration:=True` for SLAM mapping, `exploration:=False` for navigation on a saved map

### 2.1 Launching the robot

**Simulation — SLAM (build a map):**
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```

**Simulation — Navigation on a saved map:**
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=False map_file:=/path/to/your_map.yaml
```

**Real Robot — SLAM (build a map):**
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True
```

**Real Robot — Navigation on a saved map:**
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=False map_file:=/path/to/your_map.yaml
```

**Save a map after SLAM exploration:**
```bash
ros2 launch tortoisebot_navigation save_map.launch.py map_name:=/path/to/your_map
```

### 2.2 Launch files for reference
#### Main Bringup
- `autobringup.launch.py` — all-in-one launch (sim + real, SLAM + nav)
- `bringup.launch.py` — simulation only (no nav stack)
#### SLAM
- `cartographer.launch.py` — Cartographer SLAM node
#### Navigation
- `navigation_slam.launch.py` — Nav2 stack for use during SLAM
- `navigation_mapbased.launch.py` — Nav2 stack with AMCL on a saved map
- `save_map.launch.py` — save the current Cartographer map to disk
#### Visualisation
- `rviz.launch.py`
#### Ignition Sim
- `ignition_sim.launch.py` — Ignition Gazebo simulation
#### Gazebo Classic
- `gazebo.launch.py` — Gazebo Classic simulation

### 2.3 Remote PC

When performing `colcon build` on a remote PC, ignore the hardware-specific packages since the LiDAR and camera will not be connected:

```bash
colcon build --packages-ignore ydlidar_sdk ydlidar_ros2_driver v4l2_camera tortoisebot_firmware tortoisebot_imu
```

# 3. Demos

<!-- Simulation | Visualisation of Sensors (Lidar, Odometery, Camera) 
:-------------------------:|:-------------------------:
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/002.png) |![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/005.png) 

Teleop |  Mapping | Navigation 
:-------------------------:|:-------------------------:|:-------------------------: 
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/007.png) |  ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/006.png) | ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/010.png) -->

# The TortoiseBot 🐢🤖

The ReadMe is divided into several sections as per different topics and is constantly been updated and maintained with new updates by our talented and dedicated 👥 Team at RigBetel Labs LLP. So don't forget to often come here and check on it for the latest and greatest software updates, projects & skills for your TortoiseBot. Also don't forget to 🌟 Star this repository on top-right corner of the screen to show your 💖 Love and Support 🤗 for our Team. 🤩 It will make us happy and encourage us to make and bring more such projects for you. 😍 Click [here](https://github.com/rigbetellabs/tortoisebot/wiki/1.-Getting-Started) to get started.

1. [Getting Started](https://github.com/rigbetellabs/tortoisebot/wiki/1.-Getting-Started)
2. [Hardware Assembly](https://github.com/rigbetellabs/tortoisebot/wiki/2.-Hardware-Assembly)
3. [TortoiseBot Setup](https://github.com/rigbetellabs/tortoisebot/wiki/3.-TortoiseBot-Setup)
4. [Server PC Setup](https://github.com/rigbetellabs/tortoisebot/wiki/4.-Server-PC-Setup)
5. [Running Demos](https://github.com/rigbetellabs/tortoisebot/wiki/5.-Running-Demos)

[Join](https://discord.gg/qDuCSMTjvN) our community for Free. Post your projects or ask questions if you need any help.


## TortosieBot is sourced, assembled, made & maintained by our team 🧑🏻‍🤝‍🧑🏻 at<br>

RigBetel Labs LLP®, Charholi Bk., via. Loheagaon, Pune - 412105, MH, India 🇮🇳<br>
🌐 [RigBetelLabs.com](https://rigbetellabs.com) 📞 [+91-8432152998](https://wa.me/918432152998) 📨 getintouch.rbl@gmail.com , info@rigbetellabs.com <br>
[LinkedIn](http://linkedin.com/company/rigbetellabs/) | [Instagram](http://instagram.com/rigbetellabs/) | [Facebook](http://facebook.com/rigbetellabs) | [Twitter](http://twitter.com/rigbetellabs) | [YouTube](https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA) | [Discord Community](https://discord.gg/qDuCSMTjvN)
