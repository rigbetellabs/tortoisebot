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
## 1.1 Required Dependences: 
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-joint-state-publisher ros-melodic-robot-state-publisher ros-melodic-slam-gmapping ros-melodic-dwa-local-planner ros-melodic-joint-state-publisher-gui ros-melodic-cartographer-ros ros-melodic-cartographer-rviz
```
```
cd ~/your workscpace
catkin_make
```
## 1.2 Clone this repo 
Make sure you clone the repo in your robot and your remote PC 
```
git clone --recursive https://github.com/rigbetellabs/tortoisebot.git
```
```
cd ~/your workscpace
catkin_make
```
# 2. Setup
### 2.1 Running on Simulation
- Run bringup.launch
- Run server_bringup.launch
### SLAM
- slam.launch
### Navigation
- navigation.launch
### 2.2 Running on Real Robot
### Running on ROBOT
- Run bringup.launch
### Running on REMOTE PC
- Run server_bringup.launch
for SLAM
- slam.launch
for Nav
- navigation.launch

# 3. Demos

Simulation | Vizualisation of Sensors (Lidar,Odometery,Camera) 
:-------------------------:|:-------------------------:
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/002.png) |![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/005.png) 

Teleop |  Mapping | Navigation 
:-------------------------:|:-------------------------:|:-------------------------: 
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/007.png) |  ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/006.png) | ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/010.png)

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
