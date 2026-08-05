# TortoiseBot — ROS 2 Humble

<p align="center">
  <img src="https://github.com/rigbetellabs/tortoisebot_docs/raw/master/imgs/packaging/pack_front.png" alt="TortoiseBot Banner" width="800"/>
</p>

<p align="center">
  <img src="https://img.shields.io/github/stars/rigbetellabs/tortoisebot?style=for-the-badge"/>
  <img src="https://img.shields.io/github/forks/rigbetellabs/tortoisebot?style=for-the-badge"/>
  <img src="https://img.shields.io/github/watchers/rigbetellabs/tortoisebot?style=for-the-badge"/>
  <img src="https://img.shields.io/github/repo-size/rigbetellabs/tortoisebot?style=for-the-badge"/>
  <img src="https://img.shields.io/github/contributors/rigbetellabs/tortoisebot?style=for-the-badge"/>
</p>

<p align="center">
  <a href="https://rigbetellabs.com/"><img src="https://img.shields.io/website?down_color=lightgrey&down_message=offline&label=Rigbetellabs%20Website&style=for-the-badge&up_color=green&up_message=online&url=https%3A%2F%2Frigbetellabs.com%2F"/></a>
  <a href="https://rigbetellabs.com/discord"><img src="https://img.shields.io/discord/890669104330063903?logo=Discord&style=for-the-badge"/></a>
  <a href="https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA"><img src="https://img.shields.io/youtube/channel/subscribers/UCfIX89y8OvDIbEFZAAciHEA?label=YT%20Subscribers&style=for-the-badge"/></a>
  <a href="https://www.instagram.com/rigbetellabs/"><img src="https://img.shields.io/badge/Follow_on-Instagram-pink?style=for-the-badge&logo=instagram"/></a>
</p>

---

<p align="center">
  <a href="#1-installation">Installation</a> •
  <a href="#2-workspace-setup">Workspace Setup</a> •
  <a href="#3-simulation">Simulation</a> •
  <a href="#4-real-robot">Real Robot</a> •
  <a href="#5-more-wiki-resources">More Wiki Resources</a>
</p>

---

## 1. Installation

### 1.1 Install Gazebo Ignition Fortress

TortoiseBot uses **Ignition Fortress** as its simulation backend. Install it before any ROS packages:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install ignition-fortress
```

### 1.2 Required ROS 2 Dependencies

Install all required ROS 2 Humble packages:

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

---

## 2. Workspace Setup

### 2.1 Clone the Repository

Clone the repository on both your **robot** and your **remote PC**:

```bash
mkdir -p ~/tb_ws/src && cd ~/tb_ws/src
git clone -b ros2-humble https://github.com/rigbetellabs/tortoisebot.git
```

### 2.2 Build the Workspace

> [!IMPORTANT]
> Always run `rosdep` first to resolve any remaining system dependencies automatically.

```bash
cd ~/tb_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Simulation / Remote PC build** (excludes hardware-only packages):

```bash
colcon build --packages-ignore ydlidar_sdk ydlidar_ros2_driver v4l2_camera tortoisebot_firmware tortoisebot_imu
source install/setup.bash
```

**Real Robot build** (run on the robot after the above, then source again):

```bash
colcon build --packages-select ydlidar_sdk ydlidar_ros2_driver v4l2_camera tortoisebot_firmware tortoisebot_imu
source install/setup.bash
```

### 2.3 Key Launch Arguments

`autobringup.launch.py` is the single entry point for all operating modes:

| Argument | Default | `True` | `False` |
|---|---|---|---|
| `use_sim_time` | `True` | Ignition Gazebo simulation | Real robot hardware |
| `exploration` | `True` | SLAM — build a new map | Navigation — use a saved map |
| `slam_only` | `False` | SLAM-only — Cartographer, **no Nav2** (teleop + map saving) | Standard SLAM + Nav2 stack |

> [!NOTE]
> `slam_only:=True` only takes effect when `exploration:=True`. When `exploration:=False` (map-based nav), the `slam_only` argument is ignored.

### 2.4 Available Launch Files

| Category | Launch File | Purpose |
|---|---|---|
| **Main** | `autobringup.launch.py` | All-in-one bringup (sim + real, SLAM + nav) |
| **Main** | `bringup.launch.py` | Simulation only (no nav stack) |
| **SLAM** | `cartographer.launch.py` | Cartographer SLAM node |
| **Navigation** | `navigation_slam.launch.py` | Nav2 stack during SLAM |
| **Navigation** | `navigation_mapbased.launch.py` | Nav2 with AMCL on a saved map |
| **Navigation** | `save_map.launch.py` | Save current Cartographer map to disk |
| **Visualization** | `rviz.launch.py` | RViz2 sensor & map visualization |
| **Sim** | `ignition_sim.launch.py` | Ignition Gazebo simulation |
| **Sim** | `gazebo.launch.py` | Gazebo Classic simulation |

---

## 3. Simulation

The TortoiseBot simulation runs inside **Ignition Gazebo** with full ROS 2 Humble integration. RViz2 launches automatically alongside Gazebo, providing a live side-by-side view of the robot's sensor data, SLAM map, and navigation stack.

### 3.1 Teleoperation in Simulation

Drive the robot manually using the keyboard teleop:

```bash
# Terminal 1 — Launch Gazebo + RViz + SLAM (no Nav2, teleop-only mode)
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True slam_only:=True

# Terminal 2 — Keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3.2 Autonomous Exploration & SLAM Mapping

The robot maps the environment autonomously using **Cartographer SLAM**. Use the `2D Nav Goal` button in RViz2 to send exploration goals:

```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```

<p align="center">
  <img src="media/navigation.gif" alt="SLAM Mapping Demo" width="800"/>
</p>

<p align="center">
  <img src="media/goaltravel.gif" alt="Goal Travel during SLAM" width="800"/>
</p>

### 3.3 Autonomous Navigation on a Saved Map

Once a map has been built and saved, the robot localizes itself using **AMCL** (Adaptive Monte Carlo Localization) and navigates autonomously to any goal point:

```bash
# Step 1 — Save the map after exploration
ros2 launch tortoisebot_navigation save_map.launch.py map_name:=/home/vn/tb_ws/maps/my_map

# Step 2 — Navigate using the saved map
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=False map_file:=/home/vn/tb_ws/maps/my_map.yaml
```

<p align="center">
  <img src="media/amclnav.gif" alt="AMCL Navigation on Saved Map" width="800"/>
</p>

---

## 4. Real Robot

### 4.1 Network Setup — Connecting the Robot to Your Wi-Fi

Before powering up the robot for the first time, configure Wi-Fi credentials directly on the SD card.

> **Recommended:** Perform these steps on a Linux or macOS machine.

1. Insert the SD card into your computer using a card reader.
2. Navigate to the `writable` partition, then to `/etc/netplan/`.
3. Open `01-network-manager-all.yaml` in a text editor.
4. Locate the `wifis` section and add your credentials:

```yaml
wlan0:
  optional: true
  access-points:
    "your_wifi_ssid":
      password: "your_wifi_password"
  dhcp4: true
```
<p align="center">
  <img src="media/netplancommand.png" alt="Netplan command" width="600"/>
</p>

<p align="center">
  <img src="media/wifipassentry.png" alt="Wi-Fi password entry" width="600"/>
</p>

**Example configuration:**

<p align="center">
  <img src="media/wifipassexample.png" alt="Wi-Fi password example" width="600"/>
</p>

5. Save the file and exit the text editor.
6. Safely eject the SD card and insert it into the robot.

The robot will connect to your Wi-Fi network automatically on the next boot.

---

### 4.2 SSH into the Robot

After powering on the robot, obtain its IP address (displayed on its OLED screen or found via your router's device list):

<p align="center">
  <img src="https://github.com/rigbetellabs/tortoisebot_docs/blob/master/imgs/tortoiseBot_setup/001.jpeg?raw=true" alt="Finding robot IP" width="600"/>
</p>

Connect to the robot from your PC terminal:

```bash
ssh tortoisebot@<ROBOT_IP_ADDRESS>
# Example: ssh tortoisebot@192.168.0.120
# Password: raspberry
```

---

### 4.3 Sensor Data Visualization

**Step 1 — On the Robot (SSH Terminal 1):** Source ROS 2 Humble and launch all sensors and actuators:

```bash
source /opt/ros/humble/setup.bash
# slam_only:=True starts SLAM-only (no Nav2) — lightweight mode for verifying sensors
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True slam_only:=True
```

<p align="center">
  <img src="https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/irl_robot_00.jpeg?raw=true" alt="Robot hardware bringup" width="600"/>
</p>

**Step 2 — On your PC (Terminal 1):** Source your workspace and start keyboard teleoperation:

```bash
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Step 3 — On your PC (Terminal 2):** Launch RViz2 to visualize all sensor streams:

```bash
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/setup.bash
ros2 launch tortoisebot_description rviz.launch.py
```

<p align="center">
  <img src="https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/irl_robot_viz.jpeg?raw=true" alt="Robot IRL RViz visualization" width="700"/>
</p>

---

### 4.4 Exploration Mode — SLAM Mapping on Real Robot

**Step 1 — On the Robot (SSH Terminal 1):** Launch the robot in exploration mode:

```bash
source /opt/ros/humble/setup.bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True
```

**Step 2 — On your PC:** Source and visualize the live map:

```bash
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/setup.bash
ros2 launch tortoisebot_description rviz.launch.py
```

<p align="center">
  <img src="https://github.com/rigbetellabs/tortoisebot_docs/blob/ros2/imgs/tortoiseBot_demo/irl_viz_02.jpeg?raw=true" alt="SLAM mapping visualization IRL" width="700"/>
</p>

Teleoperate the robot to map the environment, or use the **Nav2 Goal** button in RViz2 to send autonomous exploration goals.

**Step 3 — Save the map (SSH Terminal 2 on Robot):**

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room_map
```

---

### 4.5 Navigation on a Saved Map — Real Robot

**Step 1 — Place the robot** at approximately the same starting position used during mapping.

**Step 2 — On the Robot (SSH Terminal 1):** Launch navigation with the saved map:

```bash
source /opt/ros/humble/setup.bash
ros2 launch tortoisebot_bringup autobringup.launch.py \
  use_sim_time:=False \
  exploration:=False \
  map_file:=~/maps/my_room_map.yaml
```

**Step 3 — On your PC:** Visualize and send navigation goals:

```bash
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/setup.bash
ros2 launch tortoisebot_description rviz.launch.py
```

Use the **Nav2 Goal** button in RViz2 to set a target pose and watch the robot navigate autonomously!

> **Multiple Robot Setup:** If operating more than one TortoiseBot on the same network, assign a unique domain ID to each robot to prevent cross-talk:
> ```bash
> export ROS_DOMAIN_ID=0   # Set a unique integer per robot (0–101)
> ```

---

## 5. More Wiki Resources

The TortoiseBot documentation is continuously maintained and updated by the team at **RigBetel Labs**. The full wiki covers hardware assembly, OS flashing, advanced configuration, and project showcases.

| Resource | Link |
|---|---|
| 🚀 Getting Started | [Wiki — Getting Started](https://github.com/rigbetellabs/tortoisebot/wiki/1.-Getting-Started) |
| 🔧 Hardware Assembly | [Wiki — Hardware Assembly](https://github.com/rigbetellabs/tortoisebot/wiki/2.-Hardware-Assembly) |
| ⚙️ TortoiseBot Setup | [Wiki — TortoiseBot Setup](https://github.com/rigbetellabs/tortoisebot/wiki/3.-TortoiseBot-Setup) |
| 💻 Server PC Setup | [Wiki — Server PC Setup](https://github.com/rigbetellabs/tortoisebot/wiki/4.-Server-PC-Setup) |
| 🎮 Running Demos | [Wiki — Running Demos](https://github.com/rigbetellabs/tortoisebot/wiki/5.-Running-Demos) |
| 💬 Community Discord | [Join the Community](https://discord.gg/qDuCSMTjvN) |

Don't forget to ⭐ **Star this repository** to stay updated with the latest releases and show your support for the team!

---

<p align="center">
  TortoiseBot is designed, assembled, and maintained by the team at<br/><br/>
  <strong>RigBetel Labs LLP®</strong><br/>
  Charholi Bk., via. Loheagaon, Pune – 412105, MH, India 🇮🇳<br/><br/>
  🌐 <a href="https://rigbetellabs.com">RigBetelLabs.com</a> &nbsp;|&nbsp;
  📞 <a href="https://wa.me/918432152998">+91-8432152998</a> &nbsp;|&nbsp;
  📨 <a href="mailto:info@rigbetellabs.com">info@rigbetellabs.com</a><br/><br/>
  <a href="http://linkedin.com/company/rigbetellabs/">LinkedIn</a> &nbsp;|&nbsp;
  <a href="http://instagram.com/rigbetellabs/">Instagram</a> &nbsp;|&nbsp;
  <a href="http://facebook.com/rigbetellabs">Facebook</a> &nbsp;|&nbsp;
  <a href="http://twitter.com/rigbetellabs">Twitter</a> &nbsp;|&nbsp;
  <a href="https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA">YouTube</a> &nbsp;|&nbsp;
  <a href="https://discord.gg/qDuCSMTjvN">Discord</a>
</p>
