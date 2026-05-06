# Real World Setup & Troubleshooting Guide

This document contains deployment and troubleshooting fixes discovered during real-world setup of TortoiseBot on:

- Raspberry Pi 4 (Ubuntu 20.04 + ROS 1 Noetic)
- Ubuntu 24.04 Desktop (via Distrobox)
- Apple Silicon Mac (M-series)

These issues are not fully covered in the current wiki/setup flow but are commonly encountered during practical deployment.

---

## 1. WiFi Issues on Raspberry Pi

> **Note:** Ubuntu 20.04 on Raspberry Pi uses `systemd-networkd` + Netplan as the network backend — **not** NetworkManager. `nmcli wifi` commands appear to work but have no effect.

### 1.1 `wlan0` shows `NO-CARRIER` / State DOWN

**Cause:** Missing or corrupted Broadcom BCM43455 WiFi chip firmware.

**Fix:**

```bash
sudo systemctl stop unattended-upgrades  # Must run first — it holds the apt lock
sudo apt update
sudo apt install --reinstall linux-firmware -y

sudo modprobe -r brcmfmac
sleep 2
sudo modprobe brcmfmac

dmesg | grep -i brcm  # Verify firmware loaded
```

---

### 1.2 WiFi Scan Returns Empty — `Failed to initiate sched scan`

**Cause:** Kernel defaults to regulatory domain `country 00` (world domain), which marks most channels as passive-scan only. This silently prevents active network scanning.

**Fix:**

```bash
sudo iw reg set IN        # Replace IN with your country code (US, GB, DE, etc.)
sudo iw reg get           # Verify — should NOT show 'country 00'

# Make permanent across reboots:
echo 'REGDOMAIN=IN' | sudo tee /etc/default/crda
```

---

### 1.3 `apt`/`dpkg` Lock Error

**Symptoms:**

```
dpkg: error: dpkg frontend lock is held by another process
```

**Cause:** `unattended-upgrades` background service holds the apt/dpkg lock.

**Fix:**

```bash
sudo systemctl stop unattended-upgrades
sudo killall unattended-upgrade 2>/dev/null
sudo dpkg --configure -a

# Verify lock is clear:
sudo lsof /var/lib/dpkg/lock-frontend
```

---

### 1.4 Netplan YAML Configuration Errors

**Cause:** YAML does not allow tab characters for indentation. Editors inserting tabs cause `netplan apply` to fail silently or with cryptic errors.

**Fix:** Disable cloud-init from overwriting your config on reboot, then write config using Python to avoid tab issues:

```bash
sudo bash -c 'echo "network: {config: disabled}" > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg'

sudo python3 -c "
content = '''network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      access-points:
        \"YourSSID\":
          password: \"YourPassword\"
'''
open('/etc/netplan/50-cloud-init.yaml', 'w').write(content)
"

sudo netplan apply
```

---

### 1.5 IP Address Changes After Each Reboot (SSH Breaks)

**Cause:** DHCP assigns a new IP on each boot.

**Fix:**

```bash
# Always check current IP after rebooting:
ip a show wlan0

# Or force a new DHCP request:
sudo dhclient wlan0
ip a show wlan0
```

---

## 2. ROS Networking Issues

### 2.1 Unable to Contact ROS Master

**Symptoms:**

```
RLException: Unable to contact my own server at [http://10.42.0.250:xxxxx/]
This usually means that the network is not configured properly.
```

**Cause:** ROS resolves the wrong network interface IP (e.g., eth0 instead of wlan0).

**Fix:** Export these variables before every `roslaunch`:

```bash
export ROS_MASTER_URI=http://<PI_WIFI_IP>:11311
export ROS_IP=<PI_WIFI_IP>
export ROS_HOSTNAME=<PI_WIFI_IP>
source ~/ros1_ws/devel/setup.bash
```

To make permanent, add to `~/.bashrc`:

```bash
echo 'export ROS_MASTER_URI=http://10.247.26.31:11311' >> ~/.bashrc
echo 'export ROS_IP=10.247.26.31' >> ~/.bashrc
echo 'export ROS_HOSTNAME=10.247.26.31' >> ~/.bashrc
source ~/.bashrc
```

---

### 2.2 Hostname Resolves to Wrong Interface

**Cause:** The Pi hostname (e.g., `ubuntu`) maps to the Ethernet IP in `/etc/hosts`, not the WiFi IP. ROS resolves its own hostname and gets the wrong address.

**Fix:**

```bash
echo '10.247.26.31 ubuntu' | sudo tee -a /etc/hosts

# Verify — should show only wlan0 IP:
hostname -I
```

---

## 3. Ubuntu 24.04 + Distrobox Issues

### 3.1 Docker Socket Permission Denied

**Symptoms:**

```
permission denied while trying to connect to the Docker daemon socket
```

**Cause:** Docker socket is owned by root; non-root users need the `docker` group.

**Fix:**

```bash
sudo chmod 666 /var/run/docker.sock   # Resets on reboot

# Permanent fix:
sudo usermod -aG docker $USER
newgrp docker
```

---

### 3.2 Distrobox Container Corrupted

**Symptoms:**

```
cannot open .noetic_env.fifo
```

**Cause:** Container initialization was interrupted with `Ctrl+C` before first-boot mapping completed.

**Fix:**

```bash
distrobox rm noetic_env
distrobox create --name noetic_env --image ubuntu:20.04
distrobox enter noetic_env  # Allow full initialization — do NOT interrupt
```

---

### 3.3 `ros-noetic-desktop-full` Not Found Inside Distrobox

**Symptoms:**

```
E: Unable to locate package ros-noetic-desktop-full
```

**Cause:** Minimal Ubuntu 20.04 container is missing: the `universe` repository, `lsb-release` package, and/or the ROS GPG key.

**Fix (run this entire block inside the Distrobox container):**

```bash
sudo apt update && sudo apt install -y software-properties-common curl lsb-release && \
sudo add-apt-repository universe -y && \
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt update && \
sudo apt install -y ros-noetic-desktop-full
```

---

### 3.4 ROS GPG Key Error — `EXPKEYSIG F42ED6FBAB17C654`

**Fix:**

```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

---

### 3.5 `.bashrc` Errors When Entering Distrobox

**Symptoms:**

```
bash: python3: command not found
bash: /usr/share/colcon_cd/function/colcon_cd.sh: No such file or directory
bash: alias: ros2: not found (spaces around '=' are not allowed)
```

**Cause:** Distrobox shares the host `~/.bashrc`. The host `.bashrc` tries to source ROS 2 tools and colcon scripts that don't exist inside the ROS 1 container.

**Fix — correct `.bashrc` layout:**

```bash
# No spaces around = in alias
alias ros2='source ~/ros2_kilted/install/setup.bash'

# Source colcon_cd only if it exists
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
  source /usr/share/colcon_cd/function/colcon_cd.sh
fi

# Source ROS 1 only inside Distrobox container
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

# Source ROS 2 only on host (not inside container)
if [ ! -f /opt/ros/noetic/setup.bash ] && [ -f ~/ros2_kilted/install/setup.bash ]; then
  source ~/ros2_kilted/install/setup.bash
fi
```

---

## 4. Apple Silicon Mac (M-Series) Issues

### 4.1 `--network=host` Does Not Work on Docker Desktop for Mac

**Cause:** `--network=host` is Linux-only. Docker Desktop on macOS uses a VM, so host networking is not available.

**Recommendation:** Use the Raspberry Pi as the ROS master. Do not attempt to run `roscore` inside Docker on Mac with host networking.

---

### 4.2 RViz Crashes on Apple Silicon

**Symptoms:**

```
libGL error: No matching fbConfigs or visuals found
This application failed to start because no Qt platform plugin could be initialized
Aborted (core dumped)
```

**Cause:** Docker on Apple Silicon (ARM) emulates AMD64, and macOS does not expose OpenGL via XQuartz sufficiently for RViz/OGRE. This is a platform limitation.

**Recommendation:** Use [Foxglove Studio](https://foxglove.dev/) instead of RViz. Foxglove connects via `rosbridge_server` (WebSocket) and supports LaserScan, PointCloud2, TF, Camera, and other standard ROS visualization types without requiring OpenGL.

---

## 5. rosbridge Issues

### 5.1 `catkin_make` Fails — Non-Homogeneous Workspace

**Symptoms:**

```
CMake Error: This workspace contains non-catkin packages in it
and catkin cannot build a non-homogeneous workspace without isolation.
```

**Cause:** The `main` branch of `rosbridge_suite` is ROS 2 (uses `ament`). Mixing `ament` packages with `catkin` packages in the same workspace breaks `catkin_make`.

**Fix:**

```bash
cd ~/rosbridge_ws/src
rm -rf rosbridge_suite

git clone -b ros1 https://github.com/RobotWebTools/rosbridge_suite.git  # -b ros1 is mandatory

cd ~/rosbridge_ws
catkin_make
```

---

## 6. RViz over VNC Issues

### 6.1 RViz OpenGL / OGRE Crash over VNC

**Symptoms:**

```
OGRE EXCEPTION: GL_INVALID_OPERATION
Aborted (core dumped)
```

**Cause:** VNC's virtual framebuffer does not support hardware OpenGL. OGRE (RViz's 3D engine) requires it.

**Fix — force software rendering:**

```bash
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export OGRE_RTT_MODE=Copy

export ROS_MASTER_URI=http://<PI_IP>:11311
export ROS_IP=<PI_IP>
source ~/ros1_ws/devel/setup.bash

rosrun rviz rviz
```

> **Note:** GNOME does not work well with TightVNC. If you see a blank grey screen, install XFCE:
> ```bash
> sudo apt install -y xfce4 xfce4-goodies
> ```
> Then update `~/.vnc/xstartup` to run `startxfce4 &`.

---

## 7. General Recommendations

- Always check the current WiFi IP after every reboot — DHCP assigns a new address:
  ```bash
  ip a show wlan0
  ```
- Always export `ROS_MASTER_URI`, `ROS_IP`, and `ROS_HOSTNAME` before launching nodes, even if they are in `.bashrc`. Verify with `echo $ROS_MASTER_URI` in each new terminal.
- Clone `rosbridge_suite` with `-b ros1` explicitly. The default branch is ROS 2 only.
- Never interrupt first-time Distrobox initialization with `Ctrl+C`.
- Use Foxglove Studio on macOS — it is more reliable than RViz for remote visualization and has no GLX dependency.
- Be cautious with `apt upgrade` on Ubuntu 20.04 with ROS Noetic — dependency pins can be broken by system updates.
- When in doubt, restart cleanly:
  ```bash
  killall -9 rosmaster roscore
  sleep 2
  # Re-export ROS variables, then relaunch
  ```