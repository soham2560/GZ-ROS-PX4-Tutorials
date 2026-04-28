# Autonomous Aerial Navigation: RRT* & PX4 Offboard Control

**NPTEL Course:** Principles of Perception and Planning for Aerial Vehicles 

This repository provides a containerized software stack for simulating, planning, and executing quadrotor flight. It attemps to draw a connection between theoretical global planning algorithms with practical flight control architectures using ROS 2, PX4 Autopilot, and Gazebo.

## 📊 System Architecture

The environment relies on a modern, decoupled robotics stack:
* **ROS 2 (Humble):** The primary middleware for high-level path planning and visualization.
* **PX4 Autopilot:** The flight controller executing low-level attitude and rate loops.
* **Gazebo (Harmonic):** The physics engine simulating the quadrotor dynamics and environment.
* **MicroXRCE-DDS:** The bridge translating uORB messages (PX4) to DDS topics (ROS 2).
* **QGroundControl:** The Ground Control Station (GCS) for system-level monitoring.

### Core ROS 2 Packages (`px4_rrt_nav`)
This repository uses a custom ROS 2 package containing two primary nodes:
1. **`rrt_planner`**: Computes a standard collision-free global path using the **RRT*** algorithm. It inflates obstacles, plans in pixel-space, applies Greedy Pruning to remove jagged edges, and continuously monitors the drone's telemetry.
2. **`offboard_control`**: A local path follower to coordinate between PX4 (**NED**: North-East-Down) and ROS 2 (**ENU**: East-North-Up). It utilizes a simple Pure Pursuit lookahead strategy to interpolate the quadrotor along the RRT* path at a controlled velocity and heading.

---

## 🛠️ Environment Setup

This project utilizes **VS Code Dev Containers** to ensure a mostly reproducible, isolated environment across Linux, macOS(untested), and Windows (WSL2), usually eliminating the local dependency conflicts that plague most robotics projects.

### 1. Prerequisites
Ensure you have the following installed on your host machine:
* [Docker](https://docs.docker.com/engine/install/) (Ensure post-installation steps are completed so Docker runs without `sudo`).
<!-- For Windows -->
#### For Windows
- For Windows, download and install Docker Desktop from [here](https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe)
<!-- For Mac -->
#### For Mac
- For Mac, download and install Docker Desktop from [here](https://hub.docker.com/editions/community/docker-ce-desktop-mac)
<!-- For Linux -->
#### For Linux
- For Linux, run the following commands in the terminal
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
#### Test your Docker installation
```bash
docker run hello-world
```
#### Check Docker version
```bash
docker --version
```
#### Try pulling a public image
```bash
docker pull ubuntu:jammy
docker run -it ubuntu:jammy bash
```
* [Visual Studio Code](https://code.visualstudio.com/) with the **Dev Containers** extension installed.

### 2. Host GUI & GPU Configuration
To allow the Docker container to render 3D physics (Gazebo) and visualization tools (RViz2) on your host screen, install the following host dependencies:

#### X11 Forwarding (For Linux Host)
```bash
# Ubuntu/Debian
sudo apt update && sudo apt install x11-xserver-utils

# Arch Linux
sudo pacman -S xorg-xhost
```

---

## 🚀 Building & Launching the Simulator

### 1. Initialize the Dev Container
1. Clone this repository and open the directory in VS Code.
2. Press `Ctrl + Shift + P` to open the Command Palette.
3. Search for **"Dev Containers: Reopen in Container"** and select it.
4. Select your host platform (Linux, WSL, or macOS) when prompted.

> **Note:** The initial build will take several minutes as it pulls the base images (`ghcr.io/srindot/rosgzpx4`) and configures the ROS 2 workspaces.

### 2. Launch the Stack
We have provided automated VS Code tasks to orchestrate the complex startup sequence of the PX4-ROS 2 bridge. 

1. Once inside the container, press `Ctrl + Shift + P`.
2. Search for **"Tasks: Run Build Task"** (or use the shortcut `Ctrl + Shift + B`).
3. Select **`Start All: PX4 + RRT Stack`**.

This compound task will systematically open five integrated terminals, launching:
* `Session 1:` The MicroXRCE-DDS Agent.
* `Session 2:` The PX4 SITL and Gazebo Harmonic simulation.
* `Session 3:` QGroundControl.
* `Session 4:` The PX4-ROS Sensor Combined Listener.
* `Session 5:` The custom `px4_rrt_nav` launch file (initiating the Map Server, RRT Planner, Offboard Controller, and RViz2).

---

## 🎯 Executing Autonomous Navigation

Once the `Start All` sequence completes, **RViz2** and **Gazebo** will be open on your screen. The quadrotor will automatically arm, transition into Offboard mode, and hover at an altitude of 5 meters.

### Using the RRT* Planner
1. In the top toolbar of the RViz2 window, click the **2D Goal Pose** tool (green arrow).
2. Click anywhere in the free space of the 2D map.
3. **Observation:**
   * The RRT* search tree will instantly populate in faint green, demonstrating the algorithm's exploration nodes.
   * The optimal, pruned path will overlay in bright green.
   * The quadrotor (`base_link` TF axes) will smoothly navigate the path, adjusting its heading (yaw) to face the direction of flight.

### Debugging the Algorithm
If you select a goal that is mathematically unreachable (e.g., inside an inflated obstacle), the planner will safely reject the input. Check the `Session 5` terminal output for explicit error messages regarding coordinate boundaries or obstacle collisions.

---

## 📘 Code Structure & Academic Notes

For students reviewing the codebase, pay special attention to the mathematical handling of coordinate frames. 

* **`rrt_planner.py`:** Operates entirely in **ROS 2 ENU** (East-North-Up) coordinates and grid-pixel space. Obstacle inflation utilizes a Minkowski sum approach via OpenCV morphological dilations.
* **`offboard_control.py`:** Serves as the mathematical bridge. It converts ENU path targets into **PX4 NED** (North-East-Down) setpoints, ensuring the Z-axis (altitude) and quaternion orientations are inverted correctly before transmission to the flight controller.

## 🤝 Support & Contributions
If you encounter permission issues rendering Gazebo or RViz, ensure `xhost +local:docker` has been executed on your host machine. For persistent issues, please open an Issue in this repository.

*This repository builds upon architectures provided by [Waypoint_Navigation_in_ROS-PX4](https://github.com/Srindot/Waypoint_Navigation_in_ROS-PX4).*