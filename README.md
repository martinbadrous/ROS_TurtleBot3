# 🐢 ROS TurtleBot3 — Workspace & Demos

A clean, practical ROS workspace for **TurtleBot3** basics: bringup, teleop, Gazebo simulation, **SLAM**, and **Navigation**.  
Tailored for **ROS Melodic (Ubuntu 18.04)** with notes for Noetic / ROS 2.

Created & maintained by **[Martin Badrous](https://github.com/martinbadrous)**.

---

## 📌 What’s inside

- ✅ Ready-to-use **catkin** workspace layout (`catkin_ws/src`)  
- ✅ Gazebo simulation with TurtleBot3 models (Burger/Waffle)  
- ✅ Teleop (keyboard) & RViz visualization  
- ✅ **SLAM** (gmapping / Cartographer) to build and save maps  
- ✅ **Navigation** (AMCL + move_base) using saved maps  
- ✅ Real-robot bringup notes (networking, environment variables)

> Uses official TurtleBot3 packages and workflows. See the e‑Manual for details. [Docs] [1], [ROS Wiki] [2].

---

## 🧱 Repository Structure (recommended)

```
ROS_TurtleBot3/
├── README.md
├── maps/                          # saved maps (PGM/PNG + YAML)
├── launch/                        # convenience launch files (optional)
│   ├── sim_world.launch           # spawn TurtleBot3 in Gazebo + RViz
│   ├── slam_gmapping.launch       # SLAM (gmapping) in sim
│   ├── nav_amcl.launch            # Navigation with AMCL using a saved map
│   └── teleop.launch              # keyboard teleop
└── scripts/                       
    └── map_save.sh                # helper to save map to ./maps
```

> If your repo already has a different structure, keep it — these are **optional** quality‑of‑life files you can add.

---

## 🛠️ Installation

### Option A — ROS **Melodic** (recommended for this repo)

```bash
# 0) System (Ubuntu 18.04)
sudo apt update

# 1) ROS Melodic (Desktop-Full + Gazebo + TB3 sims)
sudo apt install ros-melodic-desktop-full
sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
sudo apt install ros-melodic-turtlebot3 ros-melodic-turtlebot3-msgs

# 2) Workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
# If you're cloning this repo:
git clone https://github.com/martinbadrous/ROS_TurtleBot3.git
cd .. && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 3) Set model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
export TURTLEBOT3_MODEL=burger
```

### Option B — ROS **Noetic** (Ubuntu 20.04)

```bash
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-turtlebot3 ros-noetic-gazebo-ros-pkgs
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

### Option C — **ROS 2** (Humble/Jazzy) — reference only
Follow the official e‑Manual quick‑start to install TurtleBot3 on ROS 2 and build with **colcon**. [Quick Start] [3].

---

## 🧪 Run: Simulation (Gazebo)

**Terminal 1 — Gazebo world**
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
# or: turtlebot3_house.launch / turtlebot3_empty_world.launch
```

**Terminal 2 — RViz**
```bash
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
# (or use: roslaunch turtlebot3_bringup turtlebot3_model.launch)
```

**Optional — Keyboard Teleop**
```bash
rosrun turtlebot3_teleop turtlebot3_teleop_key
# Move with WASD; Ctrl+C to exit
```

> Official worlds and teleop are provided by the TurtleBot3 packages. [Docs] [1].

---

## 🗺️ Build a Map (SLAM)

You can use **gmapping** (ROS 1) or **Cartographer**.

**Gmapping (simple & light):**
```bash
# With Gazebo running
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
# Drive the robot around (teleop) until the map is complete

# Save the map
rosrun map_server map_saver -f ~/catkin_ws/src/ROS_TurtleBot3/maps/my_lab
# Produces my_lab.yaml + my_lab.pgm (or .png)
```

**Cartographer (better loop‑closure):**
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer
# Save the map as above
```

References: [Docs] [1], [ROS Wiki] [2].

---

## 🧭 Navigation (AMCL)

Use a saved map to localize and plan paths.

```bash
# 1) Launch Gazebo world (Terminal 1)
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 2) Launch Navigation with your saved map (Terminal 2)
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/ROS_TurtleBot3/maps/my_lab.yaml

# 3) RViz: set 2D Pose Estimate, then 2D Nav Goal
```

> The standard stack uses **AMCL** for localization and **move_base** for path planning. [Docs] [1].

---

## 🤖 Real Robot Bringup (notes)

On the robot (Raspberry Pi):  
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

On your PC (replace IPs accordingly):  
```bash
# Networking
export ROS_MASTER_URI=http://<robot_ip>:11311
export ROS_HOSTNAME=<your_pc_ip>
export TURTLEBOT3_MODEL=burger

# Teleop or RViz
rosrun turtlebot3_teleop turtlebot3_teleop_key
rviz
```

> See the e‑Manual for full networking setup and bringup instructions. [Docs] [1].

---

## 🧯 Troubleshooting

- **No model in Gazebo / RViz** → ensure `export TURTLEBOT3_MODEL=burger` is sourced in your shell.  
- **Cannot connect to robot** → check `ROS_MASTER_URI` and `ROS_HOSTNAME` on both sides (ping both ways).  
- **Laser or camera topics missing** → check `turtlebot3_robot.launch` output and `rostopic list`.  
- **Map looks torn / distorted** → slow down the robot while mapping; ensure stable FPS in simulation.  
- **Navigation stuck** → increase inflation radius or reduce acceleration limits in `move_base` params.

---

## 🙌 Credits

- **TurtleBot3 e‑Manual** — official documentation, worlds, SLAM, nav, teleop. [Docs] [1], [Quick Start] [3]  
- **TurtleBot3 GitHub** — open‑source packages and examples. [GitHub] [4]

---

## Author

**Martin Badrous** — Computer Vision & Robotics Engineer  
📧 martin.badrous@gmail.com • 🔗 https://github.com/martinbadrous

---

[1]: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/  
[2]: https://wiki.ros.org/turtlebot3  
[3]: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/  
[4]: https://github.com/ROBOTIS-GIT/turtlebot3
