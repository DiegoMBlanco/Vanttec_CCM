# Vanttec_CCM
---

## About this repository
Hello! This is the GitHub repository where everything related to the MPC is stored and documented. As of now, the code living in this repository only works with DC motors and the simulated ROS2 turtlebot, which you can download following this [tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). **Please note that the packages shown here were made using ROS2 Humble in Ubuntu 22.04 LTS**, so if you work with another version of ROS2 it might be convenient to use a docker container. We might release a dedicated version for Jazzy but it is still unsure.


We sincerely hope that the packages of this repository will be useful to you, your projects and your team.
---
## How to setup our repository

**To use our repository:**
1. Open a terminal and navigate through your file system to your `src` directory:

```bash
cd ~/your_ros2_ws/src/
```

2. Clone this repository:

```bash
git clone https://github.com/DiegoMBlanco/Vanttec_CCM.git
```

3. Navigate back to the root of your workspace and build the package (using the turtlebot package as an example):

```bash
cd ~/your_ros2_ws
colcon build --packages-select mpc_turtlebot
```

4. Source the setup file:

```bash
source install/setup.bash
```

**Now you should be all set**
---
## How to use our repository (Demo)

1. Open one terminal and type the following commands to open the gazebo simulation:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

2. In a new terminal, launch the cartographer node:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

3. In a third terminal, launch our MPC and path drawer nodes:
```bash
ros2 launch mpc_turtlebot launcher.launch.py
```

4. Go back to Rviz and publish the reference points for the trajectory. When the path is ready, call the start service with the command:
```bash
ros2 service call /start_execution std_srvs/srv/Empty {}
```
4. Finally, to erase the path and start drawing again, run the next command:
```bash
ros2 service call /clear_drawn_plan std_srvs/srv/Empty {}
```

### Mini troubleshooting guide:

- Remember to always execute the `colcon build` and `source install/setup.bash` commands before launching the MPC node.

- Remember to execute the command: `source /opt/ros/humble/setup.bash` if it isn't in your .bashrc file.

- Use Rviz with the **odom** frame!

- If the published points are not drawing any path, please click on the "Add" button in the left panel and in the section "By topic" search drawn_path and select path. Do this after you launched the MPC launch file.

---
## Current status and examples:

Currently, this repository contains two main components:

* **`mpc_turtlebot` (Simulation):** A fully functional ROS 2 package that uses the Turtlebot3 as an example to demonstrate the MPC algorithm tracking a reference in a simulated environment. We recommend using this package to test the algorithm safely.
* **`mpc_dc_motor` (Hardware - WIP):** This package contains the implementation for a physical DC motor communicating via microROS. *Note: We are currently migrating to a new motor model, so this package is under active testing and might be temporarily incomplete.*
---

## Feedback
If you have any suggestions, find a bug, or just want to leave a comment, please feel free to [open an issue](https://github.com/DiegoMBlanco/Vanttec_CCM/issues).

