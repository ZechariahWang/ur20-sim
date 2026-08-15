# robim_ur20_sim

Simulated UR20 arm you can move between named waypoints, built on the official
Universal Robots ROS 2 driver (mock hardware mode) and MoveIt 2.

## Prerequisites

Ubuntu 22.04 + ROS 2 Humble, plus:

```bash
sudo apt install ros-humble-ur ros-humble-moveit
```

## Build

```bash
cd ~/robim_ur20_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1 — bring up the simulated robot (driver with fake hardware, MoveIt, RViz):

```bash
ros2 launch robim_ur20_sim sim.launch.py
```

Terminal 2 — move through the waypoints defined in `config/waypoints.yaml`:

```bash
ros2 launch robim_ur20_sim move_waypoints.launch.py
```

You can also drag the interactive marker in RViz and use MoveIt's
"Plan & Execute" directly.

## Editing waypoints

`src/robim_ur20_sim/config/waypoints.yaml` defines named joint-space waypoints
(radians, in the order shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2,
wrist_3) and the visit order in `waypoint_names`. Set `cycle: true` to loop
forever. With `--symlink-install`, YAML edits take effect on next run without
rebuilding.

## Moving to URSim or a real UR20

The stack is identical — only the hardware endpoint changes:

```bash
ros2 launch robim_ur20_sim sim.launch.py use_fake_hardware:=false robot_ip:=<ip>
```

For URSim, run UR's controller simulator in Docker and load an External
Control program in Polyscope first:

```bash
docker run --rm -it -p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 \
  --name ursim universalrobots/ursim_e-series
```
# ur20-sim
