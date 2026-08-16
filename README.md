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

Terminal 1

```bash
ros2 launch robim_ur20_sim sim.launch.py
```

Terminal 2

```bash
ros2 launch robim_ur20_sim move_waypoints.launch.py
```

You can also drag the interactive marker in RViz and use MoveIt's
"Plan & Execute" directly.

## Linear sweep

`sweep_mover` moves to one end of a line, then sweeps the TCP in a straight
Cartesian line across the workspace with the TCP pointing down at the ground.

```bash
ros2 launch robim_ur20_sim sweep.launch.py
```

Line geometry (x/z height, y extent) and speeds are in `config/sweep.yaml`.
Keep the line's radial distance `sqrt(x² + y²)` well inside the UR20's
1.75 m reach but not too close in: very close-in lines make the down-view
sweep fail partway (the arm folds into itself).

## Viewing in Foxglove

`sim.launch.py` also starts `foxglove_bridge` (disable with
`launch_foxglove:=false`). To view:

1. Open Foxglove Studio (`foxglove-studio`).
2. "Open connection" → Foxglove WebSocket → `ws://localhost:8765`.
3. Add a **3D** panel. In the panel settings, enable the robot model under
   **Topics → /robot_description** (meshes are fetched through the bridge).

The arm will animate live as trajectories execute. Add a **Plot** panel on
`/joint_states.position[0]` etc. to graph joint motion. If you don't want
RViz at the same time, launch with `launch_rviz:=false`.

## Editing waypoints

`src/robim_ur20_sim/config/waypoints.yaml` holds one flat `waypoints` list;
each row of 6 values is one waypoint (radians, in the order shoulder_pan,
shoulder_lift, elbow, wrist_1, wrist_2, wrist_3), visited top to bottom.
Set `cycle: true` to loop forever. With `--symlink-install`, YAML edits take
effect on next run without rebuilding.

## Moving to URSim or a real UR20

The stack is identical, only the hardware endpoint changes:

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
