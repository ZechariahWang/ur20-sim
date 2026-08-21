# ur20_sim hello

## Install

```bash
sudo apt install ros-humble-ur ros-humble-moveit
```

## ZED camera dependency

The ZED wrapper is a source dependency, not tracked in this repo. After
cloning, fetch it (needs the ZED SDK 4.2 installed from stereolabs.com):

```bash
cd src
git clone --branch humble-v4.2.5 --depth 1 https://github.com/stereolabs/zed-ros2-wrapper.git
```

## Build

```bash
cd ~/robim_ur20_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run in sim

```bash
# terminal 1: sim + MoveIt + RViz + Foxglove bridge + room
ros2 launch ur20_sim sim.launch.py

# terminal 2: the production sweep (room-validated)
ros2 launch ur20_sim main.launch.py

# calibration tests
ros2 launch ur20_sim tcp_orientate.launch.py
ros2 launch ur20_sim axis_test.launch.py
ros2 launch ur20_sim sweep.launch.py
```

## Run on the real robot

```bash
# terminal 1 (then press play on the External Control program on the pendant)
ros2 launch ur20_sim sim.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10

# terminal 2
ros2 launch ur20_sim main.launch.py
```

## Foxglove

Open Foxglove Studio -> Open connection -> `ws://localhost:8765`.
3D panel: enable `/robot_description` (robot) and `/room_markers` (room).

## Config

- `config/main_sweep.yaml`: production sweep geometry, speeds, room margin
- `config/sweep.yaml`: calibration sweep geometry and speeds
- `config/room.yaml`: real cell dimensions (floor, roof, walls, pillar)
- `config/initial_positions.yaml`: sim spawn pose (real parked pose)

YAML edits apply on next launch, no rebuild needed.
