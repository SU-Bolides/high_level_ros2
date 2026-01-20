# navigation_master

This package provides the `navigation_master` node: a simple master controller that selects among several high-level navigation algorithms and republishes the selected algorithm's steering and speed commands on `/cmd_dir` and `/cmd_vel` for the low-level nodes (STM32 and direction nodes) to execute.

## How it works

- The `navigation_master` node subscribes to namespaced outputs of algorithms, e.g. `/potential_field/cmd_vel`, `/wall_follow_pid/cmd_dir`.
- It keeps the latest `(vel, dir)` from each algorithm and periodically republishes the one from the configured `active_algorithm` on `/cmd_vel` and `/cmd_dir`.
- When `/emergency_stop` (`std_msgs/Bool`) is `true`, the node stops republishing and lets the emergency/obstacle checker handle the car.

## Parameters

- `active_algorithm` (string, default: `potential_field`)
  - Supported values: `potential_field`, `wall_follow_pid`, `camera3d`.
- `publish_rate` (float, default: `10.0`)
  - Frequency (Hz) at which the master republishes the selected algorithm's outputs.

## Launch file

This package provides a launch file `launch/navigation_master.launch.py` that:
- Starts the `navigation_master` node, passing `active_algorithm` as a parameter.
- Conditionally includes one of the algorithm launches:
  - `potential_field.launch.py` (if `active_algorithm=="potential_field"`)
  - `wall_follow_pid.launch.py` (if `active_algorithm=="wall_follow_pid"`)

Both of them should launch the `sllidar_ros2`, `stm32_node`, `cmd_vel_node`, `cmd_dir_node`, and `obstacle_checker` nodes as well as the main chosen algorithm.
This makes it convenient to start *all* nodes required to run the car in one command.

## How to run

1) Build and source your workspace:

```bash
colcon build
source install/setup.bash   #we have the alias srcw for this
```

2) Launch with default algorithm (potential field):

```bash
ros2 launch navigation_master navigation_master.launch.py
```

3) Launch selecting the wall-follow algorithm:

```bash
ros2 launch navigation_master navigation_master.launch.py active_algorithm:=wall_follow_pid
```

Notes
-----
- The launch includes low-level nodes (`sllidar_ros2`, `stm32_node`, `cmd_vel_node`, `cmd_dir_node`) via the selected algorithm's launch file, so you do not need to start those separately when using the launch described above.
- If you prefer to run nodes manually for debugging, you can run `ros2 run navigation_master navigation_master --ros-args -p active_algorithm:=wall_follow_pid` (runs only the master node, not the included algorithm-specific nodes).