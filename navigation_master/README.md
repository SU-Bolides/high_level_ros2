# navigation_master

Node that acts as a master controller for our multiple navigation algorithms. It subscribes to the outputs of different navigation algorithms (e.g. potential field, wall following PID, 3D camera-based navigation) and republishes the selected algorithm's commands to the car. For now, choice is binary and set at launch time, later we will maybe add dynamic switching and analysis of commands to choose the best one at each time step.

Summary
-------
- Subscribes to namespaced algorithm outputs (e.g. `/potential_field/cmd_vel`, `/wall_follow_pid/cmd_vel`).
- Republishes the selected algorithm's latest `Float32` values on `/cmd_vel` and `/cmd_dir` at `publish_rate` Hz.
- If `/emergency_stop` (`std_msgs/Bool`) is true, we let the emergency node handle the car and do not publish any commands.

Parameters
----------
- `active_algorithm` (string, default: `potential_field`)
  - Supported: `potential_field`, `wall_follow_pid`, `camera3d`.
  - You need to change the launch file to modify the algorithm you want to use.
- `publish_rate` (default: `10.0`) — publish frequency in Hz.

Quick start
-----------
- Launch :
  `ros2 launch navigation_master navigation_master.launch.py`
- Run with a different startup algorithm (or change in the launch file):
  `ros2 run navigation_master navigation_master --ros-args -p active_algorithm:=wall_follow_pid`
