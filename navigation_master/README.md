# navigation_master

Navigation Master
=================

Overview
--------
`navigation_master` is a small ROS2 Python node that selects among multiple high-level navigation algorithms and republishes the selected algorithm's speed and steering commands on the actuator topics `/cmd_vel` and `/cmd_dir`.

Design and rationale
--------------------
- Algorithms publish their outputs on *namespaced* topics (for example `/potential_field/cmd_vel`, `/wall_follow/cmd_vel`). This keeps algorithm outputs separate and avoids accidental collisions when multiple algorithms are running.
- For backward compatibility, the navigation algorithms also keep publishing on the original topics (`/cmd_vel` and `/cmd_dir`). You can safely comment those extra publishers in the algorithm code if you prefer.
- The master subscribes to the algorithms' namespaced topics and **stores the raw numeric values** (the `Float32.data`) from the subscribers. Storing raw numbers keeps the master logic simple and avoids reusing potentially stale message objects.
- At a fixed publish rate (configurable via the `publish_rate` parameter), the master publishes the latest numeric values on the actuator topics `/cmd_vel` and `/cmd_dir`.
- When an emergency condition is published on `/emergency_stop`, the master behavior depends on the `emergency_policy` parameter:
  - `yield` (default): the master abstains from publishing to `/cmd_vel` and `/cmd_dir`, allowing the `obstacle_checker` node to handle stop / neutral / reversing sequences by publishing to those topics directly.
  - `override`: the master actively publishes zeros to `/cmd_vel` and `/cmd_dir` while emergency is active (legacy behavior).

Note: `obstacle_checker` performs more than a simple stop: it implements a state machine (stopping -> neutral -> reversing) and publishes the appropriate speed/direction commands on `/cmd_vel` and `/cmd_dir` to safely maneuver away from obstacles. Use `emergency_policy='yield'` to let `obstacle_checker` manage vehicle control during emergencies.

Why subscribe and store numeric values?
--------------------------------------
- Simplicity: the master only needs the numeric values to forward to low-level controllers; extracting `msg.data` in the subscriber callback is straightforward and efficient.
- Decoupling: by keeping algorithm outputs namespaced, the master can aggregate or switch sources without modifying the algorithms.
- Robustness: the master sets sensible defaults (0.0) if an algorithm hasn't published yet or messages are invalid.

Usage
-----
- Launch the master:
  `ros2 launch navigation_master navigation_master.launch.py`
- Switch algorithm at runtime:
  - Publish on the control topic: `ros2 topic pub /nav_master/set_mode std_msgs/String "data: 'wall_follow'" -1`
  - Or use parameters: `ros2 param set /navigation_master active_algorithm wall_follow`
- Emergency stop: publish a Bool on `/emergency_stop` (true to stop, false to resume).

Notes
-----
- The package currently listens to `/potential_field/*`, `/wall_follow/*` and a placeholder `/camera3d/*` for a future camera-based algorithm.
- If you want the master to perform arbitration (e.g., select best algorithm based on quality metrics), we can extend the data model to include timestamps and quality scores instead of plain floats.