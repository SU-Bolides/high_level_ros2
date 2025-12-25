# potential_field_nav
This package contains a Potential Field based navigator designed to navigate unknown circuits.

### General information
The navigator combines an attractive forward vector with repulsive forces computed from LIDAR scans to produce smooth steering and speed commands suitable for navigation in unknown tracks.
high_level_ros2\images\potential_nav_scheme.png
## How it works
![](/high_level_ros2/images/potential_nav_scheme.png)

Basically, we make the car being "pushed" away from obstacles detected by the LIDAR while being "pulled" forward along the track centerline.

- The node subscribes to `/scan` and computes a repulsive vector from obstacles within an influence distance.
- An attractive vector points forward and the sum of attractive and repulsive vectors defines the navigation command.
- The node publishes a normalized steering command on `/cmd_dir` and a speed command on `/cmd_vel`.
- It listens to `/emergency_stop` (Bool) to disable navigation when an obstacle checker triggers an emergency stop.

## Parameters and tuning
Several parameters such as `influence_distance`, `k_repulsive`, `k_attractive` and speed limits are defined in the node and can be adjusted in launch files if needed (it's not yet added in the launch file but you can add them easily).

## Troubleshooting
- If navigation is oscillating, try reducing `k_repulsive` or increasing `smoothing_alpha`.
- If the car is not stopping on close obstacles, verify that `/emergency_stop` is being published by the `check_obstacle` package or try fiddling with `influence_distance`.
