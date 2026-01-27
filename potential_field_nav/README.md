# Potential Field Navigator
This package contains a Potential Field based navigator designed to navigate unknown circuits.

### General information
The navigator combines an attractive forward vector with repulsive forces computed from LIDAR scans to produce smooth steering and speed commands suitable for navigation in unknown tracks.
high_level_ros2\images\potential_nav_scheme.png
## How it works
![](/images/potential_nav_scheme.png)

Basically, we make the car being "pushed" away from obstacles detected by the LIDAR while being "pulled" forward along the track centerline.

- The node subscribes to `/scan` and computes a repulsive vector from obstacles within an influence distance.
- An attractive vector points forward and the sum of attractive and repulsive vectors defines the navigation command.
- The node publishes a normalized steering command on `/cmd_dir` and a speed command on `/cmd_vel`.
- It listens to `/emergency_stop` (Bool) to disable navigation when an obstacle checker triggers an emergency stop.

--> The ![potential_field_nav](/potential_field_nav/potential_field_nav/potential_field_navigator.py) is the base version of this algorithm : simple but working well for basic navigation. It oscillates a bit more but is easier to understand and modify. You can use it for testing some new things but keep the current version somewhere to avoid losing something working. (for example, copy and paste the code and write the modified version below the first one whic you can comment to test the modifs).

--> The better and upgraded version of this code is the ![unified_potential_field_navigator](/potential_field_nav/potential_field_nav/unified_potential_field_navigator.py). This one implements three main things :
- The basic logic found in the first version
- The usage of obstacle proximity to slow down the car when approaching obstacles proportionnally to their distance
- A smoothing logic based on the magbitude of the resulting forces so when the car is between two straight walls for example and it's pretty centered, we try to not tamper with the steering commands to avoid zigzaging behaviors if we can. Tweak the `repulsive_deadzone` and `smoothing_alpha` parameters to adjust this behavior. Use logging infos to debug and adjust better these parameters.
## Parameters and tuning
Several parameters such as `influence_distance`, `k_repulsive`, `k_attractive` and speed limits are defined in the node and can be adjusted in launch files if needed . Tweak them to get the desired navigation behavior.
When you have the right value, write it somewhere maybe in comment in the launch file or add a section on the README.md to keep track of the best parameters for your specific setup.
Keep in mind the car will not react the same way wether you test in in Escanglon or wether you test it at St-Cyr, so you may need to adjust parameters depending on the track.

## Troubleshooting
- If navigation is oscillating, try reducing `k_repulsive` or increasing `smoothing_alpha`.
- If the car is not stopping on close obstacles, verify that `/emergency_stop` is being published by the `check_obstacle` package or try fiddling with `influence_distance`. Try augmenting the `obstacle_distance` in the `check_obstacle` package if needed : the more your car goes fast the more this distance needs to be important to stop in time. The Lidar does not see below a certain distance (usually 10-15cm and below are not detected) so be sure to take this into account when tuning this parameter.
