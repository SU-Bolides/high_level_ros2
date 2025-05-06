# high_level_ros2
The official Sorbonne University ROS2 repository for the CoVaPsy course at ENS Paris-Saclay. High-Level packages only
**When cloning this repository, please put it in the src folder of you workspace**

### General informations
- Raspberry Pi 5 (RPi5) - OS : Ubuntu 24.04
- ROS2 Jazzy
- Repository for [Low-Level nodes](https://github.com/SU-Bolides/course_ros2)
- [Official repository of the race (for schematics)](https://github.com/ajuton-ens/CourseVoituresAutonomesSaclay.git)
- [Repository of last year (2024)](https://github.com/SU-Bolides/course_2025_slam_pkgs.git) : There is the main branch in ROS and the ROS2 branch

### High-Level packages
#### Wall Follower
This package is a simple PID controller to keep the car at a given distance from a wall. The controller is based on the controller created for the F1 Tenth course project at the University of Pennsylvania, you can find the lessons [here](https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleB/lecture04.html) and the code [here](https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main/src/wall_follow).