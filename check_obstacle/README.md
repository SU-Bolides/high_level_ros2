# Check_obstacle
This package is used for verification of close obstacles in front of the car.

### General informations
This package check for walls or close obstacles in the front of the car by using the Lidar data and checking the smallest distance to an obstacle.

For the race, the car needs to be able to stop if faced with a barrier and back a little to get out of its situation.

However, the car, like mentionned in the documentation of the [Low-level packages](https://github.com/SU-Bolides/low_level_ros2) needs to operate based on a sort of state machine for going backwards. So we need to be careful when commanding it.

## How it works
This package contains a bunch of differents parameters to adjust the behavior of the car. You can change their values in your launch files if you need to.
The state machine is configured so you need to pass by a neutral state before reversing the vehicle.

The check for obstacles is done in a small cone around the 180 degrees of the Lidar (wich seems to correspond to the front of the car). You can also modify the values used to scan (for now it's +=30 around 180).

We check the smallest distance to an object found and we compare it to the parameter defined as the distance needed to make the car stop. If it's inferior, we send a neutral command to the car (0 for speed and direction) before commanding the reverse.

The car is now ready to go back to navigation.

## Troubleshooting

- If you see that the car is not reversing, try sending it more neutral before reversing (lenghten the time of neutral_duration), we don't really know exactly how much is needed, still work in progress.

- If the car is not moving at all before or after a obstacle detection, try pressing the button that is on the bottom right of the little screen and relaunch your code. It should help.