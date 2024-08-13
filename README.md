# Obstacle-Avoidance-using-VFF

## Project Overview

This project implements a simple behavior that allows a turtle bot to move forward while avoiding obstacles using the Virtual Force Field (VFF) algorithm. The algorithm uses three 2D vectors to calculate the speed command that deviates the robot away from obstacles:
1. Attractive Vector: Always points forward as the robot aims to move in a straight line when no obstacles are present.
2. Repulsive Vector: Calculated from the laser sensor readings. It is inversely proportional to the distance from the nearest obstacle.
3. Resultant Vector: The sum of the attractive and repulsive vectors, used to calculate the robot's control speed and turn angle.


## Project Structure
omniverse_navigation/ <br>
├── CMakeLists.txt <br>
├── package.xml<br>
├── config/<br>
│   └── AvoidanceNodeConfig.yaml <br>
├── include/<br>
│   └── omniverse_navigation/<br>
│       └── avoidance_node.hpp <br>
├── launch/<br>
│   └── launch.py <br>
└── src/<br>
    ├── avoidance_node.cpp<br>
        


 ## Control Logic

The AvoidanceNode implements the VFF algorithm to generate control commands based on laser scan readings:

- Executes the algorithm iteratively at 20Hz using a timer.
- Ensures the laser scan message is valid (pointer not null, and message not older than 2 seconds).
- Retrieves the three vectors and uses the resultant vector to determine output speed.
- Clamps the speeds to a safe range and generates the output cmd_vel message.       


## Debugging with Visual Markers

The AvoidanceNode publishes visual markers for debugging purposes in Rviz2:

- Publishes messages of type visualization_msgs/msg/MarkerArray, with each marker as visualization_msgs/ msg/Marker of type arrow.
- Publishes debugging messages only if there are subscribers to the debugging topic.


## How to Clone and Build the Project

### Cloning the Repository

To clone the repository, use the following command:

```sh
git clone git@github.com:AnthonyElMoundalak/GroupAAA-VFF-Avoidance-Assignment11.git
```

### Building the project

Navigate to the workspace directory and use the following commands

```sh
colcon build
source install/setup.bash
```

### Running the project 

After successfully compiling the project, you can run the executable:

```sh
ros2 launch omniverse_navigation launch.py

```
## Conclusion

This projects showcases the working of the virtual force field algorithm in navigating an obstacle course, as well as visualization techniques for debugging purposes.

## Team members

- Anthony Bassil  
- Anthony Moundalak 
- Ahmad Chaachouh
