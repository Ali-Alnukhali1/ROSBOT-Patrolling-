# ROSbot Patrolling Project

## Overview

The **ROSbot Patrolling Project** is a demonstration of understanding and applying the **ROS2 Navigation Stack**. The primary objective of this course project is to:

- Launch a full ROS2 navigation stack along with simulation and RViz visualization.
- Enable the robot to perform autonomous tasks by following a set of predefined waypoints.
- Illustrate how ROS2 nodes interact and coordinate to achieve a high-level task.

This project allows users to define specific patrol routes, adjust the number of patrol cycles, and observe feedback in real-time.

---

## Features

- Launch the **navigation stack**, **simulation**, and **RViz** with a single command.
- Define up to **five waypoints** for the robot to patrol.
- Customize the **number of patrol cycles**.
- Real-time feedback on the robot’s progress through the waypoints.
- Modular structure, allowing easy extension and integration with other nodes.

---

## Usage

### Launch the full stack

This will launch the ROS2 Navigation Stack, Gazebo simulation, and RViz visualization:

```bash
ros2 launch rosbot_patrolling rosbot_full_stack.launch.py
```

### Launch the patrolling task only

Once the full stack is running, you can launch the patrol node:

```bash
ros2 launch rosbot_patrolling patrol.launch.py
```

## Configuration

### Waypoints

The robot’s patrol route is configured via `config/waypoints.yaml`:

```yaml
patrolling_action_client:
  ros__parameters:
    cycle_count: 3  
    waypoint1: [3.271313190460205, -3.657531261444092]
    waypoint2: [2.4051167964935303, 0.08290701359510422]
    waypoint3: [-4.635148048400879, 0.0460820198059082]
    waypoint4: [-6.4781622886657715, -3.712772846221924]
    waypoint5: [9.555916786193848, -11.72805118560791]
```

## Notes for the User

- To add new points or change the number of cycles, edit `waypoints.yaml`.
- Only 5 waypoints are supported in the current implementation.
- `cycle_count` defines how many times the robot repeats the patrol route.

## Feedback

During patrol, the node provides real-time logging in the terminal:

- Current patrol cycle number
- Current waypoint being approached
- Number of waypoints missed (if any)

This feedback allows the user to monitor the robot’s progress and ensure correct execution of patrol tasks.

