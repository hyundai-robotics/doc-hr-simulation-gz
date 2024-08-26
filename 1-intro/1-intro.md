# 1. ROS2 Control Gazebo Simulation

This project provides a ROS2 Gazebo simulation environment for HD Hyundai Robotics robots.

**[https://github.com/hyundai-robotics/hr_simulation_gz](https://github.com/hyundai-robotics/hr_simulation_gz)**

# Features

- Robot status querying and control
- I/O and PLC control
- File and log management
- Task and program control
- System time management

<div style="background-color: white; padding: 10px;">
    <img src="../_assets/communication.png" alt="communication">
</div>

As can be seen from the structure below, the parts currently implemented using ROS Service are connected to the Open API, and the part related to robot movement is connected through UDP communication. This structure is planned to be separated or proceed in a different direction in the future.
