# Running Executable

This guide provides instructions for running various executables in your ROS2 workspace.

## Preparation

Before running any executable, make sure to source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Launching Simulations

### Basic Simulation Control

To launch the basic HR simulation control:

```bash
ros2 launch hr_simulation hr_sim_control.launch.py
```

### MoveIt with Simulated Robot

To use MoveIt with the simulated robot:

```bash
ros2 launch hr_simulation hr_sim_moveit.launch.py
```

## Connecting to Robot Controller

To connect to the actual robot controller:

```bash
ros2 launch api_agent api_agent.launch.py
```

## Notes

- Ensure that all necessary packages are installed and built in your workspace before running these commands.
- Check the console output for any error messages or additional instructions after launching each executable.
- Some launch files may require additional arguments or configurations. Refer to the specific package documentation for more details.
