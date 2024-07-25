# ROS2 Workspace Setup Guide

## Configure and Build Workspace

### 1. Git Clone Package

Navigate to the `src` directory of your ROS2 workspace and clone the necessary package:

```bash
cd ~/ros2_ws/src
git clone https://github.com/hyundai-robotics/hr_simulation_gz.git
```

Replace the URL with the actual repository URL for your project.

### 2. Install Dependency Packages

#### 2.1 Install General Dependencies

Download the required repositories and install package dependencies:

```bash
cd ~/ros2_ws/
rosdep update && rosdep install --ignore-src --from-paths src -y
```

This command updates `rosdep` and automatically installs dependencies for all packages in the `src` folder.

#### 2.2 Install Project-Specific Dependencies

Run the project-specific dependency installation script:

```bash
cd ~/ros2_ws/src/hr_simulation_gz
bash ros2-install-deps.sh
```

This script installs additional dependencies specific to your project.

### 3. Configure and Install Workspace

Finally, configure and install the workspace:

```bash
cd ~/ros2_ws/src/hr_simulation_gz
cp install ~/ros2_ws/
source ~/ros2_ws/install/setup.bash
```

This process involves:

1. Changing to the `hr_simulation_gz` directory.
2. Copying the `install` directory to the root of `ros2_ws`.
3. Sourcing the `setup.bash` file to make the newly installed packages available in the current shell session.

## Conclusion

After completing these steps, your workspace should be fully configured and built. You're now ready to run your ROS2 project.

Remember to source the setup file (`source ~/ros2_ws/install/setup.bash`) in each new terminal where you want to use this ROS2 workspace.