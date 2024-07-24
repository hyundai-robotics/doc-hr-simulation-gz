# 1. Setup ROS2 Install

This guide provides instructions for downloading and executing the `ros2-humble-desktop-main.sh` script to install ROS2 Humble Desktop.

## Installation Steps

1. Download the `ros2-humble-desktop-main.sh` script from the package repository.

2. Open a terminal and navigate to the directory where you downloaded the script.

3. Make the script executable and run it by executing the following command:

   ```bash
   bash ros2-humble-desktop-main.sh
   ```

   > **NOTE:** A `~/ros2_ws/src` folder will be created. If desired, users can create and use a different folder without any issues.

4. Follow any on-screen prompts during the installation process.

5. Once the installation is complete, restart your system or source the ROS2 setup file:

    ```bash
    source /opt/ros/humble/setup.bash
    ```

## Next Steps

After successful installation, you can proceed with configuring your ROS2 workspace and installing any project-specific dependencies.

Remember to source the ROS2 setup file in each new terminal session where you want to use ROS2, or add it to your `.bashrc` file for automatic sourcing.