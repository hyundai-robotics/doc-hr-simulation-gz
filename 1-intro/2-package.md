# 2. Packages

| Package Name     | Description |
|------------------|-------------|
| hr_description   | Package containing description files for HR robots, including various configuration files and visual/physical parameter files for the models. |
| hr_moveit_config | Folder containing HR MoveIt configuration files, including setup files for integration with MoveIt. |
| hr_simulation    | Main folder of the HR Simulation package, containing simulation control and related configuration files. |
| api_agent        | Package for communication with the HR Hi6 controller and usage of ROS2 Services. |
| api_msgs         | Service Message Package for api_agent. |


# ROS2 Package Structure, Launch Files, and Parameters

## api_agent

### Launch Files
- **api_agent.launch.py**: Launches the API agent for communication with the robot controller.

| Parameter | Description |
|-----------|-------------|
| openapi_ip | Remote host for OPENAPI communication |
| openapi_port | Remote port for OPENAPI communication |
| udp_remote_host | Remote host for UDP communication |
| udp_remote_port | Remote port for UDP communication |
| udp_local_port | Local port for UDP communication |
| motor_pos_on_off | Enable/disable motor position updates |
| motor_pos_interval_ms | Minimum allowed interval for motor position updates |
| fake_hardware | Indicate whether robot is running with fake hardware mirroring command to its states |

## hr_description

### Folders
- **config**: Contains physical parameter information for each robot model
- **meshes**: Contains STL files representing the external appearance of each robot model
- **rviz**: Contains RViz configuration files
- **urdf**: Contains URDF files for robot descriptions

### Launch Files
- **view_hr.launch.py**: Launches the HR robot model visualization in RViz.

| Parameter | Description |
|-----------|-------------|
| hr_type | Type/series of used HR robot |
| safety_limits | Enables the safety limits controller if true |
| safety_pos_margin | The margin to lower and upper limits in the safety controller |
| safety_k_position | k-position factor in the safety controller |
| description_package | Description package with robot URDF/XACRO files. Usually the argument is not set, it enables use of a custom description |
| description_file | URDF/XACRO description file with the robot |
| tf_prefix | Multi-robot setup. If changed, joint names in the controllers' configuration have to be updated |

## hr_moveit_config

### Folders
- **config**: Contains MoveIt configuration files
- **rviz**: Contains RViz configuration files
- **srdf**: Contains Semantic Robot Description Format files

### Launch Files
- **hr_moveit.launch.py**: Launches the MoveIt configuration for the HR robot.

| Parameter | Description |
|-----------|-------------|
| hr_type | Type/series of used HR robot |
| use_fake_hardware | Indicate whether robot is running with fake hardware mirroring command to its states |
| safety_limits | Enables the safety limits controller if true |
| safety_pos_margin | The margin to lower and upper limits in the safety controller |
| safety_k_position | k-position factor in the safety controller |
| description_package | Description package with robot URDF/XACRO files. Usually the argument is not set, it enables use of a custom description |
| description_file | URDF/XACRO description file with the robot |
| moveit_config_package | MoveIt config package with robot SRDF/XACRO files. Usually the argument is not set, it enables use of a custom moveit config |
| moveit_config_file | MoveIt SRDF/XACRO description file with the robot |
| moveit_joint_limits_file | MoveIt joint limits that augment or override the values from the URDF robot_description |
| warehouse_sqlite_path | Path where the warehouse database should be stored |
| use_sim_time | Make MoveIt use simulation time. This is needed for trajectory planning in simulation |
| prefix | Multi-robot setup. If changed, joint names in the controllers' configuration have to be updated |

## hr_simulation

### Folders
- **config**: Contains HR controller-related configuration files

### Launch Files
- **hr_sim_control.launch.py**: Launches the HR robot simulation with basic control.

| Parameter | Description |
|-----------|-------------|
| hr_type | Type/series of used HR robot |
| safety_limits | Enables the safety limits controller if true |
| safety_pos_margin | The margin to lower and upper limits in the safety controller |
| safety_k_position | k-position factor in the safety controller |
| runtime_config_package | Package with the controller's configuration in "config" folder. Usually the argument is not set, it enables use of a custom setup |
| controllers_file | YAML file with the controllers configuration |
| description_package | Description package with robot URDF/XACRO files. Usually the argument is not set, it enables use of a custom description |
| description_file | URDF/XACRO description file with the robot |
| prefix | Multi-robot setup. If changed, joint names in the controllers' configuration have to be updated |
| start_joint_controller | Enable headless mode for robot control |
| initial_joint_controller | Robot controller to start |

- **hr_sim_moveit.launch.py**: Launches the HR robot simulation with MoveIt integration.

| Parameter | Description |
|-----------|-------------|
| hr_type | Type/series of used HR robot |
| use_fake_hardware | Indicate whether robot is running with fake hardware mirroring command to its states |
| safety_limits | Enables the safety limits controller if true |
| runtime_config_package | Package with the controller's configuration in "config" folder. Usually the argument is not set, it enables use of a custom setup |
| controllers_file | YAML file with the controllers configuration |
| description_package | Description package with robot URDF/XACRO files. Usually the argument is not set, it enables use of a custom description |
| description_file | URDF/XACRO description file with the robot |
| moveit_config_package | MoveIt config package with robot SRDF/XACRO files. Usually the argument is not set, it enables use of a custom moveit config |
| moveit_config_file | MoveIt SRDF/XACRO description file with the robot |
| prefix | Multi-robot setup. If changed, joint names in the controllers' configuration have to be updated |