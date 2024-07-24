# Services

This document provides a comprehensive list of ROS2 services available in the HD Hyundai Robotics Robots ROS2 GZ Simulation project. The services are categorized by their functionality and include descriptions and usage examples.

## Table of Contents

- [Version Services](#version-services)
- [Project Services](#project-services)
- [Control Services](#control-services)
- [Robot Services](#robot-services)
- [I/O PLC Services](#io-plc-services)
- [Log Manager Services](#log-manager-services)
- [File Manager Services](#file-manager-services)
- [Task Services](#task-services)
- [Clock Services](#clock-services)

## Version Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/get/api_ver` | `std_srvs::srv::Trigger` | Get Open API schema version | `ros2 service call /api_agent/get/api_ver std_srvs/srv/Trigger` |
| `/api_agent/get/sysver` | `std_srvs::srv::Trigger` | Get robot controller system software version | `ros2 service call /api_agent/get/sysver std_srvs/srv/Trigger` |

## Project Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/project/get/rgen` | `std_srvs::srv::Trigger` | Read general information set in the controller | `ros2 service call /api_agent/project/get/rgen std_srvs/srv/Trigger` |
| `/api_agent/project/get/jobs_info` | `std_srvs::srv::Trigger` | Get information related to job programs | `ros2 service call /api_agent/project/get/jobs_info std_srvs/srv/Trigger` |
| `/api_agent/project/post/reload_updated_jobs` | `std_srvs::srv::Trigger` | Request to update job files | `ros2 service call /api_agent/project/post/reload_updated_jobs std_srvs/srv/Trigger` |
| `/api_agent/project/post/delete_job` | `api_msgs::srv::FilePath` | Request to delete a job file | `ros2 service call /api_agent/project/post/delete_job api_msgs/srv/FilePath "{path: '0001.job'}"` |

## Control Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/control/get/op_cnd` | `std_srvs::srv::Trigger` | Get condition setting values | `ros2 service call /api_agent/control/get/op_cnd std_srvs/srv/Trigger` |
| `/api_agent/control/get/ios_di` | `api_msgs::srv::IoRequest` | Get user IO values (Digital Input) | `ros2 service call /api_agent/control/get/ios_di api_msgs/srv/IoRequest "{type: 'di', blk_no: 1, sig_no: 1}"` |
| `/api_agent/control/get/ios_do` | `api_msgs::srv::IoRequest` | Get user IO values (Digital Output) | `ros2 service call /api_agent/control/get/ios_do api_msgs/srv/IoRequest "{type: 'do', blk_no: 1, sig_no: 1}"` |
| `/api_agent/control/get/ios_si` | `api_msgs::srv::IoRequest` | Get system IO values (System Input) | `ros2 service call /api_agent/control/get/ios_si api_msgs/srv/IoRequest "{type: 'si', blk_no: 1, sig_no: 1}"` |
| `/api_agent/control/get/ios_so` | `api_msgs::srv::IoRequest` | Get system IO values (System Output) | `ros2 service call /api_agent/control/get/ios_so api_msgs/srv/IoRequest "{type: 'so', blk_no: 1, sig_no: 1}"` |
| `/api_agent/control/get/ucs_nos` | `std_srvs::srv::Trigger` | Get list of currently used user coordinate systems | `ros2 service call /api_agent/control/get/ucs_nos std_srvs/srv/Trigger` |
| `/api_agent/control/post/ios_do` | `api_msgs::srv::IoRequest` | Change digital output | `ros2 service call /api_agent/control/post/ios_do api_msgs/srv/IoRequest "{type: 'do', blk_no: 1, sig_no: 1, val: 1}"` |
| `/api_agent/control/put/op_cnd` | `api_msgs::srv::OpCnd` | Change robot's condition setting values | `ros2 service call /api_agent/control/put/op_cnd api_msgs/srv/OpCnd "{playback_mode: 1, step_goback_max_spd: 130, ucrd_num: 2}"` |

## Robot Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/robot/get/motor_state` | `std_srvs::srv::Trigger` | Get motor on state | `ros2 service call /api_agent/robot/get/motor_state std_srvs/srv/Trigger` |
| `/api_agent/robot/get/po_cur` | `api_msgs::srv::PoseCur` | Get current robot pose | `ros2 service call /api_agent/robot/get/po_cur api_msgs/srv/PoseCur "{crd: 0, mechinfo: 1}"` |
| `/api_agent/robot/get/cur_tool_data` | `std_srvs::srv::Trigger` | Get current tool data | `ros2 service call /api_agent/robot/get/cur_tool_data std_srvs/srv/Trigger` |
| `/api_agent/robot/get/tools` | `std_srvs::srv::Trigger` | Get all tool information | `ros2 service call /api_agent/robot/get/tools std_srvs/srv/Trigger` |
| `/api_agent/robot/get/tools_t` | `api_msgs::srv::Number` | Get specific tool setting information | `ros2 service call /api_agent/robot/get/tools_t api_msgs/srv/Number "{data: 0}"` |
| `/api_agent/robot/post/motor_control` | `std_srvs::srv::SetBool` | Turn robot motor ON/OFF | `ros2 service call /api_agent/robot/post/motor_control std_srvs/srv/SetBool "{data: true}"` |
| `/api_agent/robot/post/robot_control` | `std_srvs::srv::SetBool` | Start/Stop robot | `ros2 service call /api_agent/robot/post/robot_control std_srvs/srv/SetBool "{data: true}"` |
| `/api_agent/robot/post/tool_no` | `api_msgs::srv::Number` | Set current tool number | `ros2 service call /api_agent/robot/post/tool_no api_msgs/srv/Number "{data: 0}"` |
| `/api_agent/robot/post/crd_sys` | `api_msgs::srv::Number` | Set current jog coordinate system | `ros2 service call /api_agent/robot/post/crd_sys api_msgs/srv/Number "{data: 0}"` |

## I/O PLC Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/plc/get/relay_value` | `api_msgs::srv::IoplcGet` | Get relay values for the entire object type | `ros2 service call /api_agent/plc/get/relay_value api_msgs/srv/IoplcGet` |
| `/api_agent/plc/post/relay_value` | `api_msgs::srv::IoplcPost` | Set relay values | `ros2 service call /api_agent/plc/post/relay_value api_msgs/srv/IoplcPost` |

## Log Manager Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/log/get/manager` | `api_msgs::srv::LogManager` | View event log with specified filter conditions | `ros2 service call /api_agent/log/get/manager api_msgs/srv/LogManager` |

## File Manager Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/file/get/files` | `api_msgs::srv::FilePath` | Get file contents from controller | `ros2 service call /api_agent/file/get/files api_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` |
| `/api_agent/file/get/file_info` | `api_msgs::srv::FilePath` | Get file information based on file path | `ros2 service call /api_agent/file/get/file_info api_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` |
| `/api_agent/file/get/file_list` | `api_msgs::srv::FileList` | Get list of files and directories | `ros2 service call /api_agent/file/get/file_list api_msgs/srv/FileList "{path: 'project/jobs', incl_file: true, incl_dir: false}"` |
| `/api_agent/file/get/file_exist` | `api_msgs::srv::FilePath` | Check if target file exists | `ros2 service call /api_agent/file/get/file_exist api_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` |
| `/api_agent/file/post/rename_file` | `api_msgs::srv::FileRename` | Rename target file | `ros2 service call /api_agent/file/post/rename_file api_msgs/srv/FileRename "{pathname_from: 'project/jobs/0001.job',pathname_to: 'project/jobs/4321.job'}"` |
| `/api_agent/file/post/mkdir` | `api_msgs::srv::FilePath` | Create directory at target path | `ros2 service call /api_agent/file/post/mkdir api_msgs/srv/FilePath "{path: 'project/jobs/special'}"` |
| `/api_agent/file/post/files` | `api_msgs::srv::FileSend` | Send file to target path | `ros2 service call /api_agent/file/post/files api_msgs::srv::FileSend "{target_file: 'project/jobs/test.job', source_file: '/home/test/test.job'}"` |
| `/api_agent/file/delete/file` | `api_msgs::srv::FilePath` | Delete target file or directory | `ros2 service call /api_agent/file/delete/file api_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` |

## Task Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/task/post/cur_prog_cnt` | `api_msgs::srv::ProgramCnt` | Set current program counter of the task | `ros2 service call /api_agent/task/post/cur_prog_cnt api_msgs/srv/ProgramCnt "{pno: -1, sno: -1, fno: -1, ext_sel: 0}"` |
| `/api_agent/task/post/reset` | `std_srvs::srv::Trigger` | Reset all tasks | `ros2 service call /api_agent/task/post/reset std_srvs/srv/Trigger` |
| `/api_agent/task/post/reset_t` | `api_msgs::srv::Number` | Perform reset on a task | `ros2 service call /api_agent/task/post/reset_t api_msgs/srv/Number "{data: 0}"` |
| `/api_agent/task/post/assign_var` | `api_msgs::srv::ProgramVar` | Reassign variable of current task statement | `ros2 service call /api_agent/task/post/assign_var api_msgs/srv/ProgramVar "{name: 'a', scope: 'local', expr: '14 + 2', save: 'true'}"` |
| `/api_agent/task/post/release_wait` | `std_srvs::srv::Trigger` | Release statement stop | `ros2 service call /api_agent/task/post/release_wait std_srvs/srv/Trigger` |
| `/api_agent/task/post/set_cur_pc_idx` | `api_msgs::srv::Number` | Position current cursor at index line | `ros2 service call /api_agent/task/post/set_cur_pc_idx api_msgs/srv/Number "{data: 0}"` |
| `/api_agent/task/post/solve_expr` | `api_msgs::srv::ProgramVar` | Solve expression and set result to task's local or global variable | `ros2 service call /api_agent/task/post/solve_expr api_msgs/srv/ProgramVar "{name: 'a', scope: 'local'}"` |

## Clock Services

| Service Name | Service Type | Description | Example Usage |
|--------------|--------------|-------------|---------------|
| `/api_agent/clock/get/date_time` | `std_srvs::srv::Trigger` | Get set system time | `ros2 service call /api_agent/clock/get/date_time std_srvs/srv/Trigger` |
| `/api_agent/clock/put/date_time` | `api_msgs::srv::DateTime` | Change system time | `ros2 service call /api_agent/clock/put/date_time api_msgs/srv/DateTime "{year: 2024, mon: 7, day: 11, hour: 15, min: 13, sec: 0}"` |
