# 1. Operation of `gz_trigger_callback`

The `gz_trigger_callback` function is responsible for setting the robot's initial pose and publishing related information. When this function is called, it operates in the following sequence:

1. **Acquire Current Robot Joint Positions**
   - Calls `robot::robot_internal_po_cur(this->client)` to retrieve the current joint positions of the robot.

2. **Log Joint Positions**
   - Outputs the position of each joint to the console.

3. **Generate Joint Trajectory Message**
   - Creates a message of type `trajectory_msgs::msg::JointTrajectory`.

4. **Set Joint Names**
   - Dynamically sets joint names based on the number of robot joints:
     - For 6 joints: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
     - For 7 joints: The above 6 plus `conveyor_to_robot_base`

5. **Configure Trajectory Point**
   - Creates a trajectory point using the acquired joint positions.
   - Sets the execution time to 3 seconds.

6. **Publish Joint Trajectory**
   - Publishes the generated trajectory message via `joint_trajectory_publisher_`.

7. **Set Response**
   - Sets the `success` field of the service response to `true`.
   - Sets the response message to "Trigger was successful".

8. **Output Log**
   - Logs a message indicating that the joint trajectory command has been sent.

This function is used to move the robot to its initial pose, generating and publishing a new trajectory based on the current joint state. This allows the robot to smoothly transition from its current position to the initial pose.

---

# 2. Timer Callback Data Transmission Criteria and State Management

## State and Status Triggers

### START State
The `current_state_` is set to "START" in two cases:

1. In the `feedback_callback` function:
   - Topic: `/execute_trajectory/_action/feedback`
   - Code:
     ```cpp
     if (state == "MONITOR")
     {
         current_state_ = "START";
     }
     ```

2. In the `action_feedback_callback` function:
   - Topic: `/move_action/_action/feedback`
   - Code:
     ```cpp
     if (state == "PLANNING" && flag_state_)
     {
         current_state_ = "START";
     }
     ```

### execute_status
In the `execute_status_callback` function:
- Topic: `/execute_trajectory/_action/status`
- Code:
  ```cpp
  if (!msg->status_list.empty())
  {
      execute_status_ = true;
      RCLCPP_INFO(this->get_logger(), "Execute status received. Setting execute_status_ to true.");
      // ... (status logging)
  }
  ```

### Other Related Callback Functions
| Function | Topic | Purpose |
|----------|-------|---------|
| `action_status_callback` | `/move_action/_action/status` | Updates `flag_state_` |
| `jointStateCallback` | `/joint_states` | Updates `latest_joint_state_` |
| `planningDataCallback` | `/display_planned_path` | Processes planned trajectory data |

## Timer Callback Data Transmission Criteria

The Timer Callback function in the `ApiAgent` class transmits data based on the current state, hardware configuration, and execution status:

### 1. Real Hardware Mode

**Condition:** 
```cpp
current_state_ == "START" && !latest_joint_state_.position.empty() && !fake_hardware_
```

**Process:**
1. Create joint data map from `latest_joint_state_`
2. Convert joint positions from radians to degrees
3. Sort data in predefined joint order
4. Call `send_data()` to transmit data
5. Log joint information

### 2. Fake Hardware Mode

**Condition:** 
```cpp
execute_status_ && fake_hardware_ && trajectory_in_progress_
```

**Process:**
1. Retrieve next planned position from `planned_positions_`
2. Reorder data to desired joint sequence
3. Call `send_data()` to transmit data
4. Log position information
5. Update trajectory progress

## Key Points

- State transitions are managed by feedback from various topics.
- `execute_status_` is updated by messages from the `/execute_trajectory/_action/status` topic.
- Shared data access is protected by mutexes.
- Supports 6 or 7 joint configurations.
- In fake hardware mode, planned positions are transmitted sequentially.
- Detailed logging for both real and fake hardware modes.

## Data Order

Joint order:
1. `shoulder_pan_joint`
2. `shoulder_lift_joint`
3. `elbow_joint`
4. `wrist_1_joint`
5. `wrist_2_joint`
6. `wrist_3_joint`
7. `conveyor_to_robot_base` (for 7-joint configuration)

## Related Publishers

| Publisher | Topic | Purpose |
|-----------|-------|---------|
| `joint_trajectory_publisher_` | `/joint_trajectory_controller/joint_trajectory` | Publishes joint trajectory commands |
| `robot_state_publisher_` | `/api_agent/motor_state` | Publishes motor state |
| `robot_pos_publisher_` | `/api_agent/motor_position` | Publishes motor position |

This Timer Callback system monitors the robot's state through various ROS topics and transmits joint data at appropriate times. It supports both real hardware and simulation modes, responding to various stages of robot operation.

---

# 3. `ApiAgent::start_receive()` Function Data Flow

## 1. Data Reception
- **Input**: Raw data via UDP socket.
- **Format**: String (e.g., `"[10.5,20.3,30.1,40.8,50.2,60.7]"`).

## 2. Data Parsing
- **Process**: 
  1. Remove brackets.
  2. Split by commas.
  3. Convert strings to floating-point numbers.

## 3. Unit Conversion
- **Input**: Joint angles in degrees.
- **Process**: Convert degrees to radians (multiply by π/180).
- **Output**: List of joint angles in radians.

## 4. Robot Command Generation
- **Input**: List of joint angles in radians.
- **Process**: 
  1. Create `JointTrajectory` message.
  2. Set joint names (6-axis or 7-axis).
  3. Set position data.
  4. Set execution time (3 seconds).

## 5. Command Publication
- **Output**: `JointTrajectory` message.
- **Target**: Sent to robot control system via `joint_trajectory_publisher_`.

## Key Data Transformation Example
1. **Input data**: `"[10.5,20.3,30.1,40.8,50.2,60.7]"`
2. **After parsing**: `[10.5, 20.3, 30.1, 40.8, 50.2, 60.7]`
3. **After radian conversion**: `[0.183, 0.354, 0.525, 0.712, 0.876, 1.059]`
4. **Final output**: `JointTrajectory` message (containing 6 joint positions).

## Important Notes
- This process is only performed when `fake_hardware_` mode is `true`.
- In case of data reception errors, it logs the error and attempts to receive again.
- After processing, it immediately prepares for the next data reception.

## Error Handling
- Logs error messages for invalid number formats or out-of-range values.
- Continues to listen for new data even after encountering an error.

## Continuous Operation
- The function recursively calls itself to maintain continuous data reception.
- This ensures uninterrupted monitoring of incoming UDP messages.

---

This Markdown document provides a detailed explanation of how the `gz_trigger_callback` function operates, the criteria for data transmission in the Timer Callback, and the data flow within the `ApiAgent::start_receive()` function. This should give a comprehensive overview of the robot's operational logic and state management within these components.