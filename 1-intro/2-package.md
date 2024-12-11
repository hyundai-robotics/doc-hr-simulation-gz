# 2. 패키지

| 패키지 이름     | 설명 |
|-------------------|--------------------------------------------------------------------------------------------------|
| hdr_description   | 로봇 모델에 대한 다양한 구성 파일과 시각적/물리적 매개변수 파일을 퐇마하여 HR로봇에 대한 파일이 들어 있는 패키지 입니다. |
| hdr_moveit_config | HDR Moveit 구성 파일이 들어 있는 패키지 입니다.                                                         |
| hdr_simulation    | HDR 시뮬레이션 패키지의 주요 폴더로, 시뮬레이션 제어 및 관련 구성 패키지 입니다.                                |


# ROS2 패키지 구조, Launch 파일 및 Parameters

## hdr_description

### 폴더

- **config**: 각 로봇 모델에 대한 물리적 매개변수 정보를 포함합니다.
- **meshes**: 각 로봇 모델의 외부 모양을 나타내는 STL 파일이 포함되어 있습니다.
- **rviz**: Rviz 구성 파일이 포함되어 있습니다.
- **urdf**: 로봇 설명을 위한 URDF 파일이 포함되어 있습니다.

### Launch Files

- **view_hdr.launch.py**: Rviz 에서 HR 로봇 모델 시각화를 실행합니다.

| 매개변수 | 설명 |
|-----------|-------------|
| hdr_type  | HDR 로봇의 종류/시리즈 |
| safety_limits | true로 설정 할 경우 safety limits 컨트롤러를 활성화합니다. |
| safety_pos_margin | 안전 컨트롤러에서 상한선과 하한선까지의 여유 범위|
| safety_k_position | 안전 컨트롤러의 k-position 계수|
| description_package | 로봇 URDF/XACRO 파일이 포함된 설명 패키지. 일반적으로 이 인수는 설정하지 않으며, 사용자 지정 설명을 사용할 수 있도록 합니다. |
| description_file | 로봇이 포함된 URDF/XACRO 설명 파일 |
| tf_prefix | 다중 로봇 설정. 변경된 경우 컨트롤러 구성의 조인트 이름을 업데이트해야 합니다. |

## hdr_moveit_config

### 폴더

- **config**: Moveit 구성 파일이 포함되어 있습니다.
- **rviz**: Rviz 구성 파일이 포함되어 있습니다.
- **srdf**: Semantic Robot Description Format(SRDF) 파일을 포함합니다.

### Launch Files

- **hdr_moveit.launch.py**: HDR 로봇에 대한 Moveit 구성을 실행합니다.

| 매개변수 | 설명 |
|-----------|-------------|
| hdr_type  | HDR 로봇의 종류/시리즈 |
| use_fake_hardware | 로봇이 명령을 상태로 반영하는 가상 하드웨어로 실행 중인 여부를 나타냅니다. |
| safety_limits | true로 설정 할 경우 safety limits 컨트롤러를 활성화합니다. |
| safety_pos_margin | 전 컨트롤러에서 상한선과 하한선까지의 여유 범위 |
| safety_k_position | 안전 컨트롤러의 k-position 계수 |
| description_package | 로봇 URDF/XACRO 파일이 포함된 설명 패키지. 일반적으로 이 인수는 설정하지 않으며, 사용자 지정 설명을 사용할 수 있도록 합니다. |
| description_file | 로봇이 포함된 URDF/XACRO 설명 파일 |
| moveit_config_package | 로봇 SRDF/XACRO 파일이 포함된 Moveit 설정 패키지. 일반적으로 이 인수는 설정하지 않으며, 사용자 지정 Moveit 설정을 사용할 수 있도록 합니다. |
| moveit_config_file | 로봇의 Moveit SRDF/XACRO 설명 파일 |
| moveit_joint_limits_file | URDF의 robot_description에서 제공된 값을 보완하거나 재정의하는 Moveit 관절 제한 |
| warehouse_sqlite_path | warehouse 데이터베이스가 저장될 경로 |
| use_sim_time | Moveit에서 시뮬레이션 시간을 사용하도록 설정합니다. 이는 시뮬레이션에서 경로 계획에 필요합니다. |
| prefix | 다중로봇 설정 변경시 컨트롤러 구성의 관절 이름을 업데이트해야 합니다. |

## hdr_simulation

### 폴더

- **config**: HR 컨트롤러와 관련된 구성 파일을 포함합니다.

### Launch Files

- **hdr_sim_control.launch.py**: HR 로봇 시뮬레이션을 기본 제어와 함께 실행합니다.

| 매개변수 | 설명 |
|-----------|-------------|
| hdr_type  | HDR 로봇의 종류/시리즈 |
| safety_limits | true로 설정 할 경우 safety limits 컨트롤러를 활성화합니다. |
| safety_pos_margin | 전 컨트롤러에서 상한선과 하한선까지의 여유 범위 |
| safety_k_position | 안전 컨트롤러의 k-position 계수 |
| runtime_config_package | "config" 폴더에 컨트롤러 구성이 있는 패키지. 일반적으로 인수로 설정되지 않으며 사용자 지정 설정을 사용할 수 있습니다. |
| controllers_file | 컨트롤러 구성이 포함된 YAML 파일. |
| description_package | 로봇 URDF/XACRO 파일이 포함된 설명 패키지. 일반적으로 이 인수는 설정하지 않으며, 사용자 지정 설명을 사용할 수 있도록 합니다. |
| description_file | 로봇이 포함된 URDF/XACRO 설명 파일 |
| prefix |다중로봇 설정 변경시 컨트롤러 구성의 관절 이름을 업데이트해야 합니다. |
| start_joint_controller | 로봇 제어를 위한 headless 모드를 활성화합니다. |
| initial_joint_controller | 시작할 로봇 컨트롤러를 설정합니다. |

- **hdr_sim_moveit.launch.py**: HR로봇 시뮬레이션을 Moveit통합과 함께 실행합니다.

| 매개변수 | 설명 |
|-----------|-------------|
| hdr_type  | HDR 로봇의 종류/시리즈 |
| use_fake_hardware | 로봇이 명령을 상태로 반영하는 가상 하드웨어로 실행 중인 여부를 나타냅니다. |
| safety_limits | true로 설정 할 경우 safety limits 컨트롤러를 활성화합니다. |
| runtime_config_package | "config" 폴더에 컨트롤러 구성이 있는 패키지. 일반적으로 인수로 설정되지 않으며 사용자 지정 설정을 사용할 수 있습니다. |
| controllers_file | 컨트롤러 구성이 포함된 YAML 파일. |
| description_package | 로봇 URDF/XACRO 파일이 포함된 설명 패키지. 일반적으로 이 인수는 설정하지 않으며, 사용자 지정 설명을 사용할 수 있도록 합니다. |
| description_file | 로봇이 포함된 URDF/XACRO 설명 파일 |
| moveit_config_package | 로봇 SRDF/XACRO 파일이 포함된 Moveit 설정 패키지. 일반적으로 이 인수는 설정하지 않으며, 사용자 지정 Moveit 설정을 사용할 수 있도록 합니다. |
| moveit_config_file | 로봇의 Moveit SRDF/XACRO 설명 파일 |
| prefix | 다중로봇 설정 변경시 컨트롤러 구성의 관절 이름을 업데이트해야 합니다. |
