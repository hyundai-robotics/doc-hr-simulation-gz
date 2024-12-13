# 실행 파일 실행 가이드

이 가이드는 ROS2 워크스페이스에서 다양한 실행 파일을 실행하는 방법에 대한 지침을 제공합니다.

## 준비

어떤 실행 파일을 실행하기 전에 워크스페이스를 반드시 소스하십시오.

```bash
source ~/ros2_ws/install/setup.bash
```

## 시뮬레이션 실행

### 기본 시뮬레이션 제어

기본 HDR Simulation 컨트롤을 실행하려면 다음 명령어를 입력하세요.

```bash
ros2 launch hdr_simulation hdr_sim_control.launch.py
```

### MoveIt 과 시뮬레이션 로봇 사용

Moveit을 사용하여 시뮬레이션 로봇을 실행하려면 다음 명령어를 사용하십시오.

```bash
ros2 launch hdr_simulation hdr_sim_moveit.launch.py
```

## 시뮬레이션 로봇과 캐리지 사용

시뮬레이션 로봇과 캐리지를 함께 사용하려면 아래 명령어를 입력하세요.

```bash
 ros2 launch hdr_simulation hdr_sim_moveit_conveyor.launch.py 
```

## 로봇 컨틀롤러에 연결

실제 로봇 컨트롤러에 연결하려면 다음 명령어를 실행하세요.

```bash
 ros2 launch hdr_driver_launch ros2_driver.launch.py
```

## 참고 사항

- 실행 전에 필요한 모든 패키지가 설치 및 빌드되었는지 확인하십시오.
- 실행 후 출력된 콘솔 메시지에서 에러 메시지나 추가 지침을 확인하세요.
- 일부 실행 파일은 추가 인수나 설정이 필요할 수 있습니다. 세부 사항은 해당 패키지의 문서를 참조하십시오.