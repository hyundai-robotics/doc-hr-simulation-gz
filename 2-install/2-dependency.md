# ROS2 작업 공간 설정 가이드

## 작업 공간 구성 및 빌드

### 1. Git 패지키 복제

ROS2 작업 공간의 `src` 디렉토리로 이동하여 필요한 패키지를 복제합니다:

```bash
cd ~/ros2_ws/src
git clone https://github.com/hyundai-robotics/hdr_simulation_gz.git
```

프로젝트에 해당하는 실제 저장소 URL로 URL을 교체하세요.

### 2. 의존성 패키지 설치

#### 2.1 일반 의존성 설치

필요한 저장소를 다운로드하고 패키지 의존성을 설치합니다.:

```bash
cd ~/ros2_ws/
rosdep update && rosdep install --ignore-src --from-paths src -y
```

이 명령은 `rosdep`을 업데이트 하고, `src`폴더에 있는 모든 패키지의 의존성을 자동으로 설치합니다.

#### 2.2 프로젝트별 의존성 설치

프로젝트의 의존성 설치 스크립트를 실행합니다.:

```bash
cd ~/ros2_ws/src/hdr_simulation_gz
bash ros2-install-deps.sh
```

이 스크립트는 프로젝트에 필요한 추가 의존성을 설치합니다.

### 3. 작업 공간 구성 및 설치

마지막으로, 시뮬레이션을 사용하려면 아래 명을 실행하여 작업 공간을 빌드해야 합니다.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

이 `colcon build` 명령은 작업 공간의 모든 패키지를 빌드하고 심볼릭 링크를 사용하여 설치합니다.
이를 통해 소스 파일의 변경 사항이 즉시 반영되며, 재빌드가 필요하지 않습니다.

빌드가 완료된 후, 새로운 터미널을 열거나 현재 터미널에서 아래 명령을 실행하여 업데이트된 환경을 적용하세요:

```bash
source ~/ros2_ws/install/setup.bash
```

이 과정은 다음을 포함합니다.
1. `hdr_simulation_gz` 디렉토리로 이동
2. `install` 디렉토리를 `ros2_ws` 루트로 복사
3. `setup.bash` 파일을 소싱하여 새로 설치된 패키지를 현재 셀 세션에서 사용할 수 있도록 설정.

## 결론

이 단계를 완료하면 작업 공간이 완전히 구성되고 빌드됩니다. 이제 ROS2 프로젝트를 실행할 준비가 되었습니다.
새로운 터미널에서 이 ROS2 작업 공간을 사용하려면 항상 설정 파일을 소싱해야 합니다.

```bash
source ~/ros2_ws/install/setup.bash
```