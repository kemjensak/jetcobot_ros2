# Picker Action GUI

이 GUI 프로그램은 TagPicker Action Server와 통신하여 pick-and-place 작업을 제어할 수 있는 사용자 인터페이스를 제공합니다.

## 기능

- **다양한 명령 지원**: HOME, SCAN, SCAN_FRONT, SCAN_LEFT, SCAN_RIGHT, PICK_AND_PLACE
- **실시간 피드백**: 작업 진행 상황을 실시간으로 모니터링
- **결과 표시**: 작업 성공/실패와 오류 메시지 표시
- **사용자 입력**: Source Tag ID, Target Tag ID, Target TF Frame 설정 가능
- **로그 기능**: 모든 작업의 타임스탬프가 포함된 로그

## 사용법

### 1. 기본 실행

```bash
# 터미널에서 직접 실행
cd /home/jetcobot/colcon_ws
source install/setup.bash
ros2 run jetcobot_moveit_picker picker_gui.py

# 또는 런치 파일로 실행
ros2 launch jetcobot_moveit_picker picker_gui.launch.py
```

### 2. GUI 사용 방법

#### 기본 명령어 (SCAN, HOME 등)
1. **Command** 드롭다운에서 원하는 명령 선택
2. **🚀 Send Goal** 버튼 클릭
3. 오른쪽 패널에서 실시간 피드백 확인

#### PICK_AND_PLACE 명령
1. **Command**를 "PICK_AND_PLACE"로 선택
2. **Source Tag ID**: 집을 태그의 ID 입력 (필수)
3. **Target Tag ID** 또는 **Target TF Frame** 중 하나 입력:
   - **Target Tag ID**: 놓을 위치의 태그 ID
   - **Target TF Frame**: TF 프레임 이름 (예: "target_pose_1")
4. **🚀 Send Goal** 버튼 클릭

### 3. 인터페이스 설명

#### 왼쪽 패널 - Goal Configuration
- **Command**: 실행할 명령 선택
- **Source Tag ID**: 집을 객체의 태그 ID
- **Target Tag ID**: 놓을 위치의 태그 ID
- **Target TF Frame**: 놓을 위치의 TF 프레임 이름
- **🚀 Send Goal**: 목표 전송
- **🗑️ Clear**: 입력 필드 초기화

#### 오른쪽 패널 - Status & Feedback
- **Current Status**: 현재 작업 단계와 진행률
- **Feedback & Results**: 실시간 피드백과 최종 결과
- **Logs**: 타임스탬프가 포함된 모든 작업 로그

### 4. 상태 메시지

- 🔍 **Searching**: 태그 탐색 중
- ➡️ **Approaching source**: 소스 태그로 접근 중
- 🤏 **Picking**: 객체 집기 중
- 🚚 **Moving to target**: 목표 위치로 이동 중
- 📦 **Placing**: 객체 놓기 중
- ✅ **Completed**: 작업 완료
- 🏠 **Moving to home**: 홈 위치로 이동 중

## 예제 사용 시나리오

### 시나리오 1: 태그 스캔
1. Command: "SCAN"
2. Send Goal 클릭
3. 로봇이 현재 위치에서 태그를 스캔

### 시나리오 2: Pick and Place (태그 ID 사용)
1. Command: "PICK_AND_PLACE"
2. Source Tag ID: "1"
3. Target Tag ID: "2"
4. Send Goal 클릭
5. 로봇이 태그 1의 객체를 집어서 태그 2 위치에 놓음

### 시나리오 3: Pick and Place (TF 프레임 사용)
1. Command: "PICK_AND_PLACE"
2. Source Tag ID: "1"
3. Target TF Frame: "target_location_1"
4. Send Goal 클릭
5. 로봇이 태그 1의 객체를 집어서 지정된 TF 프레임 위치에 놓음

## 트러블슈팅

### GUI가 시작되지 않는 경우
```bash
# PyQt5 설치 확인
python3 -c "import PyQt5; print('PyQt5 OK')"

# 필요시 PyQt5 설치
sudo apt install python3-pyqt5
```

### Action Server 연결 실패
```bash
# TagPicker 노드가 실행 중인지 확인
ros2 node list | grep tag_picker

# Action Server 상태 확인
ros2 action list | grep picker_action
```

### 권한 문제
```bash
# 스크립트 실행 권한 확인
chmod +x /home/jetcobot/colcon_ws/src/jetcobot/jetcobot_moveit_picker/scripts/picker_gui.py
```

## 시스템 요구사항

- ROS2 Jazzy
- Python 3.8+
- PyQt5
- jetcobot_interfaces 패키지
- TagPicker Action Server 실행 중

## 주의사항

- GUI 실행 전에 TagPicker 노드가 먼저 실행되어 있어야 합니다
- PICK_AND_PLACE 명령 시 Source Tag ID는 필수입니다
- Target Tag ID와 Target TF Frame 중 하나는 반드시 입력해야 합니다
- 로봇이 안전한 상태에서만 명령을 실행하세요
