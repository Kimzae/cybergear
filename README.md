# CyberGear ROS2 Torque Control

Xiaomi CyberGear 모터를 **Linux(Ubuntu 24.04) + ROS2 Jazzy** 환경에서 **토크 제어**하는 예제입니다.
리눅스와 ROS2를 처음 접하는 사람도 순서대로 따라 하면 모터를 돌릴 수 있도록 작성했습니다.

- 테스트 환경: LattePanda Delta, Ubuntu 24.04.5, ROS2 Jazzy
- 통신: CyberGear 전용 USB-CAN 어댑터 (CH340)

---

## 목차
1. [시스템 구조](#1-시스템-구조)
2. [준비물](#2-준비물)
3. [안전 주의사항](#3-안전-주의사항)
4. [설치](#4-설치)
5. [모터 ID 찾기](#5-모터-id-찾기)
6. [하드웨어 테스트](#6-하드웨어-테스트)
7. [ROS2 노드로 토크 제어](#7-ros2-노드로-토크-제어)
8. [문제 해결](#8-문제-해결)
9. [동작 원리](#9-동작-원리)

---

## 1. 시스템 구조

```
[Linux PC]  --USB-->  [CH340 USB-CAN 어댑터]  --CAN-->  [CyberGear]
 ROS2 노드       시리얼 921600 bps            CAN 1 Mbps       24V 전원
```

- PC 입장에서 어댑터는 **시리얼 포트**(`/dev/ttyUSB0`)로 보입니다.
- PC가 시리얼로 `AT...` 형식의 프레임을 보내면, 어댑터가 CAN 신호로 바꿔 모터에 전달합니다.
- 시리얼 baudrate는 **921600**, CAN 통신 속도는 **1 Mbps**입니다.

## 2. 준비물

| 항목 | 내용 |
|---|---|
| 모터 | Xiaomi CyberGear |
| 어댑터 | CyberGear USB-CAN 어댑터 (CH340 칩) |
| 전원 | 24V DC 전원 |
| PC | Ubuntu 24.04가 설치된 PC (예: LattePanda Delta) |
| 네트워크 | 설치할 때 인터넷 연결 필요 |

어댑터가 인식되는지 확인하려면:
```bash
lsusb
```
`QinHeng Electronics CH340 serial converter`가 보이면 정상입니다.

## 3. 안전 주의사항

> **반드시 읽고 시작하세요.**

- 모터를 **바이스나 브래킷으로 단단히 고정**하세요.
- 토크 제어에서는 모터에 부하가 없으면 **계속 가속**합니다.
- 처음에는 **0.2 Nm** 정도의 작은 토크로 시작하세요. (최대 12 Nm)
- 노드에는 다음 안전 기능이 들어 있습니다.
  - 최대 토크 제한 (`max_torque`)
  - 명령이 끊기면 토크 0 (`cmd_timeout`)
  - 속도가 너무 빨라지면 토크 0 (`max_velocity`)

---

## 4. 설치

### 4-1. 저장소 받기
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Kimzae/cybergear
```

이 저장소에 들어 있는 파일:

| 파일 | 역할 |
|---|---|
| `install_ros2.sh` | ROS2 Jazzy 설치 + 시리얼 포트 권한 설정 |
| `install_cybergear.sh` | ROS2 패키지 `cybergear_ros2` 생성 + 빌드 |
| `cybergear_scan.py` | 모터 ID를 모를 때 찾아주는 진단 도구 |

> **`.sh` 파일이란?** 터미널 명령어들을 순서대로 적어 둔 "명령어 모음집"입니다.
> `bash 파일이름.sh`로 실행하면 안에 적힌 명령을 위에서부터 차례로 실행합니다. (Windows의 `.bat` 파일과 비슷합니다.)

### 4-2. ROS2 설치 (처음 한 번만)
```bash
bash ~/ros2_ws/src/cybergear/install_ros2.sh
```
- 비밀번호를 물어보면 로그인 비밀번호를 입력하세요. 입력한 글자가 화면에 보이지 않는 것이 정상입니다.
- 인터넷 속도에 따라 20분~1시간 정도 걸립니다.

이 스크립트가 하는 일:
1. ROS2 Jazzy와 개발 도구 설치
2. 파이썬 시리얼 라이브러리(`python3-serial`) 설치
3. CH340 장치를 가로채는 `brltty` 제거
4. 내 계정에 시리얼 포트 사용 권한(`dialout` 그룹) 추가

설치가 끝나면 **반드시 재부팅**하세요. 권한 설정은 재부팅한 뒤에 적용됩니다.
```bash
sudo reboot
```

### 4-3. CyberGear 패키지 설치
```bash
bash ~/ros2_ws/src/cybergear/install_cybergear.sh
```
출력 마지막에 `Summary: 1 package finished`가 나오면 성공입니다.

설치가 끝나면 **새 터미널**을 여세요. 설치 스크립트가 `~/.bashrc`에 설정을 추가해 두었기 때문에, 새 터미널에서는 ROS2와 이 패키지를 바로 쓸 수 있습니다.

설치 후 폴더 구조:
```
~/ros2_ws/
├── src/
│   ├── cybergear/          ← git clone 한 폴더 (설치 스크립트)
│   └── cybergear_ros2/     ← 설치 스크립트가 만든 실제 ROS2 패키지
│       └── cybergear_ros2/
│           ├── cybergear_protocol.py   통신 모듈
│           ├── cybergear_test.py       하드웨어 테스트
│           └── torque_node.py          ROS2 토크 제어 노드
├── build/
├── install/
└── log/
```

---

## 5. 모터 ID 찾기

CyberGear의 공장 기본 ID는 **127**이지만, 이미 다른 값으로 바뀌어 있는 경우가 많습니다.
ID가 맞지 않으면 모터가 명령을 무시하고, `피드백이 없습니다`라는 메시지가 뜹니다.

ID를 모른다면 스캔하세요. 스캔 중에 모터는 **움직이지 않습니다**.
```bash
python3 ~/ros2_ws/src/cybergear/cybergear_scan.py
```
```
FOUND motor ID(s): [1]  -> use --id 1
```

## 6. 하드웨어 테스트

ROS2 노드를 실행하기 전에, 모터와 통신이 되는지 먼저 확인합니다.

체크리스트:
- [ ] 24V 전원 ON
- [ ] CAN_H / CAN_L 연결
- [ ] 모터 고정
- [ ] USB 어댑터 연결

```bash
ros2 run cybergear_ros2 cybergear_test --id 1 --torque 0.2
```

성공하면 이렇게 출력됩니다.
```
4) 0.2 Nm 토크를 2.0초 동안 인가
   pos=+0.512 rad  vel=+2.103 rad/s  torque=+0.198 Nm  temp=31.2C  fault=0
...
성공! 모터에서 피드백을 받았습니다.
```
`--torque -0.2`로 실행하면 반대 방향으로 돕니다.

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | 시리얼 포트 |
| `--baud` | `921600` | 시리얼 속도 |
| `--id` | `127` | 모터 CAN ID |
| `--torque` | `0.2` | 테스트 토크 [Nm] |
| `--seconds` | `2.0` | 토크를 주는 시간 [s] |

---

## 7. ROS2 노드로 토크 제어

### ROS2 기본 용어

| 용어 | 뜻 |
|---|---|
| 워크스페이스 | ROS2 코드를 모아두는 폴더 (`~/ros2_ws`) |
| 패키지 | 기능 단위 묶음 (`cybergear_ros2`) |
| 노드 | 실행 중인 프로그램 하나 |
| 토픽 | 노드끼리 데이터를 주고받는 채널 |

### 7-1. 터미널 1: 노드 실행
```bash
ros2 run cybergear_ros2 torque_node --ros-args -p motor_id:=1 -p max_torque:=0.5
```

`-p 이름:=값` 형식으로 파라미터를 바꿀 수 있습니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 시리얼 포트 |
| `baudrate` | `921600` | 시리얼 속도 |
| `motor_id` | `127` | 모터 CAN ID |
| `host_id` | `253` | PC(호스트) ID |
| `max_torque` | `1.0` | 최대 토크 제한 [Nm] |
| `max_velocity` | `10.0` | 이 속도를 넘으면 토크 0 [rad/s] |
| `kd` | `0.0` | 댐핑 계수 (0이면 순수 토크 제어) |
| `rate_hz` | `100.0` | 제어 주기 [Hz] |
| `cmd_timeout` | `0.5` | 명령이 끊긴 뒤 토크를 0으로 만들 때까지의 시간 [s] |

### 7-2. 터미널 2: 토크 명령 보내기
```bash
ros2 topic pub -r 20 /cybergear/torque_cmd std_msgs/msg/Float64 "{data: 0.3}"
```
- `-r 20`은 초당 20번 반복해서 보낸다는 뜻입니다.
- 명령은 **계속 보내야** 합니다. 0.5초 동안 명령이 없으면 안전을 위해 토크가 0이 됩니다.
- `Ctrl + C`를 누르면 멈춥니다.

### 7-3. 터미널 3: 모터 상태 보기
```bash
ros2 topic echo /cybergear/joint_states
```

### 토픽 목록

| 토픽 | 타입 | 방향 | 내용 |
|---|---|---|---|
| `/cybergear/torque_cmd` | `std_msgs/Float64` | 입력 | 목표 토크 [Nm] |
| `/cybergear/joint_states` | `sensor_msgs/JointState` | 출력 | position [rad], velocity [rad/s], effort [Nm] |
| `/cybergear/temperature` | `std_msgs/Float64` | 출력 | 모터 온도 [°C] |

### 유용한 명령
```bash
ros2 node list                            # 실행 중인 노드 목록
ros2 topic list                           # 토픽 목록
ros2 topic hz /cybergear/joint_states     # 피드백 주기 확인
ros2 run rqt_plot rqt_plot /cybergear/joint_states/velocity[0] /cybergear/joint_states/effort[0]   # 그래프
```

---

## 8. 문제 해결

| 증상 | 원인 | 해결 |
|---|---|---|
| `ls /dev/ttyUSB*`에 아무것도 안 나옴 | `brltty`가 CH340을 가로챔 | `sudo apt remove -y brltty` 후 USB를 다시 꽂기 |
| `Permission denied: '/dev/ttyUSB0'` | 시리얼 권한 없음 | `sudo usermod -aG dialout $USER` 후 **재부팅** |
| `ModuleNotFoundError: serial` | pyserial 미설치 | `sudo apt install python3-serial` |
| `$'\r': command not found` | Windows에서 편집하면서 줄바꿈 형식이 바뀜 | `sed -i 's/\r$//' ~/ros2_ws/src/cybergear/*.sh` |
| `ROS2 Jazzy not installed` | ROS2 미설치 | `install_ros2.sh`를 먼저 실행 |
| `Package 'cybergear_ros2' not found` | 터미널이 워크스페이스를 모름 | `source ~/ros2_ws/install/setup.bash` 또는 새 터미널 열기 |
| 위 방법으로도 패키지를 찾지 못함 | 빌드 실패 또는 잘못된 위치에서 빌드 | 아래 "깨끗하게 다시 빌드" 참고 |
| `피드백이 없습니다` | 모터 ID 불일치 (가장 흔함) | `cybergear_scan.py`로 ID 확인 후 `--id` 지정 |
| 여전히 피드백 없음 | 전원·배선 문제 | 24V 전원, CAN_H/CAN_L 뒤바뀜, 120Ω 종단저항 확인 |
| `fault` 값이 0이 아님 | 과전류·과열·저전압 등 | 전원 전압 확인 후 노드 재시작 (시작할 때 에러를 해제함) |
| 모터가 너무 빨리 돔 | 토크 제어의 정상 동작 | `max_torque`를 낮추거나 `-p kd:=0.1`로 댐핑 추가 |

**깨끗하게 다시 빌드:**
```bash
cd ~/ros2_ws                 # 반드시 워크스페이스 최상위에서!
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```
> `src` 폴더 안에서 `colcon build`를 하면 엉뚱한 위치에 `install` 폴더가 생겨서 패키지를 찾지 못합니다.

> 다른 프로그램(CyberGear 윈도우 툴, 시리얼 모니터 등)이 포트를 쓰고 있으면 연결되지 않습니다. 포트는 한 프로그램만 쓸 수 있습니다.

---

## 9. 동작 원리

### 토크 제어 방법
CyberGear의 "운동 제어 모드(run_mode = 0)"에서 출력 토크는 다음과 같이 계산됩니다.

```
출력 토크 = Kp × (목표위치 − 현재위치) + Kd × (목표속도 − 현재속도) + 토크(feedforward)
```

**Kp = 0, Kd = 0**으로 두면 `출력 토크 = 입력 토크`가 되어 **순수 토크 제어**가 됩니다.

### 어댑터 시리얼 프레임
```
'A' 'T' | ID 4바이트 | 데이터 길이 1바이트 | 데이터 0~8바이트 | '\r' '\n'
ID 4바이트 = (29비트 CAN ID << 3) | 0x04
```

### CyberGear 29비트 CAN ID
```
[28:24] 통신 타입 | [23:8] 데이터 영역 (호스트 ID 또는 토크) | [7:0] 모터 ID
```

| 통신 타입 | 의미 |
|---|---|
| 0 | 장치 ID 요청 (스캔에 사용) |
| 1 | 운동 제어 (토크는 ID 안의 16비트, 위치·속도·Kp·Kd는 데이터 8바이트) |
| 2 | 모터 → PC 피드백 (위치, 속도, 토크, 온도) |
| 3 | 모터 활성화 |
| 4 | 모터 정지 (`data[0] = 1`이면 에러 해제) |
| 6 | 현재 위치를 기계 원점으로 설정 |
| 18 | 파라미터 쓰기 (예: `0x7005` run_mode) |

### 값 변환 범위
실수 값을 아래 범위 안에서 0~65535 정수로 선형 변환해서 보냅니다.

| 값 | 범위 |
|---|---|
| 토크 | −12 ~ 12 Nm |
| 위치 | −4π ~ 4π rad |
| 속도 | −30 ~ 30 rad/s |
| Kp | 0 ~ 500 |
| Kd | 0 ~ 5 |

---

## License
MIT
