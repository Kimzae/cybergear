#!/bin/bash
# CyberGear ROS2 package installer
set -e
PKG=~/ros2_ws/src/cybergear_ros2
if [ ! -f /opt/ros/jazzy/setup.bash ]; then echo "ROS2 Jazzy not installed. Run install_ros2.sh first."; exit 1; fi
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
[ -d cybergear_ros2 ] || ros2 pkg create --build-type ament_python cybergear_ros2 --dependencies rclpy std_msgs sensor_msgs
cat > $PKG/cybergear_ros2/cybergear_protocol.py << 'CG_EOF'
"""
CyberGear Comm. Module 

Structure:  PC --(USB, 921600bps Serial)--> CH340 USB-CAN Adapter --(CAN 1Mbps)--> CyberGear

Adapter Serial Frame:
  'A' 'T' | ID 4byte | Data length 1 byte | Data 0~8bytes | '\r' '\n'
  ID 4 Bytes = (29 bits CAN ID << 3) | 0x04   (0x04 = Expansion frame)

CyberGear 29 BIT CAN ID structure:
  [28:24] Comm. type | [23:8] Data area (Generally its Host ID or Torque) | [7:0] motor ID
"""
import math
import struct

import serial

# ---- Comm. type ----
TYPE_MOTION = 1        # Motion Control (Torque + Position + Velocity + Kp + Kd)
TYPE_FEEDBACK = 2      # Motor -> PC status feedback
TYPE_ENABLE = 3        # Enable motor
TYPE_STOP = 4          # Disable motor (data[0]=1 means No error)
TYPE_SET_ZERO = 6      # Go to zero position
TYPE_WRITE_PARAM = 18  # write parameter

# ---- Param. Index ----
PARAM_RUN_MODE = 0x7005   # 0:Motion control 1:Position 2:Velocity 3:Current
PARAM_IQ_REF = 0x7006     # Current mode goal value [A]
PARAM_LIMIT_TORQUE = 0x700B

# ---- value range (According to manual) ----
P_MIN, P_MAX = -4 * math.pi, 4 * math.pi   # position [rad]
V_MIN, V_MAX = -30.0, 30.0                 # velocity [rad/s]
T_MIN, T_MAX = -12.0, 12.0                 # torque [Nm]
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0


def float_to_uint16(x, x_min, x_max):
    """real number -> 0~65535 integer (Cut it if its more than given range)"""
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * 65535.0 / (x_max - x_min))


def uint16_to_float(u, x_min, x_max):
    """0~65535 integer -> real number"""
    return u * (x_max - x_min) / 65535.0 + x_min


class CyberGear:
    def __init__(self, port='/dev/ttyUSB0', baudrate=921600,
                 motor_id=127, host_id=0xFD):
        self.motor_id = motor_id
        self.host_id = host_id
        # timeout=0 : does not wait when reading (non-blocking)
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self._rx = bytearray()
        # Reset command that sets an adapter to AT mode 
        self.ser.write(b'AT+AT\r\n')

    # ------------------------------------------------------------
    # SEND
    # ------------------------------------------------------------
    def _send(self, comm_type, data16, data=b'\x00' * 8):
        can_id = ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (self.motor_id & 0xFF)
        frame = (b'AT'
                 + struct.pack('>I', (can_id << 3) | 0x04)
                 + bytes([len(data)])
                 + bytes(data)
                 + b'\r\n')
        self.ser.write(frame)

    def enable(self):
        self._send(TYPE_ENABLE, self.host_id)

    def stop(self, clear_fault=False):
        data = bytearray(8)
        data[0] = 1 if clear_fault else 0
        self._send(TYPE_STOP, self.host_id, data)

    def set_zero(self):
        data = bytearray(8)
        data[0] = 1
        self._send(TYPE_SET_ZERO, self.host_id, data)

    def write_param_float(self, index, value):
        data = struct.pack('<HH', index, 0) + struct.pack('<f', float(value))
        self._send(TYPE_WRITE_PARAM, self.host_id, data)

    def write_param_u8(self, index, value):
        data = struct.pack('<HH', index, 0) + bytes([value & 0xFF, 0, 0, 0])
        self._send(TYPE_WRITE_PARAM, self.host_id, data)

    def set_run_mode(self, mode):
        """Mode change must done when the current status is stop()"""
        self.write_param_u8(PARAM_RUN_MODE, mode)

    def motion_control(self, torque, position=0.0, velocity=0.0, kp=0.0, kd=0.0):
        """
        Motion control command. Actual motor output torque:
            T = kp*(position - current position) + kd*(velocity - current velocity) + torque
        if kp=0, kd=0 -> PURE TORQUE CONTORL!
        """
        data16 = float_to_uint16(torque, T_MIN, T_MAX)   # Torque goes into ID 
        data = struct.pack('>HHHH',
                           float_to_uint16(position, P_MIN, P_MAX),
                           float_to_uint16(velocity, V_MIN, V_MAX),
                           float_to_uint16(kp, KP_MIN, KP_MAX),
                           float_to_uint16(kd, KD_MIN, KD_MAX))
        self._send(TYPE_MOTION, data16, data)

    # ------------------------------------------------------------
    # Receive
    # ------------------------------------------------------------
    def poll(self):
        """Read Serial Buffer, return feedback(dict) list"""
        n = self.ser.in_waiting
        if n:
            self._rx += self.ser.read(n)
        results = []
        while True:
            start = self._rx.find(b'AT')
            if start < 0:
                self._rx.clear()      # if there is no 'AT', then it is garbage, dispose it.
                break
            del self._rx[:start]
            if len(self._rx) < 7:     # 'AT' + ID4 + length 1
                break
            dlc = self._rx[6]
            total = 7 + dlc + 2
            if dlc > 8:               # Wrong frame -> dispose one byte and search again
                del self._rx[:1]
                continue
            if len(self._rx) < total:
                break                 # Still receiving
            frame = bytes(self._rx[:total])
            del self._rx[:total]
            if frame[-2:] != b'\r\n':
                continue
            can_id = struct.unpack('>I', frame[2:6])[0] >> 3
            fb = self._parse(can_id, frame[7:7 + dlc])
            if fb is not None:
                results.append(fb)
        return results

    @staticmethod
    def _parse(can_id, data):
        comm_type = (can_id >> 24) & 0x1F
        if comm_type != TYPE_FEEDBACK or len(data) < 8:
            return None
        pos_u, vel_u, tor_u, temp_u = struct.unpack('>HHHH', data)
        return {
            'motor_id': (can_id >> 8) & 0xFF,
            'fault': (can_id >> 16) & 0x3F,   # if its not '0', then it's error
            'mode': (can_id >> 22) & 0x03,    # 0:reset 1:in calibration 2:in operation
            'position': uint16_to_float(pos_u, P_MIN, P_MAX),
            'velocity': uint16_to_float(vel_u, V_MIN, V_MAX),
            'torque': uint16_to_float(tor_u, T_MIN, T_MAX),
            'temperature': temp_u / 10.0,
        }

    def close(self):
        self.ser.close()
CG_EOF
cat > $PKG/cybergear_ros2/cybergear_test.py << 'CG_EOF'
"""
ROS2 없이 모터와 통신이 되는지 확인하는 테스트.
0.2 Nm 토크를 2초 동안 주고, 피드백을 출력한 뒤 정지합니다.

실행:  ros2 run cybergear_ros2 cybergear_test --port /dev/ttyUSB0 --torque 0.2
"""
import argparse
import time

from cybergear_ros2.cybergear_protocol import CyberGear


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--id', type=int, default=127, help='모터 CAN ID (공장 기본값 127)')
    ap.add_argument('--torque', type=float, default=0.2, help='테스트 토크 [Nm]')
    ap.add_argument('--seconds', type=float, default=2.0)
    args = ap.parse_args()

    motor = CyberGear(args.port, args.baud, motor_id=args.id)
    time.sleep(0.1)
    motor.poll()                      # 초기화 응답 버리기

    print('1) 정지 + 에러 해제')
    motor.stop(clear_fault=True)
    time.sleep(0.05)
    print('2) 운동 제어 모드(run_mode=0) 설정')
    motor.set_run_mode(0)
    time.sleep(0.05)
    print('3) 모터 활성화')
    motor.enable()
    time.sleep(0.05)

    got_feedback = False
    print(f'4) {args.torque} Nm 토크를 {args.seconds}초 동안 인가')
    t_end = time.time() + args.seconds
    last_print = 0.0
    try:
        while time.time() < t_end:
            motor.motion_control(torque=args.torque)   # kp=kd=0 -> 순수 토크
            for fb in motor.poll():
                got_feedback = True
                if time.time() - last_print > 0.2:
                    last_print = time.time()
                    print(f"   pos={fb['position']:+.3f} rad  vel={fb['velocity']:+.3f} rad/s  "
                          f"torque={fb['torque']:+.3f} Nm  temp={fb['temperature']:.1f}C  "
                          f"fault={fb['fault']}")
            time.sleep(0.01)                            # 100 Hz
    finally:
        print('5) 토크 0 -> 정지')
        motor.motion_control(torque=0.0)
        time.sleep(0.02)
        motor.stop()
        motor.close()

    if got_feedback:
        print('성공! 모터에서 피드백을 받았습니다.')
    else:
        print('피드백이 없습니다. 전원/배선/모터 ID/포트/baudrate를 확인하세요.')


if __name__ == '__main__':
    main()
CG_EOF
cat > $PKG/cybergear_ros2/torque_node.py << 'CG_EOF'
"""
CyberGear 토크 제어 ROS2 노드

구독(Subscribe):  /cybergear/torque_cmd   (std_msgs/Float64)  목표 토크 [Nm]
발행(Publish):    /cybergear/joint_states (sensor_msgs/JointState) 위치/속도/토크
                  /cybergear/temperature  (std_msgs/Float64)  모터 온도 [C]

안전 기능:
  - max_torque   : 명령 토크를 이 값 이하로 제한
  - cmd_timeout  : 이 시간 동안 명령이 안 오면 토크 0
  - max_velocity : 속도가 이 값을 넘으면 토크 0 (헛돌며 폭주 방지)
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from cybergear_ros2.cybergear_protocol import CyberGear


class CyberGearTorqueNode(Node):
    def __init__(self):
        super().__init__('cybergear_torque_node')

        # ---- 파라미터 (실행할 때 바꿀 수 있는 설정값) ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('motor_id', 127)
        self.declare_parameter('host_id', 253)
        self.declare_parameter('max_torque', 1.0)      # [Nm] 처음엔 작게!
        self.declare_parameter('max_velocity', 10.0)   # [rad/s]
        self.declare_parameter('kd', 0.0)              # 댐핑 (0 = 순수 토크)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('cmd_timeout', 0.5)     # [s]

        p = lambda name: self.get_parameter(name).value
        self.max_torque = float(p('max_torque'))
        self.max_velocity = float(p('max_velocity'))
        self.kd = float(p('kd'))
        self.cmd_timeout = float(p('cmd_timeout'))

        # ---- 모터 연결 & 초기화 ----
        self.motor = CyberGear(p('port'), int(p('baudrate')),
                               motor_id=int(p('motor_id')), host_id=int(p('host_id')))
        time.sleep(0.1)
        self.motor.poll()
        self.motor.stop(clear_fault=True)
        time.sleep(0.05)
        self.motor.set_run_mode(0)      # 운동 제어 모드
        time.sleep(0.05)
        self.motor.enable()
        self.get_logger().info(f"CyberGear 활성화 (port={p('port')}, id={p('motor_id')}, "
                               f"max_torque={self.max_torque} Nm)")

        # ---- 상태 변수 ----
        self.cmd_torque = 0.0
        self.last_cmd_time = 0.0
        self.last_velocity = 0.0
        self.last_fb_time = time.time()

        # ---- ROS 통신 ----
        self.create_subscription(Float64, 'cybergear/torque_cmd', self.on_cmd, 10)
        self.js_pub = self.create_publisher(JointState, 'cybergear/joint_states', 10)
        self.temp_pub = self.create_publisher(Float64, 'cybergear/temperature', 10)
        self.create_timer(1.0 / float(p('rate_hz')), self.loop)

    def on_cmd(self, msg):
        t = max(-self.max_torque, min(self.max_torque, msg.data))
        if t != msg.data:
            self.get_logger().warn(f'명령 {msg.data:.2f} Nm -> {t:.2f} Nm 으로 제한',
                                   throttle_duration_sec=1.0)
        self.cmd_torque = t
        self.last_cmd_time = time.time()

    def loop(self):
        now = time.time()
        torque = self.cmd_torque

        if now - self.last_cmd_time > self.cmd_timeout:
            torque = 0.0                             # 명령 끊김 -> 0
        if abs(self.last_velocity) > self.max_velocity:
            torque = 0.0                             # 과속 -> 0
            self.get_logger().warn('속도 제한 초과! 토크 0', throttle_duration_sec=1.0)

        self.motor.motion_control(torque=torque, kd=self.kd)

        for fb in self.motor.poll():
            self.last_fb_time = now
            self.last_velocity = fb['velocity']
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['cybergear']
            js.position = [fb['position']]
            js.velocity = [fb['velocity']]
            js.effort = [fb['torque']]
            self.js_pub.publish(js)
            self.temp_pub.publish(Float64(data=fb['temperature']))
            if fb['fault']:
                self.get_logger().error(f"모터 에러 코드: {fb['fault']:#04x}",
                                        throttle_duration_sec=1.0)

        if now - self.last_fb_time > 1.0:
            self.get_logger().warn('모터 피드백이 1초 이상 없음 (전원/배선 확인)',
                                   throttle_duration_sec=2.0)

    def shutdown_motor(self):
        try:
            self.motor.motion_control(torque=0.0)
            time.sleep(0.02)
            self.motor.stop()
            self.motor.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = CyberGearTorqueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_motor()           # Ctrl+C 해도 모터는 안전하게 정지
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
CG_EOF
cat > $PKG/setup.py << 'CG_EOF'
from setuptools import find_packages, setup

package_name = 'cybergear_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zae',
    maintainer_email='zae@todo.todo',
    description='Xiaomi CyberGear torque control with ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'torque_node = cybergear_ros2.torque_node:main',
            'cybergear_test = cybergear_ros2.cybergear_test:main',
        ],
    },
)
CG_EOF
cd ~/ros2_ws && colcon build --symlink-install
grep -q "ros2_ws/install/setup.bash" ~/.bashrc || echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo; echo "DONE! Open a new terminal and run: ros2 run cybergear_ros2 cybergear_test --torque 0.2"
