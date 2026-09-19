#!/usr/bin/env python3
# CyberGear diagnostic scanner (pyserial only, no ROS needed)
# Usage: python3 cybergear_scan.py [port] [baud]
# Step 1: shows ANY raw bytes coming from the USB-CAN adapter
# Step 2: asks every motor ID 1..127 "who are you?" (type 0, motor does NOT move)
import struct, sys, time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
HOST = 0xFD

ser = serial.Serial(port, baud, timeout=0)
print(f'Opened {port} @ {baud}')

def send(comm_type, data16, motor_id, data=b'\x00' * 8):
    can_id = (comm_type << 24) | (data16 << 8) | motor_id
    ser.write(b'AT' + struct.pack('>I', (can_id << 3) | 4) + bytes([len(data)]) + data + b'\r\n')

def read_all(wait):
    time.sleep(wait)
    return ser.read(ser.in_waiting or 1)

# ---- Step 1: adapter check ----
ser.write(b'AT+AT\r\n')
raw = read_all(0.3)
print('[1] Adapter reply to AT+AT :', raw.hex(' ') if raw else '(nothing)', repr(raw) if raw else '')

# ---- Step 2: scan IDs ----
print('[2] Scanning motor IDs 1..127 ...')
buf = bytearray()
found = set()
for mid in range(1, 128):
    send(0, HOST, mid)
    time.sleep(0.01)
    buf += ser.read(ser.in_waiting or 1)
time.sleep(0.3)
buf += ser.read(ser.in_waiting or 1)

i = 0
while True:
    i = buf.find(b'AT', i)
    if i < 0 or i + 7 > len(buf):
        break
    cid = struct.unpack('>I', bytes(buf[i + 2:i + 6]))[0] >> 3
    ctype = (cid >> 24) & 0x1F
    mid = (cid >> 8) & 0xFF
    print(f'    frame: type={ctype} can_id=0x{cid:08X} -> motor ID = {mid}')
    found.add(mid)
    i += 7

if found:
    print(f'FOUND motor ID(s): {sorted(found)}  -> use --id {sorted(found)[0]}')
elif buf:
    print('Got bytes but no valid frame. Raw:', bytes(buf[:64]).hex(' '))
    print('-> baudrate may be wrong. Try: python3 cybergear_scan.py', port, '115200')
else:
    print('NO reply at all.')
    print('-> check: 24V power ON? CAN_H/CAN_L swapped? 120ohm termination? right port?')
ser.close()
