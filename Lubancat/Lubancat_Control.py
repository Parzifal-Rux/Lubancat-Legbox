#!/usr/bin/env python3
"""
鲁班猫4 程序：MPU6050 姿态检测 + 目标角度控制 → ESP32
- I2C:    /dev/i2c-6, 地址 0x69
- 串口:   /dev/ttyACM0, 115200 bps
- 帧格式 (25 字节):
    [0xAA][0x55]
    [Roll:f32][Pitch:f32][Yaw:f32]          ← 传感器姿态
    [TgtRoll:f32][TgtPitch:f32][TgtYaw:f32] ← 目标角度 (由鲁班猫设定)
    [Cmd:u8]                                 ← 命令字
    [xor:u8]                                 ← 校验 (2~22)
    [0x0D][0x0A]                             ← 帧尾
- 频率:   70 Hz

Cmd 命令字:
    0x00 = 停止 (电机停机, PID 输出 0)
    0x01 = 平衡 (PID 闭环, 目标由 TgtPitch 等决定)
    0x02 = 走路 (步态模式, 鲁班猫周期性改 TgtPitch)

依赖:
    sudo pip3 install smbus2 pyserial

运行:
    sudo python3 Lubancat_Control.py
"""

import math
import time
import struct
import threading
from smbus2 import SMBus
import serial

# ========== 配置 ==========
I2C_BUS      = 6
MPU_ADDR     = 0x69
SERIAL_PORT  = "/dev/ttyACM0"
BAUDRATE     = 115200
LOOP_HZ      = 70
CALIB_SAMPLES = 500

# MPU6050 寄存器
PWR_MGMT_1   = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG  = 0x1B
ACCEL_XOUT   = 0x3B
GYRO_XOUT    = 0x43

# 量程 LSB
ACCEL_SENS = 8192.0
GYRO_SENS  = 16.4

# 帧头/帧尾
FRAME_HEAD = b"\xAA\x55"
FRAME_TAIL = b"\x0D\x0A"
FRAME_LEN  = 30   # 2+12+12+1+1+2 = 30 字节

# 命令字
CMD_STOP   = 0x00
CMD_BALANCE = 0x01
CMD_WALK   = 0x02

# ========== 控制参数 ==========
# 这些可以通过键盘交互修改
target_roll  = 0.0
target_pitch = 0.0
target_yaw   = 0.0
current_cmd  = CMD_BALANCE
cmd_names    = {CMD_STOP: "停止", CMD_BALANCE: "平衡", CMD_WALK: "走路"}

# 步态参数 (走路模式)
walk_speed     = 0.3   # 步态速度 (Hz)
walk_amplitude = 8.0   # 俯仰角摆幅 (度)


class MPU6050:
    """MPU6050 姿态读取（互补滤波）"""

    def __init__(self, bus=I2C_BUS, addr=MPU_ADDR):
        self.bus = SMBus(bus)
        self.addr = addr
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.98
        self.last_time = None

    @staticmethod
    def _word(hi, lo):
        val = (hi << 8) | lo
        if val >= 0x8000:
            val -= 0x10000
        return val

    def _write_byte(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def init(self):
        self._write_byte(PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self._write_byte(GYRO_CONFIG, 0x18)
        self._write_byte(ACCEL_CONFIG, 0x08)
        time.sleep(0.1)

    def read_accel(self):
        data = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT, 6)
        return (self._word(data[0], data[1]) / ACCEL_SENS,
                self._word(data[2], data[3]) / ACCEL_SENS,
                self._word(data[4], data[5]) / ACCEL_SENS)

    def read_gyro_raw(self):
        data = self.bus.read_i2c_block_data(self.addr, GYRO_XOUT, 6)
        return (self._word(data[0], data[1]) / GYRO_SENS,
                self._word(data[2], data[3]) / GYRO_SENS,
                self._word(data[4], data[5]) / GYRO_SENS)

    def read_gyro(self):
        gx, gy, gz = self.read_gyro_raw()
        return (gx - self.gyro_offset[0],
                gy - self.gyro_offset[1],
                gz - self.gyro_offset[2])

    def calibrate_gyro(self):
        print(f"校准中... 请保持 MPU6050 静止 {CALIB_SAMPLES} 次")
        gx_sum = gy_sum = gz_sum = 0.0
        for _ in range(CALIB_SAMPLES):
            gx, gy, gz = self.read_gyro_raw()
            gx_sum += gx; gy_sum += gy; gz_sum += gz
            time.sleep(0.002)
        self.gyro_offset = [gx_sum / CALIB_SAMPLES,
                            gy_sum / CALIB_SAMPLES,
                            gz_sum / CALIB_SAMPLES]
        print(f"校准完成: offset=({self.gyro_offset[0]:.2f}, "
              f"{self.gyro_offset[1]:.2f}, {self.gyro_offset[2]:.2f})")

    def _get_dt(self):
        now = time.monotonic()
        if self.last_time is None:
            self.last_time = now
            return 0.0
        dt = now - self.last_time
        self.last_time = now
        return dt

    def update(self):
        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()

        acc_roll  = math.atan2(ay, az) * 180.0 / math.pi
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

        dt = self._get_dt()
        self.roll  = self.alpha * (self.roll  + gx * dt) + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * acc_pitch
        self.yaw  += gz * dt


def build_frame(roll, pitch, yaw, tgt_roll, tgt_pitch, tgt_yaw, cmd):
    """构造 25 字节二进制帧"""
    payload = struct.pack("<ffffffB",
                          roll, pitch, yaw,
                          tgt_roll, tgt_pitch, tgt_yaw,
                          cmd)
    # payload[0:22] = 6*4+1 = 25? 不对: 6*4=24 + 1cmd = 25? 不对
    # 帧: 头(2) + 姿态(12) + 目标(12) + cmd(1) + xor(1) + 尾(2) = 30?
    # 重新算:
    # [0xAA][0x55] = 2
    # Roll,Pitch,Yaw = 12
    # TgtRoll,TgtPitch,TgtYaw = 12
    # Cmd = 1
    # Xor = 1
    # [0x0D][0x0A] = 2
    # 总计 = 30 字节
    payload = struct.pack("<ffffffB",
                          roll, pitch, yaw,
                          tgt_roll, tgt_pitch, tgt_yaw,
                          cmd)
    xor = 0
    for b in payload:
        xor ^= b
    return FRAME_HEAD + payload + bytes([xor]) + FRAME_TAIL


def walk_pattern(t):
    """走路步态: 周期性 target_pitch 正弦"""
    return walk_amplitude * math.sin(2 * math.pi * walk_speed * t)


def keyboard_listener():
    """键盘交互线程: 切换命令/调节目标角度"""
    global target_pitch, target_roll, target_yaw, current_cmd
    print("\n键盘控制:")
    print("  s = 停止, b = 平衡, w = 走路")
    print("  j/k = Pitch ±2°, h/l = Roll ±2°, u/o = Yaw ±5°")
    print("  r = 复位目标全0\n")
    while True:
        ch = input().strip().lower()
        if ch == 's':
            current_cmd = CMD_STOP
            print(f"→ 命令: 停止")
        elif ch == 'b':
            current_cmd = CMD_BALANCE
            print(f"→ 命令: 平衡 (tgt_pitch={target_pitch:.1f}°)")
        elif ch == 'w':
            current_cmd = CMD_WALK
            print(f"→ 命令: 走路 (速度={walk_speed}Hz, 摆幅={walk_amplitude}°)")
        elif ch == 'j':
            target_pitch -= 2.0
        elif ch == 'k':
            target_pitch += 2.0
        elif ch == 'h':
            target_roll -= 2.0
        elif ch == 'l':
            target_roll += 2.0
        elif ch == 'u':
            target_yaw -= 5.0
        elif ch == 'o':
            target_yaw += 5.0
        elif ch == 'r':
            target_pitch = target_roll = target_yaw = 0.0
            current_cmd = CMD_BALANCE
            print("→ 全部复位: 平衡模式, 目标全0")


def main():
    global target_pitch, current_cmd

    print("=" * 60)
    print("鲁班猫4 平衡控制 (带目标角度 + 命令字)")
    print(f"I2C:  /dev/i2c-{I2C_BUS}, 地址 0x{MPU_ADDR:02X}")
    print(f"串口: {SERIAL_PORT} @ {BAUDRATE} bps, {LOOP_HZ}Hz")
    print("=" * 60)

    mpu = MPU6050()
    mpu.init()
    print("MPU6050 初始化成功")
    mpu.calibrate_gyro()

    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
    print(f"串口已打开: {SERIAL_PORT}")

    # 启动键盘监听线程
    threading.Thread(target=keyboard_listener, daemon=True).start()

    print(f"\n{'Time':>8} {'Roll':>7} {'Pitch':>7} {'Yaw':>7} "
          f"| {'TgtP':>6} {'Cmd':>4} | {'FPS':>4}")
    print("-" * 75)

    loop_dt = 1.0 / LOOP_HZ
    last_time = time.monotonic()
    frame_count = 0
    fps_timer = last_time
    fps = 0
    walk_t = 0.0

    try:
        while True:
            mpu.update()
            roll, pitch, yaw = mpu.roll, mpu.pitch, mpu.yaw

            # 根据命令决定目标角度
            if current_cmd == CMD_WALK:
                walk_t += loop_dt
                tgt_pitch = walk_pattern(walk_t)
            else:
                tgt_pitch = target_pitch

            frame = build_frame(roll, pitch, yaw,
                                target_roll, tgt_pitch, target_yaw,
                                current_cmd)
            ser.write(frame)

            # FPS + 状态打印
            now = time.monotonic()
            frame_count += 1
            if now - fps_timer >= 1.0:
                fps = frame_count
                frame_count = 0
                fps_timer = now
                cmd_name = cmd_names.get(current_cmd, "?")
                print(f"\r{now%100:7.2f}s {roll:7.2f}° {pitch:7.2f}° {yaw:7.2f}° "
                      f"| {tgt_pitch:6.2f}° {cmd_name:>4} | {fps:4d}",
                      end="", flush=True)

            sleep_time = loop_dt - (time.monotonic() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n退出")
    finally:
        # 发停止命令
        stop_frame = build_frame(0, 0, 0, 0, 0, 0, CMD_STOP)
        ser.write(stop_frame)
        ser.close()


if __name__ == "__main__":
    main()
