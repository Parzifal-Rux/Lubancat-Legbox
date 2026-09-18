#!/usr/bin/env python3
"""
鲁班猫4 程序：读取 MPU6050 姿态，通过串口发送给 ESP32-S3
- I2C:    /dev/i2c-6, 地址 0x69
- 串口:   /dev/ttyS1 (40pin UART), 115200 bps
- 帧格式: [0xAA][0x55][Roll:f32][Pitch:f32][Yaw:f32][xor][0x0D][0x0A] = 14 字节
- 频率:   70 Hz

依赖:
    sudo pip3 install smbus2 pyserial

运行:
    sudo python3 MPU6050_ToESP32.py
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
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# 量程 LSB
ACCEL_LSB = 8192.0   # ±4g
GYRO_LSB  = 16.4     # ±2000°/s

# 帧头/帧尾
FRAME_HEAD = b"\xAA\x55"
FRAME_TAIL = b"\x0D\x0A"


class MPU6050:
    """MPU6050 姿态读取（互补滤波）"""

    def __init__(self, bus=I2C_BUS, addr=MPU_ADDR):
        self.bus = SMBus(bus)
        self.addr = addr
        # 唤醒 + 量程配置
        self.bus.write_byte_data(addr, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(addr, GYRO_CONFIG, 0x18)    # ±2000°/s
        self.bus.write_byte_data(addr, ACCEL_CONFIG, 0x08)   # ±4g
        time.sleep(0.01)
        # 滤波参数
        self.alpha = 0.98
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # 陀螺仪零偏
        self.g_offset = [0.0, 0.0, 0.0]

    def _read_word(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        val = (data[0] << 8) | data[1]
        if val >= 0x8000:
            val -= 0x10000
        return val

    def read_accel(self):
        ax = self._read_word(ACCEL_XOUT_H)     / ACCEL_LSB
        ay = self._read_word(ACCEL_XOUT_H + 2) / ACCEL_LSB
        az = self._read_word(ACCEL_XOUT_H + 4) / ACCEL_LSB
        return ax, ay, az

    def read_gyro_raw(self):
        gx = self._read_word(GYRO_XOUT_H)     / GYRO_LSB
        gy = self._read_word(GYRO_XOUT_H + 2) / GYRO_LSB
        gz = self._read_word(GYRO_XOUT_H + 4) / GYRO_LSB
        return gx, gy, gz

    def read_gyro(self):
        gx, gy, gz = self.read_gyro_raw()
        return (gx - self.g_offset[0],
                gy - self.g_offset[1],
                gz - self.g_offset[2])

    def calibrate(self, samples=CALIB_SAMPLES):
        print(f"校准中... 请保持 MPU6050 静止 {samples} 次")
        sx = sy = sz = 0.0
        for _ in range(samples):
            gx, gy, gz = self.read_gyro_raw()
            sx += gx; sy += gy; sz += gz
            time.sleep(0.002)
        self.g_offset[0] = sx / samples
        self.g_offset[1] = sy / samples
        self.g_offset[2] = sz / samples
        print(f"校准完成: offset=({self.g_offset[0]:.2f}, "
              f"{self.g_offset[1]:.2f}, {self.g_offset[2]:.2f})")

    def update(self, dt):
        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()

        # 加速度计计算 Roll/Pitch
        roll_acc  = math.atan2(ay, az) * 180.0 / math.pi
        pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi

        # 互补滤波
        self.roll  = self.alpha * (self.roll  + gx * dt) + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * pitch_acc
        self.yaw  += gz * dt   # Yaw 纯积分（会漂移）

        return self.roll, self.pitch, self.yaw, ax, ay, az


def build_frame(roll, pitch, yaw):
    """构造 14 字节二进制帧"""
    payload = struct.pack("<fff", roll, pitch, yaw)  # 12 字节
    xor = 0
    for b in payload:
        xor ^= b
    return FRAME_HEAD + payload + bytes([xor]) + FRAME_TAIL


def main():
    print("=" * 60)
    print("鲁班猫4 MPU6050 姿态检测 → ESP32 串口发送")
    print(f"I2C:  /dev/i2c-{I2C_BUS}, 地址 0x{MPU_ADDR:02X}")
    print(f"串口: {SERIAL_PORT} @ {BAUDRATE} bps, 70Hz")
    print("=" * 60)

    # 初始化 MPU6050
    mpu = MPU6050()
    print("MPU6050 初始化成功")

    # 校准
    mpu.calibrate()

    # 打开串口
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
    print(f"串口已打开: {SERIAL_PORT}")

    print("\n开始发送姿态数据（Ctrl+C 退出）")
    print(f"{'Roll':>8} {'Pitch':>8} {'Yaw':>8}  {'FPS':>5}")
    print("-" * 40)

    loop_dt = 1.0 / LOOP_HZ
    last_time = time.monotonic()
    frame_count = 0
    fps_timer = last_time
    fps = 0

    try:
        while True:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            roll, pitch, yaw, _, _, _ = mpu.update(dt)

            # 发送给 ESP32
            frame = build_frame(roll, pitch, yaw)
            ser.write(frame)

            # FPS 统计 + 打印
            frame_count += 1
            if now - fps_timer >= 1.0:
                fps = frame_count
                frame_count = 0
                fps_timer = now
                print(f"\r{roll:7.2f}° {pitch:7.2f}° {yaw:7.2f}°  {fps:5d}", end="", flush=True)

            # 控制循环频率
            elapsed = time.monotonic() - now
            sleep_time = loop_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
