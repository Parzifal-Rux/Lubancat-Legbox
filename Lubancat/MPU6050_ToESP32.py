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
ACCEL_XOUT = 0x3B
GYRO_XOUT  = 0x43

# 量程 LSB
ACCEL_SENS = 8192.0   # ±4g
GYRO_SENS  = 16.4     # ±2000°/s

# 帧头/帧尾
FRAME_HEAD = b"\xAA\x55"
FRAME_TAIL = b"\x0D\x0A"


class MPU6050:
    """MPU6050 姿态读取（互补滤波，来自 mpu6050.py）"""

    def __init__(self, bus=I2C_BUS, addr=MPU_ADDR):
        self.bus = SMBus(bus)
        self.addr = addr
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.98
        self.last_time = None

    # ===== I2C 读写 =====
    @staticmethod
    def _word(hi, lo):
        """两个字节转 16 位有符号整数"""
        val = (hi << 8) | lo
        if val >= 0x8000:
            val -= 0x10000
        return val

    def _read_word(self, reg):
        # 读取 16 位有符号值（大端）
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return self._word(data[0], data[1])

    def _write_byte(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    # ===== 初始化 =====
    def init(self):
        self._write_byte(PWR_MGMT_1, 0x00)   # 唤醒
        time.sleep(0.1)
        self._write_byte(GYRO_CONFIG, 0x18)   # 陀螺仪 ±2000°/s
        self._write_byte(ACCEL_CONFIG, 0x08)   # 加速度 ±4g
        time.sleep(0.1)

    # ===== 读加速度 (g) — burst read 6 字节 =====
    def read_accel(self):
        data = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT, 6)
        ax = self._word(data[0], data[1]) / ACCEL_SENS
        ay = self._word(data[2], data[3]) / ACCEL_SENS
        az = self._word(data[4], data[5]) / ACCEL_SENS
        return ax, ay, az

    # ===== 读陀螺仪 (°/s) 未校准 — burst read 6 字节 =====
    def read_gyro_raw(self):
        data = self.bus.read_i2c_block_data(self.addr, GYRO_XOUT, 6)
        gx = self._word(data[0], data[1]) / GYRO_SENS
        gy = self._word(data[2], data[3]) / GYRO_SENS
        gz = self._word(data[4], data[5]) / GYRO_SENS
        return gx, gy, gz

    # ===== 读陀螺仪 (校准后) =====
    def read_gyro(self):
        gx, gy, gz = self.read_gyro_raw()
        gx -= self.gyro_offset[0]
        gy -= self.gyro_offset[1]
        gz -= self.gyro_offset[2]
        return gx, gy, gz

    # ===== 陀螺仪零偏校准 =====
    def calibrate_gyro(self):
        print(f"校准中... 请保持 MPU6050 静止 {CALIB_SAMPLES} 次")
        gx_sum = gy_sum = gz_sum = 0.0
        for _ in range(CALIB_SAMPLES):
            gx, gy, gz = self.read_gyro_raw()
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz
            time.sleep(0.002)
        self.gyro_offset = [gx_sum / CALIB_SAMPLES,
                            gy_sum / CALIB_SAMPLES,
                            gz_sum / CALIB_SAMPLES]
        print(f"校准完成: offset=({self.gyro_offset[0]:.2f}, "
              f"{self.gyro_offset[1]:.2f}, {self.gyro_offset[2]:.2f})")

    # ===== 获取时间差 (秒) =====
    def _get_dt(self):
        now = time.monotonic()
        if self.last_time is None:
            self.last_time = now
            return 0.0
        dt = now - self.last_time
        self.last_time = now
        return dt

    # ===== 互补滤波更新姿态 =====
    def update(self):
        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()

        # 加速度算角度
        acc_roll = math.atan2(ay, az) * 180.0 / math.pi
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

        dt = self._get_dt()

        # 互补滤波
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * acc_pitch
        self.yaw += gz * dt   # Yaw 靠积分，会漂移


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
    mpu.init()
    print("MPU6050 初始化成功")

    # 校准
    mpu.calibrate_gyro()

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
            # 更新姿态 (mpu6050.py 内部管理 dt)
            mpu.update()
            roll, pitch, yaw = mpu.roll, mpu.pitch, mpu.yaw

            # 发送给 ESP32
            frame = build_frame(roll, pitch, yaw)
            ser.write(frame)

            # FPS 统计 + 打印
            now = time.monotonic()
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
