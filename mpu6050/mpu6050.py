#!/usr/bin/env python3
# MPU6050 姿态读取（互补滤波）
# 运行：sudo python3 MPU6050.py
# 依赖：pip install smbus2

import math
import time
from smbus2 import SMBus

# ===== 配置区 =====
I2C_BUS = 6            # 鲁班猫4 扩展板 MPU6050 接口
MPU_ADDR = 0x69        # AD0 拉高
CALIB_SAMPLES = 500    # 校准采样次数

# MPU6050 寄存器
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT = 0x3B
GYRO_XOUT = 0x43

# 量程配置
ACCEL_SENS = 8192.0    # ±4g → 8192 LSB/g
GYRO_SENS = 16.4       # ±2000°/s → 16.4 LSB/(°/s)


class MPU6050:
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
    def _read_word(self, reg):
        # 读取 16 位有符号值（大端）
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        val = (data[0] << 8) | data[1]
        if val >= 0x8000:
            val -= 0x10000
        return val

    def _write_byte(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    # ===== 初始化 =====
    def init(self):
        self._write_byte(PWR_MGMT_1, 0x00)   # 唤醒
        time.sleep(0.1)
        self._write_byte(GYRO_CONFIG, 0x18)   # 陀螺仪 ±2000°/s
        self._write_byte(ACCEL_CONFIG, 0x08)  # 加速度 ±4g
        time.sleep(0.1)

    # ===== 读加速度 (g) =====
    def read_accel(self):
        ax = self._read_word(ACCEL_XOUT) / ACCEL_SENS
        ay = self._read_word(ACCEL_XOUT + 2) / ACCEL_SENS
        az = self._read_word(ACCEL_XOUT + 4) / ACCEL_SENS
        return ax, ay, az

    # ===== 读陀螺仪 (°/s) 未校准 =====
    def read_gyro_raw(self):
        gx = self._read_word(GYRO_XOUT) / GYRO_SENS
        gy = self._read_word(GYRO_XOUT + 2) / GYRO_SENS
        gz = self._read_word(GYRO_XOUT + 4) / GYRO_SENS
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


def main():
    mpu = MPU6050()
    print(f"MPU6050 初始化成功 (addr=0x{MPU_ADDR:02X}, bus=/dev/i2c-{I2C_BUS})")

    mpu.init()
    mpu.calibrate_gyro()

    print("\n开始读姿态数据（Ctrl+C 退出）")
    print(f"{'Roll':>9} {'Pitch':>9} {'Yaw':>9} {'AccX':>8} {'AccY':>8} {'AccZ':>8}")
    print("-" * 55)

    try:
        while True:
            mpu.update()
            ax, ay, az = mpu.read_accel()
            print(f"\r{mpu.roll:7.2f}° {mpu.pitch:7.2f}° {mpu.yaw:7.2f}° "
                  f"{ax:6.2f}g {ay:6.2f}g {az:6.2f}g", end="", flush=True)
            time.sleep(0.02)   # 50Hz
    except KeyboardInterrupt:
        print("\n已退出")


if __name__ == "__main__":
    main()
