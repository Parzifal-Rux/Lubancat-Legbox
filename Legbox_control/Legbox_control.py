/#!/usr/bin/env python3
# Legbox_control.py — 鲁班猫4 端主控
# 功能: 读取 MPU6050 姿态 + 通过串口控制 ESP32 (Go1 电机)
# 运行: sudo python3 Legbox_control.py
# 依赖: pip install smbus2 pyserial

import math
import time
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None
try:
    import serial
except ImportError:
    serial = None

# ===== 配置区 =====
# MPU6050
I2C_BUS = 6            # 鲁班猫4 扩展板 MPU6050 接口
MPU_ADDR = 0x69        # AD0 拉高
CALIB_SAMPLES = 500

# ESP32 串口
ESP32_SERIAL = '/dev/ttyS1'   # 鲁班猫4 扩展板 UART 接口（需根据实际修改）
ESP32_BAUD = 115200

# MPU6050 寄存器
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT = 0x3B
GYRO_XOUT = 0x43

ACCEL_SENS = 8192.0
GYRO_SENS = 16.4


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

    def _read_word(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        val = (data[0] << 8) | data[1]
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
        ax = self._read_word(ACCEL_XOUT) / ACCEL_SENS
        ay = self._read_word(ACCEL_XOUT + 2) / ACCEL_SENS
        az = self._read_word(ACCEL_XOUT + 4) / ACCEL_SENS
        return ax, ay, az

    def read_gyro_raw(self):
        gx = self._read_word(GYRO_XOUT) / GYRO_SENS
        gy = self._read_word(GYRO_XOUT + 2) / GYRO_SENS
        gz = self._read_word(GYRO_XOUT + 4) / GYRO_SENS
        return gx, gy, gz

    def read_gyro(self):
        gx, gy, gz = self.read_gyro_raw()
        gx -= self.gyro_offset[0]
        gy -= self.gyro_offset[1]
        gz -= self.gyro_offset[2]
        return gx, gy, gz

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
        acc_roll = math.atan2(ay, az) * 180.0 / math.pi
        acc_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi
        dt = self._get_dt()
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * acc_pitch
        self.yaw += gz * dt


class ESP32Controller:
    """鲁班猫 → ESP32 → Go1 电机的串口控制"""

    def __init__(self, port=ESP32_SERIAL, baud=ESP32_BAUD):
        self.ser = None
        if serial is None:
            print("警告: pyserial 未安装，ESP32 控制不可用")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            time.sleep(0.5)
            # 等待 ESP32 就绪
            self.ser.write(b"PING\n")
            resp = self.ser.readline().decode().strip()
            if resp == "PONG":
                print(f"ESP32 已连接: {port}@{baud}")
            else:
                print(f"ESP32 响应异常: {resp}")
        except Exception as e:
            print(f"ESP32 串口打开失败: {e}")
            self.ser = None

    def send_motor_cmd(self, motor_id, mode, pos, vel, kp, kd, tau):
        """
        motor_id: 0-14
        mode: 0=停止, 1=FOC, 2=校准
        pos: 目标位置 (rad)
        vel: 目标速度 (rad/s)
        kp: 位置刚度
        kd: 速度阻尼
        tau: 前馈力矩 (Nm)
        返回: dict 或 None
        """
        if not self.ser:
            return None
        cmd = f"CMD:{motor_id}:{mode}:{pos:.3f}:{vel:.3f}:{kp:.3f}:{kd:.3f}:{tau:.3f}\n"
        self.ser.write(cmd.encode())
        # 等待反馈
        line = self.ser.readline().decode().strip()
        if line.startswith("FB:"):
            parts = line.split(":")
            return {
                'id': int(parts[1]),
                'pos': float(parts[2]),
                'vel': float(parts[3]),
                'torque': float(parts[4]),
                'temp': float(parts[5])
            }
        elif line.startswith("ERR:"):
            print(f"电机错误: {line}")
            return None
        return None

    def stop_motor(self, motor_id):
        """紧急停止"""
        return self.send_motor_cmd(motor_id, 0, 0, 0, 0, 0, 0)


def main():
    # 初始化 MPU6050
    mpu = MPU6050()
    print(f"MPU6050 初始化成功 (addr=0x{MPU_ADDR:02X}, bus=/dev/i2c-{I2C_BUS})")
    mpu.init()
    mpu.calibrate_gyro()

    # 初始化 ESP32 控制器
    esp = ESP32Controller()

    print("\n开始读姿态并控制电机（Ctrl+C 退出）")
    print(f"{'Roll':>9} {'Pitch':>9} {'Yaw':>9} {'Motor_fb':>20}")
    print("-" * 55)

    try:
        while True:
            mpu.update()

            # 示例: 根据 Roll 角度计算电机力矩 (简单 PD 平衡)
            # 这里只是演示，实际控制算法需要根据机器人结构调整
            target_tau = -0.5 * mpu.roll  # 简单比例控制
            target_tau = max(-3.0, min(3.0, target_tau))  # 限幅 ±3Nm

            # 发送给电机 ID=0
            fb = esp.send_motor_cmd(
                motor_id=0, mode=1,
                pos=0.0, vel=0.0, kp=0.0, kd=0.0,
                tau=target_tau
            )

            fb_str = (f"{fb['pos']:.2f},{fb['torque']:.2f}Nm"
                      if fb else "no_response")
            print(f"\r{mpu.roll:7.2f}° {mpu.pitch:7.2f}° {mpu.yaw:7.2f}° "
                  f"{fb_str:>20}", end="", flush=True)
            time.sleep(0.02)  # 50Hz
    except KeyboardInterrupt:
        print("\n退出前停止电机...")
        esp.stop_motor(0)
        print("已退出")


if __name__ == "__main__":
    main()
