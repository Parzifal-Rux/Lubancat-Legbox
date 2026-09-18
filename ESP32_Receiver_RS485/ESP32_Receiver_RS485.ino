/*
 * ESP32-S3 程序：接收鲁班猫姿态数据 + RS485 控制宇树 Go1 电机
 *
 * 硬件连接:
 *   - USB Type-C 接鲁班猫4 (原生 USB CDC, 显示为 /dev/ttyACM0)
 *   - GPIO17 (TX) → MAX485 DI
 *   - GPIO18 (RX) ← MAX485 RO
 *   - GPIO15      → MAX485 DE/RE (短接)
 *   - MAX485 A/B  → Go1 电机 A/B
 *   - 24V 电源   → 电机供电
 *
 * 依赖:
 *   - Arduino IDE + ESP32 板卡支持 (https://github.com/espressif/arduino-esp32)
 *   - 选板: ESP32S3 Dev Module
 *   - USB CDC On Boot: ON (使 Serial 走 USB)
 *
 * 帧格式 (鲁班猫 → ESP32, 14 字节):
 *   [0xAA][0x55][Roll:f32][Pitch:f32][Yaw:f32][xor][0x0D][0x0A]
 *
 * Go1 电机控制帧 (17 字节, 宇树协议):
 *   参考 https://www.unitree.com/go1.html 电机协议文档
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ========== 配置 ==========
#define RS485_TX     17
#define RS485_RX     18
#define RS485_DE_PIN 15
#define RS485_BAUD   4000000   // Go1 电机 4Mbps

// 鲁班猫 → ESP32 帧定义
// 帧: [0xAA][0x55][Roll:f32][Pitch:f32][Yaw:f32][xor][0x0D][0x0A] = 17 字节
#define FRAME_LEN     17
#define FRAME_HEAD0  0xAA
#define FRAME_HEAD1  0x55
#define FRAME_TAIL0  0x0D
#define FRAME_TAIL1  0x0A

// Go1 电机 ID 和模式
#define MOTOR_ID     0          // 电机 ID (0~14)
#define MOTOR_MODE   1          // 0=停机 1=速度 2=位置

// 速度控制参数 (根据实际调试)
#define SPEED_SCALE  1.0f       // 速度缩放

// RGB LED (板载 WS2812, ESP32-S3 默认 GPIO48)
#define LED_PIN        48
#define NUM_PIXELS     1
#define STABLE_THRESH  5.0f     // 平稳阈值 (度): |角度|<此值视为平稳

Adafruit_NeoPixel led(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ========== 全局变量 ==========
HardwareSerial RS485(1);

uint8_t rx_buf[64];
int rx_idx = 0;

float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

unsigned long last_frame_ms = 0;

// ========== 鲁班猫帧解析 ==========
bool parse_frame() {
  if (rx_idx < FRAME_LEN) return false;

  // 找帧头
  int head = -1;
  for (int i = 0; i <= rx_idx - FRAME_LEN; i++) {
    if (rx_buf[i] == FRAME_HEAD0 && rx_buf[i+1] == FRAME_HEAD1) {
      head = i;
      break;
    }
  }
  if (head < 0) {
    // 丢弃一半数据
    if (rx_idx > FRAME_LEN) {
      memmove(rx_buf, rx_buf + rx_idx - FRAME_LEN + 1, FRAME_LEN - 1);
      rx_idx = FRAME_LEN - 1;
    }
    return false;
  }

  // 帧尾校验
  if (rx_buf[head + FRAME_LEN - 2] != FRAME_TAIL0 ||
      rx_buf[head + FRAME_LEN - 1] != FRAME_TAIL1) {
    memmove(rx_buf, rx_buf + head + 1, rx_idx - head - 1);
    rx_idx -= head + 1;
    return false;
  }

  // XOR 校验
  uint8_t xor_val = 0;
  for (int i = 2; i < FRAME_LEN - 3; i++) {
    xor_val ^= rx_buf[head + i];
  }
  if (xor_val != rx_buf[head + FRAME_LEN - 3]) {
    memmove(rx_buf, rx_buf + head + 1, rx_idx - head - 1);
    rx_idx -= head + 1;
    return false;
  }

  // 解析 (小端 float)
  memcpy(&roll,  &rx_buf[head + 2], 4);
  memcpy(&pitch, &rx_buf[head + 6], 4);
  memcpy(&yaw,   &rx_buf[head + 10], 4);

  // 丢弃已处理的帧
  memmove(rx_buf, rx_buf + head + FRAME_LEN, rx_idx - head - FRAME_LEN);
  rx_idx -= head + FRAME_LEN;

  last_frame_ms = millis();
  return true;
}

// ========== Go1 电机控制帧 ==========
// 宇树 Go1 电机控制帧 (17 字节)
// 头: 0xFE 0x0F 0x00 ID
// 数据: 位置(2) 速度(2) 力矩(2) KP(2) KD(2) 预留(2)
// CRC: CRC16 (2 字节, Modbus)
// 详细字段定义参考宇树电机协议
uint16_t crc16_modbus(const uint8_t *data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

void build_motor_frame(uint8_t *frame, uint8_t motor_id,
                       float pos, float vel, float torque,
                       float kp, float kd) {
  // 简化版: 实际宇树协议参考官方文档
  // 这里用 0xFE 0x0F 头 + 数据 + CRC16
  frame[0] = 0xFE;
  frame[1] = 0x0F;       // 数据长度
  frame[2] = 0x00;      // 保留
  frame[3] = motor_id;  // 电机 ID
  // 数值映射到 int16 (示例, 实际按协议)
  int16_t pos_i   = (int16_t)(pos * 1000);    // 位置 → 0.001°/LSB
  int16_t vel_i   = (int16_t)(vel * 100);     // 速度 → 0.01°/s/LSB
  int16_t torque_i= (int16_t)(torque * 100);  // 力矩 → 0.01 N·m/LSB
  int16_t kp_i    = (int16_t)(kp * 100);
  int16_t kd_i    = (int16_t)(kd * 100);
  // 写入帧 (大端)
  frame[4]  = (pos_i    >> 8) & 0xFF; frame[5]  = pos_i    & 0xFF;
  frame[6]  = (vel_i   >> 8) & 0xFF; frame[7]  = vel_i    & 0xFF;
  frame[8]  = (torque_i>> 8) & 0xFF; frame[9]  = torque_i & 0xFF;
  frame[10] = (kp_i    >> 8) & 0xFF; frame[11] = kp_i     & 0xFF;
  frame[12] = (kd_i    >> 8) & 0xFF; frame[13] = kd_i     & 0xFF;
  frame[14] = 0x00; frame[15] = 0x00;  // 预留
  // CRC16
  uint16_t crc = crc16_modbus(frame, 16);
  frame[16] = crc & 0xFF;       // 小端
  // 注: 帧长度 = 17 (frame[0..16])
}

void send_motor_cmd() {
  uint8_t frame[17];

  // 简单示例: 用 Pitch 角度映射到电机速度
  // 正常应做 PID 闭环, 这里只是演示链路
  float target_vel = -pitch * 5.0f * SPEED_SCALE;   // 俯仰角 → 速度 (deg/s → scale)
  target_vel = constrain(target_vel, -50.0f, 50.0f);

  build_motor_frame(frame, MOTOR_ID,
                    0.0f,          // 位置 (0 = 不限制位置)
                    target_vel,    // 速度
                    0.0f,          // 力矩前馈
                    0.0f,          // KP (位置增益)
                    0.5f);         // KD (速度阻尼)

  // 发送时拉高 DE
  digitalWrite(RS485_DE_PIN, HIGH);
  RS485.write(frame, 17);
  RS485.flush();                  // 等待发送完
  digitalWrite(RS485_DE_PIN, LOW);

  // 等电机反馈 (16 字节, 约 40us @ 4Mbps)
  delayMicroseconds(50);
}

// ========== RGB LED 状态指示 ==========
// 规则: Pitch >  +5° → 蓝灯
//       Pitch <  -5° → 红灯
//       |Pitch| ≤ 5° → 不亮 (平稳)
void update_led() {
  uint8_t r = 0, g = 0, b = 0;
  if (pitch >  STABLE_THRESH) b = 255;   // 正 → 蓝
  else if (pitch < -STABLE_THRESH) r = 255;  // 负 → 红
  // 平稳 → r=g=b=0 (黑, 不亮)

  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// ========== setup ==========
void setup() {
  Serial.begin(115200);                  // USB CDC: 接鲁班猫
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);       // 默认接收

  // 初始化板载 RGB LED
  led.begin();
  led.setBrightness(30);                  // 亮度 0-255 (避免刺眼)
  led.setPixelColor(0, 0);                // 初始不亮
  led.show();

  Serial.println("ESP32-S3 启动: 鲁班猫姿态接收 + Go1 电机控制 + RGB 状态指示");
}

// ========== loop ==========
void loop() {
  // 1. 接收鲁班猫数据
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (rx_idx < (int)sizeof(rx_buf)) {
      rx_buf[rx_idx++] = c;
    } else {
      // 缓冲区满, 丢前面
      memmove(rx_buf, rx_buf + 1, sizeof(rx_buf) - 1);
      rx_buf[sizeof(rx_buf) - 1] = c;
    }

    if (parse_frame()) {
      Serial.printf("Rcv: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", roll, pitch, yaw);
      // 更新 RGB LED 状态指示
      update_led();
      // 收到姿态后立即发送电机命令
      send_motor_cmd();
    }
  }

  // 2. 超时检测 (500ms 没收到帧)
  if (millis() - last_frame_ms > 500 && last_frame_ms != 0) {
    Serial.println("警告: 鲁班猫数据超时");
    last_frame_ms = millis();
  }
}
