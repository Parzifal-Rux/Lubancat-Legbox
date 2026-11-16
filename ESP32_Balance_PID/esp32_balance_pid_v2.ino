/*
 * ESP32-S3 平衡控制 V2: 接收鲁班猫扩展帧 (带目标角度 + 命令字)
 *
 * 与 V1 (ESP32_Balance_PID.ino) 的区别:
 *   - 帧长度 25 → 30 字节, 增加 TgtRoll/TgtPitch/TgtYaw + Cmd
 *   - Cmd 控制启停: 0=停止, 1=平衡, 2=走路 (步态由鲁班猫周期性改 TgtPitch)
 *   - PID 目标不再硬编码, 用鲁班猫发来的 TgtPitch
 *   - 走路模式: ESP32 只负责 PID 跟随, 步态在鲁班猫端生成
 *
 * 硬件连接:
 *   - USB Type-C 接鲁班猫4 (原生 USB CDC, /dev/ttyACM0)
 *   - GPIO17 (TX) → MAX485 DI
 *   - GPIO18 (RX) ← MAX485 RO
 *   - GPIO15      → MAX485 DE/RE (短接)
 *   - MAX485 A/B  → Go1 电机 A/B
 *   - 24V 电源   → 电机供电
 *
 * Arduino IDE 配置:
 *   - 板子: ESP32S3 Dev Module
 *   - USB CDC On Boot: ON
 *
 * 帧格式 (鲁班猫 → ESP32, 30 字节):
 *   [0xAA][0x55]
 *   [Roll:f32][Pitch:f32][Yaw:f32]          ← 传感器 (12B)
 *   [TgtRoll:f32][TgtPitch:f32][TgtYaw:f32] ← 目标角度 (12B)
 *   [Cmd:u8]                                 ← 命令字 (1B)
 *   [xor:u8]                                 ← 校验 (1B)
 *   [0x0D][0x0A]                             ← 帧尾 (2B)
 *   总计 = 2 + 12 + 12 + 1 + 1 + 2 = 30 字节
 *
 * Cmd 命令字:
 *   0x00 = 停止 (电机停机)
 *   0x01 = 平衡 (PID 闭环, 目标=TgtPitch)
 *   0x02 = 走路 (同平衡, 但鲁班猫端周期性改 TgtPitch)
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ========== 引脚配置 ==========
#define RS485_TX     17
#define RS485_RX     18
#define RS485_DE_PIN 15
#define RS485_BAUD   4000000

#define LED_PIN      48
#define NUM_PIXELS   1

// ========== 鲁班猫帧定义 (30 字节) ==========
#define FRAME_LEN     30
#define FRAME_HEAD0  0xAA
#define FRAME_HEAD1  0x55
#define FRAME_TAIL0  0x0D
#define FRAME_TAIL1  0x0A

// 帧内偏移
#define OFF_ROLL      2
#define OFF_PITCH     6
#define OFF_YAW      10
#define OFF_TGT_ROLL 14
#define OFF_TGT_PITCH 18
#define OFF_TGT_YAW  22
#define OFF_CMD      26
#define OFF_XOR      27
// 帧尾 28,29

// ========== 命令字 ==========
#define CMD_STOP    0x00
#define CMD_BALANCE 0x01
#define CMD_WALK    0x02

// ========== 电机配置 ==========
#define MOTOR_ID     0
#define FALL_ANGLE   30.0f
#define VEL_MAX      50.0f

// ========== PID 参数 ==========
// 调参: 先 P 后 D 最后 I
#define PID_KP  8.0f
#define PID_KI  0.05f
#define PID_KD  1.2f

// ========== LED 阈值 ==========
#define STABLE_THRESH  5.0f

// ========== 对象 ==========
Adafruit_NeoPixel led(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial rs485(1);

// ========== PID 控制器类 ==========
class PID {
public:
  float kp, ki, kd;
  float integral;
  float last_error;
  float output_min, output_max;
  unsigned long last_ms;

  PID(float _kp, float _ki, float _kd, float _min, float _max)
    : kp(_kp), ki(_ki), kd(_kd),
      integral(0), last_error(0),
      output_min(_min), output_max(_max),
      last_ms(0) {}

  void reset() {
    integral = 0;
    last_error = 0;
    last_ms = 0;
  }

  float compute(float target, float current) {
    unsigned long now = millis();
    float dt = (last_ms == 0) ? 0.001f : (now - last_ms) / 1000.0f;
    last_ms = now;
    if (dt <= 0 || dt > 0.1f) dt = 0.001f;

    float error = target - current;
    integral += error * dt;
    integral = constrain(integral, -output_max / (ki + 0.001f),
                         output_max / (ki + 0.001f));
    float derivative = (error - last_error) / dt;
    last_error = error;

    float out = kp * error + ki * integral + kd * derivative;
    return constrain(out, output_min, output_max);
  }
};

// ========== 全局变量 ==========
PID pid(PID_KP, PID_KI, PID_KD, -VEL_MAX, VEL_MAX);

uint8_t rx_buf[128];
int rx_idx = 0;

// 传感器姿态
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;
// 目标角度 (由鲁班猫设定)
float tgt_roll  = 0.0f;
float tgt_pitch = 0.0f;
float tgt_yaw   = 0.0f;
// 命令字
uint8_t cmd = CMD_BALANCE;

unsigned long last_frame_ms = 0;
bool motor_enabled = true;

// ========== CRC16 Modbus ==========
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

// ========== 鲁班猫帧解析 (30 字节) ==========
bool parse_frame() {
  if (rx_idx < FRAME_LEN) return false;

  int head = -1;
  for (int i = 0; i <= rx_idx - FRAME_LEN; i++) {
    if (rx_buf[i] == FRAME_HEAD0 && rx_buf[i+1] == FRAME_HEAD1) {
      head = i;
      break;
    }
  }
  if (head < 0) {
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

  // XOR 校验 (2~26, 共 25 字节: Roll/Pitch/Yaw/TgtRoll/TgtPitch/TgtYaw/Cmd)
  uint8_t xor_val = 0;
  for (int i = 2; i <= 26; i++) {
    xor_val ^= rx_buf[head + i];
  }
  if (xor_val != rx_buf[head + OFF_XOR]) {
    memmove(rx_buf, rx_buf + head + 1, rx_idx - head - 1);
    rx_idx -= head + 1;
    return false;
  }

  // 解析姿态
  memcpy(&roll,  &rx_buf[head + OFF_ROLL],  4);
  memcpy(&pitch, &rx_buf[head + OFF_PITCH], 4);
  memcpy(&yaw,   &rx_buf[head + OFF_YAW],   4);
  // 解析目标
  memcpy(&tgt_roll,  &rx_buf[head + OFF_TGT_ROLL],  4);
  memcpy(&tgt_pitch, &rx_buf[head + OFF_TGT_PITCH], 4);
  memcpy(&tgt_yaw,   &rx_buf[head + OFF_TGT_YAW],   4);
  // 命令字
  cmd = rx_buf[head + OFF_CMD];

  // 丢弃已处理的帧
  memmove(rx_buf, rx_buf + head + FRAME_LEN, rx_idx - head - FRAME_LEN);
  rx_idx -= head + FRAME_LEN;

  last_frame_ms = millis();
  return true;
}

// ========== Go1 电机控制帧 ==========
void build_motor_frame(uint8_t *frame, uint8_t motor_id,
                       float pos, float vel, float kp, float kd) {
  frame[0] = 0xFE;
  frame[1] = 0x0F;
  frame[2] = 0x00;
  frame[3] = motor_id;

  int16_t pos_i  = (int16_t)(pos * 1000);
  int16_t vel_i  = (int16_t)(vel * 100);
  int16_t kp_i   = (int16_t)(kp * 100);
  int16_t kd_i   = (int16_t)(kd * 100);

  frame[4]  = (pos_i >> 8) & 0xFF; frame[5]  = pos_i  & 0xFF;
  frame[6]  = (vel_i >> 8) & 0xFF; frame[7]  = vel_i  & 0xFF;
  frame[8]  = 0; frame[9]  = 0;
  frame[10] = (kp_i >> 8) & 0xFF; frame[11] = kp_i   & 0xFF;
  frame[12] = (kd_i >> 8) & 0xFF; frame[13] = kd_i   & 0xFF;
  frame[14] = 0; frame[15] = 0;

  uint16_t crc = crc16_modbus(frame, 16);
  frame[16] = crc & 0xFF;
}

void send_motor_cmd(uint8_t motor_id, float pos, float vel, float kp, float kd) {
  uint8_t frame[17];
  build_motor_frame(frame, motor_id, pos, vel, kp, kd);

  digitalWrite(RS485_DE_PIN, HIGH);
  rs485.write(frame, 17);
  rs485.flush();
  digitalWrite(RS485_DE_PIN, LOW);
  delayMicroseconds(50);
}

// ========== 停电机 (命令字 0) ==========
void stop_motor() {
  send_motor_cmd(MOTOR_ID, 0.0f, 0.0f, 0.0f, 0.5f);
}

// ========== LED 状态指示 ==========
// Pitch >  +5° → 蓝灯 | Pitch < -5° → 红灯 | |Pitch| ≤ 5° → 不亮
// 命令字 STOP → 紫灯 (停) | WALK → 绿灯 (走路) | 摔倒保护 → 红灯快闪
void update_led() {
  uint8_t r = 0, g = 0, b = 0;

  if (fabs(pitch) > FALL_ANGLE) {
    // 摔倒保护: 红灯快闪
    r = (millis() / 100) % 2 ? 255 : 0;
  } else if (cmd == CMD_STOP) {
    // 停止: 紫色
    r = 255; b = 255;
  } else if (cmd == CMD_WALK) {
    // 走路: 绿色
    g = 255;
  } else if (pitch >  STABLE_THRESH) {
    b = 255;
  } else if (pitch < -STABLE_THRESH) {
    r = 255;
  }

  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// ========== setup ==========
void setup() {
  Serial.begin(115200);
  rs485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  led.begin();
  led.setBrightness(30);
  led.setPixelColor(0, 0);
  led.show();

  Serial.println("========================================");
  Serial.println("ESP32-S3 平衡控制 V2 (扩展帧 + 命令字)");
  Serial.println("========================================");
  Serial.printf("PID: Kp=%.2f Ki=%.3f Kd=%.2f\n", PID_KP, PID_KI, PID_KD);
  Serial.printf("帧长度: %d 字节 (扩展目标角度 + Cmd)\n", FRAME_LEN);
  Serial.printf("命令字: 0=停止 1=平衡 2=走路\n");
  Serial.println("等待鲁班猫数据...\n");
}

// ========== loop ==========
void loop() {
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (rx_idx < (int)sizeof(rx_buf)) {
      rx_buf[rx_idx++] = c;
    } else {
      memmove(rx_buf, rx_buf + 1, sizeof(rx_buf) - 1);
      rx_buf[sizeof(rx_buf) - 1] = c;
    }

    if (parse_frame()) {
      // 摔倒保护
      if (fabs(pitch) > FALL_ANGLE) {
        if (motor_enabled) {
          motor_enabled = false;
          pid.reset();
          Serial.printf("!!! 摔倒保护: Pitch=%.2f° !!! 电机停止\n", pitch);
          stop_motor();
        }
        update_led();
        continue;
      } else if (!motor_enabled) {
        motor_enabled = true;
        pid.reset();
        Serial.println("--- 已扶正, 电机恢复 ---");
      }

      // 根据命令字决定动作
      if (cmd == CMD_STOP) {
        stop_motor();
        pid.reset();
        static unsigned long last_stop_print = 0;
        if (millis() - last_stop_print > 500) {
          last_stop_print = millis();
          Serial.println("[命令] 停止, 电机停机");
        }
      } else {
        // CMD_BALANCE 或 CMD_WALK: PID 闭环
        float target = tgt_pitch;                       // 用鲁班猫发来的目标
        float target_vel = pid.compute(target, pitch);
        send_motor_cmd(MOTOR_ID, 0.0f, target_vel, 0.0f, 0.5f);

        // 10Hz 调试打印
        static unsigned long last_print = 0;
        if (millis() - last_print >= 100) {
          last_print = millis();
          const char *cmd_name = (cmd == CMD_WALK) ? "走路" : "平衡";
          Serial.printf("PID %s: P=%.2f Tgt=%.2f→%.2f V=%.2f\n",
                        cmd_name, target, pitch,
                        target_vel);
        }
      }

      update_led();
    }
  }

  // 超时: 500ms 没收到帧 → 停电机
  if (millis() - last_frame_ms > 500 && last_frame_ms != 0) {
    static unsigned long last_timeout_print = 0;
    if (millis() - last_timeout_print > 1000) {
      last_timeout_print = millis();
      Serial.println("警告: 鲁班猫数据超时, 电机停机");
    }
    if (motor_enabled) {
      motor_enabled = false;
      stop_motor();
    }
  }
}
