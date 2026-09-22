// rs485test.ino — RS485 通信测试（宇树 Go1 电机）
// 功能：定时发送控制帧，打印发送字节 + 读取反馈

#define RS485_TX_PIN 1
#define RS485_RX_PIN 2
#define RS485_DE_PIN 15
#define MOTOR_ID     0
#define BAUDRATE     4000000  // 4Mbps

// ===== 宇树 Go1 控制帧 (17 字节) =====
// 实际字段定义请对照官方协议文档
typedef struct __attribute__((packed)) {
  uint8_t  head1;    // 0xFE
  uint8_t  head2;    // 0xFE
  uint8_t  id;       // 电机 ID
  uint8_t  cmd;      // 0x14 = 控制命令
  uint8_t  data[10]; // 位置/速度/力矩/KP/KD (定标后)
  uint16_t crc;      // CRC16
} MotorFrame;

MotorFrame frame;
uint8_t tx_buf[17];
uint8_t rx_buf[64];
uint32_t send_count = 0;

// ===== CRC16-CCITT (宇树用) =====
uint16_t crc16(const uint8_t *data, int len) {
  uint16_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ===== 构建控制帧 =====
void build_frame(uint8_t motor_id, float target_pos, float target_vel) {
  frame.head1 = 0xFE;
  frame.head2 = 0xFE;
  frame.id    = motor_id;
  frame.cmd   = 0x14;
  // 简化：把数据填 0，实际要按官方定标填
  for (int i = 0; i < 10; i++) frame.data[i] = 0;
  // 计算 CRC（不含 head 和 crc 本身）
  memcpy(tx_buf, &frame, 15);
  uint16_t crc = crc16(tx_buf + 2, 13);  // 从 id 开始算
  tx_buf[15] = crc & 0xFF;
  tx_buf[16] = (crc >> 8) & 0xFF;
}

// ===== 发送控制帧 =====
void send_motor_cmd(uint8_t motor_id, float target_pos, float target_vel) {
  build_frame(motor_id, target_pos, target_vel);

  // ===== 调试：打印发送字节 =====
  Serial.print("RS485 TX #");
  Serial.print(send_count++);
  Serial.print(":");
  for (int i = 0; i < 17; i++) {
    Serial.printf(" %02X", tx_buf[i]);
  }
  Serial.println();

  // 发送
  digitalWrite(RS485_DE_PIN, HIGH);   // 发送模式
  delayMicroseconds(1);
  Serial1.write(tx_buf, 17);
  Serial1.flush();
  digitalWrite(RS485_DE_PIN, LOW);    // 切回接收模式
  delayMicroseconds(50);

  // ===== 读取电机反馈 =====
  int n = Serial1.available();
  if (n > 0) {
    int len = Serial1.readBytes(rx_buf, min(n, 64));
    Serial.print("RS485 RX (");
    Serial.print(len);
    Serial.print("):");
    for (int i = 0; i < len; i++) {
      Serial.printf(" %02X", rx_buf[i]);
    }
    Serial.println();
  } else {
    Serial.println("RS485 RX: 无反馈");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // RS485 串口
  Serial1.begin(BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);  // 默认接收模式

  Serial.println("=== RS485 通信测试 ===");
  Serial.printf("波特率: %d bps\n", BAUDRATE);
  Serial.printf("TX=GPIO%d RX=GPIO%d DE=GPIO%d\n", RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN);
  Serial.println("每秒发送一次控制帧...");
  Serial.println();
}

void loop() {
  // 每 1 秒发一次，位置目标在 0 和 1.0 之间切换
  static uint32_t last_send = 0;
  static bool toggle = false;

  if (millis() - last_send >= 1000) {
    last_send = millis();
    toggle = !toggle;
    float target = toggle ? 1.0 : 0.0;

    Serial.printf("\n--- 第 %d 次发送, 目标位置 %.2f ---\n", send_count, target);
    send_motor_cmd(MOTOR_ID, target, 0.0);
  }
}
