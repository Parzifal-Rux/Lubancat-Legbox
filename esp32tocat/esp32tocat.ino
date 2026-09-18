#include <Adafruit_NeoPixel.h>

// ESP32-S3 DevKitC-1 板载 WS2812 LED 接在 GPIO48
#define LED_PIN 48
#define LED_COUNT 1

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    Serial.begin(115200);
    strip.begin();
    strip.show();               // 初始化关灯
    delay(500);
    Serial.println("ESP32 ready");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        Serial.print("[recv] ");
        Serial.println(cmd);

        if (cmd == "BLUE_ON") {
            Serial.println("Blue LED ON for 3 seconds");
            // 蓝色：R=0, G=0, B=255
            strip.setPixelColor(0, strip.Color(0, 0, 255));
            strip.show();
            delay(3000);
            strip.clear();
            strip.show();
            Serial.println("Blue LED OFF");
        } else {
            Serial.println("unknown cmd");
        }
    }
}