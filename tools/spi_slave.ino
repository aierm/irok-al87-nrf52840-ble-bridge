#include <SPI.h>

const int CS_PIN = PA4;
static const uint32_t SPI_FREQ = 100000;

struct KeyStep {
  const char *name;
  uint8_t modifier;
  uint8_t keycode;
  uint16_t hold_ms;
  uint16_t gap_ms;
};

static const KeyStep kSequence[] = {
  {"a", 0x00, 0x04, 20, 120},
  {"b", 0x00, 0x05, 20, 120},
  {"c", 0x00, 0x06, 20, 120},
  {"A", 0x02, 0x04, 20, 120},
  {"1", 0x00, 0x1E, 20, 120},
  {"2", 0x00, 0x1F, 20, 120},
  {"3", 0x00, 0x20, 20, 120},
  {"space", 0x00, 0x2C, 20, 120},
  {"enter", 0x00, 0x28, 20, 220},
  {"tab", 0x00, 0x2B, 20, 220},
  {"backspace", 0x00, 0x2A, 20, 220},
};

void printHex8(const uint8_t *buf) {
  for (int i = 0; i < 8; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    if (i != 7) Serial.print(' ');
  }
}

void spiSendReport(const uint8_t tx_report[8], const char *tag) {
  uint8_t rx_report[8] = {0};

  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(2);

  for (int i = 0; i < 8; i++) {
    rx_report[i] = SPI.transfer(tx_report[i]);
  }

  delayMicroseconds(2);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  Serial.print(tag);
  Serial.print("  TX: ");
  printHex8(tx_report);
  Serial.print("  RX: ");
  printHex8(rx_report);

  if (rx_report[0] == 0xA5 && rx_report[1] == 0x5A) {
    uint16_t cnt = (uint16_t)rx_report[2] | ((uint16_t)rx_report[3] << 8);
    Serial.print("  ACK cnt=");
    Serial.print(cnt);
  } else {
    Serial.print("  (No ACK header)");
  }
  Serial.println();
}

void sendKeyTap(uint8_t modifier, uint8_t keycode, const char *name, uint16_t hold_ms) {
  uint8_t press[8] = {modifier, 0x00, keycode, 0, 0, 0, 0, 0};
  static const uint8_t release[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  Serial.print("KEY=");
  Serial.println(name);
  spiSendReport(press, "PRESS  ");
  delay(hold_ms);
  spiSendReport(release, "RELEASE");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  Serial.println("STM32 SPI HID sender ready");
  Serial.println("Auto loop: a b c A 1 2 3 space enter tab backspace");
}

void loop() {
  static size_t index = 0;
  const KeyStep &step = kSequence[index];
  sendKeyTap(step.modifier, step.keycode, step.name, step.hold_ms);
  delay(step.gap_ms);
  index = (index + 1) % (sizeof(kSequence) / sizeof(kSequence[0]));
}
