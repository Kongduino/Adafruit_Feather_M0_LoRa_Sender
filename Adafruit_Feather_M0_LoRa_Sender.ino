#include <SPI.h>
#include <LoRa.h>
#include<Arduino.h>

// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

#define SS      8
#define RST     4
#define DI0     3
#define BAND    433.250E6
#define REG_OCP 0x0B
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_OP_MODE 0x01
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_MODEM_CONFIG_3 0x26
#define REG_PA_DAC 0x4D
#define PA_DAC_HIGH 0x87
#define REG_PA_DAC 0x4D
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

int counter = 0;
uint8_t ix = 0;
char buff[256];
String output = "";
double t0, t2;

void hexDump(unsigned char *buf, uint16_t len) {
  String s = "|", t = "| |";
  Serial.println(F("  |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f |"));
  Serial.println(F("  +------------------------------------------------+ +----------------+"));
  for (uint16_t i = 0; i < len; i += 16) {
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j >= len) {
        s = s + "   "; t = t + " ";
      } else {
        char c = buf[i + j];
        if (c < 16) s = s + "0";
        s = s + String(c, HEX) + " ";
        if (c < 32 || c > 127) t = t + ".";
        else t = t + (char)c;
      }
    }
    uint8_t index = i / 16;
    Serial.print(index, HEX); Serial.write('.');
    Serial.println(s + t + "|");
    s = "|"; t = "| |";
  }
  Serial.println(F("  +------------------------------------------------+ +----------------+"));
}

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  while (!Serial); //If just the the basic function, must connect to a computer
  SPI.begin();
  LoRa.setPins(SS, RST, DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  // 0x72: BW = 7: 125 kHz, CR = 1: 4/5, HM = 0
  uint8_t reg1 = 0x48; // 0x48: BW 4, 31.25 kHz; CR 4/8
  // 0x70: SF = 7: 7, CRC = 0
  uint8_t reg2 =  0x94; // 0x94: SF 9; CRC enabled;
  // 0x08: LDRO = 1, AGCAutoOn = 0
  uint8_t reg3 = 0x0C; // 0x0C: LNA gain set by the internal AGC loop; LowDataRateOptimize Enabled
  // PaSelect = 1, MaxPower = 7: 15 dBm, OutputPower = 15: 17 dBm
  uint8_t regpaconfig = 0xFF;
  // 7:5 LnaGain 001 -> G1 = maximum gain
  // 1:0 LnaBoostHf 11 -> Boost on, 150% LNA current
  uint8_t reglna = 0b00100011;
  LoRa.setTxPower(17);
  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
  LoRa.writeRegister(REG_PA_CONFIG, 0xFF);
  LoRa.writeRegister(REG_MODEM_CONFIG_1, reg1);
  LoRa.writeRegister(REG_MODEM_CONFIG_2, reg2);
  LoRa.writeRegister(REG_MODEM_CONFIG_3, reg3);
  LoRa.writeRegister(REG_LNA, reglna);
  LoRa.writeRegister(REG_PA_DAC, REG_PA_DAC); // That's for the receiver
  LoRa.writeRegister(REG_OCP, 0b00111111);
  delay(10);
  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  delay(10);
  LoRa.setCodingRate4(8);
  Serial.println(F("LoRa init succeeded."));
  t0 = millis();
  t2 = millis();
}

void loop() {
  double t3 = millis() - t2;
  if (t3 > 999) digitalWrite(13, LOW);
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    memset(buff, 0, 256);
    // received a packet
    // print RSSI of packet
    Serial.print(F("Received packet with RSSI "));
    int rssiLvl = LoRa.packetRssi();
    Serial.println(rssiLvl);
    ix = 0;
    output = "";
    while (LoRa.available()) {
      char c = LoRa.read();
      buff[ix++] = c;
    }
    hexDump((unsigned char*)buff, ix);
  }
  double t1 = millis() - t0;
  if (t1 > 2999) {
    digitalWrite(13, HIGH);
    t2 = millis();
    Serial.print("Sending packet: ");
    Serial.println(counter);
    memset(buff, 0, 256);
    sprintf(buff, "{\"from\":\"Feather M0\", \"msg\":\"Hi there!\", \"sendCount\":\"%d\"}", counter++);
    Serial.print("Sending "); Serial.println(buff);
    delay(10);
    LoRa.beginPacket();
    LoRa.print(buff);
    LoRa.endPacket();
    t0 = millis();
  }
}
