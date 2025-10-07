#include <Arduino.h>
#include <FilterBase.h>
#include <IMU.h>
#include <NoFilter.h>
#include <LowPassFilter.h>
#include <ComplementaryFilter.h>
#include <WiFi.h>
#include <MadgwickFilter.h>
#include <MahonyFilter.h>
#include <ExtendedKF.h>

IMU imu(21, 22, 25);

NoFilter noFilter;

LowPassFilter lowPassFilter(10.0f);
ComplementaryFilter complementaryFilter(1.0f);
MadgwickFilter madgwickFilter(0.01f);
MahonyFilter mahonyFilter(1.5f, 0.05f);
// EKF
BLA::Matrix<6, 6> Q = {2e-4f, 0, 0, 0, 0, 0,
                       0, 2e-4f, 0, 0, 0, 0,
                       0, 0, 2e-4f, 0, 0, 0,
                       0, 0, 0, 4e-8f, 0, 0,
                       0, 0, 0, 0, 4e-8f, 0,
                       0, 0, 0, 0, 0, 4e-8f};
BLA::Matrix<3, 3> R = {0.0025f, 0, 0,
                       0, 0.0025f, 0,
                       0, 0, 0.0025f};
BLA::Matrix<7, 7> P = BLA::Eye<7, 7>() * 1e-4f;
ExtendedKF EKF(Q, R, P);

// pointers to generic filter base class
FilterBase* filter1 = nullptr;
FilterBase* filter2 = nullptr;

// setting up WiFi to stream data to PC
// using UDP
const char* ssid = "Hosseini";
const char* password = "Tofigh1377";
WiFiUDP udp;
// IP of the PC (receiver)
const char* hostIP = "192.168.1.21";
// port for UDP (can be 1024-65535)
const int udpPort = 3333; 

// PACKET LAYOUT 
// 1 byte header 0xAA
// 4 bytes sequence 
// 4 bytes timestamp - micros() truncated to 32bits
// 8 floats = 32 bytes (filter data)
const size_t SAMPLE_BYTES = 1 + 4 + 4 + 32; // header + seq + ts + 8 floats

uint32_t seq = 0;
uint32_t lastMicros = 0;

// we send a message from python script
// [0xAA][1 byte filterConfig]
// High nibble (bits 7–4) → filter1 ID
// Low nibble (bits 3–0) → filter2 ID
// 0x0 -> NoFilter
// 0x1 -> Lowpass
// 0x2 -> complementary
// 0x3 -> Madgwick
// 0x4 -> Mahony
// 0x5 -> EKF
void selectFilters(uint8_t config) {
  uint8_t f1 = (config >> 4) & 0x0F;
  uint8_t f2 = config & 0x0F;

  switch (f1) {
    case 0x0: filter1 = &noFilter; break;
    case 0x1: filter1 = &lowPassFilter; break;
    case 0x2: filter1 = &complementaryFilter; break;
    case 0x3: filter1 = &madgwickFilter; break;
    case 0x4: filter1 = &mahonyFilter; break;
    case 0x5: filter1 = &EKF; break;
    default: filter1 = &noFilter; break;
  }

  switch (f2) {
    case 0x0: filter2 = &noFilter; break;
    case 0x1: filter2 = &lowPassFilter; break;
    case 0x2: filter2 = &complementaryFilter; break;
    case 0x3: filter2 = &madgwickFilter; break;
    case 0x4: filter2 = &mahonyFilter; break;
    case 0x5: filter2 = &EKF; break;
    default: filter2 = &noFilter; break;
  }
}

// we will wait for filter config to be sent to ESP32
// from python script. When received we will set filter config
// then we send ACK back to python script for handshake
void waitForConfig() {
  Serial.println("Waiting for filter config...");
  uint8_t buf[2];
  while(true) {
    int len = udp.parsePacket();
    if (len == 2) {
      udp.read(buf, 2);
      if (buf[0] == 0xAA) { // header
        selectFilters(buf[1]); 
        // send ACK
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(0x55);
        udp.endPacket();
        Serial.println("Filter Config Received.");
        break;
      }
    }
    delay(10);
  }
}

void setup() {
  Serial.begin(921600);
  // begining wifi connection
  WiFi.begin(ssid, password);
  // waiting for wifi to connect
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(1234);
  waitForConfig();
  if (!imu.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }
  imu.setConfig(0x47, 0x07, 4.0f, 2000.f);
  lastMicros = micros();
}

void loop() {
  // reading imu at its own pace
  if (!imu.available()) return;
  // after reading the first data
  // we batch bunch of data with timestamps 
  // publish them over UDP
  unsigned long now = micros();
  ImuData d = imu.readScaled();
  float dt = (now - lastMicros) / 1e6f;
  lastMicros = now;
  filter1->update(d, dt);
  filter2->update(d, dt);

  Quaternion q_1 = filter1->getQuaternion();
  Quaternion q_2 = filter2->getQuaternion();
  uint8_t packetBuf[SAMPLE_BYTES];
  size_t offset = 0;

  // header
  packetBuf[offset++] = 0xAA;
  // sequence number (little endian)
  uint32_t seqLE = seq;
  memcpy(packetBuf + offset, &seqLE, sizeof(seqLE));
  offset += sizeof(seqLE);
  // timestamp
  uint32_t ts = (uint32_t)(micros() & 0xFFFFFFFFUL);
  memcpy(packetBuf + offset, &ts, sizeof(ts));
  offset += sizeof(ts);
  // 8 floats of filtered data
  memcpy(packetBuf + offset, &q_1.q0, sizeof(float)); offset += sizeof(float);
  memcpy(packetBuf + offset, &q_1.q1, sizeof(float)); offset += sizeof(float);
  memcpy(packetBuf + offset, &q_1.q2, sizeof(float)); offset += sizeof(float);
  memcpy(packetBuf + offset, &q_1.q3, sizeof(float)); offset += sizeof(float);
  memcpy(packetBuf + offset, &q_2.q0, sizeof(float)); offset += sizeof(float);
  memcpy(packetBuf + offset, &q_2.q1, sizeof(float)); offset += sizeof(float);
  memcpy(packetBuf + offset, &q_2.q2, sizeof(float)); offset += sizeof(float);
  memcpy(packetBuf + offset, &q_2.q3, sizeof(float)); offset += sizeof(float);

  // converting host IP address from string
  IPAddress remoteIP;
  if (!remoteIP.fromString(hostIP)) {
    Serial.println("Invalid IP adress.");
    return;
  }

  // sending udp packet
  udp.beginPacket(remoteIP, udpPort);
  udp.write(packetBuf, SAMPLE_BYTES);
  if (!udp.endPacket()) {
    Serial.println("Failed to send packet");
  }

  // iterating seq
  seq++;
}
