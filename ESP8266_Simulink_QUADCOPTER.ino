#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "F";
const char* password = "12345678";

WiFiUDP udp;
unsigned int localUdpPort = 4210; // Receiving T

// IP and port of the PC running Simulink
IPAddress pcIP(172, 20, 10, 3);
unsigned int pcPort = 5005; // Send z here

// Physics variables
double T = 0;      // Thrust (received)
double z = 0;      // Altitude
double vz = 0;     // Vertical velocity

// Timing
unsigned long lastUpdate = 0;
const unsigned long physicsInterval = 10; // ms

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ Connected to WiFi!");
  Serial.print("📡 ESP IP Address: ");
  Serial.println(WiFi.localIP());

  udp.begin(localUdpPort);
  Serial.printf("🟢 Listening on UDP port %d\n", localUdpPort);

  lastUpdate = millis();
}

void loop() {
  // ✅ 1. Check for new T from Simulink
  int packetSize = udp.parsePacket();
  if (packetSize == 8) {
    byte buffer[8];
    udp.read(buffer, 8);
    memcpy(&T, buffer, 8);

    Serial.print("📥 Received T = ");
    Serial.println(T, 3);
  }

  // ✅ 2. Run physics & send z at fixed intervals
  unsigned long now = millis();
  if (now - lastUpdate >= physicsInterval) {
    double dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    // Clamp dt to avoid rare glitches
    if (dt < 0.005 || dt > 0.05) {
      dt = 0.01;
    }

    // 🚀 Physics simulation
    double m = 2.0;
    double g = 9.81;
    double a = (T - m * g) / m;

    // Integrate velocity & position
    vz += a * dt;
    z += vz * dt;

    // Clamp
    if (z < 0) {
      z = 0;
      vz = 0;
    }
    if (z > 1000) z = 1000;
    if (fabs(vz) > 100) vz = 0;

    // Debug
    Serial.print("📤 Sending z = ");
    Serial.println(z, 3);

    // ✅ 3. Send z to Simulink
    byte zBytes[8];
    memcpy(zBytes, &z, 8);
    udp.beginPacket(pcIP, pcPort);
    udp.write(zBytes, 8);
    udp.endPacket();
  }

  // No delay — stay responsive
}