#include <WiFiS3.h>
#include <WiFiUdp.h>

const char* ssid = "wip13";
const char* password = "123456789";
const char* laptopIP = "172.20.10.3";
const int PORT = 4210;

const int ACT_LED = 39;

WiFiUDP udp;
char buf[255];
bool killed = false;

void setup() {
  Serial.begin(9600);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, LOW);

  connectWiFi();
  udp.begin(PORT);
  Serial.println("WiFi kill switch test — send 'Stop' or 'Go' via UDP");
}

void loop() {
  String msg = receive();
  if (msg.length() > 0) {
    Serial.print("Got: ");
    Serial.println(msg);

    if (msg == "Stop") {
      killed = true;
      digitalWrite(ACT_LED, HIGH);
      send("Stopped");
      Serial.println("KILLED — red LED flashing");
    } else if (msg == "Go") {
      killed = false;
      digitalWrite(ACT_LED, LOW);
      send("Running");
      Serial.println("ALIVE — normal operation");
    } else {
      send("?");
    }
  }

  delay(10);
}

void connectWiFi() {
  Serial.print("Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

String receive() {
  int size = udp.parsePacket();
  if (size) {
    int len = udp.read(buf, 255);
    if (len > 0) buf[len] = 0;
    return String(buf);
  }
  return "";
}

void send(const String& msg) {
  udp.beginPacket(laptopIP, PORT);
  udp.print(msg);
  udp.endPacket();
}
