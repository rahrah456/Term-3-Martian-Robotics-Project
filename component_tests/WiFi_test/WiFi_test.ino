#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

// WiFi credentials
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;

// Target (your computer)
const char* targetIP = "172.20.10.3";
const int targetPort = 4210;

// UDP
WiFiUDP udp;
const int localPort = 4210;

// Buffer
char incomingPacket[255];

//Death Pin
const int activation = 8; 

void setup() {
  Serial.begin(9600);
  delay(1000); // give serial time to start

  pinMode(activation, OUTPUT);

  initWiFi();

  Serial.print("UDP listening on port ");
  Serial.println(localPort);
}

void loop() {
  String msg = receiveUDP();

  if (msg.length() > 0) {
    Serial.print("Received: ");
    Serial.println(msg);

    if (msg == "Die") {
      sendUDP("Dead");
      digitalWrite(activation, HIGH);
    }
    else if (msg == "Live") {
      sendUDP("Alive");
      digitalWrite(activation, LOW);
    }
    else{
      sendUDP("Huh?");
    }
  }

  delay(10);
}

void initWiFi() {
  Serial.print("Connecting to WiFi");

  WiFi.begin(ssid, password);

  // Wait until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");

  IPAddress ip = WiFi.localIP();

  while (ip == IPAddress(0, 0, 0, 0)) {
    Serial.println("Waiting for valid IP...");
    delay(500);
    ip = WiFi.localIP();
  }

  Serial.print("Stable IP Address: ");
  Serial.println(ip);

  udp.begin(localPort);
}

String receiveUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;

    return String(incomingPacket);
  }
  return "";
}

void sendUDP(const String& message) {
  udp.beginPacket(targetIP, targetPort);
  udp.print(message);
  udp.endPacket();
}