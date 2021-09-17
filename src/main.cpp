#include <Arduino.h>
#include "esp_wifi.h"
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Your ssid";
const char* password = "password";
const char* mqtt_server = "your ip";
const char* ca_cert = "";

#define mqtt_port 8883

float humidity
float temperature
float heatIndex
bool sound;
bool light_status
float fan_speed
float servo_angle

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}