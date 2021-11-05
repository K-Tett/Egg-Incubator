/**
 * TODO:
 * - [ ] Complete the code for sensor and relay control
 * - [ ] Add kalman filtering to improve the sensor reading
 * - [ ] Sleep code
 * - [ ] serial print when debugging
 * - [ ] Connect to wifi
 * - [ ] Setup the MQTT protocol to communicate with PI
 * - [ ] Tune the control 
 */ 

#include <Arduino.h>
#include "esp_wifi.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Stepper.h>

const char* ssid = "Your ssid";
const char* password = "password";
const char* mqtt_server = "your ip";
const char* ca_cert = "";

#define mqtt_port 8883

float humidity;
float temperature;
float heatIndex;
bool light_status;
bool fan_status;
bool stepper_motor_status = false;
const int microstep_per_revolution = 2048;
const int rounds_per_minutes = 12;
Stepper stepper_motor = Stepper(microstep_per_revolution, pin_x, pin_y, pin_z, pin_n);

#define DHTType DHT22
#define DHT22Pin
#define relay_pin
#define uS_TO_S_FACTOR 
#define TIME_TO_SLEEP

//Print serialprint easier way
void StreamPrint_progmem(Print &out,PGM_P format,...)
{
  // program memory version of printf - copy of format string and result share a buffer
  // so as to avoid too much memory use
  char formatString[128], *ptr;
  strncpy_P( formatString, format, sizeof(formatString) ); // copy in from program mem
  // null terminate - leave last char since we might need it in worst case for result's \0
  formatString[ sizeof(formatString)-2 ]='\0'; 
  ptr=&formatString[ strlen(formatString)+1 ]; // our result buffer...
  va_list args;
  va_start (args,format);
  vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString)-1 ]='\0'; 
  out.print(ptr);
}
#define Serialprint(format, ...) StreamPrint_progmem(Serial,PSTR(format),##__VA_ARGS__)

void relay(){

  return
}

void stepper_start(){
  //Move stepper motor to 75 degrees angle
  stepper_motor.step(microstep_per_revolution / 4.8); 
  
  return
}

void Sensor_Reading(){
  

  return 
}

//Print in the serial monitor (disable if wifi)
void serialPrintFunction(){
  Serialprint("The temperature of the Incubator: %fC\n", temperature)
  Serialprint("The humidity of the incubator: %f%%\n", humidity)
  Serialprint("Feel like: %.2fC\n", heatIndex)
  Serialprint("The light status is on:", light_status)
}

//Inital setup to connect to router
void setup_wifi(){
  delay(10);
  Serialprint();
  Serialprint("Connecting to ");
  Serialprint(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serialprint(".");
    delay(100);
  }
  randomSeed(micros());
  Serialprint("");
  Serialprint("WiFi connected");
  Serialprint("IP address: ");
  Serialprint(WiFi.localIP());
  
  client.setCaCert(ca_cert);
}

//Reconnect if initial connection failed
void reconnect(){
    // Loop until we're reconnected
  while (!client.connected()) {
    Serialprint("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serialprint("connected");
      //Once connected, publish an announcement...
      client.publish("/icircuit/presence/ESP32/", "hello world");
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serialprint("failed, rc=");
      Serialprint(client.state());
      Serialprint(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode();
  dht.begin();
  stepper_motor.setSpeed(rounds_per_minutes);

  setup_wifi();
  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);
  reconnect();

}

void loop() {
  esp_wifi_start();
  
  //functions start
  Sensor_Reading();
  stepper_start();
  serialPrintFunction();

  esp_wifi_stop();

  esp_sleep_enable_timer_wakeup(TIME_TOSLEEP * uS_TO_S_FACTOR)
  esp_deep_sleep_start();
}