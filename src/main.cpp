/**
 * TODO:
 * - [x] Complete the code for sensor and relay control
 * - [x] Add kalman filtering to improve the sensor reading
 * - [x] Sleep code
 * - [x] serial print when debugging
 * - [x] Connect to wifi
 * - [x] Setup the MQTT protocol to communicate with PI
 * - [ ] Tune the control 
 * - [X] RTC memory for stepper motor
 * - [ ] Send message over the wifi for the node red
 * - [X] Hold pin
 * - [ ] Use HTTPS protocol (optional)
 * ex:
 * https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFiClientSecure/examples/WiFiClientSecure/WiFiClientSecure.ino
 * https://www.iotsharing.com/2017/08/how-to-use-https-in-arduino-esp32.html
 */ 

#include <Arduino.h>
#include <math.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Stepper.h>
#include <DHT.h>
#include <SimpleKalmanFilter.h>

PubSubClient client;

const char* ssid = "OZ_HOME1_2G";
const char* password = "0859078228";
const char* mqtt_server = "192.168.1.187";

#define mqtt_port 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_SERIAL_RECEIVER_CH ""

RTC_DATA_ATTR unsigned int record_counter = 0;
RTC_DATA_ATTR unsigned int eggs_turn_counter = 0;
RTC_DATA_ATTR unsigned int embryo_rest_counter = 0;
float humidity;
float temperature;
float heatIndex;
bool light_status;
bool fan_status;
bool stepper_motor_status = false;
const int microstep_per_revolution = 2048;
const int rounds_per_minutes = 12;

char temperature_string[8];
char humidity_string[8];
char heat_index_string[8];
char light_status_string[4];
char fan_status_string[4];

#define ledPin GPIO_NUM_2
#define DHTType DHT22
#define DHT22Pin GPIO_NUM_34
#define light_relay_pin_1 GPIO_NUM_35
#define fan1_relay_pin_2 GPIO_NUM_32
#define fan2_relay_pin_3 GPIO_NUM_33
#define stepper_pin_1 GPIO_NUM_25
#define stepper_pin_2 GPIO_NUM_26
#define stepper_pin_3 GPIO_NUM_27
#define stepper_pin_4 GPIO_NUM_14
#define uS_TO_S_FACTOR 1000000 //Conversion factor for micro second to second
#define TIME_TO_SLEEP 60 // Time ESP32 will go to sleep in seconds

DHT dht(DHT22Pin, DHTType);

Stepper stepper_motor = Stepper(microstep_per_revolution, stepper_pin_1, stepper_pin_2, stepper_pin_3, stepper_pin_4);

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
  light_status = digitalRead(light_relay_pin_1);
  fan_status = digitalRead(fan1_relay_pin_2);
  //Wire fans and light to normally closed relay
  if(temperature>38){
    //Turn OFF
    if (light_status != HIGH) {
      gpio_hold_dis(light_relay_pin_1);
    }
    digitalWrite(light_relay_pin_1, HIGH);
    gpio_hold_en(light_relay_pin_1);
  } else if (temperature<36){
    //Turn On
    if (light_status != LOW) {
      gpio_hold_dis(light_relay_pin_1);
    }
    digitalWrite(light_relay_pin_1, LOW);
    gpio_hold_en(light_relay_pin_1);
  }
  if(humidity>53){
    //Turn On
    if (fan_status != LOW){
      gpio_hold_dis(fan1_relay_pin_2);
      gpio_hold_dis(fan2_relay_pin_3);
    }
    digitalWrite(fan1_relay_pin_2, LOW);
    digitalWrite(fan2_relay_pin_3, LOW);
    gpio_hold_en(fan1_relay_pin_2);
    gpio_hold_en(fan2_relay_pin_3);
  } else if (humidity<44){
    //Turn Off
    if (fan_status != HIGH){
      gpio_hold_dis(fan1_relay_pin_2);
      gpio_hold_dis(fan2_relay_pin_3);
    }
    digitalWrite(fan1_relay_pin_2, HIGH);
    digitalWrite(fan2_relay_pin_3, HIGH);
    gpio_hold_en(fan1_relay_pin_2);
    gpio_hold_en(fan2_relay_pin_3);
  }
  light_status = digitalRead(light_relay_pin_1);
  fan_status = digitalRead(fan1_relay_pin_2);

  return;
}

void stepper_start(){
  //Move stepper motor to 75 degrees angle
  stepper_motor.step(microstep_per_revolution / 4.8); 

  return;
}

void Sensor_Reading(){
  temperature = dht.readTemperature(false);//Read temperature in celsius
  humidity = dht.readHumidity();
  heatIndex = dht.computeHeatIndex(temperature, humidity, false);//Feels like temperature

  return ;
}

//Print in the serial monitor (disable if wifi)
void serialPrintFunction(){
  Serialprint("The temperature of the Incubator: %fC\n", temperature);
  Serialprint("The humidity of the incubator: %f%%\n", humidity);
  Serialprint("Feel like: %.2fC\n", heatIndex);
  Serialprint("The light status is High/Low(OFF/ON): %c\n", light_status);
  Serialprint("The fan status is High/Low(OFF/ON): %c\n", fan_status);

  return;
}

//Inital setup to connect to router
void setup_wifi(){
  delay(10);

  //connecting to the wifi
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
  Serial.print(WiFi.localIP());
}

//Callback function
void callback(char* topic, byte* message, unsigned int length) {
  Serialprint("Message arrived on topic: ");
  Serialprint(topic);
  Serialprint(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serialprint("on\n");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serialprint("off\n");
      digitalWrite(ledPin, LOW);
    }
  }
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
      Serial.print(client.state());
      Serialprint(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//Client publishing data to MQTT broker
void client_publish(){
  client.publish("esp32/temperature", temperature_string);
  client.publish("esp32/humidity", humidity_string);
  client.publish("esp32/heat_index", heat_index_string);
  client.publish("esp32/light_status", light_status_string);
  client.publish("esp32/fan_status", fan_status_string);
  
  return;
}

void setup() {
  Serial.begin(115200);

  pinMode(light_relay_pin_1, OUTPUT);
  pinMode(fan1_relay_pin_2, OUTPUT);
  pinMode(fan2_relay_pin_3, OUTPUT);
  pinMode(ledPin, OUTPUT);
  stepper_motor.setSpeed(rounds_per_minutes);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
}

void loop() {

  if (!client.connected()){
    reconnect();
  }
  client.loop();

  esp_wifi_start();
  
  //functions start
  Sensor_Reading();
  relay();
  if (embryo_rest_counter < 180){
    if (eggs_turn_counter < 3){
      if (record_counter >= 36){
        stepper_start();
        record_counter = 0;
        eggs_turn_counter++;
      }
    } else {
      eggs_turn_counter = 0;
    }
  } else {
    embryo_rest_counter = 0;
  }
  client_publish();
  serialPrintFunction();

  record_counter++;

  esp_wifi_stop();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  gpio_deep_sleep_hold_en();
  esp_deep_sleep_start();
}