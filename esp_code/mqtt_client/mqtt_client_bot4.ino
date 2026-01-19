#include "WiFi.h"
#include "PubSubClient.h"
#include <string.h>  // For strcmp
#include <ESP32Servo.h>
#include <Arduino.h>

const char* ssid = "nithish_reddy";       //WiFi SSID
const char* password = "xxxxxxxxx";  //WiFi password
const char* broker_ip = "10.186.39.156"; //Broker's IP
const int broker_port = 1883;

#define CLIENT_ID "ESPClient4"
#define CMD_TOPIC "bot_cmd/4"
#define SENSOR_TOPIC "esp/sensor/4"
#define SOL_TOPIC "esp/sol/4"

Servo servo1; 
Servo servo2; 
Servo servo3; 
Servo micro_servo1;
Servo micro_servo2;

// Defining pins
#define servo1_pin  27
#define servo2_pin  26
#define servo3_pin  25
#define micro_servo1_pin 18
#define micro_servo2_pin 5

const int SOL_PIN = 23;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 1000; 
const int PWM_RES = 8; 



#define IR_PIN 15


WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup_wifi() {
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {  // ~10s timeout
        Serial.println(F("Waiting for hotspot..."));
        delay(500);
        attempts++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WiFi connection failed! Restarting..."));
        ESP.restart();  // Restart on failure
    }
    Serial.println(F("WiFi connected."));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    Serial.print(". Message: ");
    String message;

    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
        message += (char)payload[i];
    }

    if (strcmp(topic, CMD_TOPIC) == 0) {
        Serial.print("Command received: ");
        Serial.println(message);
        int v1, v2, v3, mv1, mv2;
        sscanf(message.c_str(), "%d,%d,%d,%d,%d",&v1, &v2, &v3, &mv1, &mv2);
        servo1.writeMicroseconds(v1);
        servo2.writeMicroseconds(v2);
        servo3.writeMicroseconds(v3);
        micro_servo1.write(mv1);
        micro_servo2.write(mv2);

        Serial.print("Wheel1: "); Serial.println(v1);  
        Serial.print("Wheel2: "); Serial.println(v2); 
        Serial.print("Wheel3: "); Serial.println(v3); 

        Serial.print("u_servo1: "); Serial.println(mv1); 
        Serial.print("u_servo2: "); Serial.println(mv2); 
    }

    if (String(topic) == SOL_TOPIC) {
        Serial.print("Command received: ");
        Serial.println(message);
        int v1;
        sscanf(message.c_str(), "%d",&v1);
        ledcWrite(SOL_PIN, v1); 
        Serial.print("Solenoid PWM: "); Serial.println(v1); 
    }
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(CLIENT_ID)) {
            Serial.println("connected");
            mqttClient.subscribe(CMD_TOPIC);
            mqttClient.subscribe(SOL_TOPIC);
            Serial.println(F("Subscribed to command topic."));
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void publishSensorData() {
    int state = digitalRead(IR_PIN); 
    String payload = String(state, 2);
    mqttClient.publish(SENSOR_TOPIC, payload.c_str());
    Serial.print(": ");
    Serial.println(payload);
}

void servo_init(){
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  micro_servo1.attach(micro_servo1_pin);
  micro_servo2.attach(micro_servo2_pin);
}


void setup() {
    Serial.begin(115200);
    pinMode(IR_PIN, INPUT_PULLUP);
    setup_wifi();
    servo_init();

    ledcAttach(SOL_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(SOL_PIN, 0);

    mqttClient.setServer(broker_ip, broker_port);
    mqttClient.setCallback(mqttCallback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 2000) {
        lastMsg = millis();
        publishSensorData();
    }
}

