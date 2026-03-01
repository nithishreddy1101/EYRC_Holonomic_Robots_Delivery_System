/*
 * Team Id: 1060
 * Author List: A. Nithish , Rohan A Khamitkar 
 * Filename: esp32_mqtt_servo_control.ino
 * Theme: Mobile Robot / IoT Control
 * Functions: setup_wifi(), mqttCallback(), reconnect(), publishSensorData(),
 *            servo_init(), setup(), loop()
 * Global Variables: ssid, password, broker_ip, broker_port,
 *                   mqttClient, servo objects, pin definitions
 */

#include "WiFi.h"           // ESP32 WiFi library
#include "PubSubClient.h"   // MQTT client library
#include <string.h>         // For strcmp()
#include <ESP32Servo.h>     // Servo control for ESP32
#include <Arduino.h>        // Arduino core definitions

/* -------------------- WiFi Credentials -------------------- */
// ssid: Name of the WiFi hotspot to connect to
const char* ssid = "nithish_reddy";

// password: Password of the WiFi hotspot
const char* password = "xxxxxxxxx";

/* -------------------- MQTT Broker Details -------------------- */
// broker_ip: IP address of MQTT broker
const char* broker_ip = "10.37.195.156";

// broker_port: Port number of MQTT broker (default: 1883)
const int broker_port = 1883;

/* -------------------- MQTT Topics & Client ID -------------------- */
#define CLIENT_ID     "ESPClient0"
#define CMD_TOPIC     "bot_cmd/0"     // Topic for servo commands
#define SENSOR_TOPIC  "esp/sensor/0"  // Topic to publish IR sensor data
#define SOL_TOPIC     "esp/sol/0"     // Topic to control solenoid PWM

/* -------------------- Servo Objects -------------------- */
// servo1, servo2, servo3: Continuous rotation or wheel servos
Servo servo1;
Servo servo2;
Servo servo3;

// micro_servo1, micro_servo2: Small angle-position servos
Servo micro_servo1;
Servo micro_servo2;

/* -------------------- Pin Definitions -------------------- */
// servo*_pin: GPIO pins connected to servo signal wires
#define servo1_pin        27
#define servo2_pin        26
#define servo3_pin        25
#define micro_servo1_pin  18
#define micro_servo2_pin  5

// SOL_PIN: GPIO pin connected to solenoid driver (PWM controlled)
const int SOL_PIN = 23;

// PWM configuration for solenoid
const int PWM_CHANNEL = 0;   // PWM channel number
const int PWM_FREQ    = 1000; // PWM frequency in Hz
const int PWM_RES     = 8;    // PWM resolution in bits (0–255)

// IR_PIN: GPIO pin connected to IR sensor output
#define IR_PIN 15

/* -------------------- Network Objects -------------------- */
// espClient: WiFi client object
WiFiClient espClient;

// mqttClient: MQTT client using WiFi connection
PubSubClient mqttClient(espClient);

/*
 * Function Name: setup_wifi
 * Input: None
 * Output: None
 * Logic:
 *   - Attempts to connect ESP32 to the specified WiFi network
 *   - Retries for a fixed number of attempts
 *   - Restarts ESP32 if connection fails
 * Example Call: setup_wifi();
 */
void setup_wifi() {
    WiFi.begin(ssid, password);
    int attempts = 0;

    // Try connecting to WiFi for ~10 seconds
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        Serial.println(F("Waiting for hotspot..."));
        delay(500);
        attempts++;
    }

    // Restart ESP32 if WiFi connection fails
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WiFi connection failed! Restarting..."));
        ESP.restart();
    }

    // Print IP address after successful connection
    Serial.println(F("WiFi connected."));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
}

/*
 * Function Name: mqttCallback
 * Input:
 *   topic   -> MQTT topic on which message was received
 *   payload -> Message data
 *   length  -> Length of received message
 * Output: None
 * Logic:
 *   - Parses incoming MQTT messages
 *   - Controls servos based on CMD_TOPIC
 *   - Controls solenoid PWM based on SOL_TOPIC
 * Example Call: Called automatically by MQTT client
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {

    Serial.print("Message arrived on topic: ");
    Serial.println(topic);

    String message = "";

    // Convert payload bytes into String
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    /* -------- Servo Control Command -------- */
    if (strcmp(topic, CMD_TOPIC) == 0) {

        int v1, v2, v3, mv1, mv2;

        // Parse comma-separated servo values
        sscanf(message.c_str(), "%d,%d,%d,%d,%d",
               &v1, &v2, &v3, &mv1, &mv2);

        // Apply servo commands
        servo1.writeMicroseconds(v1);
        servo2.writeMicroseconds(v2);
        servo3.writeMicroseconds(v3);
        micro_servo1.write(mv1);
        micro_servo2.write(mv2);

        // Debug output
        Serial.println("Servo command applied");
    }

    /* -------- Solenoid PWM Control -------- */
    if (String(topic) == SOL_TOPIC) {

        int pwm_value;

        // Extract PWM value
        sscanf(message.c_str(), "%d", &pwm_value);

        // Write PWM to solenoid
        ledcWrite(PWM_CHANNEL, pwm_value);

        Serial.print("Solenoid PWM: ");
        Serial.println(pwm_value);
    }
}

/*
 * Function Name: reconnect
 * Input: None
 * Output: None
 * Logic:
 *   - Reconnects to MQTT broker if connection is lost
 *   - Subscribes to required topics after reconnection
 * Example Call: reconnect();
 */
void reconnect() {
    while (!mqttClient.connected()) {

        Serial.print("Attempting MQTT connection...");

        if (mqttClient.connect(CLIENT_ID)) {
            Serial.println("connected");

            // Subscribe to command topics
            mqttClient.subscribe(CMD_TOPIC);
            mqttClient.subscribe(SOL_TOPIC);

            Serial.println("Subscribed to MQTT topics");
        }
        else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}

/*
 * Function Name: publishSensorData
 * Input: None
 * Output: None
 * Logic:
 *   - Reads IR sensor digital state
 *   - Publishes sensor value over MQTT
 * Example Call: publishSensorData();
 */
void publishSensorData() {

    int ir_state = digitalRead(IR_PIN);

    // Convert sensor reading to string
    String payload = String(ir_state);

    // Publish sensor data
    mqttClient.publish(SENSOR_TOPIC, payload.c_str());

    Serial.print("IR Sensor State: ");
    Serial.println(payload);
}

/*
 * Function Name: servo_init
 * Input: None
 * Output: None
 * Logic:
 *   - Attaches servo objects to corresponding GPIO pins
 * Example Call: servo_init();
 */
void servo_init() {
    servo1.attach(servo1_pin);
    servo2.attach(servo2_pin);
    servo3.attach(servo3_pin);
    micro_servo1.attach(micro_servo1_pin);
    micro_servo2.attach(micro_servo2_pin);
}

/*
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic:
 *   - Initializes serial communication
 *   - Configures pins, WiFi, MQTT, PWM, and servos
 * Example Call: Called automatically by Arduino
 */
void setup() {

    Serial.begin(115200);

    // Configure IR sensor pin
    pinMode(IR_PIN, INPUT_PULLUP);

    // Initialize WiFi and servos
    setup_wifi();
    servo_init();

    // Configure PWM for solenoid control
    ledcAttach(SOL_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(SOL_PIN, 0);

    // Configure MQTT client
    mqttClient.setServer(broker_ip, broker_port);
    mqttClient.setCallback(mqttCallback);
}

/*
 * Function Name: loop
 * Input: None
 * Output: None
 * Logic:
 *   - Maintains MQTT connection
 *   - Publishes sensor data periodically
 * Example Call: Called repeatedly by Arduino runtime
 */
void loop() {

    // Reconnect to MQTT broker if disconnected
    if (!mqttClient.connected()) {
        reconnect();
    }

    mqttClient.loop();

    // Publish sensor data every 2 seconds
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 2000) {
        lastMsg = millis();
        publishSensorData();
    }
}