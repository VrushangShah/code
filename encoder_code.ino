//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Add your required sensor/component libraries here
// --
#define SLAVE_ADDRESS 0x04
#define SCL_PIN 22
#define SDA_PIN 21
long enc1_count;
long enc2_count;
int16_t leftMotor_speed;
int16_t rightMotor_speed;
int servoAngle;
// --

// Replace the next variables with your SSID/Password combination
const char* ssid = "c08shah";                      //CHANGE ME
const char* password = "notmyproblem";              //CHANGE ME     

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";                      
const char* mqtt_server = "192.168.137.138";          //CHANGE ME

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(); //initialize the I2C bus with the specified pin numbers
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
  
  // --
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      
      // Add your subscribe topics here
      // --
      
      // --
         
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
  }
    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --
  // measure the distance to an object using the ultrasonic sensor

leftMotor_speed = 0;
rightMotor_speed =                                                                                                                                                                                                                                                      0;
Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(leftMotor_speed >> 8); // send bits 16 to 9 of leftMotor_speed
  Wire.write(leftMotor_speed & 0xFF); // send bits 8 to 1 of leftMotor_speed
  Wire.write(rightMotor_speed >> 8); // send bits 16 to 9 of rightMotor_speed
  Wire.write(rightMotor_speed & 0xFF); // send bits 8 to 1 of rightMotor_speed
  Wire.write(servoAngle >> 8); // send bits 16 to 9 of servoAngle
  Wire.write(servoAngle & 0xFF); // send bits 8 to 1 of servoAngle
  Wire.endTransmission();

  // request the encoder values from the Arduino Nano

  Wire.requestFrom(SLAVE_ADDRESS, 4);
 if (Wire.available() >= 4) { // wait until 4 bytes are available
enc1_count = Wire.read() << 8; // receive bits 16 to 9 of enc1_count
enc1_count |= Wire.read(); // receive bits 8 to 1 of enc1_count
enc2_count = Wire.read() << 8; // receive bits 16 to 9 of enc2_count
enc2_count |= Wire.read(); // receive bits 8 to 1 of enc2_count
}

//Serial.println(enc1_count);

  // Convert the value to a char array
    char encString[8];
    dtostrf(enc1_count, 1, 2, encString);
    Serial.print("Encoders: ");
    Serial.println(encString);
    client.publish("esp32/enc", encString);

    // --
  
}
