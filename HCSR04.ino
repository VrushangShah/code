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
// the pin numbers for the ultrasonic sensor
#define TRIGGER_PIN 27
#define ECHO_PIN 26
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
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(TRIGGER_PIN, OUTPUT); // set the trigger pin as an output
  pinMode(ECHO_PIN, INPUT); // set the echo pin as an input
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
  if (now - lastMsg > 250) {
    lastMsg = now;
    
    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --
  // measure the distance to an object using the ultrasonic sensor
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;

  // Convert the value to a char array
    char distString[8];
    dtostrf(distance, 1, 2, distString);
    Serial.print("Distance: ");
    Serial.println(distString);
    client.publish("esp32/distance", distString);

    // --
  }
}
