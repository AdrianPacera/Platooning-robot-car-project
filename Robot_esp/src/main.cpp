#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

// Replace with your network credentials
const char *ssid = "12connect";
const char *password = "";

// Replace with your MQTT broker credentials
const char *mqtt_server = "test.mosquitto.org";
//const char *mqtt_server = "broker.hivemq.com";

const int mqtt_port = 1883;

// Define the Serial port connected to the external device
HardwareSerial externalSerial(1);

// Create an instance of the PubSubClient
WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length)
{
  // Convert the payload to a string
  String payloadStr = "";
  for (unsigned int i = 0; i < length; i++)
  {
    payloadStr += (char)payload[i];
  }

  // Convert the string to an integer
  int value = payloadStr.toInt();

  externalSerial.println(value);
  
  externalSerial.write(value);
  // externalSerial.write(\0");
  Serial.println(value);
}

void reconnect()
{
  Serial.println("reconnecting . . . ");
  // Attempt to connect to MQTT broker
  if (client.connect("test.mosquitto.org"))
  {
    // Subscribe to the MQTT topic after successful connection
    client.subscribe("ROBOT");
    client.subscribe("DISTANCE");
  }
  else
  {
    Serial.print("Failed to connect to MQTT broker, rc=");
    Serial.println(client.state());
  }
}

void setup()
{
  // Start the serial communication
  Serial.begin(9600);
  externalSerial.begin(9600, SERIAL_8N1, 16, 17);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set up MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Initial MQTT connection
  reconnect();
}

void loop()
{
  //Serial.println("looping");
  // If not connected to MQTT broker, attempt to reconnect
  if (!client.connected())
  {
    reconnect();
  }

  // Read data from the external device's serial port
  if (externalSerial.available() > 0)
  {
    Serial.println("reading");
    String data = externalSerial.readStringUntil('\n');

    // Publish the data to the MQTT topic
    char topic[50];
    data.toCharArray(topic,10);
    client.publish("DISTANCE", topic);


    Serial.print("Data sent to MQTT: ");
    Serial.println(data);
  }

  // Maintain MQTT connection and handle incoming messages
  client.loop();
}