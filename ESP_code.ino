#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>

#define RX_PIN D1  // Define the RX pin for SoftwareSerial
#define TX_PIN D2  // Define the TX pin for SoftwareSerial
#define GPS_RX_PIN D3  // Define the RX pin for GPS
#define GPS_TX_PIN D4  // Define the TX pin for GPS

SoftwareSerial arduinoSerial(RX_PIN, TX_PIN);
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
WiFiClient client;
float lpgValue, coValue, smokeValue, tempValue, humidityValue;
bool gpsDataAvailable = false;

const char* ssid = "";
const char* password = "";
const unsigned long channelID = ;
const char* writeAPIKey = "";

void setup() {
  Serial.begin(9600);
  arduinoSerial.begin(9600);
  gpsSerial.begin(9600);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  ThingSpeak.begin(client);
}

void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        // Set flag to indicate GPS data is available
        gpsDataAvailable = true;
        // Send data to ThingSpeak
        sendDataToThingSpeak(latitude, longitude);
      }
    }
  }

  // Read sensor data from Arduino only if GPS data is available
  if (gpsDataAvailable && arduinoSerial.available() > 0) {
    // Read data and parse it
    String sensorData = arduinoSerial.readStringUntil('\n');
    processSensorData(sensorData);
    // Reset GPS data flag
    gpsDataAvailable = false;
  }
}

void processSensorData(String sensorData) {
  // Example: if sensorData is comma-separated, assuming the order is LPG, CO, Smoke, Temperature, Humidity
  String values[5];
  int index = 0;

  while (sensorData.length() > 0) {
    int commaIndex = sensorData.indexOf(',');
    if (commaIndex != -1) {
      values[index] = sensorData.substring(0, commaIndex);
      sensorData = sensorData.substring(commaIndex + 1);
    } else {
      values[index] = sensorData;
      sensorData = "";
    }

    index++;
  }

  // Now, values[] contains individual sensor readings
  float lpgValue = values[0].toFloat();
  float coValue = values[1].toFloat();
  float smokeValue = values[2].toFloat();
  float tempValue = values[3].toFloat();
  float humidityValue = values[4].toFloat();

  // Now you can use these values as needed
  Serial.println("Received Sensor Data:");
  Serial.print("LPG: "); Serial.println(lpgValue);
  Serial.print("CO: "); Serial.println(coValue);
  Serial.print("Smoke: "); Serial.println(smokeValue);
  Serial.print("Temperature: "); Serial.println(tempValue);
  Serial.print("Humidity: "); Serial.println(humidityValue);
}

void sendDataToThingSpeak(float latitude, float longitude) {
  // Set data for each field
  ThingSpeak.setField(1, lpgValue);
  ThingSpeak.setField(2, coValue);
  ThingSpeak.setField(3, smokeValue);
  ThingSpeak.setField(4, tempValue);
  ThingSpeak.setField(5, humidityValue);
  ThingSpeak.setField(6, latitude);
  ThingSpeak.setField(7, longitude);

  Serial.print("Latitude: "); Serial.printf("%.8f\n", latitude);
  Serial.print("Longitude: "); Serial.printf("%.8f\n", longitude);

  // Send data to ThingSpeak
  int httpCode = ThingSpeak.writeFields(channelID, writeAPIKey);

  if (httpCode == 200) {
    Serial.println("Data sent to ThingSpeak successfully");
  } else {
    Serial.println("Error sending data to ThingSpeak");
  }
  Serial.println();
  delay(1000); // Adjust the delay based on your desired interval
}