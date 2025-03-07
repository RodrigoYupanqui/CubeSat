/*
 *  Project:    CubeSAT Implementation (1U) for "Competencia Nacional de Pequeños Satélites 2024"
 *  Author:     Massimo Bruschi / Rodrigo Yupanqui / Josh Yauri (PUCP)
 *  Created:    26/12/2024
 *  Revised:    03/03/2025 (Revision 3.0)
 *  Description: Receipt of all data via LoRa communication
 *  
 *  Development Board: ESP32 Dev Module (ESP-WROOM-32 Chip)
 *  Hardware Configuration:
 *  - SPI pins - Pin 18 (SCK), Pin 19 (MISO) and Pin 23 (MOSI) (These are for the LoRa Module Ra-02)
 *  - Everything operates on +3.3V (Could be external source or USB cable connection)
 *  
 */

#include <SPI.h>
#include <LoRa.h>

// Define pins for LoRa module
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23

// Variable to store the complete received message
String CompleteMessage = "";
// Flag to indicate if a complete message has been received
bool Reception = false;

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial); // Wait until serial communication is available
  
  Serial.println("Initializing LoRa RX...");
  
  // Initialize SPI communication with the specified pins
  SPI.begin(18, 19, 23, LORA_SS);
  
  // Set LoRa module pins
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Attempt to start LoRa communication at 433 MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("Error starting LoRa");
    // If LoRa fails to start, retry every second until successful
    while (!LoRa.begin(433E6)){
      delay(1000);
    }
  }
  
  // Set the LoRa sync word
  LoRa.setSyncWord(0xF3);
  
  Serial.println("LoRa RX ready");
}

void loop() {
  // Check if there is a packet available to read
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    // Initialize a string to store part of the message
    String PartOfMessage = "";
    
    // Read the packet and append it to the string
    while (LoRa.available()) {
      char c = LoRa.read();
      PartOfMessage += c;
    }
    
    // Check if this is the start of a new message
    if (PartOfMessage.startsWith("{")) {
      // Reset the complete message and reception flag
      CompleteMessage = PartOfMessage;
      Reception = false;
    } 
    // Check if this is the end of a message
    else if (PartOfMessage.endsWith("}")) {
      // Append to the complete message and set reception flag
      CompleteMessage += PartOfMessage;
      Reception = true;
    } 
    // If it's neither start nor end, just append to the complete message
    else {
      CompleteMessage += PartOfMessage;
    }

    // If a complete message has been received, process it
    if (Reception) {
      ProcessMessage(CompleteMessage);
      // Reset variables for the next message
      CompleteMessage = "";
      Reception = false;
    }
  }
}

void ProcessMessage(String message) {
  // Convert the message to a character array
  char datos[512];
  message.toCharArray(datos, 512);
  
  // Define variables to store sensor readings
  float voltage, current, temperature, humidity, pressure;
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  int azimuth;
  char direction[20];
  float latitude, longitude, altitude, speed, HDOP;
  
  // Parse the JSON message into the variables
  sscanf(datos, "{\"Voltage\":%f,\"Current\":%f,\"Temperature\":%f,\"Humidity\":%f,\"Pressure\":%f,\"AccelerationX\":%f,\"AccelerationY\":%f,\"AccelerationZ\":%f,\"RotationX\":%f,\"RotationY\":%f,\"RotationZ\":%f,\"MagneticFieldX\":%f,\"MagneticFieldY\":%f,\"MagneticFieldZ\":%f,\"Azimuth\":%d,\"Direction\":\" %19[^\"]\",\"Latitude\":%f,\"Longitude\":%f,\"Altitude\":%f,\"Speed\":%f,\"HDOP\":%f}",
         &voltage, &current, &temperature, &humidity, &pressure, &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &azimuth, direction, &latitude, &longitude, &altitude, &speed, &HDOP);

  // Print the received sensor readings
  Serial.print(voltage); Serial.print(",");
  Serial.print(current); Serial.print(",");
  Serial.print(temperature); Serial.print(",");
  Serial.print(humidity); Serial.print(",");
  Serial.print(pressure); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(mx); Serial.print(",");
  Serial.print(my); Serial.print(",");
  Serial.print(mz); Serial.print(",");
  Serial.print(azimuth); Serial.print(",");
  Serial.print(latitude, 6); Serial.print(",");
  Serial.print(longitude, 6); Serial.print(",");
  Serial.print(altitude); Serial.print(",");
  Serial.print(speed); Serial.print(",");
  Serial.println(HDOP);
}
