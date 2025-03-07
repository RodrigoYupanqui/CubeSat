/*
 *  Project:    CubeSat Implementation (1U) for "Competencia Nacional de Pequeños Satélites 2024"
 *  Author:     Massimo Bruschi (Sensor configuration)
 *              Rodrigo Yupanqui (Sensor configuration)
 *              Josh Yauri (Data transmission and reception)
 *  Created:    26/12/2024
 *  Revised:    03/03/2025
 *  Description: Monitoring system for parameters such as acceleration, relative 
 *               humidity, geolocation, acceleration, among others. Sending of 
 *               data from a LoRa communication. 
 *  
 *  Development Board: ESP32 Dev Module (ESP-WROOM-32 Chip) 30 pins
 *  Hardware Configuration:
 *  - I2C BUS pins - Pin 21 (SDA) and Pin 22 (SCL) (These are for BME280, 
 *    MPU6050, INA219 and QMC5883L modules)
 *  - SPI pins - Pin 18 (SCK), Pin 19 (MISO) and Pin 23 (MOSI) (These are for the LoRa Module Ra-02)
 *  - Serial pins - Pin 16 (RX) and Pin 17 (Tx) (These are for NEO-6M module)
 *  - Everything operates on +3.3V (Could be external source or USB cable connection)
 *
 *  Massimo Bruschi, Rodrigo Yupanqui and Josh Yauri are 3 outstanding students of the Faculty of 
 *  Science and Engineering of PUCP, belonging to the top 5% of students (according to Craest).
 *
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>     
#include <QMC5883LCompass.h>
#include <Adafruit_BME280.h>        
#include <Adafruit_MPU6050.h>
#include <Adafruit_INA219.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <LoRa.h>                  
#include <ESP32Servo.h>

// --- Pin Definitions ---

// Definition of pins for the I2C bus, used by multiple sensors:
//   - BME280 (pressure, temperature, and humidity)
//   - MPU6050 (accelerometer and gyroscope)
//   - INA219 (current sensor)
//   - QMC5883L (magnetometer)
#define SDA 21  // Pin for the I2C data line
#define SCL 22  // Pin for the I2C clock line

// Definition of pins for the GPS module
#define GPS_RX 16   // Pin to receive data from the GPS module (RX: Receive)
// Note: This pin should be connected to the TX pin of the GPS module.
#define GPS_TX 17   // Pin to send data to the GPS module (TX: Transmit)
// Note: This pin should be connected to the RX pin of the GPS module.

// Definition of pins for the LoRa module
#define LORA_SS 5   // Slave Select pin for the LoRa module
#define LORA_RST 14 // Reset pin for the LoRa module
#define LORA_DIO0 2 // Digital Input/Output pin 0 for the LoRa module (used to indicate events)
#define LORA_SCK 18 // Clock pin for the LoRa module's SPI bus
#define LORA_MISO 19 // Master In Slave Out pin for the LoRa module's SPI bus
#define LORA_MOSI 23 // Master Out Slave In pin for the LoRa module's SPI bus

// Initialize instances of various sensor and component classes
Adafruit_BME280 bme;      // Instance of the BME280 sensor for pressure, temperature, and humidity
Adafruit_MPU6050 mpu;     // Instance of the MPU6050 sensor for accelerometer and gyroscope data
Adafruit_INA219 ina219;   // Instance of the INA219 current sensor
TinyGPSPlus gps;          // Instance of the TinyGPSPlus library for GPS data processing
HardwareSerial serialGPS(2); // Serial interface for GPS communication (using UART2)
QMC5883LCompass compass;  // Instance of the QMC5883L magnetometer for compass readings
Servo servo1;              // First servo motor instance
Servo servo2;              // Second servo motor instance

// Variables to store sensor readings
float temperature, humidity, pressure; // BME280 Readings
float ax, ay, az, gx, gy, gz; // MPU6050 Readings (Accelerometer and Gyroscope)
float mx, my, mz, azimuth; // QMC5883L Readings (Magnetometer)
char direction[3]; // QMC5883L Readings (Direction based on magnetometer data)
float voltage, current; // INA219 Readings (Current sensor)
float latitude, longitude, altitude, speed, HDOP;  // NEO-6M GPS Readings
float starting_height, height; // BME280 Altitude calculations
bool sentinel1 = false; // Flag for altitude threshold
bool sentinel2 = true; // Flag for initial height setup

// Security configuration (by Josh Yauri)
const char* networkID = "MiRedLoRa";
const char* deviceID = "Dispositivo1";
const char* claveAutenticacion = "MiClaveSegura";

void setup() {
  // Configure PWM pins for servo motors
  // Recommended PWM pins for ESP32 are 2, 4, 12-19, 21-23, 25-27, 32-33
  servo1.attach(4); // Pin 4 for the first servo
  servo2.attach(13); // Pin 13 for the second servo

  // Initialize servo motors to 0 degrees
  servo1.write(0);
  servo2.write(0);

  Serial.begin(115200); // Initialize serial communication at 115200 baud
  Wire.begin(); // Initialize I2C communication

  // GPS configuration
  serialGPS.begin(9600, SERIAL_8N1, 16, 17); // Initialize GPS serial communication
  delay(1000); // Wait for GPS module to stabilize

  // Magnetometer configuration
  compass.init(); // Initialize magnetometer
  compass.setCalibrationOffsets(-276.00, 269.00, -348.00); // Set magnetometer calibration offsets
  compass.setCalibrationScales(1.08, 1.11, 0.86); // Set magnetometer calibration scales

  // Temperature, Humidity, and Pressure Sensor
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found.");
    while(!bme.begin(0x76)){
      delay(1000);
    }
    Serial.println("BME280 found.");
  }

  // Accelerometer / Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while(!mpu.begin()) {
      delay(1000);
    }
    Serial.println("MPU6050 Found!");
  }
  
  // Current Sensor
  if (!ina219.begin()) {
    Serial.println("INA219 not found.");
    while(!ina219.begin()) {
      delay(1000);
    }
    Serial.println("INA219 Found!");
  }
  
  // LoRa Module
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS); // Initialize SPI pins for LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0); // Set LoRa module pins
  if (!LoRa.begin(433E6)) {  // LoRa frequency (example: 433MHz)
    Serial.println("Error starting LoRa.");
    while(!LoRa.begin(433E6)) {
      delay(1000);
    }
    Serial.println("LoRa initialized.");
  }
  LoRa.setSyncWord(0xF3); // Set LoRa sync word
  Serial.println("Complete initialization.");
}

void loop() {
  // Read sensor data
  readBME280();
  readMPU6050();
  readQMC5883L();
  readINA219();
  readGPS();

  // Construct message to send via LoRa
  String message = buildMessage();
  sendLoRa(message);

  // Control servo motors based on altitude threshold
  if(height > 85){
    sentinel1 = true;
  }
  if(sentinel1){
    if(80 > height){
      servomotores();   
    }
  }
  delay(500); // Wait before next loop iteration
}

void servomotores() {
  // Set servo motors to 90 degrees
  servo1.write(90);
  servo2.write(90);
}

void readBME280() {
  // Read temperature, humidity, and pressure from BME280
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = (1 - pow((pressure / 1013.25), (1 / 5.2559))) / 0.0000225577;
  
  // Set initial height if not already set
  if(sentinel2){
    sentinel2 = false;
    starting_height = altitude;
  }
  height = altitude - starting_height;
}

void readMPU6050() {
  // Read accelerometer and gyroscope data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
}

void readQMC5883L() {
  // Read magnetometer data from QMC5883L
  compass.read();
  mx = compass.getX();
  my = compass.getY();
  mz = compass.getZ();
  azimuth = compass.getAzimuth();
  compass.getDirection(direction, azimuth);
}

void readINA219() {
  // Read voltage and current from INA219
  voltage = ina219.getBusVoltage_V();
  current = ina219.getCurrent_mA();
}

void readGPS() {
  // Read GPS data
  while (serialGPS.available() > 0) {
    if (gps.encode(serialGPS.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        speed = gps.speed.kmph();
        HDOP = gps.hdop.value() / 100.0;
        break;
      } else {
        Serial.println("GPS doesn't read coherent things");
      }
    }
  }
}

// --- Message Construction ---
String buildMessage() {
  // Create a JSON object with sensor readings
  String message = "{";
  message += "\"Voltage\": " + String(voltage) + ",";
  message += "\"Current\": " + String(current) + ",";
  message += "\"Temperature\": " + String(temperature) + ",";
  message += "\"Humidity\": " + String(humidity) + ",";
  message += "\"Pressure\": " + String(pressure) + ",";
  message += "\"AccelerationX\": " + String(ax, 3) + ",";
  message += "\"AccelerationY\": " + String(ay, 3) + ",";
  message += "\"AccelerationZ\": " + String(az, 3) + ",";
  message += "\"RotationX\": " + String(gx, 3) + ",";
  message += "\"RotationY\": " + String(gy, 3) + ",";
  message += "\"RotationZ\": " + String(gz, 3) + ",";
  message += "\"MagneticFieldX\": " + String(mx) + ",";
  message += "\"MagneticFieldY\": " + String(my) + ",";
  message += "\"MagneticFieldZ\": " + String(mz) + ",";
  message += "\"Azimuth\": " + String(azimuth) + ",";
  message += "\"Direction\": \"" + String(direction) + "\",";
  message += "\"latitude\": " + String(latitude, 3) + ",";
  message += "\"longitude\": " + String(longitude, 3) + ",";
  message += "\"altitude\": " + String(height) + ","; // Corrected variable name
  message += "\"Speed\": " + String(speed) + ",";
  message += "\"HDOP\": " + String(HDOP);
  message += "}";
  return message;
}

// --- Sending Data by LoRa ---
void sendLoRa(String message) {
  int messageLength = message.length();
  int maxCharsPerPacket = 50; // Maximum characters per LoRa packet
  
  // Split message into packets if necessary
  int start = 0;
  while (start < messageLength) {
    String partOfMessage = message.substring(start, min(start + maxCharsPerPacket, messageLength));
    LoRa.beginPacket();
    LoRa.print(partOfMessage);
    LoRa.endPacket();
    Serial.println("Message sent by LoRa: " + partOfMessage);
    delay(50); // Short delay between transmissions
    start += maxCharsPerPacket;
  }
  Serial.println("Message sent in full");
}
