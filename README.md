# CubeSat

La Primera Competencia Nacional de Pequeños Satélites en Perú fue un evento educativo organizado por el Instituto de Radioastronomía de la PUCP (INRAS-PUCP), el Vicerrectorado de Investigación (VRI-PUCP) y la Agencia Espacial del Perú – CONIDA. Después de varias fases, la competencia seleccionó a 17 equipos de diversas universidades del país para participar en la fase final y presencial. Los estudiantes debían diseñar, construir y probar un CubeSat, que luego era elevado por un dron a 100 metros de altura antes de ser liberado en caída libre.

<img src="https://i.imgur.com/GWN2etC.png" width="1000">

## 1. Selección y conexión de los componentes
Después de una evaluación de precios, disponibilidad y tecnología, obtuvimos los componentes necesarios y adecuados para nuestro objetivo. Indicaré cuales son dichos sensores, al igual que sus especificaciones técnicas.

### **NodeMCU ESP32 DevKit V1 (Microcontrolador)**

<img align="right" src="https://grobotronics.com/images/detailed/123/esp32-4-1_grobo.jpg" width="290">

    - Chip: ESP32-WROOM-32
    
    - Procesador: Dual-Core 32-bit Tensilica Xtensa LX6 a 240 MHz
    
    - Memoria: 4MB Flash, 520KB SRAM
    
    - Interfaces: UART, SPI, I2C, I2S, ADC, DAC, PWM
    
    - Pines: 30 (25 GPIOs configurables como ADC, DAC, PWM, etc)
    
    - Voltaje de Operación: 3.3V (con regulador interno, soporta alimentación de 5V vía USB)
    
    - Compatible: Arduino IDE, MicroPython, Espressif IDF

### **MPU6050 (Acelerómetro + Giroscopio)**

<img align="right" src="https://i.imgur.com/TyWNgdh.jpeg" width="210">

    - Sensor MEMS 6 ejes (3-axis acelerómetro + 3-axis giroscopio)
    
    - Rango de aceleración: ±2g, ±4g, ±8g, ±16g
    
    - Rango angular: ±250, ±500, ±1000, ±2000°/s
    
    - Protocolo de comunicación: I2C
    
    - ADC de 16 bits para alta precisión

### **BME280 (Sensor Ambiental)**

<img align="right" src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTZnv_aqz1EngN4Rf3QH1ZFQ6ZEDc9N1B4NxA&s" width="275">

    - Sensor integrado de Presión, Humedad y Temperatura
    
    - Rango de presión: 300-1100 hPa
    
    - Rango de humedad: 0-100% HR
    
    - Rango de temperatura: -40°C a +85°C
  
    - Protocolo de comunicación: I2C/SPI
    
    - Bajo consumo de energía

### **QMC5883 (Magnetómetro)**

<img align="right" src="https://electronics.semaf.at/media/image/product/659/lg/triple-axis-magnetometer-compass-breakout-hmc5883l-qmc5883.jpg" width="210">

    - Sensor de campo magnético de 3 ejes
    
    - Rango de medición: ±8 Gauss
    
    - Resolución: 1-2 milliGauss
    
    - Protocolo de comunicación: I2C
    
    - Compensación de temperatura integrada

### **INA219 (Sensor de Corriente y Voltaje)**

<img align="right" src="https://naylampmechatronics.com/4480-superlarge_default/monitor-de-corriente-voltaje-high-side.jpg" width="210">

    - Mide corriente y voltaje en el lado alto
    
    - Rango de voltaje de bus: 0 a +26V
    
    - Rango de corriente: ±3.2A
    
    - Resolución: 0.8mA (corriente), 4mV (voltaje)
    
    - Protocolo de comunicación: I2C

### **GPS NEO-6M (Módulo GPS)**

<img align="right" src="https://sumador.com/cdn/shop/products/ModuloGPSUBLOXNEO-6M2_grande.jpg?v=1588376046" width="210">

    - 50 canales de rastreo
    
    - Sensibilidad: -161 dBm
    
    - Protocolo de comunicación: UART
    
    - Antena integrada
    
    - TTFF (Tiempo para la primera fijación): 27s (arranque en frío)

### **Servomotor SG90**

<img align="right" src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRnp32QGJB-StDuJ-GVG60-a93YiZZBe4qLlA&s" width="260">

    - Tipo: Micro servo
    
    - Voltaje de operación: 3.3V a 5V
    
    - Torque: 1.8 kg-cm
    
    - Velocidad de operación: 0.1 s/60 grados
    
    - Rango de rotación: 0 a 180 grados
    
    - Interfaz: PWM

### **SX1278 (Módulo LoRa)**

<img align="right" src="https://www.cparkb2c.com/cdn/shop/files/Ra-01_1.jpg?v=1719989019" width="335">

    - Chip: Semtech SX1278
    
    - Frecuencia: 433MHz/470MHz/868MHz/915MHz
    
    - Potencia de salida: +20dBm
    
    - Sensibilidad: Hasta -148dBm
    
    - Modulación: LoRa/FSK/GFSK/OOK
    
    - Interfaz: SPI

La siguiente imagen es un ejemplo de conexión para todos los modulos con los pines del ESP32 (Imagen brindada por Massimo Bruschi)

<img src="https://i.imgur.com/Cmlhuhz.jpeg" width="600">

También, en el código, se indica que pines son los utilizados para conectar correctamente. De todas formas, los indicó a continuación:

````cpp
// From CubeSatTX.ino
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
````

## 2. Diseño de los circuitos electrónicos / PCBs

Como parte del equipo Arquitectos Galácticos, mis funciones incluyeron:
- Matriz Morfológica y Diagrama de Flujo: Elaboración de la matriz morfológica, diagrama de flujo principal y criterios para seleccionar los componentes más convenientes.
- Investigación de Componentes: Investigación sobre los componentes necesarios para realizar los objetivos planteados y su debida cotización.
- Programación del Microcontrolador: Elaboración del código para interconectar módulos y sensores con un microcontrolador ESP32, evaluando parámetros como temperatura, aceleración, presión y humedad.
- Diseño de Diagramas Eléctricos y PCBs: Diseño de los diagramas eléctricos y PCBs necesarios, siguiendo requerimientos específicos para el correcto encaje en el cubo.
