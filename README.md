# CubeSat

La Primera Competencia Nacional de Pequeños Satélites en Perú fue un evento educativo organizado por el Instituto de Radioastronomía de la PUCP (INRAS-PUCP), el Vicerrectorado de Investigación (VRI-PUCP) y la Agencia Espacial del Perú – CONIDA. Después de varias fases, la competencia seleccionó a 17 equipos de diversas universidades del país para participar en la fase final y presencial. Los estudiantes debían diseñar, construir y probar un CubeSat, que luego era elevado por un dron a 100 metros de altura antes de ser liberado en caída libre.

![Imagen de WhatsApp 2025-03-16 a las 18 00 06_cc1e3546](https://github.com/user-attachments/assets/c6378456-362f-4281-bf51-3e9f60e6ac92)

<img src="https://i.imgur.com/GWN2etC.png" width="1050">

Como parte del equipo Arquitectos Galácticos, mis funciones incluyeron:
- Matriz Morfológica y Diagrama de Flujo: Elaboración de la matriz morfológica, diagrama de flujo principal y criterios para seleccionar los componentes más convenientes.
- Investigación de Componentes: Investigación sobre los componentes necesarios para realizar los objetivos planteados y su debida cotización.
- Programación del Microcontrolador: Elaboración del código para interconectar módulos y sensores con un microcontrolador ESP32, evaluando parámetros como temperatura, aceleración, presión y humedad.
- Diseño de Diagramas Eléctricos y PCBs: Diseño de los diagramas eléctricos y PCBs necesarios, siguiendo requerimientos específicos para el correcto encaje en el cubo.

En este documento, describiré mis labores durante la competencia en orden secuencial. En el último inciso, se muestran los aportes de mis compañeros. Esto les podrá servir de referencia para otros proyectos o intentar replicar un CubeSat.

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

Después de realizar las diferentes conexiones y pruebas en protoboard, ya era momento de empezar a diseñar los PCBs para imprimirlos y soldar los componentes correspondientes. Dejo fotos en el siguiente orden: 

<p align="center"><b>Diseño PCB en EasyEDA ➡️ Modelo 3D brindado por EasyEDA ➡️ Impresión real</b></p>

Piso 1: En este piso, solo se encontraban las pilas 2A de 1.5V marca Duracell para alimentar todo el circuito (Recomendamos usar baterías Ion-Litio de 3.7V si se asegura un buen despliegue)

Piso 2: En este piso, se encuentra el ESP32, el GPS NEO-6M, el SX1278, el INA219, el QMC5883 y el MPU6050.

<p align="center">
  <img src="https://i.imgur.com/Q54q2ht.png" width="300">
  <img src="https://i.imgur.com/phjtCcb.png" width="465">
  <img src="https://github.com/user-attachments/assets/dd8754d2-1192-4456-8db3-03df59ef4a85" width="225">
</p>

Piso 3: En este piso, se encuentra el BME280, los 2 servomotores SG90 y el paracaídas que se iba a desplegar al momento de que los servomotores girarán.

<p align="center">
  <img src="https://i.imgur.com/EGLJfHs.png" width="300">
  <img src="https://i.imgur.com/81v1I4O.png" width="340">
  <img src="https://github.com/user-attachments/assets/30411c85-d884-44a9-a2d3-0f8f53125d6f" width="300">

</p>

## 3. Telemetría

### **Construcción del Mensaje (CubeSatTX.ino)**
La función buildMessage() en el código CubeSatTX.ino es responsable de tomar las lecturas de todos los sensores y construir un mensaje estructurado en formato JSON.

### **Transmisión de Datos (CubeSatTX.ino)**
La función sendLoRa() en el código CubeSatTX.ino se encarga de transmitir los mensajes JSON a través del módulo LoRa. Dada la limitación en el tamaño de los paquetes LoRa, el mensaje se divide en fragmentos más pequeños si excede los 50 caracteres.

- División en Paquetes: El mensaje se divide en fragmentos de hasta 50 caracteres.

- Transmisión LoRa: Cada fragmento se envía como un paquete LoRa independiente utilizando LoRa.beginPacket(), LoRa.print(), y LoRa.endPacket().

- Retardo: Se introduce un pequeño retardo (delay(50)) entre las transmisiones para evitar saturar el canal.

### **Recepción en la Estación Terrestre (CubeSatRX.ino)**
El código CubeSatRX.ino en la estación terrestre escucha continuamente las transmisiones LoRa del CubeSat. Utiliza la función loop() para verificar la llegada de paquetes y la función ProcessMessage() para procesar los datos recibidos.
La función ProcessMessage() convierte el mensaje recibido en un array de caracteres y utiliza sscanf para extraer los valores de cada sensor.

### **Control Basado en Altitud (CubeSatTX.ino)**
El sistema también incluye lógica para controlar servomotores en función de la altitud medida por el BME280. En la función loop() del código CubeSatTX.ino, se verifica si la altitud supera un umbral de 85 metros. Si se cumple esta condición, se activa la función servomotores().
La función servomotores() establece los servos en una posición predefinida (90 grados).

### **Interfaz**

También contamos con una interfaz que permitía ver los datos recibidos por la estación a tierra de manera más dinámica. Esto no hubiera sido posible sin Josh Yauri, un destacado estudiante de Ingeniería de Telecomunicaciones PUCP (el 2do mejor de su especialidad si nos basamos en Craest) y un gran amigo mío. Dejo una imagen de su interfaz mientras realizabamos las pruebas térmicas en el INRAS:

![1741226307030](https://github.com/user-attachments/assets/b0f7e1a9-0b45-4ef1-8f56-5dd217865905)

## 4. Diseño Mecánico

Mis compañeros se encargaron de un diseño mecánico que permita cumplir el peso requerido y sea sólido al momento de un impacto. Para evitar volcaduras, analizaron los pesos de los componentes al posicionar los elementos de manera que el centro de gravedad se encuentre lo más centrado posible.

<p align="center">
  <img src="https://github.com/user-attachments/assets/9018fef8-c56c-412d-97f2-c3cc83c80e6f" width="400">
  <img src="https://github.com/user-attachments/assets/384b8ad9-b5a1-4828-969c-59d403010f96" width="290">
</p>


## 5. Demostración casera

Les muestro una demostración del envío y recepción de datos que hice en mi casa. Se muestra como envía por tramas y recepciona la estación a tierra (la cual es un ESP32 más un módulo LoRa). Se utilizaron antenas helicoidales para mayor rango al momento de la transmisión.

https://github.com/user-attachments/assets/5067e69a-c475-48ee-a354-8e5d27d49ae5

También lo pueden checar en mi canal, en el cual subiré videos de algunas pruebas de nuevos proyectos para compartir mis avances continuamente (antes de hacer divulgaciones oficiales en GitHub, donde toma más tiempo): www.youtube.com/@BitsYBobinas



## Agradecimientos a los colaboradores

Agradezco a Massimo Bruschi, uno de los estudiantes más brillantes de Ingeniería Electrónica, por su apoyo tanto en la parte electrónica como en la estación terrena. Una gran persona y muy buen amigo, al cual le espera un futuro brillante en el mundo de la electrónica indudablemente.
También a Josh Fernando Yauri Salas por su apoyo en la parte de telemetría. Como futuro Ingeniero de Telecomunicaciones, nos apoyó en establecer una comunicación mediante LoRa y realizó una interfaz increíble en la estación terrena, lo cual refuerza que es un estudiante destacado en nuestra institución.
Nuestro compañero Cristhian Tarazona, destacado estudiante de Ingeniería Mecánica y líder de una área importante en la Ingeniería como lo es la Estructura, nos pudo brindar un diseño mecánico estético y sólido, el cual aseguraba resistencia ante choques mediante cálculos detallados. Junto con Gonzalo David Romero Jamanca y Yazira Denise Martinez Laverde, los cuales apoyaron en la sección mecánica mediante los planos requeridos y la realización del póster.
Finalmente, pero no por eso menos importante, gracias a Renato Perez, porque, aparte de ser el líder del equipo, apoyó en todas las secciones constantemente. Una persona resiliente y con ganas de aportar a lo que se compromete.
