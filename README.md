# CubeSat

La Competencia Nacional de Pequeños Satélites es un evento educativo que integra la ciencia y las ingenierías para desarrollar soluciones tecnológicas innovadoras.

## Proceso

Hemos cubierto 4 etapas virtuales y la última etapa presencial, en la cual estaban presentes los mejores equipos a nivel nacional.

### 1. Selección de componentes
#### XD
Después de una evaluación de precios, disponibilidad y tecnología, obtuvimos los componentes necesarios y adecuados para nuestro objetivo. Indicaré cuales son dichos sensores, al igual que sus especificaciones técnicas.

- **MPU6050 (Acelerómetro + Giroscopio)**
  - Sensor MEMS 6 ejes (3-axis acelerómetro + 3-axis giroscopio)
  - Rango de aceleración: ±2g, ±4g, ±8g, ±16g.
  - Rango angular: ±250, ±500, ±1000, ±2000°/s.
  - Protocolo de comunicación: I2C.
  - ADC de 16 bits para alta precisión.
  - Imagen de referencia:
  <img src="https://i.imgur.com/TyWNgdh.jpeg" width="300">
  
- **BME280**
- **QMC5883**
- **INA219**
- **GPS NEO-6M**
- **Servomotor SG90**

<img src="https://i.imgur.com/ZKGq4aQ.jpeg" width="500">


Como parte del equipo Arquitectos Galácticos, mis funciones incluyeron:
- Matriz Morfológica y Diagrama de Flujo: Elaboración de la matriz morfológica, diagrama de flujo principal y criterios para seleccionar los componentes más convenientes.
- Investigación de Componentes: Investigación sobre los componentes necesarios para realizar los objetivos planteados y su debida cotización.
- Programación del Microcontrolador: Elaboración del código para interconectar módulos y sensores con un microcontrolador ESP32, evaluando parámetros como temperatura, aceleración, presión y humedad.
- Diseño de Diagramas Eléctricos y PCBs: Diseño de los diagramas eléctricos y PCBs necesarios, siguiendo requerimientos específicos para el correcto encaje en el cubo.
