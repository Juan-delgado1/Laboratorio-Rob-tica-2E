# Laboratorio No. 02 - Robótica Industrial - Análisis y Operación del Manipulador Motoman MH6.
## Integrantes

**Juan Angel Vargas Rodríguez**
juvargasro@unal.edu.co

**Santiago Mariño Cortés**
smarinoc@unal.edu.co

**Juan José Delgado Estrada**
judelgadoe@unal.edu.co

## Introducción

## Planteamiento del problema

## Objetivos

## Configuraciones y características del manipulador Motoman MH6 y sus herramientas

### Comparación entre manipuladores IRB140 y Motoman MH6
- Comparar las especificaciones técnicas del Motoman MH6 y el IRB140 en un cuadro comparativo.

Ejes del Motoman MH6:

![Motoman MH6](./Images/Motoman-MH6.png)

Ejes del ABB 140:

![ABB 140](./Images/ABB-140.png)

<div align="center">

| **Característica** | **Motoman MH6 (Yaskawa)** | **ABB IRB 140-6/0.8** |
|---------------------|---------------------------|-------------------------|
| **Fabricante** | Yaskawa Motoman | ABB Robotics |
| **Modelo** | MH6 | IRB 140-6/0.8 |
| **Carga útil máxima** | 6 kg | 6 kg |
| **Alcance máximo** | 1373 mm | 800 mm |
| **Número de ejes / Grados de libertad** | 6 | 6 |
| **Peso del manipulador** | 130 kg | 98 kg |
| **Velocidad máxima por eje** | E1: 220°/s<br>E2: 200°/s<br>E3: 220°/s<br>E4: 410°/s<br>E5: 410°/s<br>E6: 610°/s | E1: 200°/s<br>E2: 200°/s<br>E3: 260°/s<br>E4: 360°/s<br>E5: 360°/s<br>E6: 450°/s |
| **Rango de movimiento por eje** | E1: ±170°<br>E2: +155° a –90°<br>E3: +250° a –175°<br>E4: ±180°<br>E5: +225° a -45°<br>E6: ±360° | E1: ±180°<br>E2: +110° a –90°<br>E3: +50° a –230°<br>E4: ±200°<br>E5: ±115°<br>E6: ±400° |
| **Repetibilidad (precisión)** | ±0.08 mm | ±0.03 mm |
| **Aplicaciones comunes** | Ensamblaje, soldadura, manipulación de materiales, dispensado, manufactura aditiva, inspección visual. | Ensamblaje de precisión, laboratorio, manejo de piezas pequeñas, dispensado, pick and place, entornos de sala limpia o fundición. |



</div>



### Configuraciones iniciales del manipulador Motoman MH6
- Descripci´on de las configuraciones home1 y home2 del Motoman MH6, indicando la posici´on de cada articulaci´on, ¿Cual posici´on es mejor?, justifique su respuesta.
- Describir las diferencias entre el home1 y el home2 del Motoman MH6.

### Movimientos manuales
- Procedimiento detallado para realizar movimientos manuales, especificando c´omo cambiar entre modos de operaci´on (articulaciones, cartesiano) y realizar traslaciones y rotaciones en los ejes X, Y, Z.
- Describir el procedimiento y cu´ales teclas se usan para realizar el movimiento manual del manipulador Motoman por articulaciones, cambiar a movimientos cartesianos y realizar movimientos de traslaci´on y rotaci´on en los ejes X, Y, Z.

### Control de velocidad
- Explicaci´on completa sobre los niveles de velocidad para movimientos manuales, el proceso para cambiar entre niveles y c´omo identificar el nivel establecido en la interfaz del robot.
- Detallar los niveles de velocidad del Motoman para movimientos manuales y su configuraci´on, ¿C´omo se hace el cambio entre niveles de velocidad?, ¿C´omo se identifica en la pantalla el nivel de velocidad establecido?

### Software RoboDK 
- Descripci´on de las principales funcionalidades de RoboDK, explicando c´omo se comunica con el manipulador Motoman y qu´e procesos realiza para ejecutar movimientos.
- Explicar las aplicaciones principales de RoboDK y c´omo se comunica con el manipulador, ¿Qu´e hace RoboDK para mover el manipulador?
- ¿C´omo se comunica RoboDK con el manipulador?

### Comparación de herramientas RoboDK y RobotStudio
- An´alisis comparativo entre RoboDK y RobotStudio, destacando ventajas, limitaciones y aplicaciones de cada herramienta.
- Analizar las diferencias entre RoboDK y RobotStudio y describir los usos espec´ıficos de cada herramienta, ¿Qu´e significa para usted cada una de esas herramientas?

## Desarrollo de práctica: Trayectoria polar
- Explicar que vamos a hacer Xd
- Programar una trayectoria polar en RoboDK, hacer que se ejecute virtualmente en RoboDK y hacer que el manipulador Motoman la realice f´ısicamente comandado desde el PC. https://www.monografias.com/trabajos33/coordenadas-polares/coordenadas-polares.
- Bajo la secci´on de la trayectoria polar, incluir los nombres de los integrantes del equipo.

### Diagrama de flujo de acciones del robot
### Plano de planta 
### Código desarrollado en RoboDK

### Video de la simulación en RoboDK
### Video de la implementación física con el manipulador Motoman MH6

## Conclusiones



