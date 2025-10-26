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

*Ejes del Motoman MH6:*

![Motoman MH6](./Images/Motoman-MH6.png)

*Ejes del ABB 140:*

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
- Describir las diferencias entre el home1 y el home2 del Motoman MH6.

**Home 1**
<p align="center">

<img src="./Images/Home1.1.jpg" width="600">

</p>
<div align="center">
  
|**Eje**|**Posición**|
|-------|------------|
|1|0 deg|
|2|88.6629 deg|
|3|-81.0787 deg|
|4|-0.0017 deg|
|5|51.7618 deg|
|6|-3.2709 deg|

</div>

En esta posición, el robot MH6 se encuentra completamente replegado o recogido, con los ejes articulados de forma compacta.
Esta configuración se utiliza principalmente para almacenamiento, transporte, ya que minimiza el espacio ocupado y reduce el riesgo de colisiones con el entorno.
Además, al mantener el brazo dentro de su radio mínimo de trabajo, se protege la integridad mecánica del manipulador y se facilita su mantenimiento o desconexión segura del sistema.

**Home 2**

<p align="center">
<img src="./Images/Home2.jpg" width="400">
</p>

<div align="center">
  
|**Eje**|**Posición**|
|-------|------------|
|1|0 deg|
|2|-1.5665 deg|
|3|1.7135 deg|
|4|0 deg|
|5|-0.1468 deg|
|6|-3.2709 deg|
</div>

En esta posición, el robot MH6 adopta una postura extendida, con el brazo y el efector final orientados hacia el área de trabajo.
Esta configuración se considera el punto de partida ideal para operaciones de manipulación o movimientos automáticos, ya que ofrece una visión clara del entorno y una cinemática favorable para alcanzar distintas posiciones sin limitaciones articulares.
También puede emplearse como posición de referencia o inicio de trayectorias programadas, facilitando tanto la enseñanza manual como la ejecución de rutinas automáticas.

No existe una posición “mejor” que otra, cada configuración del robot cumple un propósito distinto.
La posición Home 1 (recogido) es ideal para almacenamiento, transporte o seguridad, mientras que la Home 2 (extendida) se usa como punto de partida para tareas de manipulación o enseñanza.
La elección depende del tipo de operación y del contexto de trabajo.


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
- Análisis comparativo entre RoboDK y RobotStudio, destacando ventajas, limitaciones y aplicaciones de cada herramienta.
#### RoboDK
Herramienta multimarca, flexible y extensible para simulación y programación offline; fuerte en integración de muchos modelos, scripting (Python) y generación de programas para distintos controladores.
  - **Ventajas:**
    - Compatibilidad multi-marca: soporte para decenas de fabricantes y modelos, por lo que sirve para entornos heterogéneos o para pruebas con distintos brazos.
    - Extensible y scriptable: API en Python y posibilidad de crear post-procesadores personalizados y plugins; ideal para automatización de flujos y para integrar CAM/CNC → robot.
    - Rápida puesta en marcha: interfaz orientada a generar programas offline con poco esfuerzo; buena para prototipado y pruebas conceptuales.
  - **Limitaciones:**
    - Fidelidad vs controlador real: aunque produce programas que funcionan en el robot real, la simulación no siempre reproduce exactamente el comportamiento del controlador del fabricante (pequeñas diferencias en cinemática, límites, comportamiento de control). Para validación final puede requerir ajustes y verificación en el controlador real.
    - Dependencia de post-procesadores: para generar código listo para el robot se necesitan post-procesadores correctos; si no existe uno perfecto para un controlador/versión concreta, será necesario crearlo o adaptarlo.
  - **Aplicaciones:**
    - Integraciones multi-marca (cuando tu célula tiene robots de distintos fabricantes).
    - Prototipado rápido, generación de trayectorias y post-procesado personalizado (p. ej. conversión de CAM a programas robot).
    - Escuelas, investigación y startups que necesitan flexibilidad y scripting en Python. 
#### RobotStudio
Herramienta propietaria ABB enfocada a simulación de alta fidelidad y programación offline para robots ABB (IRC5/RobotWare), con integración nativa, control virtual y simulación muy fiel al controlador real.
  - **Ventajas:**
    - Integración nativa con ABB (IRC5 / RobotWare): virtual controller y emulación que reproduce con alta fidelidad cómo el robot responderá en el mundo real (RAPID, I/O, señales, tiempos). Esto reduce riesgos al pasar a producción.
    - Alta fidelidad en trayectoria y tiempos de ciclo: modelos y física optimizados para validar trayectoria, verificación de colisiones y obtener estimaciones reales de tiempos.
    - Herramientas avanzadas de análisis: Signal Analyzer, Event Manager, verificación de I/O y conexión con controladores reales para pruebas integradas.
  - **Limitaciones:**
    - Enfoque propietario: optimizado para ABB; si trabajas con robots de otras marcas no es útil.
    - Costo y licencias: algunas funciones avanzadas (virtual controller, add-ins premium) pueden requerir licencias o módulos adicionales; hay conversas de usuarios sobre la curva de licenciamiento.
  - **Aplicaciones:**
    - Proyectos con robots ABB donde la fidelidad al controlador IRC5 es crítica: puesta en marcha virtual, verificación de colisiones, I/O y estimación de tiempos realistas.
    - Validación final de programas antes de subirlos a producción en sistemas ABB (reduce riesgos y tiempos de parada).    

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



