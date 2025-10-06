# Laboratorio No. 01 Robótica Industrial - Trayectorias, Entradas y Salidas Digitales.

## Integrantes

**Juan Ángel Vargas Rodríguez**
juvargasro@unal.edu.co

**Santiago Mariño Cortés**
smarinoc@unal.edu.co

**Juan José Delgado Estrada**
judelgadoe@unal.edu.co

## Introducción
En la práctica se buscaba manipular un robot industrial IRB 140 con el objetivo de familiarizarse con esta máquina mediante la realización de una rutina de trabajo que consistía en la decoración de una superficie que simulara a una torta de cumpleaños y de esta forma aplicar conceptos vistos en clase de forma práctica. Para ello se tuvieron que identificar las partes IRB 140 y su funcionamiento para poder realizar el proceso de diseño una herramienta que se acoplara al manipulador y desarrollar una rutina de código conociendo las limitaciones y características claves tanto del IRB 140 como del entorno de RobotStudio.

## Planteamiento del problema
En la industria de alimentos, en especial panadería, quieren mejorar su proceso de producción. La decoración de tortas es una actividad que se cree que puede ser ejecutada por robots, es por ello que se propuso realizar dicha actividad sobre una superficie plana, escribiendo los nombres de cada integrante del grupo y dibujar algún tipo de decoración adicional. El desarrollo de la actividad tenía las siguientes limitaciones:

- El tamaño de la torta debe ser para 20 personas.
- Las trayectorias desarrolladas deberán realizarse en un rango de velocidades entre 100 y 1000.
- La zona tolerable de errores máxima debe ser de z10.
- El movimiento debe partir de una posición home especificada (puede ser el home del robot) y realizar la trayectoria de cada palabra y decoración con un trazo continuo. El movimiento debe finalizar en la misma posición de home en la que se inició.
- La decoración de la torta debe ser realizada sobre una torta virtual.
- Los nombres deben estar separados.

## Objetivos
- Conocer los elementos de un robot industrial.
- Realizar la calibración de herramientas en el robot real, así como en RobotStudio.
- Identificar los tipos de movimientos en el espacio de la herramienta útiles para trabajos de manipulación.
- Ampliar el manejo de funciones proporcionadas por RobotStudio.
- Utilizar diversas funciones de RAPID.
- Utilizar el módulo de entradas y salidas digitales dispuesto en el controlador IRC5.

## Desarrollo de la práctica

En la etapa inicial de la práctica se realizó el reconocimiento del entorno de robotstudio así como la familiarización con el IRB 140 haciendo reconocimiento de sus partes y realizando movimientos básicos del manipulador de forma manual. Posteriormente vino la etapa de diseño de la herramienta para fijar el marcador al flanche del robot en el software CAD Inventor.

### Diseño de la herramienta

![Dimensiones Flanche](Images/especificaciones_flanche.png)

El diseño de la herramienta se realizó con base en las especificaciones del flanche encontradas en el manual de producto del robot IRB 140 y teniendo en cuenta la posición del marcador debería tener cierta inclinación para evitar que se presentaran singularidades en las articulaciones del manipulador y se debía colocar cierta tolerancia mediante un resorte que se contraía de acuerdo a la presión con la que el marcador hiciera contacto con la superficie de trabajo. De acuerdo a esto se realizó el siguiente diseño:

<p align="center">
  <img src="Images/herramienta.png" alt="Diseño CAD de la herrameinta" width="400">
</p>

La inclinación seleccionada fue de 45°, 2cm de espacio para un resorte de este largo; la unión de la herramienta al flanche se hace mediante tornillos M6.

Debido a que el marcador se debía poder colocar y retirar de la herramienta que lo acopla al flanche se optó por un diseño en dos partes, la parte inferior actúa como el soporte del marcador y como la base que se une al flanche, mientras que la parte superior sella el marcador con la herramienta y aporta seguridad para evitar movimientos no deseados respecto a la parte inferior. 

<p align="center">
  <img src="Images/base.png" alt="Soporte" width="400">
</p>

<p align="center">
  <img src="Images/tapa.png" alt="Soporte" width="400">
</p>

Otra consideración de la herramienta de acople del marcador son las dimensiones del marcador que se iba a utilizar, para ello se realizó el diseño 3D también del marcador mediante las medidas del objeto físico por medio de un calibrador pie de rey, para así con base a las dimensiones poder acoplarlo en la herramienta de soporte de forma óptima.

<p align="center">
  <img src="Images/marcador.png" alt="Marcador" width="400">
</p>

Con base al diseño CAD se realizó la manufactura de la herramienta mediante impresión 3D y el resultado fue el siguiente:

<p align="center">
  <img src="Images/impresion.jpeg" alt="Impresión" width="400">
</p>

Después de tener la herramienta ya fabricada se procedió con el montaje en el robot y la calibración del TCP que correspondía al punto final de la herramienta y será el contacto con la superficie. Para esto se calibró con una referencia (como se muestra en la siguiente imagen) la cuál debía ser alcanzada desde 4 posiciones diferentes del robot, logrando el menor error posible, después de varios intentos se logró un error de 4mm.

<p align="center">
  <img src="Images/calibración.jpeg" alt="Marcador" width="400">
</p>
Cómo siguiente paso se tenía la calibración del workObject que sería nuestra "torta" a decorar, para ello se usó una caja de madera con dos capas de papel contact en la parte superior para simular un tablero y poder borrar los trazos del marcador en caso de que se necesitara. Las dimensiones de la caja eran 203.55 mm de largo, 171.65 mm de ancho y 63.52 mm de alto.

<p align="center">
  <img src="Images/caja.jpeg" alt="Marcador" width="400">
</p>

### Planos de la herramienta
Los planos de la herramienta se encuentran en [Planos](Planos/), mientras que los modelos CAD en inventor se encuentran en [Inventor](Inventor/)
## Diagrama de flujo de acciones del robot.
### Diagrama de flujo principal:

<p align="center">
  <img src="Images/Diagrama_de_flujo_general.png" alt="Diagrama de flujo principal" width="400">
</p>

### Diagramas de flujo de los paths: 
**Proceso escritura**
<p align="center">
  <img src="Images/Diagrama_de_flujo_1.png" alt="Diagrama de flujo principal" width="400">
</p>

**Path Home**
<p align="center">
  <img src="Images/Diagrama_de_flujo_2.png" alt="Diagrama de flujo principal" width="400">
</p>

**Path Config**
<p align="center">
  <img src="Images/Diagrama_de _flujo_3.png" alt="Diagrama de flujo principal" width="400">
</p>

### Funciones Utilizadas

### Código en Rapid

El código utilizado en RAPID se encuentra en el archivo [Module1](Codigos/Module1.mod), así como los archivos de calibración de la herramienta [TCP tool2E2e](Codigos/TCP_tool2E2e.MOD) y del workobject [Wobj_work2E2e](Codigos/Wobj_work2E2e.MOD).

## Video de la simulación

## Video de la implementación física

<p align="center">
  <img src="Images/decorado.jpeg" alt="Diseño CAD de la herrameinta" width="400">
</p>

## Conclusiones
