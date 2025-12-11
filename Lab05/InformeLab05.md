# Laboratorio No. 05 Pincher Phantom X100- ROS Humble- RVIZ

## Integrantes

**Juan Ángel Vargas Rodríguez**
juvargasro@unal.edu.co

**Santiago Mariño Cortés**
smarinoc@unal.edu.co

**Juan José Delgado Estrada**
judelgadoe@unal.edu.co

## Introducción

El presente laboratorio se realiza para aplicar de forma práctica y en un entorno real el aprendizaje de cinemática directa mediante la manipulación de robots Phantom X Pincher, para ello se emplean entornos como python, ROS 2 y el uso de servomotores Dynamixel AX-12.

Adicionalmente con el fin de enviar comandos y poses específicas al robot se implementó una interfaz gráfica, de forma que las configuraciones del robot se puedan observar en tiempo real y validar las posiciones de los resultados de acuerdo a los modelos matemáticos como las matrices de transformación homogénea (MTH) y el modelo de Denavit-Hartenberg.

## Objetivos

- Crear todos los Joint Controllers con ROS para manipular servomotores Dynamixel AX-12 del robot Phantom X Pincher.
- Manipular los tópicos de estado y comando para todos los Joint Controllers del robot Phantom X Pincher.
- Manipular los servicios para todos los Joint Controllers del robot Phantom X Pincher.
- Conectar el robot Phantom X Pincher con Python usando ROS 2.


## Cinemática Directa Robot Pincher Phantom X100
Planteamos los siguientes ejes en el robot para realizar el DH

<p align="center">
  <img src="Images/DH_LAB_04.png" alt="Ejes" width="400">
</p>

Obteniendo el siguiente DH
## Parámetros Denavit-Hartenberg (DH) - PhantomX Pincher

| $i$ | $\theta_i$ (Ángulo) | $d_i$ (Desplazamiento) | $a_i$ (Longitud) | $\alpha_i$ (Torsión) |
| :---: | :---: | :---: | :---: | :---: |
| **1** | $q_1$ | $L_1$ (44 mm) | 0 | $\pi/2$ |
| **2** | $q_2 + \pi/2$ | 0 | $L_2$ (107.5 mm) | 0 |
| **3** | $q_3$ | 0 | $L_3$ (107.5 mm) | 0 |
| **4** | $q_4$ | 0 | $L_4$ (75.3 mm) | 0 |

> **Nota:** $q_i$ representa la variable de articulación. La articulación 2 tiene un offset de $\pi/2$.

Procedemos a poner esto en MATLAB para poder simularlo y usar sus funciones en puntos que veremos más adelante.
```C
L1 = 44
L2 = 107.5
L3 = 107.5
L4 = 75.3


%       theta    d     a     alpha
DH = [    0     L1    0.0    pi/2;  % L1
          0     0.0   L2     0   ;  % L2
          0     0.0   L3     0   ;  % L3
          0     0.0   L4     0   ]; % L4

robot = SerialLink(DH, 'name', 'Pincher')

robot.offset = [0 pi/2 0 0];  % offset para cada articulación

T_tool = trotz(-pi/2) * trotx(-pi/2);  % solo rotación, sin traslación

robot.tool = T_tool;

robot.teach()
```

Gracias a la libreria de Peter Corke obtenemos el siguiente robot en MATLAB:

<p align="center">
  <img src="Images/DH_MATLAB.png" alt="Ejes" width="400">
</p>

Ahora con esta implementación podemos realizar los puntos del taller que requieran una cinemática inversa de forma más fácil.

## Plano de planta del robot

## Ciclo Home - Posición objetivo

## Publisher - Suscriber

## Robotic Toolbox

## Interfaz gráfica y visualización con RViz

## Video de implementación de los scripts

## Conclusiones
