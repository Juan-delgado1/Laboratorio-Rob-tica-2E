#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
# Importamos TeleportAbsolute para la función Home
from turtlesim.srv import SetPen, TeleportAbsolute 
import curses
import time
import math

# --- Utilidades Matemáticas ---
def _wrap_to_pi(a: float) -> float:
    """Normaliza ángulo a (-pi, pi]."""
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # 1. Publicador de velocidad
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 2. Clientes de Servicios
        self.clear_client = self.create_client(Empty, '/clear')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # Esperar a que los servicios estén listos
        if not self.clear_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio /clear no disponible')
        if not self.pen_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio /set_pen no disponible')
        if not self.teleport_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio /teleport_absolute no disponible')

        # 3. Parámetros de dibujo
        self.lin_speed = 2.0      # Velocidad lineal (m/s)
        self.ang_speed = 2.0      # Velocidad de giro estándar (rad/s)
        self.rate_hz = 60.0       # Frecuencia de actualización

        # Dimensiones de las letras
        self.H = 3.0   # Altura
        self.W = 2.0   # Ancho
        self.SPACE = 1.0 # Espacio entre letras

        # 4. Estado "Odometría Interna" (Posición relativa estimada)
        self._lx = 0.0
        self._ly = 0.0
        self._ltheta = 0.0  # 0.0 = Mirando a la derecha

        # Configuración inicial
        self.set_pen(off=False)

    # -------------------------- UTILIDADES ROS --------------------------
    def publish_twist(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.publisher_.publish(msg)

    def stop(self):
        self.publish_twist(0.0, 0.0)

    def _run_for(self, lin_x: float, ang_z: float, duration: float):
        """Ejecuta una velocidad por un tiempo determinado."""
        if duration <= 0.0:
            return
        dt = 1.0 / self.rate_hz
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < duration:
            self.publish_twist(lin_x, ang_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
        self.stop()

    def set_pen(self, off: bool, r=255, g=255, b=255, width=3):
        """Sube o baja el lápiz y cambia color."""
        req = SetPen.Request()
        req.r, req.g, req.b = int(r), int(g), int(b)
        req.width = int(width)
        req.off = bool(off)
        self.pen_client.call_async(req)
        # Pequeña pausa para asegurar que el comando llegue
        time.sleep(0.1)

    def clear_trail(self):
        req = Empty.Request()
        self.clear_client.call_async(req)

    def go_home(self):
        """Vuelve al centro (5.5, 5.5) y resetea el estado."""
        self.set_pen(off=True)
        
        req = TeleportAbsolute.Request()
        req.x = 5.544445
        req.y = 5.544445
        req.theta = 0.0
        
        fut = self.teleport_client.call_async(req)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            
        self._reset_local_frame()
        self.set_pen(off=False)
        self.get_logger().info('Tortuga en Casa (Home)')

    # -------------------------- MOTORES DE MOVIMIENTO --------------------------
    
    def _reset_local_frame(self):
        self._lx = 0.0
        self._ly = 0.0
        self._ltheta = 0.0

    def move_dist(self, dist: float):
        """Mueve en línea recta la distancia especificada."""
        sgn = 1.0 if dist >= 0 else -1.0
        duration = abs(dist) / self.lin_speed
        self._run_for(sgn * self.lin_speed, 0.0, duration)

    def turn_angle(self, ang: float):
        """Gira sobre su eje."""
        sgn = 1.0 if ang >= 0 else -1.0
        duration = abs(ang) / self.ang_speed
        self._run_for(0.0, sgn * self.ang_speed, duration)

    def _turn_to_heading(self, target_heading):
        """Gira para mirar a una dirección absoluta (relativa al inicio de letra)."""
        diff = _wrap_to_pi(target_heading - self._ltheta)
        self.turn_angle(diff)
        self._ltheta = _wrap_to_pi(self._ltheta + diff)

    def _line_to(self, x: float, y: float):
        """Calcula giro y avance para ir al punto (x,y)."""
        dx = x - self._lx
        dy = y - self._ly
        dist = math.hypot(dx, dy)
        if dist < 1e-4: return

        theta = math.atan2(dy, dx)
        self._turn_to_heading(theta)
        self.move_dist(dist)
        
        self._lx, self._ly = x, y

    def move_arc(self, radius: float, angle: float):
        """
        Dibuja un arco suave.
        angle: positivo (izq), negativo (der).
        """
        if abs(angle) < 0.001: return

        lin_vel = self.lin_speed
        # w = v / r
        ang_vel = (lin_vel / radius) * (1.0 if angle > 0 else -1.0)
        duration = abs(angle / ang_vel)

        self._run_for(lin_vel, ang_vel, duration)

        # Actualizar odometría interna (Trigonometría de la cuerda)
        chord_len = 2.0 * radius * math.sin(abs(angle) / 2.0)
        chord_angle = self._ltheta + (angle / 2.0)
        
        self._lx += chord_len * math.cos(chord_angle)
        self._ly += chord_len * math.sin(chord_angle)
        self._ltheta = _wrap_to_pi(self._ltheta + angle)

    def draw_strokes_linear(self, strokes):
        """Helper para letras hechas solo de rectas (como A, M, E)."""
        self._reset_local_frame()
        for trazo in strokes:
            self.set_pen(off=True)
            self._line_to(trazo[0][0], trazo[0][1])
            self.set_pen(off=False)
            for (x, y) in trazo[1:]:
                self._line_to(x, y)
        
        # Preparar para siguiente letra
        self.set_pen(off=True)
        self._line_to(self.W + self.SPACE, 0.0)

    # -------------------------- LETRAS (IMPLEMENTACIÓN) --------------------------

    # --- Letras Lineales (Usan draw_strokes) ---
    def draw_A(self):
        H, W = self.H, self.W
        strokes = [
            [(0.0, 0.0), (W/2, H), (W, 0.0)],      # ^
            [(W*0.25, H*0.5), (W*0.75, H*0.5)]     # -
        ]
        self.draw_strokes_linear(strokes)

    def draw_E(self):
        H, W = self.H, self.W
        strokes = [
            [(W, H), (0.0, H), (0.0, 0.0), (W, 0.0)], # [
            [(0.0, H*0.5), (W*0.8, H*0.5)]            # -
        ]
        self.draw_strokes_linear(strokes)

    def draw_M(self):
        H, W = self.H, self.W
        strokes = [[
            (0.0, 0.0), (0.0, H), (W/2, H*0.6), (W, H), (W, 0.0)
        ]]
        self.draw_strokes_linear(strokes)

    def draw_V(self):
        H, W = self.H, self.W
        strokes = [[(0.0, H), (W/2, 0.0), (W, H)]]
        self.draw_strokes_linear(strokes)

    # --- Letras Curvas (Procedurales con move_arc) ---
    
    def draw_C(self):
        self._reset_local_frame()
        H, W = self.H, self.W
        
        # Inicio arriba-derecha
        self.set_pen(off=True)
        self._line_to(W, H*0.85)
        self._turn_to_heading(math.pi * 0.8) # Orientar hacia atras
        
        self.set_pen(off=False)
        # Arco grande
        self.move_arc(radius=W*0.55, angle=4.5) 
        
        # Salida
        self.set_pen(off=True)
        self._line_to(W + self.SPACE, 0)

    def draw_S(self):
        self._reset_local_frame()
        H, W = self.H, self.W
        
        self.set_pen(off=True)
        self._line_to(W, H*0.85)
        self._turn_to_heading(math.pi)
        
        self.set_pen(off=False)
        r = H * 0.22
        self.move_arc(r, math.pi)   # Curva superior
        self.move_arc(r, -math.pi)  # Curva inferior
        
        self.set_pen(off=True)
        self._line_to(W + self.SPACE, 0)

    def draw_J(self):
        self._reset_local_frame()
        H, W = self.H, self.W
        
        # Techo
        self.set_pen(off=True)
        self._line_to(0, H)
        self.set_pen(off=False)
        self._line_to(W, H)
        
        # Bajar y gancho
        self.set_pen(off=True)
        self._line_to(W*0.6, H)
        self._turn_to_heading(-math.pi/2)
        self.set_pen(off=False)
        
        self.move_dist(H*0.6)
        self._lx += math.cos(self._ltheta) * (H*0.6)
        self._ly += math.sin(self._ltheta) * (H*0.6)
        
        self.move_arc(W*0.35, -math.pi)
        
        self.set_pen(off=True)
        self._line_to(W + self.SPACE, 0)

    def draw_R(self):
        self._reset_local_frame()
        H, W = self.H, self.W
        
        # Palo
        self.set_pen(off=True)
        self._line_to(0,0)
        self.set_pen(off=False)
        self._line_to(0, H)
        
        # Panza
        self._turn_to_heading(0)
        self.move_dist(W*0.3)
        self._lx += W*0.3
        
        self.move_arc(H*0.25, -math.pi)
        
        self.move_dist(W*0.3)
        self._line_to(0, H*0.5) # Cerrar al palo
        
        # Pata
        self._turn_to_heading(-math.pi/4)
        self._line_to(W, 0)
        
        self.set_pen(off=True)
        self._line_to(W + self.SPACE, 0)

    def draw_D(self):
        self._reset_local_frame()
        H, W = self.H, self.W
        
        self.set_pen(off=True)
        self._line_to(0, 0)
        self.set_pen(off=False)
        self._line_to(0, H)
        
        self._turn_to_heading(0)
        self.move_dist(W*0.3)
        self._lx += W*0.3
        
        self.move_arc(H*0.5, -math.pi)
        
        self._line_to(0,0)
        
        self.set_pen(off=True)
        self._line_to(W + self.SPACE, 0)

    # -------------------------- BUCLE PRINCIPAL --------------------------
    def control_loop(self, stdscr):
        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.keypad(True)
        stdscr.clear()
        
        # Mensaje de ayuda en pantalla
        stdscr.addstr(0, 0, "--- TURTLE PLOTTER ---")
        stdscr.addstr(1, 0, "Teclas: J A V R D E S M C")
        stdscr.addstr(2, 0, "Controles: 'h' (Home), 't' (Limpiar), 'q' (Salir)")
        stdscr.addstr(3, 0, "Flechas: Movimiento manual")
        stdscr.refresh()

        while rclpy.ok():
            key = stdscr.getch()
            msg = Twist()

            if key != -1: # Si se presionó algo
                # Movimiento Manual
                if key == curses.KEY_UP:
                    msg.linear.x = 2.0
                    self.publisher_.publish(msg)
                elif key == curses.KEY_DOWN:
                    msg.linear.x = -2.0
                    self.publisher_.publish(msg)
                elif key == curses.KEY_LEFT:
                    msg.angular.z = 2.0
                    self.publisher_.publish(msg)
                elif key == curses.KEY_RIGHT:
                    msg.angular.z = -2.0
                    self.publisher_.publish(msg)
                
                # Comandos
                elif key in (ord('t'), ord('T')):
                    self.clear_trail()
                    stdscr.addstr(5, 0, "Comando: Limpiar Pantalla   ")
                elif key in (ord('h'), ord('H')):
                    stdscr.addstr(5, 0, "Comando: Volviendo a Casa...")
                    stdscr.refresh()
                    self.go_home()
                    stdscr.addstr(5, 0, "Comando: Casa (Home) Listo  ")
                elif key in (ord('q'), ord('Q')):
                    break
                
                # Letras
                else:
                    char_pressed = chr(key).upper() if 0 < key < 256 else '?'
                    if char_pressed in "JAVRDESMC":
                        stdscr.addstr(5, 0, f"Dibujando: {char_pressed}           ")
                        stdscr.refresh()
                        
                        if char_pressed == 'J': self.draw_J()
                        elif char_pressed == 'A': self.draw_A()
                        elif char_pressed == 'V': self.draw_V()
                        elif char_pressed == 'R': self.draw_R()
                        elif char_pressed == 'D': self.draw_D()
                        elif char_pressed == 'E': self.draw_E()
                        elif char_pressed == 'S': self.draw_S()
                        elif char_pressed == 'M': self.draw_M()
                        elif char_pressed == 'C': self.draw_C()
                        
                        stdscr.addstr(5, 0, f"Dibujo {char_pressed} Terminado     ")

            else:
                # Freno suave si no hay tecla
                self.publisher_.publish(msg)

            rclpy.spin_once(self, timeout_sec=0.05)
            stdscr.refresh()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        curses.wrapper(node.control_loop)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()