# ********************************************* Control + Dibujo de letras (turtlesim ROS2) *********************************************

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
import curses
import time
import math


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

        # Pub cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Servicios
        self.clear_client = self.create_client(Empty, '/clear')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /clear...')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /turtle1/set_pen...')

        # Parámetros de dibujo
        self.lin_speed = 1.8      # unidades/s (turtlesim)
        self.ang_speed = 2.2      # rad/s
        self.rate_hz = 60.0       # frecuencia de publicación durante trazos

        # Tamaño de letra (escala)
        self.H = 2.6   # altura
        self.W = 1.6   # ancho
        self.SPACE = 0.7

        # Estado "plotter" local (relativo al inicio de la letra)
        self._lx = 0.0
        self._ly = 0.0
        self._ltheta = 0.0  # 0 = dirección "derecha" relativa (heading actual al iniciar la letra)

        # Pen por defecto encendido
        self.set_pen(off=False)

    # -------------------------- utilidades ROS --------------------------
    def publish_twist(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.publisher_.publish(msg)

    def stop(self):
        self.publish_twist(0.0, 0.0)

    def _run_for(self, lin_x: float, ang_z: float, duration: float):
        if duration <= 0.0:
            return
        dt = 1.0 / self.rate_hz
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < duration:
            self.publish_twist(lin_x, ang_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
        self.stop()

    def move_dist(self, dist: float):
        sgn = 1.0 if dist >= 0 else -1.0
        duration = abs(dist) / self.lin_speed
        self._run_for(sgn * self.lin_speed, 0.0, duration)

    def turn_angle(self, ang: float):
        sgn = 1.0 if ang >= 0 else -1.0
        duration = abs(ang) / self.ang_speed
        self._run_for(0.0, sgn * self.ang_speed, duration)

    # -------------------------- servicios --------------------------
    def clear_trail(self):
        req = Empty.Request()
        fut = self.clear_client.call_async(req)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info('¡Trayectoria limpiada!')

    def set_pen(self, off: bool, r=255, g=255, b=255, width=3):
        req = SetPen.Request()
        req.r = int(r)
        req.g = int(g)
        req.b = int(b)
        req.width = int(width)
        req.off = bool(off)
        fut = self.pen_client.call_async(req)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)

    # -------------------------- “plotter” local para letras --------------------------
    def _reset_local_frame(self):
        self._lx = 0.0
        self._ly = 0.0
        self._ltheta = 0.0

    def _turn_to(self, target_theta: float):
        d = _wrap_to_pi(target_theta - self._ltheta)
        self.turn_angle(d)
        self._ltheta = _wrap_to_pi(self._ltheta + d)

    def _line_to(self, x: float, y: float):
        dx = x - self._lx
        dy = y - self._ly
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return
        theta = math.atan2(dy, dx)
        self._turn_to(theta)
        self.move_dist(dist)
        self._lx, self._ly = x, y

    def draw_strokes(self, strokes, advance=None):
        """
        strokes: lista de trazos; cada trazo es una lista de puntos [(x,y), (x,y), ...]
        advance: cuánto avanzar al final (si None, avanza W+SPACE)
        """
        self._reset_local_frame()
        adv = (self.W + self.SPACE) if advance is None else float(advance)

        for trazo in strokes:
            if not trazo:
                continue
            # mover sin dibujar al inicio del trazo
            self.set_pen(off=True)
            self._line_to(trazo[0][0], trazo[0][1])

            # dibujar el trazo
            self.set_pen(off=False)
            for (x, y) in trazo[1:]:
                self._line_to(x, y)

        # dejar listo para continuar “escribiendo” hacia la derecha
        self.set_pen(off=True)
        self._line_to(adv, 0.0)
        self.set_pen(off=False)

    # -------------------------- letras --------------------------
    def draw_J(self):
        H, W = self.H, self.W
        strokes = [[
            (0.0, H), (W, H), (W, 0.35*H), (0.65*W, 0.0), (0.20*W, 0.20*H)
        ]]
        self.draw_strokes(strokes)

    def draw_A(self):
        H, W = self.H, self.W
        strokes = [
            [(0.0, 0.0), (0.50*W, H), (W, 0.0)],
            [(0.25*W, 0.50*H), (0.75*W, 0.50*H)]
        ]
        self.draw_strokes(strokes)

    def draw_V(self):
        H, W = self.H, self.W
        strokes = [[(0.0, H), (0.50*W, 0.0), (W, H)]]
        self.draw_strokes(strokes)

    def draw_R(self):
        H, W = self.H, self.W
        strokes = [[
            (0.0, 0.0), (0.0, H), (0.75*W, H), (0.75*W, 0.60*H), (0.0, 0.60*H), (W, 0.0)
        ]]
        self.draw_strokes(strokes)

    def draw_D(self):
        H, W = self.H, self.W
        strokes = [[
            (0.0, 0.0), (0.0, H), (0.70*W, 0.85*H), (0.95*W, 0.50*H), (0.70*W, 0.15*H), (0.0, 0.0)
        ]]
        self.draw_strokes(strokes)

    def draw_E(self):
        H, W = self.H, self.W
        strokes = [
            [(0.0, 0.0), (0.0, H), (W, H)],
            [(0.0, 0.50*H), (0.70*W, 0.50*H)],
            [(0.0, 0.0), (W, 0.0)]
        ]
        self.draw_strokes(strokes)

    def draw_S(self):
        H, W = self.H, self.W
        strokes = [[
            (W, H), (0.0, H), (0.0, 0.50*H), (W, 0.50*H), (W, 0.0), (0.0, 0.0)
        ]]
        self.draw_strokes(strokes)

    def draw_M(self):
        H, W = self.H, self.W
        strokes = [[
            (0.0, 0.0), (0.0, H), (0.50*W, 0.55*H), (W, H), (W, 0.0)
        ]]
        self.draw_strokes(strokes)

    def draw_C(self):
        H, W = self.H, self.W
        strokes = [[(W, H), (0.0, H), (0.0, 0.0), (W, 0.0)]]
        self.draw_strokes(strokes)

    # -------------------------- loop teclado --------------------------
    def control_loop(self, stdscr):
        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.keypad(True)
        stdscr.clear()

        stdscr.addstr(0, 0, "↑↓←→ mover | 't' limpiar | 'q' salir | Letras: J A V R D E S M C")
        stdscr.refresh()

        while rclpy.ok():
            key = stdscr.getch()

            # si no hay tecla, igual publicamos 0 para mantener quieta la tortuga
            msg = Twist()

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

            elif key in (ord('t'), ord('T')):
                self.clear_trail()

            elif key in (ord('q'), ord('Q')):
                break

            # Letras (mayúscula/minúscula)
            elif key in (ord('j'), ord('J')):
                self.draw_J()
            elif key in (ord('a'), ord('A')):
                self.draw_A()
            elif key in (ord('v'), ord('V')):
                self.draw_V()
            elif key in (ord('r'), ord('R')):
                self.draw_R()
            elif key in (ord('d'), ord('D')):
                self.draw_D()
            elif key in (ord('e'), ord('E')):
                self.draw_E()
            elif key in (ord('s'), ord('S')):
                self.draw_S()
            elif key in (ord('m'), ord('M')):
                self.draw_M()
            elif key in (ord('c'), ord('C')):
                self.draw_C()

            else:
                # si no se presiona nada "útil", manda 0 para frenar suave
                self.publisher_.publish(msg)

            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        curses.wrapper(node.control_loop)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
