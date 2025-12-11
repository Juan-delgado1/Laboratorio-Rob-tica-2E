import math
import threading
import time

import numpy as np
import rclpy
import roboticstoolbox as rtb
from spatialmath import SE3
from roboticstoolbox.backends.PyPlot import PyPlot

from pincher_control.control_servo import PincherController


def build_pincher_robot():
    """
    Modelo DH del Pincher equivalente al de MATLAB:

    L1 = 44
    L2 = 107.5
    L3 = 107.5
    L4 = 75.3

    %       theta   d    a    alpha
    DH = [    0    L1   0.0   pi/2;
              0    0.0  L2    0   ;
              0    0.0  L3    0   ;
              0    0.0  L4    0   ];

    robot.offset = [0 pi/2 0 0];
    T_tool = trotz(-pi/2) * trotx(-pi/2);
    """

    # mm -> m
    L1 = 44.0 / 1000.0
    L2 = 107.5 / 1000.0
    L3 = 107.5 / 1000.0
    L4 = 75.3 / 1000.0

    links = [
        rtb.RevoluteDH(d=L1,  a=0.0,  alpha=math.pi/2, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L2,  alpha=0.0,        offset=math.pi/2),
        rtb.RevoluteDH(d=0.0, a=L3,  alpha=0.0,        offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L4,  alpha=0.0,        offset=0.0),
    ]

    robot = rtb.DHRobot(links, name="Pincher")

    # Herramienta: T_tool = trotz(-pi/2) * trotx(-pi/2)
    T_tool = SE3.Rz(-math.pi/2) * SE3.Rx(-math.pi/2)
    robot.tool = T_tool

    return robot


def main(args=None):
    # ------------ ROS + Dynamixel ------------
    rclpy.init(args=args)
    controller = PincherController()

    if not rclpy.ok():
        print("[ERROR] No se pudo inicializar PincherController (revisa /dev/ttyUSB0, baudrate, etc.)")
        return

    # BAJAR LA VELOCIDAD de todos los motores (por ejemplo 50 sobre 0–1023)
    controller.update_speed(50)

    # Spin del nodo en segundo plano (para que siga publicando /joint_states)
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()

    # ------------ Modelo + backend gráfico ------------
    robot = build_pincher_robot()

    pyplot = PyPlot()
    pyplot.launch()
    pyplot.add(robot)

    # Pose inicial tomada de lo que publica joint_states (current_joint_positions)
    q_actual = controller.current_joint_positions[:4]
    robot.q = np.array(q_actual)
    pyplot.step(0.01)

    print("\n=== Toolbox LIVE con ROS + Dynamixel ===")
    print("Se abrió una ventana 3D con el Pincher.")
    print("En esta terminal ingresa [waist shoulder elbow wrist] en grados.")
    print("Ejemplo:  30 -40 50 0")
    print("Límites recomendados: [-150°, 150°]")
    print("La ventana NO se cierra; se actualiza con los ángulos que el nodo publica en /joint_states.")
    print("Escribe 'q' para salir.\n")

    try:
        while True:
            line = input("q_deg (4 valores): ").strip()

            if line in ("q", "quit", "exit"):
                print("Saliendo...")
                break

            if not line:
                continue

            parts = line.split()
            if len(parts) != 4:
                print("Debes ingresar EXACTAMENTE 4 ángulos (waist shoulder elbow wrist).")
                continue

            try:
                q_deg_cmd = [float(p) for p in parts]
            except ValueError:
                print("Todos los valores deben ser números. Ejemplo: 30 -40 50 0")
                continue

            # Comprobar límites
            if any(a < -150.0 or a > 150.0 for a in q_deg_cmd):
                print("Algún ángulo está fuera de [-150°, 150°]. No se envía.")
                continue

            # ---- 1) Convertir a radianes (modelo DH) ----
            q_rad_cmd = [math.radians(a) for a in q_deg_cmd]

            # ---- 2) Enviar al robot real usando Dynamixel ----
            ids = controller.dxl_ids[:4]  # asumimos [1,2,3,4] = [waist, shoulder, elbow, wrist]

            for i, motor_id in enumerate(ids):
                sign = controller.joint_sign.get(motor_id, 1)
                motor_angle = q_rad_cmd[i] * sign   # ángulo que debe ver el motor

                goal_dxl = controller.radians_to_dxl(motor_angle)
                controller.move_motor(motor_id, goal_dxl)

            print(f"Comando enviado (deg): {q_deg_cmd}")
            print("Actualizando visualización con lo que publica /joint_states ...")

            # ---- 3) Actualizar la visualización usando LAS POSICIONES QUE PUBLICA EL NODO ----
            # Es decir, usamos controller.current_joint_positions, que es lo que se
            # manda en /joint_states. Damos un pequeño tiempo para que se actualice.
            total_time = 2.0   # segundos de "seguimiento"
            dt = 0.05
            t0 = time.time()

            while time.time() - t0 < total_time:
                q_actual = controller.current_joint_positions[:4]   # <-- mismos datos que /joint_states
                robot.q = np.array(q_actual)
                pyplot.step(dt)
                time.sleep(dt)

            # Mostrar por consola última lectura
            q_actual = controller.current_joint_positions[:4]
            q_actual_deg = [math.degrees(a) for a in q_actual]
            print(f"joint_states (deg aprox): waist={q_actual_deg[0]:.1f}, "
                  f"shoulder={q_actual_deg[1]:.1f}, elbow={q_actual_deg[2]:.1f}, "
                  f"wrist={q_actual_deg[3]:.1f}\n")

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")

    finally:
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
