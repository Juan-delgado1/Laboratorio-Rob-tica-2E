# pincher_control/terminal_control.py

import math
import threading

import rclpy
from pincher_control.control_servo import PincherController


def main(args=None):
    rclpy.init(args=args)

    # Creamos el controlador de motores (mismo que la GUI)
    controller = PincherController()

    if not rclpy.ok():
        # Si algo falló en el init del controlador (puerto, baudrate, etc.)
        return

    # Lanzamos el spin en un hilo aparte para que se sigan publicando /joint_states
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()

    dxl_ids = controller.dxl_ids  # típicamente [1, 2, 3, 4, 5]

    print("\n=== Control por terminal - PhantomX Pincher X100 ===")
    print("Motores disponibles (IDs Dynamixel):", dxl_ids)
    print("Convención (típica): 1=waist, 2=shoulder, 3=elbow, 4=wrist, 5=gripper")
    print("Escribe:  ID  angulo_en_grados")
    print("Ejemplo:  2  45   (mueve el motor 2 a +45°)")
    print("Escribe 'q' o 'quit' para salir.\n")

    try:
        while True:
            line = input(">> ").strip()

            if line in ("q", "quit", "exit"):
                print("Saliendo...")
                break

            if not line:
                continue

            parts = line.split()
            if len(parts) != 2:
                print("Formato inválido. Usa: ID angulo_en_grados   (ej: 3  30)")
                continue

            try:
                motor_id = int(parts[0])
                angle_deg_joint = float(parts[1])
            except ValueError:
                print("Error: ID debe ser entero y el ángulo un número. Ej: 2  -45")
                continue

            # Validar que el ID exista en la lista de motores
            if motor_id not in dxl_ids:
                print(f"ID {motor_id} no está en la lista de motores: {dxl_ids}")
                continue

            

            # Validar límites articulares (en grados)
            if (angle_deg_joint < -150.0 or angle_deg_joint > 150.0) and (motor_id == 1):
                print("Ángulo fuera de límites [-150°, 150°]. No se envía comando.")
                continue
            elif (angle_deg_joint < -100.0 or angle_deg_joint > 100.0) and (motor_id == 2):
                print("Ángulo fuera de límites [-100°, 100°]. No se envía comando.")
                continue
            elif (angle_deg_joint < -120.0 or angle_deg_joint > 120.0) and (motor_id == 3):
                print("Ángulo fuera de límites [-120°, 120°]. No se envía comando.")
                continue
            elif (angle_deg_joint < -120.0 or angle_deg_joint > 120.0) and (motor_id == 4):
                print("Ángulo fuera de límites [-120°, 120°]. No se envía comando.")
                continue
            elif (angle_deg_joint < 0 or angle_deg_joint > 120.0) and (motor_id == 5):
                print("Ángulo fuera de límites [0, 120°]. No se envía comando.")
                continue

            # Convertir a radianes (ángulo de la articulación en el modelo)
            angle_rad_joint = math.radians(angle_deg_joint)

            # Ajuste de signo según el mapeo motor ↔ articulación
            sign = controller.joint_sign.get(motor_id, 1)

            # Este es el ángulo que debe ver el motor (antes del signo en RViz)
            angle_rad_motor = angle_rad_joint * sign

            # Convertir radianes a valor Dynamixel (0–1023)
            goal_dxl = controller.radians_to_dxl(angle_rad_motor)

            # Mandar comando al motor
            controller.move_motor(motor_id, goal_dxl)

            print(
                f"Motor {motor_id}: {angle_deg_joint:.1f}° "
                f"(motor_rad={angle_rad_motor:.3f}, dxl={goal_dxl})"
            )

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")

    finally:
        # Apagar torque y cerrar puerto
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
