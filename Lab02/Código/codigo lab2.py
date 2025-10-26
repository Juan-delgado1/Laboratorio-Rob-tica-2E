from robodk.robolink import *    # API para comunicarte con RoboDK
from robodk.robomath import *    # Funciones matemáticas
import math
from matplotlib.textpath import TextPath
from matplotlib.patches import PathPatch
from matplotlib.transforms import Affine2D


#------------------------------------------------
# 1) Conexión a RoboDK e inicialización
#------------------------------------------------
RDK = Robolink()
robot = RDK.ItemUserPick("Selecciona un robot", ITEM_TYPE_ROBOT)

if not robot.Valid():
    raise Exception("No se ha seleccionado un robot válido.")

# Conectar al robot físico
if not robot.Connect():
    raise Exception("No se pudo conectar al robot. Verifica que esté en modo remoto y que la configuración sea correcta.")

# Confirmar conexión
if not robot.ConnectedState():
    raise Exception("El robot no está conectado correctamente. Revisa la conexión.")

print("Robot conectado correctamente.")

#------------------------------------------------
# 2) Cargar el Frame existente
#------------------------------------------------
frame_name = "Frame_from_Target1"
frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
if not frame.Valid():
    raise Exception(f'No se encontró el Frame "{frame_name}" en la estación.')

robot.setPoseFrame(frame)
#robot.setPoseTool(robot.PoseTool())

# Ajustes de velocidad
robot.setSpeed(300)
robot.setRounding(5)

#------------------------------------------------
# 3) Parámetros del cardioide
#------------------------------------------------
num_points = 400      # resolución de la curva
A = 150           # factor de escala para el tamaño
z_surface = 0         # plano de dibujo
z_safe = 50           # altura segura


#------------------------------------------------
# 4) Movimiento al centro (inicio)
#------------------------------------------------
robot.MoveJ(transl(315, 0, z_surface - z_safe))
robot.MoveL(transl(315, 0, z_surface))

#------------------------------------------------
# 5) Dibujar el cardioide r = -1 + cos(theta)
#------------------------------------------------
full_turn = 2 * math.pi

for i in range(num_points + 1):
    t = i / num_points
    theta = full_turn * t

    r = -1 + math.cos(theta)          # ecuación del cardioide
    x = A * r * math.cos(theta)+315
    y = A * r * math.sin(theta)

    robot.MoveL(transl(x, y, z_surface))

#------------------------------------------------
#7) Nombres de los integrantes
# JUAN X2 SANTI
#------------------------------------------------
texto = "JUAN X2 SANTI"         
tp = TextPath((0,0), texto, size=75)

X,Y = tp.vertices[:, 0], tp.vertices[:,1]

for i in range(len(X)):
    x = Y[i]
    y = X[i]-300
    robot.MoveL(transl(x, y, z_surface))

robot.MoveL(transl(x, y, z_surface - z_safe))
robot.MoveL(transl(0, 0, z_surface - z_safe))





#------------------------------------------------
# 6) Subir a altura segura
#------------------------------------------------
robot.MoveL(transl(0, 0, z_surface - z_safe))

print(f"¡Cardioide completado en el frame '{frame_name}'!")
