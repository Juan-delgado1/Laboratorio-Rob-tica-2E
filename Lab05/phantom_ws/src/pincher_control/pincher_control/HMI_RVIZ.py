import sys
import math
import threading
import subprocess  # para lanzar RViz como proceso externo

import numpy as np
import rclpy

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QSlider, QPushButton, QLineEdit, QTabWidget,
    QSizePolicy, QGroupBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import roboticstoolbox as rtb
from spatialmath import SE3

# Importa tu controlador actual
from pincher_control.control_servo import PincherController


# ==========================
#  Modelo DH del Pincher
# ==========================

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
        rtb.RevoluteDH(d=L1,  a=0.0,  alpha=np.pi / 2, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L2,   alpha=0.0,       offset=np.pi / 2),
        rtb.RevoluteDH(d=0.0, a=L3,   alpha=0.0,       offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L4,   alpha=0.0,       offset=0.0),
    ]

    robot = rtb.DHRobot(links, name='Pincher')
    # Herramienta: rotación Z(-90°) X(-90°)
    robot.tool = SE3.Rz(-np.pi / 2) * SE3.Rx(-np.pi / 2)
    return robot


# ==========================
#  Ventana HMI con pestañas
# ==========================

class PincherHMI(QMainWindow):
    def __init__(self, controller: PincherController):
        super().__init__()
        self.controller = controller

        self.setWindowTitle("HMI Pincher - Qt5 + ROS2 + Toolbox")
        self.resize(1200, 800)

        # Modelo DH para la visualización e IK
        self.robot_model = build_pincher_robot()

        # Estados actuales de los ángulos (5 articulaciones, deg)
        self.current_angles_deg = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Límites articulares (deg)
        self.joint_limits = {
            1: (-150.0, 150.0),
            2: (-100.0, 100.0),
            3: (-120.0, 120.0),
            4: (-120.0, 120.0),
            5: (0.0, 120.0),
        }

        # Diccionarios para widgets
        self.sliders = {}         # motor_id -> QSlider (articular)
        self.angle_labels = {}    # motor_id -> QLabel (texto al lado del slider)
        self.numeric_edits = {}   # motor_id -> QLineEdit (numérico articular)

        self.xyz_edits = {}       # 'x'/'y'/'z' -> QLineEdit
        self.xyz_slider = {}              # 'x'/'y'/'z' -> QSlider
        self.xyz_slider_value_labels = {} # 'x'/'y'/'z' -> QLabel

        # Panel de info (derecha)
        self.info_joint_labels = {}  # motor_id -> QLabel
        self.info_xyz_labels = {}    # 'x','y','z' -> QLabel
        self.info_rpy_labels = {}    # 'roll','pitch','yaw' -> QLabel

        # Proceso de RViz (ros2 launch ...)
        self.rviz_process = None

        # -------- Layout principal --------
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout()
        central.setLayout(main_layout)

        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # Estado global (barra de estado superior)
        self.status_label = QLabel("Sistema listo")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 14px; color: #98c379;")
        main_layout.addWidget(self.status_label)

        # -------- Zona central: pestañas + visualización ----------

        center_layout = QHBoxLayout()
        main_layout.addLayout(center_layout)

        # ---- Pestañas a la izquierda ----
        self.tabs = QTabWidget()
        center_layout.addWidget(self.tabs, stretch=2)

        # Tab 1: Sliders articulares
        self.tab_sliders = QWidget()
        self.tabs.addTab(self.tab_sliders, "Control por sliders")
        self._build_tab_sliders()

        # Tab 2: Control numérico articular
        self.tab_numeric = QWidget()
        self.tabs.addTab(self.tab_numeric, "Control numérico")
        self._build_tab_numeric()

        # Tab 3: Control XYZ por valores
        self.tab_xyz = QWidget()
        self.tabs.addTab(self.tab_xyz, "Control en XYZ")
        self._build_tab_xyz()

        # Tab 4: Control XYZ con sliders
        self.tab_xyz_sliders = QWidget()
        self.tabs.addTab(self.tab_xyz_sliders, "Sliders XYZ")
        self._build_tab_xyz_sliders()

        # Tab 5: Visualización en RViz
        self.tab_rviz = QWidget()
        self.tabs.addTab(self.tab_rviz, "Visualización RViz")
        self._build_tab_rviz()

        # ---- Visualización + panel de info a la derecha ----
        vis_panel_layout = QVBoxLayout()
        center_layout.addLayout(vis_panel_layout, stretch=3)

        # ----- Información del grupo -----
        group_box = QGroupBox("Información del grupo")
        group_layout = QVBoxLayout()
        group_box.setLayout(group_layout)

        lbl_group = QLabel("Grupo: Robótica / Manipulador Pincher Phantom X100")
        lbl_names = QLabel("Integrantes: Juan J. Delgado, Juan A. Vargas, Santiago Mariño")
        lbl_univ = QLabel("Universidad Nacional de Colombia - Sede Bogotá")

        for w in (lbl_group, lbl_names, lbl_univ):
            w.setAlignment(Qt.AlignLeft)
            group_layout.addWidget(w)

        vis_panel_layout.addWidget(group_box)

        # ----- Visualización 3D + panel de estado -----
        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.ax = self.fig.add_subplot(111, projection="3d")

        info_panel = QGroupBox("Estado actual del manipulador")
        info_layout = QVBoxLayout()
        info_panel.setLayout(info_layout)

        # --- Posición XYZ del TCP ---
        xyz_group = QGroupBox("Posición TCP (mm)")
        xyz_layout = QGridLayout()
        xyz_group.setLayout(xyz_layout)
        for i, coord in enumerate(['x', 'y', 'z']):
            l = QLabel(f"{coord.upper()} =")
            xyz_layout.addWidget(l, i, 0)
            val_l = QLabel("0.0")
            xyz_layout.addWidget(val_l, i, 1)
            self.info_xyz_labels[coord] = val_l
        info_layout.addWidget(xyz_group)

        # --- Orientación RPY del TCP ---
        rpy_group = QGroupBox("Orientación TCP (RPY, deg)")
        rpy_layout = QGridLayout()
        rpy_group.setLayout(rpy_layout)
        for i, comp in enumerate(['roll', 'pitch', 'yaw']):
            l = QLabel(f"{comp.capitalize()} =")
            rpy_layout.addWidget(l, i, 0)
            val_l = QLabel("0.0")
            rpy_layout.addWidget(val_l, i, 1)
            self.info_rpy_labels[comp] = val_l
        info_layout.addWidget(rpy_group)

        # --- Ángulos articulares ---
        joint_group = QGroupBox("Ángulos articulares (deg)")
        joint_layout = QGridLayout()
        joint_group.setLayout(joint_layout)
        for i, motor_id in enumerate([1, 2, 3, 4, 5]):
            l = QLabel(f"Motor {motor_id}:")
            joint_layout.addWidget(l, i, 0)
            val_l = QLabel("0.0")
            joint_layout.addWidget(val_l, i, 1)
            self.info_joint_labels[motor_id] = val_l
        info_layout.addWidget(joint_group)

        right_bottom = QHBoxLayout()
        vis_panel_layout.addLayout(right_bottom)

        right_bottom.addWidget(self.canvas, stretch=3)
        right_bottom.addWidget(info_panel, stretch=2)

        # Dibujar primera vez
        self.update_robot_plot()

        # Poner velocidad moderada
        self.controller.update_speed(80)

        # Aplicar estilos bonitos
        self.apply_styles()

    # ============ Estilos (Qt Style Sheets) ============

    def apply_styles(self):
        style = """
        QMainWindow {
            background-color: #282c34;
        }

        QTabWidget::pane {
            border: 1px solid #3e4451;
            border-radius: 6px;
            background: #21252b;
        }

        QTabBar::tab {
            background: #3e4451;
            color: #dcdcdc;
            padding: 6px 12px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
            margin-right: 2px;
            font-size: 10pt;
        }

        QTabBar::tab:selected {
            background: #61afef;
            color: #000000;
        }

        QLabel {
            color: #dcdcdc;
            font-size: 10pt;
        }

        QPushButton {
            background-color: #61afef;
            color: #000000;
            border-radius: 6px;
            padding: 6px 12px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #72b7ff;
        }
        QPushButton:pressed {
            background-color: #4fa3de;
        }

        QLineEdit {
            background-color: #21252b;
            border: 1px solid #3e4451;
            border-radius: 4px;
            padding: 2px 4px;
            color: #dcdcdc;
        }

        QSlider::groove:horizontal {
            height: 6px;
            background: #3e4451;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #61afef;
            width: 14px;
            margin: -4px 0;
            border-radius: 7px;
        }

        QGroupBox {
            border: 1px solid #3e4451;
            border-radius: 6px;
            margin-top: 12px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
            color: #abb2bf;
            font-weight: bold;
        }
        """
        self.setStyleSheet(style)

    # ============ Pestaña 1: Sliders articulares ============

    def _build_tab_sliders(self):
        layout = QVBoxLayout()
        self.tab_sliders.setLayout(layout)

        title = QLabel("Control articular mediante sliders")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        group_sliders = QGroupBox("Sliders articulares (grados)")
        g_layout = QGridLayout()
        group_sliders.setLayout(g_layout)

        for idx, motor_id in enumerate([1, 2, 3, 4, 5]):
            row = idx

            name_label = QLabel(f"Motor {motor_id}")
            g_layout.addWidget(name_label, row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(self.joint_limits[motor_id][0]))
            slider.setMaximum(int(self.joint_limits[motor_id][1]))
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.setPageStep(5)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(10)
            slider.valueChanged.connect(
                lambda value, mid=motor_id: self.on_slider_changed(mid, value)
            )
            g_layout.addWidget(slider, row, 1)

            angle_label = QLabel("0°")
            g_layout.addWidget(angle_label, row, 2)

            self.sliders[motor_id] = slider
            self.angle_labels[motor_id] = angle_label

        layout.addWidget(group_sliders)

        # Poses predefinidas
        poses_group = QGroupBox("Poses predefinidas")
        poses_layout = QHBoxLayout()
        poses_group.setLayout(poses_layout)

        poses = [
            ("HOME",   [0,   0,   0,   0,   0]),
            ("Pose 2", [25,  25,  20, -20,  0]),
            ("Pose 3", [-35, 35, -30, 30,  0]),
            ("Pose 4", [85, -20, 55,  25,  0]),
            ("Pose 5", [80, -35, 55, -45,  0]),
        ]

        for label, pose in poses:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, p=pose: self.set_pose(p))
            poses_layout.addWidget(btn)

        layout.addWidget(poses_group)

    # ============ Pestaña 2: Control numérico articular ============

    def _build_tab_numeric(self):
        layout = QVBoxLayout()
        self.tab_numeric.setLayout(layout)

        title = QLabel("Control articular numérico (grados)")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        group = QGroupBox("Ingreso manual de ángulos")
        grid = QGridLayout()
        group.setLayout(grid)

        for idx, motor_id in enumerate([1, 2, 3, 4, 5]):
            row = idx

            name_label = QLabel(f"Motor {motor_id}")
            grid.addWidget(name_label, row, 0)

            edit = QLineEdit("0")
            grid.addWidget(edit, row, 1)
            self.numeric_edits[motor_id] = edit

            btn_move = QPushButton("Mover")
            btn_move.clicked.connect(lambda _, mid=motor_id: self.on_numeric_move(mid))
            grid.addWidget(btn_move, row, 2)

        layout.addWidget(group)

        btn_all = QPushButton("Mover todos (pose completa)")
        btn_all.clicked.connect(self.on_numeric_move_all)
        layout.addWidget(btn_all)

    # ============ Pestaña 3: XYZ por valores ============

    def _build_tab_xyz(self):
        layout = QVBoxLayout()
        self.tab_xyz.setLayout(layout)

        title = QLabel("Control en espacio de tarea (XYZ)")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        info = QLabel("Ingrese la posición deseada del TCP en milímetros (mm).")
        info.setAlignment(Qt.AlignCenter)
        layout.addWidget(info)

        group = QGroupBox("Posición objetivo del TCP")
        grid = QGridLayout()
        group.setLayout(grid)

        labels = [("X [mm]", 'x'), ("Y [mm]", 'y'), ("Z [mm]", 'z')]
        defaults = {'x': "150", 'y': "0", 'z': "100"}

        for row, (text, key) in enumerate(labels):
            lbl = QLabel(text)
            grid.addWidget(lbl, row, 0)

            edit = QLineEdit(defaults[key])
            grid.addWidget(edit, row, 1)
            self.xyz_edits[key] = edit

        layout.addWidget(group)

        btn_move_xyz = QPushButton("Mover a XYZ")
        btn_move_xyz.clicked.connect(self.on_task_move_xyz)
        layout.addWidget(btn_move_xyz)

    # ============ Pestaña 4: XYZ con sliders ============

    def _build_tab_xyz_sliders(self):
        layout = QVBoxLayout()
        self.tab_xyz_sliders.setLayout(layout)

        title = QLabel("Control del TCP en XYZ mediante sliders")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        info = QLabel("Mueva los sliders para cambiar la posición del TCP (mm).")
        info.setAlignment(Qt.AlignCenter)
        layout.addWidget(info)

        group = QGroupBox("Sliders XYZ")
        grid = QGridLayout()
        group.setLayout(grid)

        defaults = {'x': 150, 'y': 0, 'z': 100}

        for row, key in enumerate(['x', 'y', 'z']):
            if key in ['x', 'y']:
                min_val, max_val = -335, 335
            else:
                min_val, max_val = 0, 335

            lbl = QLabel(f"{key.upper()} [mm]")
            grid.addWidget(lbl, row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setValue(defaults[key])
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(10)
            slider.valueChanged.connect(
                lambda value, axis=key: self.on_xyz_slider_changed(axis, value)
            )
            grid.addWidget(slider, row, 1)

            val_label = QLabel(f"{defaults[key]} mm")
            grid.addWidget(val_label, row, 2)

            self.xyz_slider[key] = slider
            self.xyz_slider_value_labels[key] = val_label

        layout.addWidget(group)

    # ============ Pestaña 5: Visualización en RViz ============

    def _build_tab_rviz(self):
        layout = QVBoxLayout()
        self.tab_rviz.setLayout(layout)

        title = QLabel("Visualización en RViz (ROS2)")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        info = QLabel(
            "Esta pestaña lanza RViz con el modelo del PhantomX Pincher:\n"
            "  ros2 launch phantomx_pincher_description display.launch.py\n\n"
            "En RViz verás el modelo siguiendo los /joint_states."
        )
        info.setAlignment(Qt.AlignCenter)
        layout.addWidget(info)

        btn_layout = QHBoxLayout()
        layout.addLayout(btn_layout)

        self.btn_launch_rviz = QPushButton("Lanzar RViz")
        self.btn_launch_rviz.clicked.connect(self.launch_rviz)
        btn_layout.addWidget(self.btn_launch_rviz)

        self.btn_stop_rviz = QPushButton("Detener RViz")
        self.btn_stop_rviz.clicked.connect(self.stop_rviz)
        self.btn_stop_rviz.setEnabled(False)
        btn_layout.addWidget(self.btn_stop_rviz)

        self.lbl_rviz_status = QLabel("RViz no iniciado")
        self.lbl_rviz_status.setAlignment(Qt.AlignCenter)
        self.lbl_rviz_status.setStyleSheet("color: #dcdcdc;")
        layout.addWidget(self.lbl_rviz_status)

    def launch_rviz(self):
        # Si ya hay un proceso vivo, no lanzar otro
        if self.rviz_process is not None and self.rviz_process.poll() is None:
            self.status_label.setText("RViz ya está en ejecución")
            self.status_label.setStyleSheet("color: #e5c07b;")
            return

        try:
            # *** AQUÍ se usa el nuevo paquete y launch ***
            cmd = ["ros2", "launch", "phantomx_pincher_description", "display.launch.py"]
            self.rviz_process = subprocess.Popen(cmd)

            self.lbl_rviz_status.setText(
                "RViz ejecutándose (phantomx_pincher_description/display.launch.py)"
            )
            self.lbl_rviz_status.setStyleSheet("color: #98c379;")

            self.btn_launch_rviz.setEnabled(False)
            self.btn_stop_rviz.setEnabled(True)

            self.status_label.setText("RViz lanzado correctamente")
            self.status_label.setStyleSheet("color: #98c379;")
        except Exception as e:
            self.lbl_rviz_status.setText(f"Error al lanzar RViz: {e}")
            self.lbl_rviz_status.setStyleSheet("color: #e06c75;")
            self.status_label.setText("Error al lanzar RViz")
            self.status_label.setStyleSheet("color: #e06c75;")

    def stop_rviz(self):
        if self.rviz_process is not None:
            try:
                self.rviz_process.terminate()
            except Exception:
                pass
            self.rviz_process = None

        self.lbl_rviz_status.setText("RViz no iniciado")
        self.lbl_rviz_status.setStyleSheet("color: #dcdcdc;")

        self.btn_launch_rviz.setEnabled(True)
        self.btn_stop_rviz.setEnabled(False)

        self.status_label.setText("RViz detenido")
        self.status_label.setStyleSheet("color: #61afef;")

    # ============ Lógica de movimiento articular ============

    def on_slider_changed(self, motor_id: int, angle_deg: int):
        self.angle_labels[motor_id].setText(f"{angle_deg}°")
        self.current_angles_deg[motor_id - 1] = float(angle_deg)

        # Sincronizar con control numérico
        if motor_id in self.numeric_edits:
            self.numeric_edits[motor_id].setText(str(angle_deg))

        # Validar límites
        min_lim, max_lim = self.joint_limits[motor_id]
        if angle_deg < min_lim or angle_deg > max_lim:
            self.status_label.setText(f"Ángulo fuera de límites para motor {motor_id}")
            self.status_label.setStyleSheet("color: #e06c75;")
            return

        angle_rad = math.radians(angle_deg)

        # Ajustar por signo según controlador
        sign = getattr(self.controller, "joint_sign", {}).get(motor_id, 1)
        motor_angle = angle_rad * sign

        # Convertir a valor Dynamixel
        try:
            dxl_value = self.controller.radians_to_dxl(motor_angle)
        except AttributeError:
            dxl_value = int(motor_angle * (2048.0 / 2.618) + 2048)

        self.controller.move_motor(motor_id, dxl_value)

        self.update_robot_plot()

        self.status_label.setText(
            f"Motor {motor_id} → {angle_deg}° (DXL: {dxl_value})"
        )
        self.status_label.setStyleSheet("color: #61afef;")

    def on_numeric_move(self, motor_id: int):
        text = self.numeric_edits[motor_id].text()
        try:
            angle_deg = float(text)
        except ValueError:
            self.status_label.setText(f"Valor inválido para motor {motor_id}")
            self.status_label.setStyleSheet("color: #e06c75;")
            return

        min_lim, max_lim = self.joint_limits[motor_id]
        angle_deg = max(min(angle_deg, max_lim), min_lim)

        self.sliders[motor_id].setValue(int(angle_deg))

    def on_numeric_move_all(self):
        pose_deg = []
        for motor_id in [1, 2, 3, 4, 5]:
            text = self.numeric_edits[motor_id].text()
            try:
                val = float(text)
            except ValueError:
                val = 0.0
            min_lim, max_lim = self.joint_limits[motor_id]
            val = max(min(val, max_lim), min_lim)
            pose_deg.append(val)

        self.set_pose(pose_deg)

    def set_pose(self, pose_deg_list):
        for motor_id, angle_deg in enumerate(pose_deg_list, start=1):
            min_lim, max_lim = self.joint_limits[motor_id]
            angle_deg = max(min(angle_deg, max_lim), min_lim)
            self.sliders[motor_id].setValue(int(angle_deg))

        self.status_label.setText("Pose enviada")
        self.status_label.setStyleSheet("color: #98c379;")

    # ============ Control en XYZ (texto + sliders) ============

    def on_task_move_xyz(self):
        try:
            x_mm = float(self.xyz_edits['x'].text())
            y_mm = float(self.xyz_edits['y'].text())
            z_mm = float(self.xyz_edits['z'].text())
        except ValueError:
            self.status_label.setText("XYZ inválido (usa números)")
            self.status_label.setStyleSheet("color: #e06c75;")
            return

        self.move_to_xyz(x_mm, y_mm, z_mm)

        # Sincronizar sliders XYZ
        for key, val in zip(['x', 'y', 'z'], [x_mm, y_mm, z_mm]):
            if key in self.xyz_slider:
                self.xyz_slider[key].setValue(int(val))

    def on_xyz_slider_changed(self, axis_key: str, value: int):
        self.xyz_slider_value_labels[axis_key].setText(f"{value} mm")

        if axis_key in self.xyz_edits:
            self.xyz_edits[axis_key].setText(str(value))

        x_mm = self.xyz_slider['x'].value()
        y_mm = self.xyz_slider['y'].value()
        z_mm = self.xyz_slider['z'].value()

        self.move_to_xyz(x_mm, y_mm, z_mm)

    def move_to_xyz(self, x_mm: float, y_mm: float, z_mm: float):
        x = x_mm / 1000.0
        y = y_mm / 1000.0
        z = z_mm / 1000.0

        T_target = SE3(x, y, z)

        q0_rad = [math.radians(a) for a in self.current_angles_deg[:4]]

        try:
            sol = self.robot_model.ikine_LM(
                T_target, q0=q0_rad, mask=[1, 1, 1, 0, 0, 0]
            )
        except Exception as e:
            self.status_label.setText(f"Error IK: {e}")
            self.status_label.setStyleSheet("color: #e06c75;")
            return

        if not sol.success:
            self.status_label.setText("No hay solución IK para esa XYZ")
            self.status_label.setStyleSheet("color: #e06c75;")
            return

        q_rad_4 = sol.q
        q_deg_4 = np.degrees(q_rad_4)

        for i, motor_id in enumerate([1, 2, 3, 4]):
            lo, hi = self.joint_limits[motor_id]
            if q_deg_4[i] < lo - 1e-3 or q_deg_4[i] > hi + 1e-3:
                self.status_label.setText(
                    f"Solución IK fuera de límites (M{motor_id}: {q_deg_4[i]:.1f}°)"
                )
                self.status_label.setStyleSheet("color: #e06c75;")
                return

        pose_full_deg = [
            q_deg_4[0],
            q_deg_4[1],
            q_deg_4[2],
            q_deg_4[3],
            self.current_angles_deg[4],
        ]

        self.set_pose(pose_full_deg)

        self.status_label.setText(
            f"IK OK → TCP = ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm"
        )
        self.status_label.setStyleSheet("color: #98c379;")

    # ============ Visualización con Toolbox ============

    def update_robot_plot(self):
        q_deg_4 = self.current_angles_deg[:4]
        q_rad_4 = [math.radians(a) for a in q_deg_4]

        try:
            T_all = self.robot_model.fkine_all(q_rad_4, old=False)
        except TypeError:
            T_all = self.robot_model.fkine_all(q_rad_4)

        points = np.array([T.t for T in T_all])

        T_ee = self.robot_model.fkine(q_rad_4)
        tcp_xyz_m = T_ee.t
        tcp_xyz_mm = tcp_xyz_m * 1000.0

        self.info_xyz_labels['x'].setText(f"{tcp_xyz_mm[0]:.1f}")
        self.info_xyz_labels['y'].setText(f"{tcp_xyz_mm[1]:.1f}")
        self.info_xyz_labels['z'].setText(f"{tcp_xyz_mm[2]:.1f}")

        try:
            rpy_deg = T_ee.rpy(order='xyz', unit='deg')
        except TypeError:
            rpy = T_ee.rpy(order='xyz')
            rpy_deg = np.degrees(rpy)

        self.info_rpy_labels['roll'].setText(f"{rpy_deg[0]:.1f}")
        self.info_rpy_labels['pitch'].setText(f"{rpy_deg[1]:.1f}")
        self.info_rpy_labels['yaw'].setText(f"{rpy_deg[2]:.1f}")

        for motor_id in [1, 2, 3, 4, 5]:
            angle = self.current_angles_deg[motor_id - 1]
            self.info_joint_labels[motor_id].setText(f"{angle:.1f}")

        self.ax.cla()
        self.ax.plot(points[:, 0], points[:, 1], points[:, 2], "-o")

        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")
        self.ax.set_title("Pincher - Configuración actual (Toolbox)")

        self.ax.set_xlim(-0.3, 0.3)
        self.ax.set_ylim(-0.3, 0.3)
        self.ax.set_zlim(0.0, 0.3)
        self.ax.view_init(elev=30, azim=45)

        self.canvas.draw()

    # ============ Cierre ordenado ============

    def closeEvent(self, event):
        # Intentar cerrar RViz si está corriendo
        if self.rviz_process is not None:
            try:
                self.rviz_process.terminate()
            except Exception:
                pass
            self.rviz_process = None

        try:
            self.controller.close()
        except Exception:
            pass

        event.accept()


# ==========================
#  main()
# ==========================

def main(args=None):
    rclpy.init(args=args)

    controller = PincherController()

    # ROS2 en hilo aparte
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()

    app = QApplication(sys.argv)

    # Tema Fusion + paleta oscura
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(40, 44, 52))
    palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.Base, QColor(33, 37, 43))
    palette.setColor(QPalette.AlternateBase, QColor(40, 44, 52))
    palette.setColor(QPalette.ToolTipBase, QColor(220, 220, 220))
    palette.setColor(QPalette.ToolTipText, QColor(220, 220, 220))
    palette.setColor(QPalette.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.Button, QColor(40, 44, 52))
    palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
    palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.Highlight, QColor(100, 149, 237))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    win = PincherHMI(controller)
    win.show()

    try:
        sys.exit(app.exec_())
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
