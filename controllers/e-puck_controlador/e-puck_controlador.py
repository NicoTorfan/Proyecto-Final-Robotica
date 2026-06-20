"""
epuck_controller.py
--------------------
Controlador principal del robot e-puck en Webots para el Proyecto Final
(Linea A: Planificacion de rutas).

Componentes implementados (ver enunciado del proyecto):
  1. Control de movimiento     -> cinematica diferencial (Sec. 7 del enunciado)
  2. Percepcion del entorno    -> sensores de proximidad ps0..ps7
  3. Estimacion de movimiento  -> odometria a partir de encoders (Ec. 3-7)
  4. Navegacion local          -> evitacion reactiva de obstaculos
  5. Navegacion global         -> A* sobre grilla de ocupacion (modulo astar.py)
  6. Evaluacion experimental   -> log en CSV (tiempo, pose, sensores, etc.)

IMPORTANTE - CALIBRACION OBLIGATORIA:
  Debes ajustar la seccion "CONFIGURACION DEL ESCENARIO" mas abajo para
  que coincida con TU mundo de Webots:
    - La matriz OCCUPANCY_GRID debe representar tus obstaculos.
    - GRID_ORIGIN_X / GRID_ORIGIN_Y / CELL_SIZE deben mapear las celdas
      de la grilla a coordenadas reales (x, y) de Webots.
    - START_CELL y GOAL_CELL deben corresponder a la posicion inicial
      del robot en el mundo y a la meta deseada.

  Para tener "al menos dos escenarios" (Sec. 9), se incluyen dos grillas
  de ejemplo (ESCENARIO_SIMPLE y ESCENARIO_COMPLEJO). Cambia la variable
  ESCENARIO ("simple" o "complejo") segun el mundo .wbt que tengas cargado,
  o crea tu propia grilla copiando el formato.
"""

import math
import csv
import os

from controller import Robot
from astar import a_star, simplificar_camino


# ============================================================
# CONFIGURACION GENERAL DEL ROBOT (e-puck)
# ============================================================

TIME_STEP = 64                 # ms, paso de simulacion
MAX_SPEED = 6.28                # rad/s, velocidad maxima de las ruedas e-puck

WHEEL_RADIUS = 0.0205           # m, radio de la rueda del e-puck
AXLE_LENGTH = 0.052              # m, distancia entre ruedas (L)

# Umbral de sensores de proximidad para considerar "obstaculo cercano"
PS_THRESHOLD_EVITAR = 300.0
# Umbral mas alto para contar "casi colision" (metrica de evaluacion)
PS_THRESHOLD_CASI_COLISION = 150.0

# Tolerancias de control
DIST_TOLERANCIA_WAYPOINT = 0.03  # m, distancia para considerar alcanzado un punto
DIST_TOLERANCIA_META = 0.05       # m, distancia para considerar alcanzada la meta

# Ganancias del controlador proporcional (velocidad lineal y angular)
KP_LINEAL = 0.6
KP_ANGULAR = 2.5


# ============================================================
# CONFIGURACION DEL ESCENARIO (CALIBRAR SEGUN TU MUNDO WEBOTS)
# ============================================================

# Cambia esto a "simple" o "complejo" segun el mundo que tengas cargado.
ESCENARIO = "simple"

# --- ESCENARIO SIMPLE: pocos obstaculos, ruta directa (Sec. 9) ---------
ESCENARIO_SIMPLE_GRID = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
]
ESCENARIO_SIMPLE_START = (0, 0)
ESCENARIO_SIMPLE_GOAL = (7, 7)

# --- ESCENARIO COMPLEJO: mas obstaculos, pasillos, curvas (Sec. 9) -----
ESCENARIO_COMPLEJO_GRID = [
    [0, 0, 0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 1, 0],
    [0, 1, 0, 1, 0, 1, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 1],
    [0, 1, 1, 1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 0, 0],
]
ESCENARIO_COMPLEJO_START = (0, 0)
ESCENARIO_COMPLEJO_GOAL = (7, 7)

# Tamano de cada celda de la grilla en METROS (ajustar segun tu mundo).
# Ejemplo: si tu arena es de 1.6m x 1.6m y la grilla es 8x8 -> 0.2 m por celda.
CELL_SIZE = 0.2

# Coordenadas (x, y) de Webots que corresponden a la celda (0,0) de la grilla,
# es decir, la esquina/origen de tu arena. Ajustar segun tu mundo.
GRID_ORIGIN_X = -0.8
GRID_ORIGIN_Y = -0.8


def seleccionar_escenario():
    if ESCENARIO == "complejo":
        return ESCENARIO_COMPLEJO_GRID, ESCENARIO_COMPLEJO_START, ESCENARIO_COMPLEJO_GOAL
    return ESCENARIO_SIMPLE_GRID, ESCENARIO_SIMPLE_START, ESCENARIO_SIMPLE_GOAL


def celda_a_mundo(celda):
    """Convierte (fila, columna) de la grilla a coordenadas (x, y) de Webots.

    Se usa el centro de la celda. La fila 'i' se interpreta como avance en Y
    y la columna 'j' como avance en X (ajustar si tu mundo usa otra orientacion).
    """
    i, j = celda
    x = GRID_ORIGIN_X + (j + 0.5) * CELL_SIZE
    y = GRID_ORIGIN_Y + (i + 0.5) * CELL_SIZE
    return x, y


# ============================================================
# CLASE PRINCIPAL DEL CONTROLADOR
# ============================================================

class EpuckNavController:
    def __init__(self):
        self.robot = Robot()

        # ---------- Motores ----------
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # ---------- Encoders (sensores de posicion de rueda) ----------
        self.left_encoder = self.robot.getDevice("left wheel sensor")
        self.right_encoder = self.robot.getDevice("right wheel sensor")
        self.left_encoder.enable(TIME_STEP)
        self.right_encoder.enable(TIME_STEP)

        # ---------- Sensores de proximidad ps0..ps7 ----------
        self.ps = []
        self.ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
        for name in self.ps_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(TIME_STEP)
            self.ps.append(sensor)

        # ---------- GPS opcional (para validar odometria, NO usar para control) ----------
        # Si tu robot tiene un nodo GPS llamado "gps", se habilita para
        # poder comparar la odometria estimada vs la posicion real en el
        # analisis experimental. Si no existe, se ignora sin error.
        self.gps = None
        try:
            self.gps = self.robot.getDevice("gps")
            self.gps.enable(TIME_STEP)
        except Exception:
            self.gps = None

        # ---------- Estado de odometria ----------
        # Pose inicial: se asume que el robot parte en el centro de START_CELL,
        # mirando en direccion +X (phi = 0). Ajustar si tu robot parte con otra
        # orientacion en el mundo.
        self.grid, self.start_cell, self.goal_cell = seleccionar_escenario()
        x0, y0 = celda_a_mundo(self.start_cell)
        self.x = x0
        self.y = y0
        self.phi = 0.0

        self.prev_left_enc = None
        self.prev_right_enc = None

        # ---------- Planificacion global (A*) ----------
        camino_celdas = a_star(self.grid, self.start_cell, self.goal_cell)
        if camino_celdas is None:
            raise RuntimeError(
                "A* no encontro una ruta entre start y goal. "
                "Revisa la grilla OCCUPANCY_GRID, START_CELL y GOAL_CELL."
            )
        camino_celdas = simplificar_camino(camino_celdas)

        # Ruta planificada en coordenadas del mundo (lista de waypoints)
        self.ruta_planeada = [celda_a_mundo(c) for c in camino_celdas]
        self.waypoint_idx = 0

        print("Escenario seleccionado:", ESCENARIO)
        print("Celdas de la ruta planificada (A*):", camino_celdas)
        print("Waypoints en coordenadas del mundo:", self.ruta_planeada)

        # ---------- Logging ----------
        self.log_rows = []
        self.t = 0.0

        # ---------- Estado de finalizacion ----------
        self.meta_alcanzada = False
        self.ps_filtrado = [0.0] * 8

    # --------------------------------------------------------
    # 1) ESTIMACION DE MOVIMIENTO (ODOMETRIA) - Ecs. 3-7 enunciado
    # --------------------------------------------------------
    def actualizar_odometria(self):
        left_enc = self.left_encoder.getValue()
        right_enc = self.right_encoder.getValue()

        if self.prev_left_enc is None:
            # Primera lectura: inicializar y no integrar todavia
            self.prev_left_enc = left_enc
            self.prev_right_enc = right_enc
            return

        d_theta_l = left_enc - self.prev_left_enc
        d_theta_r = right_enc - self.prev_right_enc
        self.prev_left_enc = left_enc
        self.prev_right_enc = right_enc

        # Ec. (3): desplazamiento de cada rueda
        d_sl = WHEEL_RADIUS * d_theta_l
        d_sr = WHEEL_RADIUS * d_theta_r

        # Ec. (4): desplazamiento y giro del robot
        d_s = (d_sr + d_sl) / 2.0
        d_phi = (d_sr - d_sl) / AXLE_LENGTH

        # Ecs. (5)-(7): actualizacion de la pose
        self.x += d_s * math.cos(self.phi + d_phi / 2.0)
        self.y += d_s * math.sin(self.phi + d_phi / 2.0)
        self.phi += d_phi
        self.phi = math.atan2(math.sin(self.phi), math.cos(self.phi))  # normalizar [-pi, pi]

    # --------------------------------------------------------
    # 2) PERCEPCION: lectura de sensores de proximidad
    # --------------------------------------------------------
    def leer_sensores(self):
        # Filtro pasa-bajos exponencial con alpha alto para evitar lag
        alpha = 0.8  
        raw_values = [s.getValue() for s in self.ps]
        
        for i in range(8):
            self.ps_filtrado[i] = (alpha * raw_values[i]) + ((1.0 - alpha) * self.ps_filtrado[i])
            
        return self.ps_filtrado

    # --------------------------------------------------------
    # 3) NAVEGACION LOCAL: evitacion reactiva de obstaculos
    # --------------------------------------------------------
    def calcular_evitacion(self, ps_values):
        """
        Retorna (activo, vl, vr) donde:
          activo: True si se debe sobreescribir el control de seguimiento
                  de ruta porque hay un obstaculo cercano.
          vl, vr: velocidades de rueda (rad/s) sugeridas para evitar.

        Disposicion tipica de sensores e-puck (vista desde arriba):
          ps7, ps0  -> frente
          ps6, ps1  -> frontal-lateral
          ps5, ps2  -> laterales
          ps4, ps3  -> traseros
        """
        front = max(ps_values[0], ps_values[7])
        front_left = ps_values[6]
        front_right = ps_values[1]

        if front > PS_THRESHOLD_EVITAR or front_left > PS_THRESHOLD_EVITAR or front_right > PS_THRESHOLD_EVITAR:
            # Obstaculo al frente: girar hacia el lado con menos obstruccion
            if front_left > front_right:
                # Mas obstaculo a la izquierda -> curva hacia la derecha
                vl = 0.5 * MAX_SPEED
                vr = 0.1 * MAX_SPEED
            else:
                # Mas obstaculo a la derecha -> curva hacia la izquierda
                vl = 0.1 * MAX_SPEED
                vr = 0.5 * MAX_SPEED
            return True, vl, vr

        return False, 0.0, 0.0

    # --------------------------------------------------------
    # 4) NAVEGACION GLOBAL: seguimiento de la ruta planificada (A*)
    # --------------------------------------------------------
    def calcular_seguimiento_ruta(self):
        if self.waypoint_idx >= len(self.ruta_planeada):
            return 0.0, 0.0, True  # ya no hay waypoints -> detener

        wx, wy = self.ruta_planeada[self.waypoint_idx]
        dx = wx - self.x
        dy = wy - self.y
        distancia = math.hypot(dx, dy)
        angulo_objetivo = math.atan2(dy, dx)

        error_angulo = angulo_objetivo - self.phi
        error_angulo = math.atan2(math.sin(error_angulo), math.cos(error_angulo))

        # Si llegamos al waypoint actual, avanzar al siguiente
        if distancia < DIST_TOLERANCIA_WAYPOINT:
            self.waypoint_idx += 1
            if self.waypoint_idx >= len(self.ruta_planeada):
                return 0.0, 0.0, True
            wx, wy = self.ruta_planeada[self.waypoint_idx]
            dx = wx - self.x
            dy = wy - self.y
            distancia = math.hypot(dx, dy)
            angulo_objetivo = math.atan2(dy, dx)
            error_angulo = angulo_objetivo - self.phi
            error_angulo = math.atan2(math.sin(error_angulo), math.cos(error_angulo))

        # Control proporcional: velocidad lineal y angular deseadas
        v = min(KP_LINEAL * distancia, 0.25)
        w = KP_ANGULAR * error_angulo

        # Si el error angular es mayor a ~5 grados, priorizar giro (freno total de avance)
        if abs(error_angulo) > 0.1:
            v = 0.0

        # Conversion a velocidades de rueda (cinematica diferencial)
        v_left = v - (w * AXLE_LENGTH / 2.0)
        v_right = v + (w * AXLE_LENGTH / 2.0)

        vl = v_left / WHEEL_RADIUS
        vr = v_right / WHEEL_RADIUS

        return vl, vr, False

    # --------------------------------------------------------
    # Utilidad: limitar velocidades al rango permitido
    # --------------------------------------------------------
    @staticmethod
    def limitar(valor, minimo, maximo):
        return max(minimo, min(maximo, valor))

    # --------------------------------------------------------
    # LOOP PRINCIPAL
    # --------------------------------------------------------
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.t += TIME_STEP / 1000.0

            # 1) Odometria
            self.actualizar_odometria()

            # 2) Percepcion
            ps_values = self.leer_sensores()
            ps_front_max = max(ps_values[0], ps_values[1], ps_values[6], ps_values[7])

            if self.meta_alcanzada:
                vl_cmd, vr_cmd = 0.0, 0.0
            else:
                # 3) Navegacion global: velocidades sugeridas por seguimiento de ruta
                vl_ruta, vr_ruta, llego_meta = self.calcular_seguimiento_ruta()

                if llego_meta:
                    self.meta_alcanzada = True
                    vl_cmd, vr_cmd = 0.0, 0.0
                    print("Meta alcanzada en t = {:.2f} s | pose final: x={:.3f}, y={:.3f}, phi={:.3f}".format(
                        self.t, self.x, self.y, self.phi))
                else:
                    # 4) Navegacion local: evitacion reactiva tiene prioridad
                    activo, vl_evi, vr_evi = self.calcular_evitacion(ps_values)
                    if activo:
                        vl_cmd, vr_cmd = vl_evi, vr_evi
                    else:
                        vl_cmd, vr_cmd = vl_ruta, vr_ruta

            # Saturar velocidades
            vl_cmd = self.limitar(vl_cmd, -MAX_SPEED, MAX_SPEED)
            vr_cmd = self.limitar(vr_cmd, -MAX_SPEED, MAX_SPEED)

            self.left_motor.setVelocity(vl_cmd)
            self.right_motor.setVelocity(vr_cmd)

            # 5) Registro de datos (evaluacion experimental)
            gps_x, gps_y = "", ""
            if self.gps is not None:
                pos = self.gps.getValues()
                gps_x, gps_y = pos[0], pos[1]

            self.log_rows.append([
                round(self.t, 3),
                round(self.x, 4),
                round(self.y, 4),
                round(self.phi, 4),
                round(vl_cmd, 3),
                round(vr_cmd, 3),
                round(ps_front_max, 2),
                self.waypoint_idx,
                int(ps_front_max > PS_THRESHOLD_CASI_COLISION),
                gps_x,
                gps_y,
            ])

            if self.meta_alcanzada:
                # Mantener un par de pasos detenido y luego exportar log
                self.guardar_log()
                break

    # --------------------------------------------------------
    # Exportar log y ruta planificada a CSV
    # --------------------------------------------------------
    def guardar_log(self):
        out_dir = os.path.join(os.path.dirname(__file__), "resultados")
        os.makedirs(out_dir, exist_ok=True)

        log_path = os.path.join(out_dir, "log_{}.csv".format(ESCENARIO))
        with open(log_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "tiempo_s", "x_odom", "y_odom", "phi_odom",
                "vel_izq_radps", "vel_der_radps",
                "ps_front_max", "waypoint_idx", "casi_colision",
                "x_gps", "y_gps",
            ])
            writer.writerows(self.log_rows)

        ruta_path = os.path.join(out_dir, "ruta_planeada_{}.csv".format(ESCENARIO))
        with open(ruta_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            writer.writerows(self.ruta_planeada)

        print("Log guardado en:", log_path)
        print("Ruta planificada guardada en:", ruta_path)


if __name__ == "__main__":
    controller = EpuckNavController()
    controller.run()