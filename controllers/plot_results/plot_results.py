"""
plot_results.py
----------------
Script de analisis EXTERNO a Webots. Lee los archivos CSV generados por
epuck_controller.py (carpeta resultados/) y produce:

  1. Grafico comparando la ruta planificada (A*) vs la trayectoria
     ejecutada por el robot (odometria).
  2. Calculo de metricas cuantitativas (Sec. 10 del enunciado):
       - Tiempo total hasta llegar a la meta
       - Longitud de la ruta planificada
       - Longitud de la trayectoria ejecutada
       - Diferencia (error) entre ambas
       - Numero de eventos "casi colision"
       - Numero de cambios de waypoint (proxy de "giros")

Uso:
    python3 plot_results.py simple
    python3 plot_results.py complejo

Requiere matplotlib y pandas instalados en tu computador
(NO se ejecuta dentro de Webots, sino despues de la simulacion).
"""

import sys
import os
import math
import pandas as pd
import matplotlib.pyplot as plt


def longitud_camino(xs, ys):
    total = 0.0
    for k in range(1, len(xs)):
        total += math.hypot(xs[k] - xs[k - 1], ys[k] - ys[k - 1])
    return total


def main():
    if len(sys.argv) < 2:
        escenario = "simple"
    else:
        escenario = sys.argv[1]

    base_dir = os.path.join(os.path.dirname(__file__), "..", "e-puck_controlador", "resultados")
    log_path = os.path.join(base_dir, f"log_{escenario}.csv")
    ruta_path = os.path.join(base_dir, f"ruta_planeada_{escenario}.csv")

    if not os.path.exists(log_path) or not os.path.exists(ruta_path):
        print(f"No se encontraron los archivos para el escenario '{escenario}' en {base_dir}")
        print("Asegurate de haber ejecutado la simulacion completa en Webots primero.")
        return

    log = pd.read_csv(log_path)
    ruta = pd.read_csv(ruta_path)

    # ---------- Metricas ----------
    tiempo_total = log["tiempo_s"].iloc[-1]

    longitud_planeada = longitud_camino(ruta["x"].tolist(), ruta["y"].tolist())
    longitud_real = longitud_camino(log["x_odom"].tolist(), log["y_odom"].tolist())
    diferencia = abs(longitud_real - longitud_planeada)

    num_casi_colisiones = int(log["casi_colision"].sum())
    num_cambios_waypoint = int((log["waypoint_idx"].diff().fillna(0) != 0).sum())

    print("===== METRICAS -", escenario.upper(), "=====")
    print(f"Tiempo total hasta la meta:        {tiempo_total:.2f} s")
    print(f"Longitud ruta planificada (A*):    {longitud_planeada:.3f} m")
    print(f"Longitud trayectoria ejecutada:    {longitud_real:.3f} m")
    print(f"Diferencia (planeada vs real):     {diferencia:.3f} m")
    print(f"Casi-colisiones detectadas:        {num_casi_colisiones}")
    print(f"Cambios de waypoint (giros):       {num_cambios_waypoint}")

    # Guardar metricas en un archivo de texto para el README
    metricas_path = os.path.join(base_dir, f"metricas_{escenario}.txt")
    with open(metricas_path, "w") as f:
        f.write(f"Escenario: {escenario}\n")
        f.write(f"Tiempo total hasta la meta: {tiempo_total:.2f} s\n")
        f.write(f"Longitud ruta planificada (A*): {longitud_planeada:.3f} m\n")
        f.write(f"Longitud trayectoria ejecutada: {longitud_real:.3f} m\n")
        f.write(f"Diferencia (planeada vs real): {diferencia:.3f} m\n")
        f.write(f"Casi-colisiones detectadas: {num_casi_colisiones}\n")
        f.write(f"Cambios de waypoint (giros): {num_cambios_waypoint}\n")
    print("Metricas guardadas en:", metricas_path)

    # ---------- Grafico: ruta planificada vs trayectoria real ----------
    plt.figure(figsize=(6, 6))
    plt.plot(ruta["x"], ruta["y"], "o--", label="Ruta planificada (A*)", color="tab:blue")
    plt.plot(log["x_odom"], log["y_odom"], "-", label="Trayectoria ejecutada (odometria)", color="tab:red")

    # Marcar puntos de casi-colision si existen
    casi_col = log[log["casi_colision"] == 1]
    if not casi_col.empty:
        plt.scatter(casi_col["x_odom"], casi_col["y_odom"], color="orange",
                     marker="x", label="Casi colision", zorder=5)

    plt.scatter(ruta["x"].iloc[0], ruta["y"].iloc[0], color="green", s=80, label="Inicio")
    plt.scatter(ruta["x"].iloc[-1], ruta["y"].iloc[-1], color="black", s=80, marker="*", label="Meta")

    plt.title(f"Ruta planificada vs trayectoria ejecutada - Escenario {escenario}")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    fig_path = os.path.join(base_dir, f"comparacion_ruta_{escenario}.png")
    plt.savefig(fig_path, dpi=150, bbox_inches="tight")
    print("Grafico guardado en:", fig_path)

    # ---------- Grafico: lectura de sensor frontal en el tiempo ----------
    plt.figure(figsize=(8, 4))
    plt.plot(log["tiempo_s"], log["ps_front_max"], color="tab:purple")
    plt.axhline(150, color="orange", linestyle="--", label="Umbral casi-colision")
    plt.title(f"Sensor frontal maximo en el tiempo - Escenario {escenario}")
    plt.xlabel("tiempo [s]")
    plt.ylabel("lectura sensor IR")
    plt.legend()
    plt.grid(True)

    fig2_path = os.path.join(base_dir, f"sensor_frontal_{escenario}.png")
    plt.savefig(fig2_path, dpi=150, bbox_inches="tight")
    print("Grafico guardado en:", fig2_path)


if __name__ == "__main__":
    main()
