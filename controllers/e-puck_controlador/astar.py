"""
astar.py
--------
Implementacion del algoritmo A* sobre una grilla de ocupacion 2D.
 
La grilla se representa como una lista de listas (matriz) donde:
    0 = celda libre
    1 = celda ocupada (obstaculo)
 
Las celdas se identifican como tuplas (fila, columna) -> (i, j)
    i: indice de fila    (eje "y" de la grilla)
    j: indice de columna (eje "x" de la grilla)
 
Este modulo es independiente de Webots, por lo que puede probarse
y depurarse fuera del simulador (por ejemplo con un pequeno script
de prueba o notebook) antes de integrarlo al controlador.
"""
 
import heapq
import math
 
 
def heuristica(a, b):
    """Distancia euclidiana entre dos celdas (i, j). Admisible para A*."""
    return math.hypot(a[0] - b[0], a[1] - b[1])
 
 
def vecinos(grid, celda):
    """
    Retorna las celdas vecinas validas (libres y dentro de la grilla)
    usando conectividad de 8 direcciones (incluye diagonales).
    """
    filas = len(grid)
    cols = len(grid[0])
    i, j = celda
 
    movimientos = [
        (-1, 0), (1, 0), (0, -1), (0, 1),   # arriba, abajo, izq, der
        (-1, -1), (-1, 1), (1, -1), (1, 1)  # diagonales
    ]
 
    resultado = []
    for di, dj in movimientos:
        ni, nj = i + di, j + dj
        if 0 <= ni < filas and 0 <= nj < cols:
            if grid[ni][nj] == 0:
                # Evitar "cortar" esquinas: si es movimiento diagonal,
                # exigir que las dos celdas ortogonales adyacentes
                # tambien esten libres.
                if di != 0 and dj != 0:
                    if grid[i + di][j] == 1 or grid[i][j + dj] == 1:
                        continue
                resultado.append((ni, nj))
    return resultado
 
 
def costo_movimiento(a, b):
    """Costo de moverse de la celda a a la celda b (1.0 recto, sqrt(2) diagonal)."""
    if a[0] != b[0] and a[1] != b[1]:
        return math.sqrt(2)
    return 1.0
 
 
def a_star(grid, inicio, meta):
    """
    Ejecuta A* sobre 'grid' desde 'inicio' hasta 'meta'.
 
    Parametros
    ----------
    grid : list[list[int]]
        Matriz de ocupacion (0 libre, 1 obstaculo).
    inicio : tuple(int, int)
        Celda inicial (fila, columna).
    meta : tuple(int, int)
        Celda meta (fila, columna).
 
    Retorna
    -------
    list[tuple(int,int)] o None
        Lista de celdas desde 'inicio' hasta 'meta' (ambas incluidas),
        o None si no existe una ruta posible.
    """
 
    if grid[inicio[0]][inicio[1]] == 1:
        raise ValueError("La celda de inicio esta marcada como obstaculo")
    if grid[meta[0]][meta[1]] == 1:
        raise ValueError("La celda meta esta marcada como obstaculo")
 
    open_set = []
    heapq.heappush(open_set, (0.0, inicio))
 
    came_from = {}
    g_score = {inicio: 0.0}
    f_score = {inicio: heuristica(inicio, meta)}
 
    visitados = set()
 
    while open_set:
        _, actual = heapq.heappop(open_set)
 
        if actual == meta:
            return reconstruir_camino(came_from, actual)
 
        if actual in visitados:
            continue
        visitados.add(actual)
 
        for vecino in vecinos(grid, actual):
            tentative_g = g_score[actual] + costo_movimiento(actual, vecino)
 
            if vecino not in g_score or tentative_g < g_score[vecino]:
                came_from[vecino] = actual
                g_score[vecino] = tentative_g
                f_score[vecino] = tentative_g + heuristica(vecino, meta)
                heapq.heappush(open_set, (f_score[vecino], vecino))
 
    # No se encontro ruta
    return None
 
 
def reconstruir_camino(came_from, actual):
    camino = [actual]
    while actual in came_from:
        actual = came_from[actual]
        camino.append(actual)
    camino.reverse()
    return camino
 
 
def simplificar_camino(camino):
    """
    Reduce el numero de puntos intermedios eliminando puntos colineales
    consecutivos (mismo vector de direccion). Util para que el robot
    no tenga que "frenar y girar" en cada celda cuando avanza en linea recta.
    """
    if len(camino) <= 2:
        return camino[:]
 
    simplificado = [camino[0]]
    for k in range(1, len(camino) - 1):
        prev_dir = (camino[k][0] - camino[k - 1][0], camino[k][1] - camino[k - 1][1])
        next_dir = (camino[k + 1][0] - camino[k][0], camino[k + 1][1] - camino[k][1])
        if prev_dir != next_dir:
            simplificado.append(camino[k])
    simplificado.append(camino[-1])
    return simplificado
 
 
if __name__ == "__main__":
    # Pequena prueba de humo (no requiere Webots)
    grid_prueba = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
    ]
    ruta = a_star(grid_prueba, (0, 0), (4, 4))
    print("Ruta encontrada:", ruta)
    print("Ruta simplificada:", simplificar_camino(ruta))
 
