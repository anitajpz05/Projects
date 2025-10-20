import heapq

def Recorridocondijkstra(matriz, inicio, objetivo):
    """
    matriz de caminos: lista de listas; cada celda es int>=1 o None (obstáculo)
    inicio, objetivo: tuplas (x, y) con 0-based indices (fila, columna)
    devuelve: (coste_total, path_as_list_of_cells) o (float('inf'), [])
    """
    rows = len(matriz)
    cols = len(matriz[0]) if rows else 0

    def in_bounds(pos):
        x, y = pos
        return 0 <= x < rows and 0 <= y < cols

    listadevecinos = [(1,0), (-1,0), (0,1), (0,-1)]

    # prioridad por coste acumulado
    pq = []
    # guardamos coste acumulado hasta la celda
    costeacumulado = {inicio: 0}
    # para reconstruir camino
    caminoreconstruido = {inicio: None}

    heapq.heappush(pq, (0, inicio))

    while pq:
        coste, actual = heapq.heappop(pq)

        if actual == objetivo:
            # reconstruir camino
            camino = []
            nodo = objetivo
            while nodo is not None:
                camino.append(nodo)
                nodo = caminoreconstruido[nodo]
            camino.reverse()
            return coste, camino

        # Si sacamos un estado con coste obsoleto, ignorar
        if coste != costeacumulado.get(actual, float('inf')):
            continue

        cx, cy = actual
        for dx, dy in listadevecinos:
            nx, ny = cx + dx, cy + dy
            vecino = (nx, ny)
            if not in_bounds(vecino):
                continue
            costecamino = matriz[nx][ny]
            if costecamino is None:
                continue  # obstáculo
            nuevo_coste = coste + costecamino
            if nuevo_coste < costeacumulado.get(vecino, float('inf')):
                costeacumulado[vecino] = nuevo_coste
                caminoreconstruido[vecino] = actual
                heapq.heappush(pq, (nuevo_coste, vecino))

    return float('inf'), []  # Devuelve lista vacia sino hay solución


    """
    Solucion basada en A*
    """


def RecorridoconAestrella(matriz, inicio, objetivo):
    """
    Devuelve (coste_total, camino) o (float('inf'), []) si no hay solución.
    Heurística admisible: h = Manhattan(start, goal) * min_cost_in_grid
    """
    fila = len(matriz)
    columna = len(matriz[0]) if fila else 0

    def in_bounds(pos):
        x, y = pos
        return 0 <= x < fila and 0 <= y < columna

    listadevecinos = [(1,0), (-1,0), (0,1), (0,-1)]

    # calcular coste mínimo positivo en la rejilla (para escalado heurístico)
    costeminimo = float('inf')
    for r in range(fila):
        for c in range(columna):
            v = matriz[r][c]
            if v is not None and v < costeminimo:
                costeminimo = v
    if costeminimo == float('inf'):
        return float('inf'), []  # todo es obstáculo

    def manhattan(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def heuristic(nodo):
        return manhattan(nodo, objetivo) * costeminimo

    # funcion de evaluacion: (f = g + h, g, node)
    open_pq = []
    heapq.heappush(open_pq, (heuristic(inicio), 0, inicio))

    nodopasado = {inicio: None}
    costoacumulado = {inicio: 0}

    while open_pq:
        f, g, actual = heapq.heappop(open_pq)

        if actual == objetivo:
            # reconstruir camino
            camino = []
            nodo = objetivo
            while nodo is not None:
                camino.append(nodo)
                nodo = nodopasado[nodo]
            camino.reverse()
            return g, camino

        # si el g extraído no coincide con el mejor guardado, ignorar
        if g != costoacumulado.get(actual, float('inf')):
            continue

        cx, cy = actual
        for dx, dy in listadevecinos:
            nx, ny = cx + dx, cy + dy
            vecino = (nx, ny)
            if not in_bounds(vecino):
                continue
            cell_cost = matriz[nx][ny]
            if cell_cost is None:
                continue
            tentative_g = g + cell_cost
            if tentative_g < costoacumulado.get(vecino, float('inf')):
                costoacumulado[vecino] = tentative_g
                nodopasado[vecino] = actual
                f_score = tentative_g + heuristic(vecino)
                heapq.heappush(open_pq, (f_score, tentative_g, vecino))

    return float('inf'), []


def mostrarcamino(matriz, camino):
    # copia para imprimir
    fila = len(matriz)
    columna = len(matriz[0]) if fila else 0
    disponible = [['.' for _ in range(columna)] for _ in range(fila)]
    for i in range(fila):
        for j in range(columna):
            if matriz[i][j] is None:
                disponible[i][j] = '#'
            else:
                # mostrar coste reducido si >1
                if matriz[i][j] == 1:
                    disponible[i][j] = '.'
                else:
                    disponible[i][j] = str(matriz[i][j])
    for (x, y) in camino:
        disponible[x][y] = 'X'
    # inicio/objetivo
    if camino:
        sx, sy = camino[0]
        gx, gy = camino[-1]
        disponible[sx][sy] = 'S'
        disponible[gx][gy] = 'G'

    for row in disponible:
        print(' '.join(row))


# Definimos un mapa ejemplo (None = obstáculo; int >=1 = coste de entrar)
matrizderelacion = [
    [1, 1, 1, 1, 1, 1, 1],
    [1, None, None, 5, 5, 5, 1],
    [1, 1, 1, 1, None, 1, 1],
    [1, 5, None, 1, 1, 1, 1],
    [1, 1, 1, None, 2, None, 1],
    [1, None, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1]
]

inicio = (0, 0)
objetivo  = (6, 6)

# Ejecuta Dijkstra
cost_dij, path_dij = Recorridocondijkstra(matrizderelacion, inicio, objetivo)
print("Dijkstra → Coste:", cost_dij, "Ruta:", path_dij)
mostrarcamino(matrizderelacion, path_dij)
print("\n")

# Ejecuta A estrella
cost_astar, path_astar = RecorridoconAestrella(matrizderelacion, inicio, objetivo)
print("A* → Coste:", cost_astar, "Ruta:", path_astar)
mostrarcamino(matrizderelacion, path_astar)


"""  Resultado de la Comparación entre Dijkstra y A*
 Algoritm Dijkstra
Garantiza encontrar la ruta de coste mínimo.
No usa heurística: explora por coste acumulado.
En mapas grandes y cuando tienes pocos motivos para guiar la búsqueda, funciona bien, pero puede expandir muchos nodos.

 Algoritmo A*
Combina coste real (g) con heurística (h) para priorizar estados que parecen llevar al objetivo.
Si h es admisible (nunca sobreestima), A* es óptimo y normalmente más eficiente que Dijkstra porque expande menos nodos.
Elegir una heurística informativa reduce mucho el tiempo de búsqueda. En grids con movimientos 4-dir, la Manhattan escalada por el coste mínimo es una buena elección."""