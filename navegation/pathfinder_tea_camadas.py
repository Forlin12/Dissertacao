import heapq
import math
import threading
import time
import numpy as np
from shapely.geometry import Point
from numba import njit


# ==========================================
# OTIMIZAÇÃO NUMBA: RAY-CASTING VETORIZADO
# ==========================================
@njit(cache=True)
def point_in_polygon_numba(x, y, poly_coords):
    """
    Algoritmo de Ray-Casting ultrarrápido compilado em C via Numba.
    Verifica se um ponto (x,y) está dentro de um polígono.
    """
    n = len(poly_coords)
    inside = False
    p1x, p1y = poly_coords[0]
    for i in range(1, n + 1):
        p2x, p2y = poly_coords[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


@njit(cache=True)
def rasterizar_bbox_numba(grid, z_idx, poly_coords, x_min, x_max, y_min, y_max, fator_escala):
    """
    Itera sobre a Bounding Box na grelha e escreve os obstáculos na matriz 3D.
    """
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            if grid[z_idx, x, y]:
                continue

            px = x * fator_escala
            py = y * fator_escala

            if point_in_polygon_numba(px, py, poly_coords):
                grid[z_idx, x, y] = True


# ==========================================
# VARIÁVEIS GLOBAIS E CACHE
# ==========================================
thread_local_data = threading.local()
_cache_lock = threading.Lock()

_DIAG_TEMPO_MIN_S = 1.0
_DIAG_ITER_MIN = 200000

_route_cache = {}
_route_cache_lock = threading.Lock()
_route_cache_hits = [0]
_route_cache_misses = [0]


def calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, vetor_camadas,
                              t_inicial=0):
    # ==========================================
    # PARÂMETRO DE OTIMIZAÇÃO: ESCALA DA GRADE
    # ==========================================
    FATOR_ESCALA = 2.5

    grid_max_x = int(math.ceil(max_x / FATOR_ESCALA))
    grid_max_y = int(math.ceil(max_y / FATOR_ESCALA))

    if not hasattr(calcular_rota_tea_camadas, "cache_grid"):
        calcular_rota_tea_camadas.cache_grid = None
        calcular_rota_tea_camadas.cache_cidade_id = None

    if calcular_rota_tea_camadas.cache_cidade_id != id(lotes_gdf):
        with _cache_lock:
            # Double-checked locking
            if calcular_rota_tea_camadas.cache_cidade_id != id(lotes_gdf):
                _t0_cache = time.perf_counter()
                num_camadas = len(vetor_camadas)

                # Matriz gerada com o tamanho reduzido pela escala
                grid = np.zeros((num_camadas, grid_max_x, grid_max_y), dtype=bool)
                inflacao = drone.raio + 1.0

                for z_idx, camada in enumerate(vetor_camadas):
                    predios_altos = lotes_gdf[lotes_gdf['altura_z'] >= camada]

                    for _, p in predios_altos.iterrows():
                        geom_inflada = p.geometry.buffer(inflacao)
                        b = geom_inflada.bounds

                        # Converte a bound box do polígono real para índices na grade escalada
                        x_min = max(0, int(math.floor(b[0] / FATOR_ESCALA)))
                        x_max = min(grid_max_x - 1, int(math.ceil(b[2] / FATOR_ESCALA)))
                        y_min = max(0, int(math.floor(b[1] / FATOR_ESCALA)))
                        y_max = min(grid_max_y - 1, int(math.ceil(b[3] / FATOR_ESCALA)))

                        # Extração de coordenadas cruas para o Numba
                        geometrias = [geom_inflada] if geom_inflada.geom_type == 'Polygon' else geom_inflada.geoms

                        for g in geometrias:
                            coords = np.array(g.exterior.coords)
                            rasterizar_bbox_numba(grid, z_idx, coords, x_min, x_max, y_min, y_max, FATOR_ESCALA)

                calcular_rota_tea_camadas.cache_grid = grid
                calcular_rota_tea_camadas.cache_cidade_id = id(lotes_gdf)

    # ATENÇÃO: grid_estatico é PARTILHADO por todas as threads/chamadas.
    grid_estatico = calcular_rota_tea_camadas.cache_grid

    z_to_idx = {z: i for i, z in enumerate(vetor_camadas)}
    idx_to_z = {i: z for i, z in enumerate(vetor_camadas)}
    z_inicial_val = vetor_camadas[0]
    z_idx_inicial = 0
    num_camadas = len(vetor_camadas)

    # Conversão do start e goal para a escala reduzida
    sx = int(round(start_loc[0] / FATOR_ESCALA))
    sy = int(round(start_loc[1] / FATOR_ESCALA))
    gx = int(round(goal_loc[0] / FATOR_ESCALA))
    gy = int(round(goal_loc[1] / FATOR_ESCALA))

    # === VERIFICAÇÃO DE CACHE COM BOUNDING BOX LOCAL ===
    MARGEM_CACHE = int(25 / FATOR_ESCALA)
    min_x_box = min(sx, gx) - MARGEM_CACHE
    max_x_box = max(sx, gx) + MARGEM_CACHE
    min_y_box = min(sy, gy) - MARGEM_CACHE
    max_y_box = max(sy, gy) + MARGEM_CACHE

    reservas_relativas = frozenset(
        (int(round(rx / FATOR_ESCALA)), int(round(ry / FATOR_ESCALA)), rz, rt - t_inicial)
        for (rx, ry, rz, rt) in reserva_global.keys()
        if rt >= t_inicial and
        min_x_box <= int(round(rx / FATOR_ESCALA)) <= max_x_box and
        min_y_box <= int(round(ry / FATOR_ESCALA)) <= max_y_box
    )

    chave_cache = (
        sx, sy, gx, gy,
        getattr(drone, 'nome_modelo', None), drone.raio,
        drone.velocidade_horiz, drone.velocidade_subida, drone.velocidade_descida,
        reservas_relativas
    )

    with _route_cache_lock:
        resultado_cache = _route_cache.get(chave_cache)

    if resultado_cache is not None:
        _route_cache_hits[0] += 1
        return list(resultado_cache)

    _route_cache_misses[0] += 1

    def _guardar_cache(resultado):
        with _route_cache_lock:
            _route_cache[chave_cache] = list(resultado)
        return resultado

    raio_limpeza = int(math.ceil(drone.raio / FATOR_ESCALA)) + 1

    pixels_livres_locais = set()
    for dx in range(-raio_limpeza, raio_limpeza + 1):
        for dy in range(-raio_limpeza, raio_limpeza + 1):
            nx_s, ny_s = sx + dx, sy + dy
            if 0 <= nx_s < grid_max_x and 0 <= ny_s < grid_max_y:
                pixels_livres_locais.add((z_idx_inicial, nx_s, ny_s))

            nx_g, ny_g = gx + dx, gy + dy
            if 0 <= nx_g < grid_max_x and 0 <= ny_g < grid_max_y:
                pixels_livres_locais.add((z_idx_inicial, nx_g, ny_g))

    def esta_bloqueado(z_idx, x, y):
        if (z_idx, x, y) in pixels_livres_locais:
            return False
        return bool(grid_estatico[z_idx, x, y])

    try:
        if esta_bloqueado(z_idx_inicial, sx, sy):
            return _guardar_cache([])
        if esta_bloqueado(z_idx_inicial, gx, gy):
            return _guardar_cache([])

        v_horiz = drone.velocidade_horiz if drone.velocidade_horiz > 0 else 1.0
        v_sub = drone.velocidade_subida if drone.velocidade_subida > 0 else 1.0
        v_des = drone.velocidade_descida if drone.velocidade_descida > 0 else 1.0

        c_cruzeiro = drone.consumo_cruzeiro + (drone.carga * drone.penalidade_carga)
        c_subida = drone.consumo_subida + (drone.carga * drone.penalidade_carga)
        c_descida = drone.consumo_descida + (drone.carga * drone.penalidade_carga)

        PESO_HEURISTICA = 1.2

        def h_fisico(x, y, z_idx):
            dist_xy_celulas = math.sqrt((gx - x) ** 2 + (gy - y) ** 2)
            dist_xy_metros = dist_xy_celulas * FATOR_ESCALA
            dist_z_metros = abs(idx_to_z[z_idx] - z_inicial_val)
            tempo_ideal = (dist_xy_metros / v_horiz) + (dist_z_metros / v_des)
            return tempo_ideal * PESO_HEURISTICA

        ocupacao_set = set()
        agentes_dict = {}
        for chaves, id_agente in reserva_global.items():
            if len(chaves) == 4:
                rx, ry, rz, rt = chaves
                if rz in z_to_idx:
                    c_idx = z_to_idx[rz]
                    sx_res = int(round(rx / FATOR_ESCALA))
                    sy_res = int(round(ry / FATOR_ESCALA))
                    chave_4d = (sx_res, sy_res, c_idx, rt)
                    ocupacao_set.add(chave_4d)
                    agentes_dict[chave_4d] = id_agente

        abertos = []
        heapq.heappush(abertos, (h_fisico(sx, sy, z_idx_inicial), t_inicial, sx, sy, z_idx_inicial))

        g_score = {(sx, sy, z_idx_inicial, t_inicial): 0.0}
        g_energia = {(sx, sy, z_idx_inicial, t_inicial): 0.0}
        veio_de = {}

        # OTIMIZAÇÃO MÁXIMA: Array Numpy estático por Thread + Tracking de Sujeira
        if not hasattr(thread_local_data, 'melhor_t_espacial') or \
                thread_local_data.melhor_t_espacial.shape != (grid_max_x, grid_max_y, num_camadas):
            # Só aloca esta matriz 1 única vez por Thread durante toda a execução do GA
            thread_local_data.melhor_t_espacial = np.full((grid_max_x, grid_max_y, num_camadas), -1, dtype=np.int32)

        melhor_t_espacial_ref = thread_local_data.melhor_t_espacial
        celulas_sujas = []  # Regista apenas as coordenadas tocadas para limpar no fim

        dist_max_t_metros = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) * FATOR_ESCALA
        max_t = t_inicial + int(dist_max_t_metros * 6.0) + 150

        LIMITE_ITERACOES = 200_0000
        iteracoes = 0

        movimentos_base = [
            (1, 0, 0, 1.0), (-1, 0, 0, 1.0), (0, 1, 0, 1.0), (0, -1, 0, 1.0),
            (1, 1, 0, 1.414), (-1, -1, 0, 1.414), (1, -1, 0, 1.414), (-1, 1, 0, 1.414)
        ]

        while abertos:
            iteracoes += 1
            if iteracoes > LIMITE_ITERACOES:
                return _guardar_cache([])

            f_cur, t, x, y, z_idx = heapq.heappop(abertos)

            if x == gx and y == gy:
                # 1. Recuperar os nós originais em escala reduzida
                caminho_reduzido = []
                curr = (x, y, z_idx, t)
                while curr in veio_de:
                    cx, cy, cz_idx, _ = curr
                    caminho_reduzido.append((cx * FATOR_ESCALA, cy * FATOR_ESCALA, idx_to_z[cz_idx]))
                    curr = veio_de[curr]

                if caminho_reduzido:
                    # CORREÇÃO 1: Substituir o nó inicial escalado pelo start_loc real
                    cz_start = caminho_reduzido[-1][2]
                    caminho_reduzido[-1] = (start_loc[0], start_loc[1], cz_start)

                    caminho_reduzido = caminho_reduzido[::-1]

                    # Garante que o destino final é rigorosamente exato
                    caminho_reduzido[-1] = (goal_loc[0], goal_loc[1], idx_to_z[z_idx])

                # 2. Interpolar os pontos para restabelecer os frames sem criar hovers falsos
                caminho_final = []
                passos_interp = int(FATOR_ESCALA)

                for i in range(len(caminho_reduzido) - 1):
                    p1 = caminho_reduzido[i]
                    p2 = caminho_reduzido[i + 1]

                    x1, y1, z1 = p1
                    x2, y2, z2 = p2

                    if x1 == x2 and y1 == y2:
                        # É um movimento puramente vertical (Z).
                        # CORREÇÃO 2: Filtro Anti-Hover
                        if not caminho_final or caminho_final[-1] != p1:
                            caminho_final.append(p1)
                    else:
                        # Movimento no plano XY: injeta os frames para escalar a distância
                        for step in range(passos_interp):
                            f = step / passos_interp
                            nx = x1 + (x2 - x1) * f
                            ny = y1 + (y2 - y1) * f
                            nz = z1 + (z2 - z1) * f

                            p_novo = (int(round(nx)), int(round(ny)), nz)

                            if not caminho_final or caminho_final[-1] != p_novo:
                                caminho_final.append(p_novo)

                p_fim = caminho_reduzido[-1]
                if not caminho_final or caminho_final[-1] != p_fim:
                    caminho_final.append(p_fim)

                return _guardar_cache(caminho_final)

            if t >= max_t: continue

            g_atual = g_score[(x, y, z_idx, t)]
            g_atual_e = g_energia.get((x, y, z_idx, t), 0.0)

            movimentos = list(movimentos_base)

            if z_idx > 0:
                dist_z_descida = abs(idx_to_z[z_idx] - idx_to_z[z_idx - 1])
                movimentos.append((0, 0, -1, dist_z_descida))
            if z_idx < num_camadas - 1:
                dist_z_subida = abs(idx_to_z[z_idx + 1] - idx_to_z[z_idx])
                movimentos.append((0, 0, 1, dist_z_subida))

            for dx, dy, dz_idx, dist in movimentos:
                nx, ny, nz_idx = x + dx, y + dy, z_idx + dz_idx

                if not (0 <= nx < grid_max_x and 0 <= ny < grid_max_y): continue
                if esta_bloqueado(nz_idx, nx, ny): continue

                if dz_idx == 0:
                    dist_fisica_m = dist * FATOR_ESCALA
                    custo_fisico = dist_fisica_m / v_horiz
                    custo_energia = dist_fisica_m * c_cruzeiro
                    nt = t + int(FATOR_ESCALA)
                else:
                    dist_fisica_m = dist
                    if dz_idx > 0:
                        custo_fisico = dist_fisica_m / v_sub
                        custo_energia = dist_fisica_m * c_subida
                    else:
                        custo_fisico = dist_fisica_m / v_des
                        custo_energia = dist_fisica_m * c_descida
                    nt = t + 1

                chave_alvo = (nx, ny, nz_idx, nt)
                if chave_alvo in ocupacao_set: continue

                chave_origem_alvo = (nx, ny, nz_idx, t)
                chave_alvo_origem = (x, y, z_idx, nt)
                if chave_origem_alvo in agentes_dict and chave_alvo_origem in agentes_dict:
                    if agentes_dict[chave_origem_alvo] == agentes_dict[chave_alvo_origem]:
                        continue

                # OTIMIZAÇÃO MÁXIMA: Leitura de índice C ultrarrápida
                melhor = melhor_t_espacial_ref[nx, ny, nz_idx]

                if melhor == -1:
                    melhor_t_espacial_ref[nx, ny, nz_idx] = nt
                    celulas_sujas.append((nx, ny, nz_idx))  # Marca a célula para limpeza posterior
                elif nt > melhor + 5:
                    continue

                novo_g = g_atual + custo_fisico

                if chave_alvo not in g_score or novo_g < g_score[chave_alvo]:
                    g_score[chave_alvo] = novo_g
                    g_energia[chave_alvo] = g_atual_e + custo_energia

                    f_novo = novo_g + h_fisico(nx, ny, nz_idx)
                    veio_de[chave_alvo] = (x, y, z_idx, t)
                    heapq.heappush(abertos, (f_novo, nt, nx, ny, nz_idx))

        return _guardar_cache([])
    finally:
        # Limpa APENAS os pixels que foram tocados nesta busca específica
        # Restaurando a matriz reciclável para o seu estado original (-1) num milissegundo
        if 'celulas_sujas' in locals():
            for cx, cy, cz in celulas_sujas:
                melhor_t_espacial_ref[cx, cy, cz] = -1