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
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            if grid[z_idx, x, y]:
                continue
            px = x * fator_escala
            py = y * fator_escala
            if point_in_polygon_numba(px, py, poly_coords):
                grid[z_idx, x, y] = True


_cache_lock = threading.Lock()


def calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, vetor_camadas,
                              t_inicial=0):
    FATOR_ESCALA = 2.5
    grid_max_x = int(math.ceil(max_x / FATOR_ESCALA))
    grid_max_y = int(math.ceil(max_y / FATOR_ESCALA))

    if not hasattr(calcular_rota_tea_camadas, "cache_grid"):
        calcular_rota_tea_camadas.cache_grid = None
        calcular_rota_tea_camadas.cache_cidade_id = None

    if calcular_rota_tea_camadas.cache_cidade_id != id(lotes_gdf):
        with _cache_lock:
            if calcular_rota_tea_camadas.cache_cidade_id != id(lotes_gdf):
                num_camadas = len(vetor_camadas)
                grid = np.zeros((num_camadas, grid_max_x, grid_max_y), dtype=bool)
                inflacao = drone.raio + 1.0

                for z_idx, camada in enumerate(vetor_camadas):
                    predios_altos = lotes_gdf[lotes_gdf['altura_z'] >= camada]
                    for _, p in predios_altos.iterrows():
                        geom_inflada = p.geometry.buffer(inflacao)
                        b = geom_inflada.bounds

                        x_min = max(0, int(math.floor(b[0] / FATOR_ESCALA)))
                        x_max = min(grid_max_x - 1, int(math.ceil(b[2] / FATOR_ESCALA)))
                        y_min = max(0, int(math.floor(b[1] / FATOR_ESCALA)))
                        y_max = min(grid_max_y - 1, int(math.ceil(b[3] / FATOR_ESCALA)))

                        geometrias = [geom_inflada] if geom_inflada.geom_type == 'Polygon' else geom_inflada.geoms
                        for g in geometrias:
                            coords = np.array(g.exterior.coords)
                            rasterizar_bbox_numba(grid, z_idx, coords, x_min, x_max, y_min, y_max, FATOR_ESCALA)

                calcular_rota_tea_camadas.cache_grid = grid
                calcular_rota_tea_camadas.cache_cidade_id = id(lotes_gdf)

    grid_estatico = calcular_rota_tea_camadas.cache_grid

    z_to_idx = {z: i for i, z in enumerate(vetor_camadas)}
    idx_to_z = {i: z for i, z in enumerate(vetor_camadas)}
    z_idx_inicial = 0
    num_camadas = len(vetor_camadas)

    sx = int(round(start_loc[0] / FATOR_ESCALA))
    sy = int(round(start_loc[1] / FATOR_ESCALA))
    gx = int(round(goal_loc[0] / FATOR_ESCALA))
    gy = int(round(goal_loc[1] / FATOR_ESCALA))

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
        if esta_bloqueado(z_idx_inicial, sx, sy) or esta_bloqueado(z_idx_inicial, gx, gy):
            print(f"      🛑 [TEA 4D] Partida ou Destino Inválido.")
            return []

        v_horiz = drone.velocidade_horiz if drone.velocidade_horiz > 0 else 1.0
        v_sub = drone.velocidade_subida if drone.velocidade_subida > 0 else 1.0
        v_des = drone.velocidade_descida if drone.velocidade_descida > 0 else 1.0

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

        def h_tempo(x, y):
            dist_xy_celulas = math.sqrt((gx - x) ** 2 + (gy - y) ** 2)
            dist_xy_metros = dist_xy_celulas * FATOR_ESCALA
            return dist_xy_metros / v_horiz

        # ==========================================
        # MOTOR TEA: A* sobre o estado (x, y, z_idx, t)
        # Custo único: tempo de voo (g)
        # ==========================================
        PESO_HEURISTICA = 1.2
        LIMITE_ITERACOES = 400000000
        iteracoes = 0

        dist_max_t_metros = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) * FATOR_ESCALA
        max_t = t_inicial + int(dist_max_t_metros * 6.0) + 150

        movimentos_base = [
            (1, 0, 0, 1.0), (-1, 0, 0, 1.0), (0, 1, 0, 1.0), (0, -1, 0, 1.0),
            (1, 1, 0, 1.414), (-1, -1, 0, 1.414), (1, -1, 0, 1.414), (-1, 1, 0, 1.414)
        ]

        estado_inicial = (sx, sy, z_idx_inicial, t_inicial)
        g_melhor = {estado_inicial: 0.0}
        pai = {estado_inicial: None}
        fechados = set()
        contador = 0
        abertos = []
        heapq.heappush(abertos, (h_tempo(sx, sy) * PESO_HEURISTICA, contador, estado_inicial))

        estado_final = None

        while abertos:
            iteracoes += 1
            if iteracoes > LIMITE_ITERACOES:
                print(f"      ⏳ [FALHA TEA 4D] Timeout computacional atingido.")
                break

            _, _, estado = heapq.heappop(abertos)
            if estado in fechados:
                continue
            fechados.add(estado)

            x, y, z_idx, t = estado
            g_atual = g_melhor[estado]

            if x == gx and y == gy:
                estado_final = estado
                break

            if t >= max_t:
                continue

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
                    c_t = dist_fisica_m / v_horiz
                    nt = t + int(FATOR_ESCALA)
                else:
                    dist_fisica_m = dist
                    if dz_idx > 0:
                        c_t = dist_fisica_m / v_sub
                    else:
                        c_t = dist_fisica_m / v_des
                    nt = t + 1

                if (nx, ny, nz_idx, nt) in ocupacao_set: continue

                chave_origem_alvo = (nx, ny, nz_idx, t)
                chave_alvo_origem = (x, y, z_idx, nt)
                if chave_origem_alvo in agentes_dict and chave_alvo_origem in agentes_dict:
                    if agentes_dict[chave_origem_alvo] == agentes_dict[chave_alvo_origem]:
                        continue

                novo_estado = (nx, ny, nz_idx, nt)
                if novo_estado in fechados: continue

                ng = g_atual + c_t
                if ng < g_melhor.get(novo_estado, float('inf')):
                    g_melhor[novo_estado] = ng
                    pai[novo_estado] = estado
                    contador += 1
                    f_score_tempo = ng + (h_tempo(nx, ny) * PESO_HEURISTICA)
                    heapq.heappush(abertos, (f_score_tempo, contador, novo_estado))

        if estado_final is not None:
            caminho_reduzido = []
            curr = estado_final
            while curr is not None:
                caminho_reduzido.append((curr[0] * FATOR_ESCALA, curr[1] * FATOR_ESCALA, idx_to_z[curr[2]]))
                curr = pai[curr]

            if caminho_reduzido:
                cz_start = caminho_reduzido[-1][2]
                caminho_reduzido[-1] = (start_loc[0], start_loc[1], cz_start)
                caminho_reduzido = caminho_reduzido[::-1]
                caminho_reduzido[-1] = (goal_loc[0], goal_loc[1], idx_to_z[estado_final[2]])

            caminho_final = []
            passos_interp = int(FATOR_ESCALA)

            for i in range(len(caminho_reduzido) - 1):
                p1 = caminho_reduzido[i]
                p2 = caminho_reduzido[i + 1]

                x1, y1, z1 = p1
                x2, y2, z2 = p2

                if x1 == x2 and y1 == y2:
                    if not caminho_final or caminho_final[-1] != p1:
                        caminho_final.append(p1)
                else:
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

            return caminho_final

        print(f"      🧱 [TEA 4D] Não foi possível encontrar uma rota válida.")
        return []

    finally:
        pass