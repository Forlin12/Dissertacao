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


# ==========================================
# ESTRUTURAS DO ALGORITMO MOSP
# ==========================================
_cache_lock = threading.Lock()


class Label:
    """Rótulo para o algoritmo MOSP contendo os múltiplos objetivos."""

    def __init__(self, obj_tempo, obj_energia, x, y, z_idx, t, pai=None):
        self.obj_tempo = obj_tempo
        self.obj_energia = obj_energia
        self.x = x
        self.y = y
        self.z_idx = z_idx
        self.t = t
        self.pai = pai

    def __lt__(self, outro):
        if math.isclose(self.obj_tempo, outro.obj_tempo, abs_tol=1e-4):
            return self.obj_energia < outro.obj_energia
        return self.obj_tempo < outro.obj_tempo

    def domina(self, outro):
        eps_t = 0.5
        eps_e = 0.2
        return (self.obj_tempo <= outro.obj_tempo + eps_t) and (self.obj_energia <= outro.obj_energia + eps_e)


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
    z_inicial_val = vetor_camadas[0]
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
            print(f"      🛑 [MOSP 4D] Partida ou Destino Inválido.")
            return []

        v_horiz = drone.velocidade_horiz if drone.velocidade_horiz > 0 else 1.0
        v_sub = drone.velocidade_subida if drone.velocidade_subida > 0 else 1.0
        v_des = drone.velocidade_descida if drone.velocidade_descida > 0 else 1.0

        c_cruzeiro = drone.consumo_cruzeiro + (drone.carga * drone.penalidade_carga)
        c_subida = drone.consumo_subida + (drone.carga * drone.penalidade_carga)
        c_descida = drone.consumo_descida + (drone.carga * drone.penalidade_carga)

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

        rotulos_permanentes = {}
        abertos = []
        rotulo_inicial = Label(obj_tempo=0.0, obj_energia=0.0, x=sx, y=sy, z_idx=z_idx_inicial, t=t_inicial)
        heapq.heappush(abertos, (rotulo_inicial.obj_tempo + h_tempo(sx, sy), rotulo_inicial))

        dist_max_t_metros = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) * FATOR_ESCALA
        max_t = t_inicial + int(dist_max_t_metros * 6.0) + 150
        LIMITE_ITERACOES = 400000000
        iteracoes = 0

        movimentos_base = [
            (1, 0, 0, 1.0), (-1, 0, 0, 1.0), (0, 1, 0, 1.0), (0, -1, 0, 1.0),
            (1, 1, 0, 1.414), (-1, -1, 0, 1.414), (1, -1, 0, 1.414), (-1, 1, 0, 1.414)
        ]

        melhores_solucoes_destino = []

        while abertos:
            iteracoes += 1
            if iteracoes > LIMITE_ITERACOES:
                print(f"      ⏳ [FALHA MOSP 4D] Timeout computacional atingido.")
                break

            _, label_atual = heapq.heappop(abertos)
            estado_espacial = (label_atual.x, label_atual.y, label_atual.z_idx)

            if len(rotulos_permanentes.get(estado_espacial, [])) >= 8:
                continue

            dominado = False
            if estado_espacial in rotulos_permanentes:
                for r_perm in rotulos_permanentes[estado_espacial]:
                    if r_perm.domina(label_atual):
                        dominado = True
                        break
            if dominado:
                continue

            if estado_espacial not in rotulos_permanentes:
                rotulos_permanentes[estado_espacial] = []
            rotulos_permanentes[estado_espacial].append(label_atual)

            if label_atual.x == gx and label_atual.y == gy:
                melhores_solucoes_destino.append(label_atual)
                if len(melhores_solucoes_destino) >= 5:
                    break

            if label_atual.t >= max_t:
                continue

            movimentos = list(movimentos_base)

            if label_atual.z_idx > 0:
                dist_z_descida = abs(idx_to_z[label_atual.z_idx] - idx_to_z[label_atual.z_idx - 1])
                movimentos.append((0, 0, -1, dist_z_descida))

            if label_atual.z_idx < num_camadas - 1:
                dist_z_subida = abs(idx_to_z[label_atual.z_idx + 1] - idx_to_z[label_atual.z_idx])
                movimentos.append((0, 0, 1, dist_z_subida))

            for dx, dy, dz_idx, dist in movimentos:
                nx, ny, nz_idx = label_atual.x + dx, label_atual.y + dy, label_atual.z_idx + dz_idx

                if not (0 <= nx < grid_max_x and 0 <= ny < grid_max_y): continue
                if esta_bloqueado(nz_idx, nx, ny): continue

                if dz_idx == 0:
                    dist_fisica_m = dist * FATOR_ESCALA
                    c_t = dist_fisica_m / v_horiz
                    c_e = dist_fisica_m * c_cruzeiro
                    nt = label_atual.t + int(FATOR_ESCALA)
                else:
                    dist_fisica_m = dist
                    if dz_idx > 0:
                        c_t = dist_fisica_m / v_sub
                        c_e = dist_fisica_m * c_subida
                    else:
                        c_t = dist_fisica_m / v_des
                        c_e = dist_fisica_m * c_descida
                    nt = label_atual.t + 1

                if (nx, ny, nz_idx, nt) in ocupacao_set: continue

                chave_origem_alvo = (nx, ny, nz_idx, label_atual.t)
                chave_alvo_origem = (label_atual.x, label_atual.y, label_atual.z_idx, nt)
                if chave_origem_alvo in agentes_dict and chave_alvo_origem in agentes_dict:
                    if agentes_dict[chave_origem_alvo] == agentes_dict[chave_alvo_origem]:
                        continue

                novo_obj_tempo = label_atual.obj_tempo + c_t
                novo_obj_energia = label_atual.obj_energia + c_e

                novo_label = Label(novo_obj_tempo, novo_obj_energia, nx, ny, nz_idx, nt, pai=label_atual)

                estado_alvo = (nx, ny, nz_idx)
                if estado_alvo in rotulos_permanentes:
                    if any(r.domina(novo_label) for r in rotulos_permanentes[estado_alvo]):
                        continue

                PESO_HEURISTICA = 1.2
                f_score_tempo = novo_obj_tempo + (h_tempo(nx, ny) * PESO_HEURISTICA)
                heapq.heappush(abertos, (f_score_tempo, novo_label))

        if melhores_solucoes_destino:
            min_t = min([l.obj_tempo for l in melhores_solucoes_destino])
            max_t = max([l.obj_tempo for l in melhores_solucoes_destino]) or 1.0
            min_e = min([l.obj_energia for l in melhores_solucoes_destino])
            max_e = max([l.obj_energia for l in melhores_solucoes_destino]) or 1.0

            melhor_label = None
            menor_dist = float('inf')

            for l in melhores_solucoes_destino:
                norm_t = (l.obj_tempo - min_t) / (max_t - min_t) if max_t > min_t else 0
                norm_e = (l.obj_energia - min_e) / (max_e - min_e) if max_e > min_e else 0
                dist_ideal = math.sqrt(norm_t ** 2 + norm_e ** 2)
                if dist_ideal < menor_dist:
                    menor_dist = dist_ideal
                    melhor_label = l

            caminho_reduzido = []
            curr = melhor_label
            while curr is not None:
                caminho_reduzido.append((curr.x * FATOR_ESCALA, curr.y * FATOR_ESCALA, idx_to_z[curr.z_idx]))
                curr = curr.pai

            if caminho_reduzido:
                cz_start = caminho_reduzido[-1][2]
                caminho_reduzido[-1] = (start_loc[0], start_loc[1], cz_start)
                caminho_reduzido = caminho_reduzido[::-1]
                caminho_reduzido[-1] = (goal_loc[0], goal_loc[1], idx_to_z[melhor_label.z_idx])

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

        print(f"      🧱 [MOSP 4D] Não foi possível encontrar uma rota válida.")
        return []

    finally:
        pass