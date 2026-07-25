import heapq
import math
import numpy as np
from shapely.geometry import Point


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
        # Epsilon-Dominância ajustada para poda agressiva de rotas semelhantes
        eps_t = 0.5  # 0.5 segundos de tolerância
        eps_e = 0.2  # 0.2 Wh de tolerância
        return (self.obj_tempo <= outro.obj_tempo + eps_t) and (self.obj_energia <= outro.obj_energia + eps_e)


def calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, vetor_camadas,
                              t_inicial=0):
    # 1. CACHE ESTÁTICO DE OBSTÁCULOS 3D
    if not hasattr(calcular_rota_tea_camadas, "cache_grid"):
        calcular_rota_tea_camadas.cache_grid = None
        calcular_rota_tea_camadas.cache_cidade_id = None

    if calcular_rota_tea_camadas.cache_cidade_id != id(lotes_gdf):
        num_camadas = len(vetor_camadas)
        grid = np.zeros((num_camadas, max_x, max_y), dtype=bool)
        inflacao = drone.raio + 1.0

        for z_idx, camada in enumerate(vetor_camadas):
            predios_altos = lotes_gdf[lotes_gdf['altura_z'] >= camada]
            for _, p in predios_altos.iterrows():
                geom_inflada = p.geometry.buffer(inflacao)
                b = geom_inflada.bounds
                x_min = max(0, int(math.floor(b[0])))
                x_max = min(max_x - 1, int(math.ceil(b[2])))
                y_min = max(0, int(math.floor(b[1])))
                y_max = min(max_y - 1, int(math.ceil(b[3])))
                for x in range(x_min, x_max + 1):
                    for y in range(y_min, y_max + 1):
                        if geom_inflada.contains(Point(x, y)):
                            grid[z_idx, x, y] = True

        calcular_rota_tea_camadas.cache_grid = grid
        calcular_rota_tea_camadas.cache_cidade_id = id(lotes_gdf)

    grid_estatico = calcular_rota_tea_camadas.cache_grid

    # 2. INICIALIZAÇÃO
    z_to_idx = {z: i for i, z in enumerate(vetor_camadas)}
    idx_to_z = {i: z for i, z in enumerate(vetor_camadas)}
    z_inicial_val = vetor_camadas[0]
    z_idx_inicial = 0
    num_camadas = len(vetor_camadas)

    sx, sy = int(round(start_loc[0])), int(round(start_loc[1]))
    gx, gy = int(round(goal_loc[0])), int(round(goal_loc[1]))

    raio_limpeza = int(math.ceil(drone.raio)) + 1
    pixels_restaurar = []
    for dx in range(-raio_limpeza, raio_limpeza + 1):
        for dy in range(-raio_limpeza, raio_limpeza + 1):
            nx_s, ny_s = sx + dx, sy + dy
            if 0 <= nx_s < max_x and 0 <= ny_s < max_y:
                pixels_restaurar.append((z_idx_inicial, nx_s, ny_s, grid_estatico[z_idx_inicial, nx_s, ny_s]))
                grid_estatico[z_idx_inicial, nx_s, ny_s] = False
            nx_g, ny_g = gx + dx, gy + dy
            if 0 <= nx_g < max_x and 0 <= ny_g < max_y:
                pixels_restaurar.append((z_idx_inicial, nx_g, ny_g, grid_estatico[z_idx_inicial, nx_g, ny_g]))
                grid_estatico[z_idx_inicial, nx_g, ny_g] = False

    try:
        if grid_estatico[z_idx_inicial, sx, sy] or grid_estatico[z_idx_inicial, gx, gy]:
            print(f"      🛑 [MOSP 4D] Partida ou Destino Inválido.")
            return []

        # 3. PERFIS FÍSICOS
        v_horiz = drone.velocidade_horiz if drone.velocidade_horiz > 0 else 1.0
        v_sub = drone.velocidade_subida if drone.velocidade_subida > 0 else 1.0
        v_des = drone.velocidade_descida if drone.velocidade_descida > 0 else 1.0

        c_cruzeiro = drone.consumo_cruzeiro + (drone.carga * drone.penalidade_carga)
        c_subida = drone.consumo_subida + (drone.carga * drone.penalidade_carga)
        c_descida = drone.consumo_descida + (drone.carga * drone.penalidade_carga)
        c_hover = drone.consumo_hover

        # 4. RESERVAS UTM
        ocupacao_set = set()
        agentes_dict = {}
        for chaves, id_agente in reserva_global.items():
            if len(chaves) == 4:
                rx, ry, rz, rt = chaves
                if rz in z_to_idx:
                    c_idx = z_to_idx[rz]
                    chave_4d = (rx, ry, c_idx, rt)
                    ocupacao_set.add(chave_4d)
                    agentes_dict[chave_4d] = id_agente

        def h_tempo(x, y):
            dx = abs(gx - x)
            dy = abs(gy - y)
            return (dx + dy + (1.414 - 2.0) * min(dx, dy)) / v_horiz

        # 5. MOSP LABEL SETTING
        rotulos_permanentes = {}
        abertos = []
        rotulo_inicial = Label(obj_tempo=0.0, obj_energia=0.0, x=sx, y=sy, z_idx=z_idx_inicial, t=t_inicial)
        heapq.heappush(abertos, (rotulo_inicial.obj_tempo + h_tempo(sx, sy), rotulo_inicial))

        max_t = t_inicial + int(math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) * 5.0) + 300
        LIMITE_ITERACOES = 400000
        iteracoes = 0

        movimentos_base = [
            (1, 0, 0, 1.0 / v_horiz, 1.0 * c_cruzeiro), (-1, 0, 0, 1.0 / v_horiz, 1.0 * c_cruzeiro),
            (0, 1, 0, 1.0 / v_horiz, 1.0 * c_cruzeiro), (0, -1, 0, 1.0 / v_horiz, 1.0 * c_cruzeiro),
            (1, 1, 0, 1.414 / v_horiz, 1.414 * c_cruzeiro), (-1, -1, 0, 1.414 / v_horiz, 1.414 * c_cruzeiro),
            (1, -1, 0, 1.414 / v_horiz, 1.414 * c_cruzeiro), (-1, 1, 0, 1.414 / v_horiz, 1.414 * c_cruzeiro),
            (0, 0, 0, 1.0, c_hover)
        ]

        melhores_solucoes_destino = []

        while abertos:
            iteracoes += 1
            if iteracoes > LIMITE_ITERACOES:
                print(f"      ⏳ [FALHA MOSP 4D] Timeout computacional atingido nas iterações.")
                break

            _, label_atual = heapq.heappop(abertos)
            estado_espacial = (label_atual.x, label_atual.y, label_atual.z_idx)

            # Gargalo restrito: corta explosão de memória
            if len(rotulos_permanentes.get(estado_espacial, [])) >= 3:
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
                # Gatilho de saída rápido
                if len(melhores_solucoes_destino) >= 5:
                    break

            if label_atual.t >= max_t:
                continue

            movimentos = list(movimentos_base)

            if label_atual.z_idx > 0:
                dist_z_descida = abs(idx_to_z[label_atual.z_idx] - idx_to_z[label_atual.z_idx - 1])
                movimentos.append((0, 0, -1, dist_z_descida / v_des, dist_z_descida * c_descida))

            if label_atual.z_idx < num_camadas - 1:
                dist_z_subida = abs(idx_to_z[label_atual.z_idx + 1] - idx_to_z[label_atual.z_idx])
                movimentos.append((0, 0, 1, dist_z_subida / v_sub, dist_z_subida * c_subida))

            for dx, dy, dz_idx, c_t, c_e in movimentos:
                nx, ny, nz_idx = label_atual.x + dx, label_atual.y + dy, label_atual.z_idx + dz_idx
                nt = label_atual.t + 1

                if not (0 <= nx < max_x and 0 <= ny < max_y): continue
                if grid_estatico[nz_idx, nx, ny]: continue
                if (nx, ny, nz_idx, nt) in ocupacao_set: continue

                chave_origem_alvo = (nx, ny, nz_idx, label_atual.t)
                chave_alvo_origem = (label_atual.x, label_atual.y, label_atual.z_idx, nt)
                if chave_origem_alvo in agentes_dict and chave_alvo_origem in agentes_dict:
                    if agentes_dict[chave_origem_alvo] == agentes_dict[chave_alvo_origem]:
                        continue

                novo_obj_tempo = label_atual.obj_tempo + c_t
                novo_obj_energia = label_atual.obj_energia + c_e
                novo_obj_energia += abs(idx_to_z[nz_idx] - z_inicial_val) * 0.01

                novo_label = Label(novo_obj_tempo, novo_obj_energia, nx, ny, nz_idx, nt, pai=label_atual)

                estado_alvo = (nx, ny, nz_idx)
                if estado_alvo in rotulos_permanentes:
                    if any(r.domina(novo_label) for r in rotulos_permanentes[estado_alvo]):
                        continue

                # Heurística firme (* 1.3) força convergência direcional e evita timeout
                f_score_tempo = novo_obj_tempo + (h_tempo(nx, ny) * 1.3)
                heapq.heappush(abertos, (f_score_tempo, novo_label))

        # 6. SELEÇÃO DE PARETO
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

            caminho_final = []
            curr = melhor_label
            while curr is not None:
                caminho_final.append((curr.x, curr.y, idx_to_z[curr.z_idx]))
                curr = curr.pai
            return caminho_final[::-1]

        print(f"      🧱 [MOSP 4D] Não foi possível encontrar uma rota válida.")
        return []

    finally:
        for z, x, y, val_original in pixels_restaurar:
            grid_estatico[z, x, y] = val_original