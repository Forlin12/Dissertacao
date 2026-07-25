import heapq
import math
import numpy as np
from shapely.geometry import Point


# =========================================================================
# ALGORITMO PRINCIPAL TEA* 4D (Alta Fidelidade Física)
# =========================================================================
def calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, vetor_camadas,
                              t_inicial=0):
    """
    Calcula a rota de um drone em um ambiente 4D (Espaço + Tempo).
    Utiliza as velocidades físicas reais do drone para calcular o custo (g_score)
    em segundos, mantendo a integridade do UTM baseada em frames lógicos.
    """

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

    # 2. INICIALIZAÇÃO E MAPEAMENTO
    z_to_idx = {z: i for i, z in enumerate(vetor_camadas)}
    idx_to_z = {i: z for i, z in enumerate(vetor_camadas)}
    z_inicial_val = vetor_camadas[0]
    z_idx_inicial = 0
    num_camadas = len(vetor_camadas)

    sx, sy = int(round(start_loc[0])), int(round(start_loc[1]))
    gx, gy = int(round(goal_loc[0])), int(round(goal_loc[1]))

    # --- AUTO-EXPURGADOR DE REBARBAS ---
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
        # Validação de Infraestrutura
        if grid_estatico[z_idx_inicial, sx, sy]:
            print(f"      🛑 [FALHA 4D] Partida Inválida em {(sx, sy)}.")
            return []
        if grid_estatico[z_idx_inicial, gx, gy]:
            print(f"      🛑 [FALHA 4D] Destino Inválido em {(gx, gy)}.")
            return []

        # 3. EXTRAÇÃO DOS PARÂMETROS FÍSICOS REAIS
        v_horiz = drone.velocidade_horiz if drone.velocidade_horiz > 0 else 1.0
        v_sub = drone.velocidade_subida if drone.velocidade_subida > 0 else 1.0
        v_des = drone.velocidade_descida if drone.velocidade_descida > 0 else 1.0

        PESO_HEURISTICA = 1.2

        def h_fisico(x, y, z_idx):
            # A heurística agora prevê o tempo real restante em segundos (em vez de distância geométrica abstrata)
            dist_xy = math.sqrt((gx - x) ** 2 + (gy - y) ** 2)
            dist_z = abs(idx_to_z[z_idx] - z_inicial_val)
            tempo_ideal = (dist_xy / v_horiz) + (dist_z / v_des)
            return tempo_ideal * PESO_HEURISTICA

        # 4. FILTRO UTM DINÂMICO (Reserva de Outros Agentes)
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

        # 5. BUSCA A*
        abertos = []
        # O heap guarda: (f_score_segundos, t_frame, x, y, z_idx)
        heapq.heappush(abertos, (h_fisico(sx, sy, z_idx_inicial), t_inicial, sx, sy, z_idx_inicial))

        # g_score agora acumula SEGUNDOS físicos
        g_score = {(sx, sy, z_idx_inicial, t_inicial): 0.0}
        veio_de = {}

        melhor_t_espacial = np.full((max_x, max_y, num_camadas), -1, dtype=np.int32)
        max_t = t_inicial + int(math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) * 6.0) + 150
        LIMITE_ITERACOES = 4000000
        iteracoes = 0

        # Os custos base passam a ser distâncias que serão divididas pela velocidade
        movimentos_base = [
            (1, 0, 0, 1.0), (-1, 0, 0, 1.0), (0, 1, 0, 1.0), (0, -1, 0, 1.0),
            (1, 1, 0, 1.414), (-1, -1, 0, 1.414), (1, -1, 0, 1.414), (-1, 1, 0, 1.414)
        ]

        while abertos:
            iteracoes += 1
            if iteracoes > LIMITE_ITERACOES:
                print(f"      ⏳ [FALHA 4D] Timeout!")
                return []

            f_cur, t, x, y, z_idx = heapq.heappop(abertos)

            # 6. SUCESSO: RECONSTRUÇÃO DA ROTA
            if x == gx and y == gy:
                caminho_final = []
                curr = (x, y, z_idx, t)
                while curr in veio_de:
                    cx, cy, cz_idx, _ = curr
                    caminho_final.append((cx, cy, idx_to_z[cz_idx]))
                    curr = veio_de[curr]
                caminho_final.append((sx, sy, z_inicial_val))
                return caminho_final[::-1]

            if t >= max_t: continue

            g_atual = g_score[(x, y, z_idx, t)]
            nt = t + 1  # O passo de tempo lógigo do UTM (frames) mantém-se imutável

            movimentos = list(movimentos_base)

            # Dinâmica Vertical com distâncias reais das camadas
            if z_idx > 0:
                dist_z_descida = abs(idx_to_z[z_idx] - idx_to_z[z_idx - 1])
                movimentos.append((0, 0, -1, dist_z_descida))
            if z_idx < num_camadas - 1:
                dist_z_subida = abs(idx_to_z[z_idx + 1] - idx_to_z[z_idx])
                movimentos.append((0, 0, 1, dist_z_subida))

            for dx, dy, dz_idx, dist in movimentos:
                nx, ny, nz_idx = x + dx, y + dy, z_idx + dz_idx

                if not (0 <= nx < max_x and 0 <= ny < max_y): continue
                if grid_estatico[nz_idx, nx, ny]: continue

                chave_alvo = (nx, ny, nz_idx, nt)
                if chave_alvo in ocupacao_set: continue

                chave_origem_alvo = (nx, ny, nz_idx, t)
                chave_alvo_origem = (x, y, z_idx, nt)
                if chave_origem_alvo in agentes_dict and chave_alvo_origem in agentes_dict:
                    if agentes_dict[chave_origem_alvo] == agentes_dict[chave_alvo_origem]:
                        continue

                melhor = melhor_t_espacial[nx, ny, nz_idx]
                if melhor == -1:
                    melhor_t_espacial[nx, ny, nz_idx] = nt
                elif nt > melhor + 15:  # Ligeiramente ampliado para permitir mais flexibilidade física
                    continue

                # Cálculo do custo em SEGUNDOS REAIS
                if dz_idx == 0:
                    custo_fisico = dist / v_horiz
                elif dz_idx > 0:
                    custo_fisico = dist / v_sub
                else:
                    custo_fisico = dist / v_des

                novo_g = g_atual + custo_fisico

                if chave_alvo not in g_score or novo_g < g_score[chave_alvo]:
                    g_score[chave_alvo] = novo_g

                    # Penalidade temporal ínfima (0.05s) apenas para desincentivar trocas de camada desnecessárias
                    f_novo = novo_g + h_fisico(nx, ny, nz_idx) + (abs(idx_to_z[nz_idx] - z_inicial_val) * 0.05)

                    veio_de[chave_alvo] = (x, y, z_idx, t)
                    heapq.heappush(abertos, (f_novo, nt, nx, ny, nz_idx))

        print(f"      🧱 [FALHA 4D] Encurralado! Sem rotas viáveis.")
        return []

    finally:
        # --- RESTAURAÇÃO DO CACHE ESTÁTICO ---
        for z, x, y, val_original in pixels_restaurar:
            grid_estatico[z, x, y] = val_original