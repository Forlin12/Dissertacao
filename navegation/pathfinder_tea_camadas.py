import heapq
import math
import numpy as np
from shapely.geometry import Point

# =========================================================================
# ALGORITMO PRINCIPAL A* 4D
# =========================================================================
def calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, vetor_camadas,
                              t_inicial=0):
    """
    Calcula a rota de um drone em um ambiente 4D (Espaço + Tempo).
    Retorna a rota bruta baseada no grid, sem pós-processamento de suavização.
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

    # --- AUTO-EXPURGADOR DE REBARBAS (COM REGISTRO PARA RESTAURAÇÃO) ---
    raio_limpeza = int(math.ceil(drone.raio)) + 1
    pixels_restaurar = []

    for dx in range(-raio_limpeza, raio_limpeza + 1):
        for dy in range(-raio_limpeza, raio_limpeza + 1):
            # Limpeza na Origem
            nx_s, ny_s = sx + dx, sy + dy
            if 0 <= nx_s < max_x and 0 <= ny_s < max_y:
                # Salva o estado original antes de modificar
                pixels_restaurar.append((z_idx_inicial, nx_s, ny_s, grid_estatico[z_idx_inicial, nx_s, ny_s]))
                grid_estatico[z_idx_inicial, nx_s, ny_s] = False

            # Limpeza no Destino
            nx_g, ny_g = gx + dx, gy + dy
            if 0 <= nx_g < max_x and 0 <= ny_g < max_y:
                # Salva o estado original antes de modificar
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

        PESO_HEURISTICA = 1.2

        def h(x, y):
            return math.sqrt((gx - x) ** 2 + (gy - y) ** 2) * PESO_HEURISTICA

        # 3. FILTRO UTM DINÂMICO (Reserva de Outros Agentes)
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

        # 4. BUSCA A*
        abertos = []
        heapq.heappush(abertos, (h(sx, sy), t_inicial, sx, sy, z_idx_inicial))
        g_score = {(sx, sy, z_idx_inicial, t_inicial): 0.0}
        veio_de = {}

        melhor_t_espacial = np.full((max_x, max_y, num_camadas), -1, dtype=np.int32)
        max_t = t_inicial + int(math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) * 3.0) + 150
        LIMITE_ITERACOES = 400000
        iteracoes = 0

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

            # 5. SUCESSO: RECONSTRUÇÃO DA ROTA (BRUTA)
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
            nt = t + 1

            movimentos = list(movimentos_base)
            if z_idx > 0: movimentos.append((0, 0, -1, 2.0))
            if z_idx < num_camadas - 1: movimentos.append((0, 0, 1, 2.0))

            for dx, dy, dz_idx, custo in movimentos:
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
                elif nt > melhor + 10:
                    continue

                novo_g = g_atual + custo
                if chave_alvo not in g_score or novo_g < g_score[chave_alvo]:
                    g_score[chave_alvo] = novo_g
                    f_novo = novo_g + h(nx, ny) + (abs(idx_to_z[nz_idx] - z_inicial_val) * 0.1)
                    veio_de[chave_alvo] = (x, y, z_idx, t)
                    heapq.heappush(abertos, (f_novo, nt, nx, ny, nz_idx))

        print(f"      🧱 [FALHA 4D] Encurralado! Sem rotas viáveis.")
        return []

    finally:
        # --- RESTAURAÇÃO DO CACHE ESTÁTICO ---
        # Este bloco executa SEMPRE, não importa qual 'return' seja acionado acima.
        for z, x, y, val_original in pixels_restaurar:
            grid_estatico[z, x, y] = val_original