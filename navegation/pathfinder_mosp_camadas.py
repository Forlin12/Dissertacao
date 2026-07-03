# pathfinder_mosp_camadas.py
import heapq
import math
import numpy as np
from shapely.geometry import Point


class Label:
    """Rótulo para o algoritmo MOSP contendo os múltiplos objetivos."""

    def __init__(self, obj_tempo, obj_energia, x, y, z_idx, t, pai=None):
        self.obj_tempo = obj_tempo  # Objetivo 1: Tempo total / Frames
        self.obj_energia = obj_energia  # Objetivo 2: Custo acumulado de manobras/alturas
        self.x = x
        self.y = y
        self.z_idx = z_idx
        self.t = t
        self.pai = pai

    def __lt__(self, outro):
        # Para a fila de prioridade, ordenamos por uma métrica lexicográfica ou soma linear.
        # Damos preferência ao tempo, usando o custo de energia como critério de desempate.
        if math.isclose(self.obj_tempo, outro.obj_tempo, abs_tol=1e-4):
            return self.obj_energia < outro.obj_energia
        return self.obj_tempo < outro.obj_tempo

    def domina(self, outro):
        """Verifica se este rótulo domina o 'outro' (Fronteira de Pareto)."""
        # Um rótulo domina outro se for estritamente melhor em pelo menos um objetivo
        # e não for pior em nenhum deles.
        return (self.obj_tempo <= outro.obj_tempo and self.obj_energia <= outro.obj_energia) and \
            (self.obj_tempo < outro.obj_tempo or self.obj_energia < outro.obj_energia)


def calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, vetor_camadas,
                              t_inicial=0):
    """
    Calcula a rota de um drone utilizando o algoritmo MOSP Label Setting em um espaço-tempo 4D.
    Substitui a lógica tradicional do TEA* mantendo total compatibilidade com o main.py.
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
        if grid_estatico[z_idx_inicial, sx, sy] or grid_estatico[z_idx_inicial, gx, gy]:
            print(f"      🛑 [MOSP 4D] Partida ou Destino Inválido.")
            return []

        # 3. FILTRO DE RESERVAS UTM DINÂMICAS
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

        # --- CORREÇÃO 2: Heurística Octile (Mais leve para a CPU) ---
        PESO_HEURISTICA = 1.0

        def h_tempo(x, y):
            dx = abs(gx - x)
            dy = abs(gy - y)
            # Distância Octile perfeitamente admissível em grelhas 8-way
            return (dx + dy + (1.414 - 2.0) * min(dx, dy)) * PESO_HEURISTICA

        # 4. ESTRUTURAS DO MOSP LABEL SETTING
        # Armazena listas de rótulos não-dominados por estado espacial (x, y, z_idx)
        rotulos_permanentes = {}

        abertos = []
        rotulo_inicial = Label(obj_tempo=0.0, obj_energia=0.0, x=sx, y=sy, z_idx=z_idx_inicial, t=t_inicial)

        # Inserimos na heap com base na estimativa do custo total do rótulo inicial (f_score)
        heapq.heappush(abertos, (rotulo_inicial.obj_tempo + h_tempo(sx, sy), rotulo_inicial))

        max_t = t_inicial + int(math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) * 3.0) + 150
        LIMITE_ITERACOES = 350000
        iteracoes = 0

        # Movimentos horizontais básicos: (dx, dy, dz, custo_tempo, custo_energia)
        movimentos_base = [
            (1, 0, 0, 1.0, 1.0), (-1, 0, 0, 1.0, 1.0), (0, 1, 0, 1.0, 1.0), (0, -1, 0, 1.0, 1.0),
            (1, 1, 0, 1.414, 1.5), (-1, -1, 0, 1.414, 1.5), (1, -1, 0, 1.414, 1.5), (-1, 1, 0, 1.414, 1.5),
            (0, 0, 0, 1.0, 1.2)  # Comando HOVER (Esperar no mesmo ponto gasta energia e tempo)
        ]

        melhores_solucoes_destino = []

        while abertos:
            iteracoes += 1
            if iteracoes > LIMITE_ITERACOES:
                break

            _, label_atual = heapq.heappop(abertos)
            estado_espacial = (label_atual.x, label_atual.y, label_atual.z_idx)

            # --- CORREÇÃO 3: Limite Rígido de Pruning Local ---
            # Se já temos 3 boas rotas alternativas para este quadrado, ignoramos o resto.
            if len(rotulos_permanentes.get(estado_espacial, [])) >= 3:
                continue

            # Critério de Pruning / Eliminação por Dominância local
            dominado = False
            if estado_espacial in rotulos_permanentes:
                for r_perm in rotulos_permanentes[estado_espacial]:
                    if r_perm.domina(label_atual):
                        dominado = True
                        break
            if dominado:
                continue

            # Adiciona à lista de não-dominados do nó
            if estado_espacial not in rotulos_permanentes:
                rotulos_permanentes[estado_espacial] = []
            rotulos_permanentes[estado_espacial].append(label_atual)

            # Verificação de Destino atingido
            if label_atual.x == gx and label_atual.y == gy:
                melhores_solucoes_destino.append(label_atual)
                # Como a Fila está ordenada por prioridade de tempo, o primeiro a chegar tende a ser o ótimo temporal.
                # Se já coletamos alternativas suficientes na fronteira de Pareto, podemos encerrar.
                if len(melhores_solucoes_destino) >= 3:
                    break

            if label_atual.t >= max_t:
                continue

            # Construção dinâmica de adjacências de mudança de camada
            movimentos = list(movimentos_base)
            if label_atual.z_idx > 0:
                movimentos.append((0, 0, -1, 2.0, 3.0))  # Descer camada (Tempo extra de transição e energia)
            if label_atual.z_idx < num_camadas - 1:
                movimentos.append((0, 0, 1, 2.0, 4.0))  # Subir camada (Gasta mais energia mecânica)

            for dx, dy, dz_idx, c_t, c_e in movimentos:
                nx, ny, nz_idx = label_atual.x + dx, label_atual.y + dy, label_atual.z_idx + dz_idx
                nt = label_atual.t + 1

                if not (0 <= nx < max_x and 0 <= ny < max_y): continue
                if grid_estatico[nz_idx, nx, ny]: continue

                # Filtro de Colisão Espaço-Tempo UTM
                chave_alvo = (nx, ny, nz_idx, nt)
                if chave_alvo in ocupacao_set: continue

                # Evita colisões frontais por troca simultânea de posição
                chave_origem_alvo = (nx, ny, nz_idx, label_atual.t)
                chave_alvo_origem = (label_atual.x, label_atual.y, label_atual.z_idx, nt)
                if chave_origem_alvo in agentes_dict and chave_alvo_origem in agentes_dict:
                    if agentes_dict[chave_origem_alvo] == agentes_dict[chave_alvo_origem]:
                        continue

                # --- CORREÇÃO 1: Arredondamento dos custos (Evita explosão de estados únicos) ---
                novo_obj_tempo = round(label_atual.obj_tempo + c_t, 2)
                novo_obj_energia = label_atual.obj_energia + c_e

                # Penalização suave para afastamento da altitude recomendada base
                novo_obj_energia += abs(idx_to_z[nz_idx] - z_inicial_val) * 0.1
                novo_obj_energia = round(novo_obj_energia, 2)

                novo_label = Label(novo_obj_tempo, novo_obj_energia, nx, ny, nz_idx, nt, pai=label_atual)

                # Pruning antes de inserir na Heap: Verifica se já é dominado no destino alvo
                estado_alvo = (nx, ny, nz_idx)
                if estado_alvo in rotulos_permanentes:
                    if any(r.domina(novo_label) for r in rotulos_permanentes[estado_alvo]):
                        continue

                f_score_tempo = novo_obj_tempo + h_tempo(nx, ny)
                heapq.heappush(abertos, (f_score_tempo, novo_label))

        # --- CORREÇÃO 4: Typo na variável de melhores soluções corrigido ---
        # 5. SELECIONAR O MELHOR RÓTULO DA FRONTEIRA DE PARETO NO DESTINO
        if melhores_solucoes_destino:
            # Seleciona o equilíbrio ideal. Aqui, optamos pelo rótulo que minimizou o tempo
            # respeitando a restrição de menor agressividade energética.
            melhor_label = min(melhores_solucoes_destino, key=lambda l: (l.obj_tempo, l.obj_energia))

            caminho_final = []
            curr = melhor_label
            while curr is not None:
                caminho_final.append((curr.x, curr.y, idx_to_z[curr.z_idx]))
                curr = curr.pai
            return caminho_final[::-1]

        print(f"      🧱 [MOSP 4D] Não foi possível encontrar uma rota válida sem dominância crítica.")
        return []

    finally:
        for z, x, y, val_original in pixels_restaurar:
            grid_estatico[z, x, y] = val_original