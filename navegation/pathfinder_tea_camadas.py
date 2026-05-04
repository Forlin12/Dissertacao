import heapq
import math
from shapely.geometry import Point


def calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, vetor_camadas,
                              t_inicial=0):
    def heuristica(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    obstaculos_por_camada = {}
    inflacao = drone.raio + 1.0

    # Gera a matriz de colisões independente para CADA altitude configurada
    # VERIFICAÇÃO EXATA: Usa geom_inflada.contains() em vez de apenas a caixa delimitadora quadrada.
    for camada in vetor_camadas:
        obs = set()
        predios_altos = lotes_gdf[lotes_gdf['altura_z'] >= camada]
        for _, p in predios_altos.iterrows():
            geom_inflada = p.geometry.buffer(inflacao)
            b = geom_inflada.bounds
            for x in range(int(math.floor(b[0])), int(math.ceil(b[2])) + 1):
                for y in range(int(math.floor(b[1])), int(math.ceil(b[3])) + 1):
                    if geom_inflada.contains(Point(x, y)):
                        obs.add((x, y))
        obstaculos_por_camada[camada] = obs

    z_inicial = vetor_camadas[0]
    start = (int(round(start_loc[0])), int(round(start_loc[1])), z_inicial)
    goal = (int(round(goal_loc[0])), int(round(goal_loc[1])))

    # ------------------ SISTEMA DE DIAGNÓSTICO: PRÉ-VOO ------------------
    obstaculos_atuais = obstaculos_por_camada[z_inicial]
    if (start[0], start[1]) in obstaculos_atuais:
        print(
            f"      🛑 [FALHA] Partida Inválida! A Base {(start[0], start[1])} está dentro de um obstáculo na camada de {z_inicial}m.")
        return []
    if (goal[0], goal[1]) in obstaculos_atuais:
        print(
            f"      🛑 [FALHA] Destino Inválido! O Cliente {(goal[0], goal[1])} está dentro de um obstáculo na camada de {z_inicial}m.")
        return []
    # ---------------------------------------------------------------------

    PESO_HEURISTICA = 1.2
    abertos = [(0 + (heuristica(start, goal) * PESO_HEURISTICA), t_inicial, start[0], start[1], start[2])]
    veio_de = {}
    g_score = {(start[0], start[1], start[2], t_inicial): 0}

    max_t = t_inicial + int(heuristica(start, goal) * 3.0) + 150
    LIMITE_ITERACOES = 400000
    iteracoes = 0
    melhor_t_espacial = {}

    while abertos:
        iteracoes += 1

        # ------------------ SISTEMA DE DIAGNÓSTICO: TIMEOUT ------------------
        if iteracoes > LIMITE_ITERACOES:
            print(f"      ⏳ [FALHA] Timeout! Atingiu {LIMITE_ITERACOES} cálculos no espaço 4D. Trânsito extremo.")
            return []
        # ---------------------------------------------------------------------

        _, t, x, y, z = heapq.heappop(abertos)

        if (x, y) == goal:
            caminho_final = []
            curr = (x, y, z, t)
            while curr in veio_de:
                caminho_final.append((curr[0], curr[1], curr[2]))
                curr = veio_de[curr]
            caminho_final.append((start[0], start[1], start[2]))
            return caminho_final[::-1]

        if t >= max_t: continue

        vizinhos = [
            (x + 1, y, z, 1.0), (x - 1, y, z, 1.0), (x, y + 1, z, 1.0), (x, y - 1, z, 1.0),
            (x + 1, y + 1, z, 1.414), (x - 1, y - 1, z, 1.414), (x + 1, y - 1, z, 1.414), (x - 1, y + 1, z, 1.414)
        ]

        # Adiciona manobras verticais (saltar para a camada de cima ou de baixo)
        idx_z = vetor_camadas.index(z)
        if idx_z > 0:
            vizinhos.append((x, y, vetor_camadas[idx_z - 1], 2.0))
        if idx_z < len(vetor_camadas) - 1:
            vizinhos.append((x, y, vetor_camadas[idx_z + 1], 2.0))

        conflito_dinamico = False
        vizinhos_validos = []

        for nx, ny, nz, custo in vizinhos:
            nt = t + 1
            if not (0 <= nx < max_x and 0 <= ny < max_y): continue
            if (nx, ny) in obstaculos_por_camada[nz]: continue

            bloqueado = False
            # O Registo UTM agora bloqueia trânsito analisando X, Y, Z e Tempo
            if (nx, ny, nz, nt) in reserva_global:
                bloqueado = True
            elif (nx, ny, nz, t) in reserva_global and (x, y, z, nt) in reserva_global:
                if reserva_global[(nx, ny, nz, t)] == reserva_global[(x, y, z, nt)]:
                    bloqueado = True

            if bloqueado:
                conflito_dinamico = True
            else:
                vizinhos_validos.append((nx, ny, nz, custo))

        # Se houver conflito, adiciona o vizinho de ESPERA TÁTICA
        if conflito_dinamico:
            vizinhos_validos.append((x, y, z, 1.0))

        for nx, ny, nz, custo in vizinhos_validos:
            nt = t + 1

            if nx != x or ny != y or nz != z:
                if (nx, ny, nz) not in melhor_t_espacial:
                    melhor_t_espacial[(nx, ny, nz)] = nt
                elif nt > melhor_t_espacial[(nx, ny, nz)] + 10:
                    continue

            novo_g = g_score[(x, y, z, t)] + custo
            if (nx, ny, nz, nt) not in g_score or novo_g < g_score[(nx, ny, nz, nt)]:
                g_score[(nx, ny, nz, nt)] = novo_g
                f = novo_g + (heuristica((nx, ny), goal) * PESO_HEURISTICA) + (abs(nz - vetor_camadas[0]) * 0.1)
                veio_de[(nx, ny, nz, nt)] = (x, y, z, t)
                heapq.heappush(abertos, (f, nt, nx, ny, nz))

    # ------------------ SISTEMA DE DIAGNÓSTICO: BECO SEM SAÍDA ------------------
    print(f"      🧱 [FALHA] Encurralado! O drone explorou todo o espaço 4D, mas não encontrou passagem nos edifícios.")
    return []
    # ----------------------------------------------------------------------------