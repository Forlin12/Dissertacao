import heapq
import math


def calcular_rota_tea(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, t_inicial=0):
    def heuristica(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    obstaculos_estaticos = set()
    inflacao = drone.raio + 1.0
    predios_altos = lotes_gdf[lotes_gdf['altura_z'] >= drone.altura_alvo]

    for _, p in predios_altos.iterrows():
        b = p.geometry.buffer(inflacao).bounds
        for x in range(int(math.floor(b[0])), int(math.ceil(b[2])) + 1):
            for y in range(int(math.floor(b[1])), int(math.ceil(b[3])) + 1):
                obstaculos_estaticos.add((x, y))

    start = (int(round(start_loc[0])), int(round(start_loc[1])))
    goal = (int(round(goal_loc[0])), int(round(goal_loc[1])))

    # 💡 A MAGIA: Weighted A* (Turbo Acelerador de Busca)
    PESO_HEURISTICA = 1.2

    abertos = [(0 + (heuristica(start, goal) * PESO_HEURISTICA), t_inicial, start[0], start[1])]
    veio_de = {}
    g_score = {(start[0], start[1], t_inicial): 0}

    max_t = t_inicial + int(heuristica(start, goal) * 3.0) + 150

    # Damos um pouco mais de memória limite (350 mil), mas com o PESO ele quase nunca vai precisar dela!
    LIMITE_ITERACOES = 350000
    iteracoes = 0

    melhor_t_espacial = {}

    while abertos:
        iteracoes += 1
        if iteracoes > LIMITE_ITERACOES:
            print(f"      🚧 [TEA*] Timeout! O motor de busca excedeu a memória ao desviar do trânsito.")
            return []

        _, t, x, y = heapq.heappop(abertos)

        if (x, y) == goal:
            caminho_final = []
            curr = (x, y, t)
            while curr in veio_de:
                caminho_final.append((curr[0], curr[1]))
                curr = veio_de[curr]
            caminho_final.append((start[0], start[1]))
            return caminho_final[::-1]

        if t >= max_t: continue

        vizinhos = [
            (x + 1, y, 1.0), (x - 1, y, 1.0), (x, y + 1, 1.0), (x, y - 1, 1.0),
            (x + 1, y + 1, 1.414), (x - 1, y - 1, 1.414), (x + 1, y - 1, 1.414), (x - 1, y + 1, 1.414)
        ]

        conflito_dinamico = False
        vizinhos_validos = []

        for nx, ny, custo in vizinhos:
            nt = t + 1
            if not (0 <= nx < max_x and 0 <= ny < max_y): continue
            if (nx, ny) in obstaculos_estaticos: continue

            bloqueado = False
            if (nx, ny, nt) in reserva_global:
                bloqueado = True
            elif (nx, ny, t) in reserva_global and (x, y, nt) in reserva_global:
                if reserva_global[(nx, ny, t)] == reserva_global[(x, y, nt)]:
                    bloqueado = True

            if bloqueado:
                conflito_dinamico = True
            else:
                vizinhos_validos.append((nx, ny, custo))

        if conflito_dinamico:
            vizinhos_validos.append((x, y, 1.0))

        for nx, ny, custo in vizinhos_validos:
            nt = t + 1

            if nx != x or ny != y:
                if (nx, ny) not in melhor_t_espacial:
                    melhor_t_espacial[(nx, ny)] = nt
                elif nt > melhor_t_espacial[(nx, ny)] + 10:
                    continue

            novo_g = g_score[(x, y, t)] + custo
            if (nx, ny, nt) not in g_score or novo_g < g_score[(nx, ny, nt)]:
                g_score[(nx, ny, nt)] = novo_g

                # Aplicação do multiplicador matemático
                f = novo_g + (heuristica((nx, ny), goal) * PESO_HEURISTICA)
                veio_de[(nx, ny, nt)] = (x, y, t)
                heapq.heappush(abertos, (f, nt, nx, ny))

    return []