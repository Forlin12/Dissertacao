import heapq
import math
from shapely.geometry import Point

def calcular_rota_tea(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, reserva_global, t_inicial=0):
    def heuristica(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    inflacao = drone.raio + 1.0

    # ==========================================
    # CACHE ESTÁTICO DE OBSTÁCULOS (MEMÓRIA)
    # ==========================================
    # Verifica se a memória já existe
    if not hasattr(calcular_rota_tea, "cache_obstaculos"):
        calcular_rota_tea.cache_obstaculos = {}
        calcular_rota_tea.cache_cidade_id = None

    # INVALIDAÇÃO DE CACHE: Se o mapa da cidade mudar (novo GeoDataFrame), limpa a memória
    if calcular_rota_tea.cache_cidade_id != id(lotes_gdf):
        calcular_rota_tea.cache_obstaculos = {}
        calcular_rota_tea.cache_cidade_id = id(lotes_gdf)

    # Se a altitude atual ainda não foi mapeada, processa e guarda
    if drone.altura_alvo not in calcular_rota_tea.cache_obstaculos:
        novo_set_obstaculos = set()
        predios_altos = lotes_gdf[lotes_gdf['altura_z'] >= drone.altura_alvo]
        for _, p in predios_altos.iterrows():
            geom_inflada = p.geometry.buffer(inflacao)
            b = geom_inflada.bounds
            for x in range(int(math.floor(b[0])), int(math.ceil(b[2])) + 1):
                for y in range(int(math.floor(b[1])), int(math.ceil(b[3])) + 1):
                    if geom_inflada.contains(Point(x, y)):
                        novo_set_obstaculos.add((x, y))
        calcular_rota_tea.cache_obstaculos[drone.altura_alvo] = novo_set_obstaculos

    # Carrega a cidade a partir da memória super-rápida
    obstaculos = calcular_rota_tea.cache_obstaculos[drone.altura_alvo]

    start = (int(round(start_loc[0])), int(round(start_loc[1])))
    goal = (int(round(goal_loc[0])), int(round(goal_loc[1])))

    # ------------------ SISTEMA DE DIAGNÓSTICO: PRÉ-VOO ------------------
    if start in obstaculos:
        print(f"      🛑 [FALHA] Partida Inválida! A Base {start} está dentro de um obstáculo físico.")
        return []
    if goal in obstaculos:
        print(f"      🛑 [FALHA] Destino Inválido! O Cliente {goal} está dentro de um obstáculo físico.")
        return []
    # ---------------------------------------------------------------------

    PESO_HEURISTICA = 1.2
    abertos = [(0 + (heuristica(start, goal) * PESO_HEURISTICA), t_inicial, start[0], start[1])]
    veio_de = {}
    g_score = {(start[0], start[1], t_inicial): 0}

    # Aumentada a tolerância de tempo máximo para lidar com tráfego muito pesado
    max_t = t_inicial + int(heuristica(start, goal) * 3.0) + 300
    LIMITE_ITERACOES = 40000000
    iteracoes = 0
    melhor_t_espacial = {}

    # Custo diagonal matematicamente exato para admissibilidade do A*
    custo_diag = math.sqrt(2)

    while abertos:
        iteracoes += 1

        # ------------------ SISTEMA DE DIAGNÓSTICO: TIMEOUT ------------------
        if iteracoes > LIMITE_ITERACOES:
            print(f"      ⏳ [FALHA] Timeout! Atingiu {LIMITE_ITERACOES} cálculos. Espaço congestionado.")
            return []
        # ---------------------------------------------------------------------

        _, t, x, y = heapq.heappop(abertos)

        if (x, y) == goal:
            caminho_final = []
            curr = (x, y, t)
            while curr in veio_de:
                caminho_final.append((curr[0], curr[1]))
                curr = veio_de[curr]
            caminho_final.append(start)
            return caminho_final[::-1]

        if t >= max_t: continue

        vizinhos = [
            (x + 1, y, 1.0), (x - 1, y, 1.0), (x, y + 1, 1.0), (x, y - 1, 1.0),
            (x + 1, y + 1, custo_diag), (x - 1, y - 1, custo_diag),
            (x + 1, y - 1, custo_diag), (x - 1, y + 1, custo_diag)
        ]

        conflito_dinamico = False
        vizinhos_validos = []

        for nx, ny, custo in vizinhos:
            nt = t + 1
            if not (0 <= nx < max_x and 0 <= ny < max_y): continue
            if (nx, ny) in obstaculos: continue

            bloqueado = False
            # Checa reserva estática e conflito de troca (Swap Conflict) para a mesma coordenada
            if (nx, ny, nt) in reserva_global:
                bloqueado = True
            elif (nx, ny, t) in reserva_global and (x, y, nt) in reserva_global:
                if reserva_global[(nx, ny, t)] == reserva_global[(x, y, nt)]:
                    bloqueado = True

            if bloqueado:
                conflito_dinamico = True
            else:
                vizinhos_validos.append((nx, ny, custo))

        # Se houver conflito dinâmico, força a avaliação do Hovering (Espera Tática)
        if conflito_dinamico:
            vizinhos_validos.append((x, y, 1.0))

        for nx, ny, custo in vizinhos_validos:
            nt = t + 1

            if nx != x or ny != y:
                if (nx, ny) not in melhor_t_espacial:
                    melhor_t_espacial[(nx, ny)] = nt
                # Alargamento do limite de poda para permitir contornar engarrafamentos massivos
                elif nt > melhor_t_espacial[(nx, ny)] + 50:
                    continue

            novo_g = g_score[(x, y, t)] + custo
            if (nx, ny, nt) not in g_score or novo_g < g_score[(nx, ny, nt)]:
                g_score[(nx, ny, nt)] = novo_g
                f = novo_g + (heuristica((nx, ny), goal) * PESO_HEURISTICA)
                veio_de[(nx, ny, nt)] = (x, y, t)
                heapq.heappush(abertos, (f, nt, nx, ny))

    # ------------------ SISTEMA DE DIAGNÓSTICO: BECO SEM SAÍDA ------------------
    print(f"      🧱 [FALHA] Encurralado! O drone explorou todo o mapa livre, mas está bloqueado por edifícios.")
    return []