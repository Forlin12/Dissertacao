import heapq
import math
from shapely.geometry import Point

_CUSTO_DIAG = math.sqrt(2)
_VIZINHOS_DELTA = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _CUSTO_DIAG), (-1, -1, _CUSTO_DIAG),
    (1, -1, _CUSTO_DIAG), (-1, 1, _CUSTO_DIAG)
)


def calcular_rota_tea(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc,
                      reserva_global, t_inicial=0, limite_iteracoes=400000):
    """
    TEA_STAR Dinâmico e Corrigido (Busca Espacial + Hashing O(1) com Sets)
    """

    inflacao = drone.raio + 1.0

    # CACHE ESTÁTICO DE OBSTÁCULOS (Mantendo Tuplas simples)
    if not hasattr(calcular_rota_tea, "cache_obstaculos"):
        calcular_rota_tea.cache_obstaculos = {}
        calcular_rota_tea.cache_cidade_id = None

    if calcular_rota_tea.cache_cidade_id != id(lotes_gdf):
        calcular_rota_tea.cache_obstaculos = {}
        calcular_rota_tea.cache_cidade_id = id(lotes_gdf)

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

    obstaculos = calcular_rota_tea.cache_obstaculos[drone.altura_alvo]

    start = (int(round(start_loc[0])), int(round(start_loc[1])))
    goal = (int(round(goal_loc[0])), int(round(goal_loc[1])))

    if start in obstaculos:
        print(f"      🛑 [FALHA] Partida Inválida! A Base {start} está dentro de um obstáculo físico.")
        return []
    if goal in obstaculos:
        print(f"      🛑 [FALHA] Destino Inválido! O Cliente {goal} está dentro de um obstáculo físico.")
        return []

    # ==========================================
    # CORREÇÃO: ISOLANDO A ALTURA E CRIANDO SETS
    # ==========================================
    # Ao invés de checar o dicionário pesado (x,y,z,t) a cada passo, extraímos
    # apenas o (x,y,t) que importa para a altura deste drone.
    # Sets do Python são escritos em C e são imbatíveis em velocidade.
    ocupacao_set = set()
    agentes_dict = {}

    for chaves, id_agente in reserva_global.items():
        # Lida tanto com chaves (x, y, z, t) do main.py quanto eventuais (x, y, t) antigas
        if len(chaves) == 4:
            rx, ry, rz, rt = chaves
        else:
            rx, ry, rz, rt = chaves[0], chaves[1], drone.altura_alvo, chaves[2]

        if rz == drone.altura_alvo:
            chave_3d = (rx, ry, rt)
            ocupacao_set.add(chave_3d)
            agentes_dict[chave_3d] = id_agente

    PESO_HEURISTICA = 1.2

    def h(x, y):
        return math.hypot(goal[0] - x, goal[1] - y) * PESO_HEURISTICA

    gx, gy = goal
    sx, sy = start

    abertos = [(h(sx, sy), t_inicial, sx, sy)]
    veio_de = {}
    g_score = {(sx, sy, t_inicial): 0}

    max_t = t_inicial + int(math.hypot(gx - sx, gy - sy) * 3.0) + 300
    iteracoes = 0
    melhor_t_espacial = {}

    while abertos:
        iteracoes += 1

        if iteracoes > limite_iteracoes:
            print(f"      ⏳ [FALHA] Timeout! Atingiu {limite_iteracoes} cálculos.")
            return []

        f_cur, t, x, y = heapq.heappop(abertos)

        if x == gx and y == gy:
            caminho_final = []
            curr = (x, y, t)
            while curr in veio_de:
                caminho_final.append((curr[0], curr[1]))
                curr = veio_de[curr]
            caminho_final.append(start)
            return caminho_final[::-1]

        if t >= max_t:
            continue

        g_cur = g_score.get((x, y, t), math.inf)

        conflito_dinamico = False
        vizinhos_validos = []
        nt = t + 1

        for dx, dy, custo in _VIZINHOS_DELTA:
            nx, ny = x + dx, y + dy

            if not (0 <= nx < max_x and 0 <= ny < max_y):
                continue
            if (nx, ny) in obstaculos:
                continue

            # LOOKUP O(1) usando Tuplas (Resolve a colisão e permite o desvio)
            chave_alvo = (nx, ny, nt)
            if chave_alvo in ocupacao_set:
                conflito_dinamico = True
                continue

            # Conflito de Troca (Swap Conflict)
            chave_origem_alvo = (nx, ny, t)
            chave_alvo_origem = (x, y, nt)

            if chave_origem_alvo in agentes_dict and chave_alvo_origem in agentes_dict:
                if agentes_dict[chave_origem_alvo] == agentes_dict[chave_alvo_origem]:
                    conflito_dinamico = True
                    continue

            vizinhos_validos.append((nx, ny, custo))

        # Hovering táctico para ceder passagem
        if conflito_dinamico:
            vizinhos_validos.append((x, y, 1.0))

        for nx, ny, custo in vizinhos_validos:
            if nx != x or ny != y:
                melhor = melhor_t_espacial.get((nx, ny))
                if melhor is None:
                    melhor_t_espacial[(nx, ny)] = nt
                elif nt > melhor + 50:
                    continue

            novo_g = g_cur + custo
            chave = (nx, ny, nt)

            if chave not in g_score or novo_g < g_score[chave]:
                g_score[chave] = novo_g
                f = novo_g + h(nx, ny)
                veio_de[chave] = (x, y, t)
                heapq.heappush(abertos, (f, nt, nx, ny))

    print(f"      🧱 [FALHA] Encurralado! O drone explorou todo o mapa livre.")
    return []