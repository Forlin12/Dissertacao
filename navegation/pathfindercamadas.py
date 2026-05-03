# pathfindercamadas.py
import networkx as nx
import math
from shapely.geometry import Point


def calcular_rota_8way_camadas(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc, vetor_camadas):
    # 1. Cria a malha base (Ruas vazias) APENAS UMA VEZ para poupar CPU
    G_base = nx.Graph()
    lim_x = int(math.ceil(max_x)) + 20
    lim_y = int(math.ceil(max_y)) + 20

    for x in range(-10, lim_x):
        for y in range(-10, lim_y):
            G_base.add_node((x, y))
            if x > -10: G_base.add_edge((x - 1, y), (x, y), weight=1.0)
            if y > -10: G_base.add_edge((x, y - 1), (x, y), weight=1.0)
            if x > -10 and y > -10:
                G_base.add_edge((x - 1, y - 1), (x, y), weight=1.414)
                G_base.add_edge((x - 1, y), (x, y - 1), weight=1.414)

    start_int = (int(round(start_loc[0])), int(round(start_loc[1])))
    goal_int = (int(round(goal_loc[0])), int(round(goal_loc[1])))

    # 2. Testa cada camada do vetor
    for altura_teste in vetor_camadas:
        # Copia o mapa limpo para começar a colocar os obstáculos desta altura
        G = G_base.copy()
        inflacao = drone.raio + 0.5

        # Filtra SÓ os prédios que são maiores que a altura que estamos a testar
        obstaculos_nesta_camada = lotes_gdf[lotes_gdf['altura_z'] >= altura_teste]

        for _, p in obstaculos_nesta_camada.iterrows():
            geom_inflada = p.geometry.buffer(inflacao)
            b = geom_inflada.bounds

            min_x, min_y = int(math.floor(b[0])), int(math.floor(b[1]))
            max_x_b, max_y_b = int(math.ceil(b[2])), int(math.ceil(b[3]))

            for x in range(min_x, max_x_b + 1):
                for y in range(min_y, max_y_b + 1):
                    if (x, y) in G and geom_inflada.contains(Point(x, y)):
                        G.remove_node((x, y))

        def dist_heuristica(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        nos_livres = list(G.nodes)

        if not nos_livres: continue  # Se o mapa inteiro for um mega-prédio, desiste logo

        s_atual = start_int
        g_atual = goal_int

        if s_atual not in G: s_atual = min(nos_livres, key=lambda n: dist_heuristica(n, s_atual))
        if g_atual not in G: g_atual = min(nos_livres, key=lambda n: dist_heuristica(n, g_atual))

        try:
            caminho = nx.astar_path(G, s_atual, g_atual, heuristic=dist_heuristica, weight='weight')

            # MAGIA: Atualiza o drone para ele voar nesta nova altitude!
            drone.altura_alvo = altura_teste
            print(f"      ✅ [A*] Rota encontrada na camada de {altura_teste}m!")

            return caminho, s_atual, g_atual
        except nx.NetworkXNoPath:
            print(f"      🚧 [A*] Rota bloqueada a {altura_teste}m. Tentando subir...")
            continue  # O loop roda e ele tenta a próxima altura do vetor!

    # Se o loop terminar e não encontrou nada
    print(f"      ❌ [A*] Rota impossível em todas as {len(vetor_camadas)} camadas fornecidas.")
    return [], start_int, goal_int