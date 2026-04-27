# pathfinder.py
import networkx as nx
import math


def calcular_rota_8way(max_x, max_y, lotes_gdf, drone, start_loc, goal_loc):
    G = nx.Graph()

    for x in range(int(max_x) + 10):
        for y in range(int(max_y) + 10):
            G.add_node((x, y))
            if x > 0: G.add_edge((x - 1, y), (x, y), weight=1.0)
            if y > 0: G.add_edge((x, y - 1), (x, y), weight=1.0)
            if x > 0 and y > 0:
                G.add_edge((x - 1, y - 1), (x, y), weight=1.414)
                G.add_edge((x - 1, y), (x, y - 1), weight=1.414)

    inflacao = drone.raio + 2.0
    for _, p in lotes_gdf[lotes_gdf['altura_z'] >= drone.altura_alvo].iterrows():
        b = p.geometry.buffer(inflacao).bounds
        for x in range(int(b[0]), int(b[2])):
            for y in range(int(b[1]), int(b[3])):
                if (x, y) in G: G.remove_node((x, y))

    def dist_heuristica(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    nos_livres = list(G.nodes)

    if start_loc not in G: start_loc = min(nos_livres, key=lambda n: dist_heuristica(n, start_loc))
    if goal_loc not in G: goal_loc = min(nos_livres, key=lambda n: dist_heuristica(n, goal_loc))

    try:
        caminho = nx.astar_path(G, start_loc, goal_loc, heuristic=dist_heuristica, weight='weight')
        return caminho, start_loc, goal_loc
    except nx.NetworkXNoPath:
        return [], start_loc, goal_loc