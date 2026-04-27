import osmnx as ox
import geopandas as gpd
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, MultiPoint, Point
from shapely.ops import voronoi_diagram
import plotly.graph_objects as go
import networkx as nx
import math


# =================================================================
# --- 1. CLASSE DRONE (CINEMÁTICA E ENERGIA) ---
# =================================================================
class Drone:
    def __init__(self, raio_m, altura_voo, velocidade_ms, carga_kg):
        self.raio = raio_m
        self.altura_alvo = altura_voo
        self.velocidade_horiz = velocidade_ms
        self.velocidade_vert = 3.0
        self.carga = carga_kg

        self.distancia_voada = 0.0
        self.tempo_voo_s = 0.0
        self.energia_consumida_kwh = 0.0

    def calcular_potencia_kw(self, estado='cruzeiro'):
        p_estatica = 0.5 + (0.15 * self.carga)
        if estado == 'subida':
            return p_estatica * 1.5 + 0.1
        elif estado == 'cruzeiro':
            return p_estatica + (0.005 * self.velocidade_horiz ** 2)
        elif estado == 'descida':
            return p_estatica * 0.7
        return p_estatica

    def checar_impacto(self, pos_xyz, obstaculos_gdf):
        hitbox = Point(pos_xyz[0], pos_xyz[1]).buffer(self.raio)
        for _, predio in obstaculos_gdf.iterrows():
            if pos_xyz[2] <= predio['altura_z']:
                if hitbox.intersects(predio.geometry):
                    return True
        return False

    def simular_missao(self, caminho_a_star, lotes_gdf, dt=0.1):
        print(f"\n🚀 Missão Iniciada | Carga: {self.carga}kg | Altitude: {self.altura_alvo}m")
        pos = np.array([caminho_a_star[0][0], caminho_a_star[0][1], 0.1], dtype=float)
        rota_xyz = [pos.copy()]
        colisao = False;
        ponto_queda = None

        # FASE 1: DESCOLAGEM VERTICAL
        while pos[2] < self.altura_alvo:
            pos[2] += self.velocidade_vert * dt
            self.tempo_voo_s += dt
            self.energia_consumida_kwh += self.calcular_potencia_kw('subida') * (dt / 3600)
            rota_xyz.append(pos.copy())
            if self.checar_impacto(pos, lotes_gdf): colisao = True; ponto_queda = pos.copy(); break

        # FASE 2: VOO DE CRUZEIRO (Com interpolação para movimentos diagonais)
        if not colisao:
            for i in range(1, len(caminho_a_star)):
                alvo = np.array([caminho_a_star[i][0], caminho_a_star[i][1], self.altura_alvo], dtype=float)
                while np.linalg.norm(alvo[:2] - pos[:2]) > (self.velocidade_horiz * dt):
                    dir_vec = (alvo - pos)[:2]
                    pos[:2] += (dir_vec / np.linalg.norm(dir_vec)) * self.velocidade_horiz * dt

                    self.tempo_voo_s += dt
                    self.distancia_voada += self.velocidade_horiz * dt
                    self.energia_consumida_kwh += self.calcular_potencia_kw('cruzeiro') * (dt / 3600)
                    rota_xyz.append(pos.copy())

                    if self.checar_impacto(pos, lotes_gdf): colisao = True; ponto_queda = pos.copy(); break
                if colisao: break
                pos[:2] = alvo[:2]

                # FASE 3: POUSO VERTICAL
        if not colisao:
            while pos[2] > 0.15:
                pos[2] -= self.velocidade_vert * dt
                self.tempo_voo_s += dt
                self.energia_consumida_kwh += self.calcular_potencia_kw('descida') * (dt / 3600)
                rota_xyz.append(pos.copy())

        return np.array(rota_xyz), colisao, ponto_queda


# =================================================================
# --- 2. CONFIGURAÇÃO DO AMBIENTE ---
# =================================================================
CENARIO_SEMPRE_NOVO = True
RAIO_M = 200
LARGURA_RUA = 10
AREA_MEDIA_LOTE = 350
ESPACO_ENTRE_LOTES = 2.0

DENSIDADE_PREDIOS = 0.8
ALTURA_MIN = 5
ALTURA_MAX = 45

START_LOC = (25, 25)
GOAL_LOC = (350, 350)
ZONA_LIVRE_INICIO = 20.0
ZONA_LIVRE_POUSO = 20.0

# =================================================================
# --- 3. GERAÇÃO DO MUNDO ---
# =================================================================
print("1. A construir o gémeo digital da cidade...")
if not CENARIO_SEMPRE_NOVO: np.random.seed(42)

graph = ox.graph_from_point((41.8058, -6.7572), dist=RAIO_M, network_type='drive')
ruas = ox.graph_to_gdfs(graph, nodes=False, edges=True).to_crs(epsg=3857)

bbox = ruas.total_bounds
ambiente = Polygon([(bbox[0], bbox[1]), (bbox[2], bbox[1]), (bbox[2], bbox[3]), (bbox[0], bbox[3])])
asfalto = ruas.geometry.buffer(LARGURA_RUA / 2).union_all()
quarteiroes = gpd.GeoDataFrame(geometry=[ambiente.difference(asfalto)]).explode(index_parts=False)

lotes_list = []
for q in quarteiroes.geometry:
    num = int(q.area / AREA_MEDIA_LOTE)
    pts = []
    minx, miny, maxx, maxy = q.bounds
    while len(pts) < num:
        p = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
        if q.contains(p): pts.append(p)
    if len(pts) >= 2:
        for poly in voronoi_diagram(MultiPoint(pts), envelope=q.buffer(10)).geoms:
            res = poly.intersection(q).buffer(-ESPACO_ENTRE_LOTES)
            if not res.is_empty and res.area > 30: lotes_list.append(res)

lotes_gdf = gpd.GeoDataFrame(geometry=lotes_list, crs=ruas.crs)
minx, miny, _, _ = lotes_gdf.total_bounds
lotes_gdf.geometry = lotes_gdf.translate(xoff=-minx, yoff=-miny)
MAX_X, MAX_Y = int(lotes_gdf.total_bounds[2]), int(lotes_gdf.total_bounds[3])

lotes_gdf['lote_id'] = range(len(lotes_gdf))

ponto_inicio = Point(START_LOC)
ponto_pouso = Point(GOAL_LOC)

lotes_gdf['altura_z'] = 0
for idx, lote in lotes_gdf.iterrows():
    if np.random.random() <= DENSIDADE_PREDIOS:
        if lote.geometry.distance(ponto_inicio) < ZONA_LIVRE_INICIO or lote.geometry.distance(
                ponto_pouso) < ZONA_LIVRE_POUSO:
            lotes_gdf.at[idx, 'altura_z'] = 0
        else:
            lotes_gdf.at[idx, 'altura_z'] = np.random.randint(ALTURA_MIN, ALTURA_MAX)

# =================================================================
# --- 4. PLANEAMENTO (AGORA COM MOVIMENTO DIAGONAL 8-WAY) ---
# =================================================================
uav = Drone(raio_m=2.0, altura_voo=25, velocidade_ms=12, carga_kg=3.0)

print("2. A calcular rota de navegação (Pathfinding 8-Way)...")
G = nx.Graph()

# Monta uma grelha onde o drone pode andar em qualquer direção (Horizontal, Vertical e Diagonal)
for x in range(MAX_X + 10):
    for y in range(MAX_Y + 10):
        G.add_node((x, y))
        if x > 0: G.add_edge((x - 1, y), (x, y), weight=1.0)
        if y > 0: G.add_edge((x, y - 1), (x, y), weight=1.0)
        if x > 0 and y > 0:
            G.add_edge((x - 1, y - 1), (x, y), weight=1.414)  # Diagonal custa √2
            G.add_edge((x - 1, y), (x, y - 1), weight=1.414)

# Aumentámos a margem de segurança visual para as esquinas
inflacao = uav.raio + 2.5

for _, p in lotes_gdf[lotes_gdf['altura_z'] >= uav.altura_alvo].iterrows():
    b = p.geometry.buffer(inflacao).bounds
    for x in range(int(b[0]), int(b[2])):
        for y in range(int(b[1]), int(b[3])):
            if (x, y) in G: G.remove_node((x, y))


def dist_heuristica(a, b): return math.hypot(b[0] - a[0], b[1] - a[1])


nos_livres = list(G.nodes)
if START_LOC not in G: START_LOC = min(nos_livres, key=lambda n: dist_heuristica(n, START_LOC))
if GOAL_LOC not in G: GOAL_LOC = min(nos_livres, key=lambda n: dist_heuristica(n, GOAL_LOC))

try:
    caminho = nx.astar_path(G, START_LOC, GOAL_LOC, heuristic=dist_heuristica, weight='weight')
    rota_final, bateu, local_queda = uav.simular_missao(caminho, lotes_gdf)
except nx.NetworkXNoPath:
    print("❌ Erro: Rota bloqueada no nível de planeamento.");
    caminho = []

# =================================================================
# --- 5. VISUALIZAÇÃO 3D ---
# =================================================================
print("3. A gerar modelo 3D...")
fig = go.Figure()

fig.add_trace(
    go.Mesh3d(x=[-20, MAX_X + 20, MAX_X + 20, -20], y=[-20, -20, MAX_Y + 20, MAX_Y + 20], z=[-0.1, -0.1, -0.1, -0.1],
              i=[0, 0], j=[1, 2], k=[2, 3], color='#1b2631', opacity=1, name='Chão'))

all_x, all_y, all_z, all_i, all_j, all_k = [], [], [], [], [], []
v_off = 0
for _, l in lotes_gdf[lotes_gdf['altura_z'] > 0].iterrows():
    geometrias = [l.geometry] if l.geometry.geom_type == 'Polygon' else l.geometry.geoms
    for geom in geometrias:
        c = list(geom.exterior.coords)[:-1];
        n = len(c);
        h = l['altura_z']
        if n < 3: continue
        all_x.extend([p[0] for p in c] * 2);
        all_y.extend([p[1] for p in c] * 2);
        all_z.extend([0] * n + [h] * n)
        for s in range(1, n - 1):
            all_i.extend([v_off, v_off + n]);
            all_j.extend([v_off + s, v_off + n + s]);
            all_k.extend([v_off + s + 1, v_off + n + s + 1])
        for v in range(n):
            nxt = (v + 1) % n
            all_i.extend([v_off + v, v_off + nxt]);
            all_j.extend([v_off + nxt, v_off + nxt + n]);
            all_k.extend([v_off + v + n, v_off + v + n])
        v_off += 2 * n

fig.add_trace(go.Mesh3d(x=all_x, y=all_y, z=all_z, i=all_i, j=all_j, k=all_k, color='#5d6d7e', flatshading=True,
                        name='Edifícios'))


def desenhar_cilindro(x, y, raio, cor, nome):
    theta = np.linspace(0, 2 * np.pi, 20)
    cx = x + raio * np.cos(theta);
    cy = y + raio * np.sin(theta)
    fig.add_trace(go.Scatter3d(x=list(cx) + [cx[0]], y=list(cy) + [cy[0]], z=[0.5] * 21, mode='lines',
                               line=dict(color=cor, width=4), name=nome))


desenhar_cilindro(START_LOC[0], START_LOC[1], ZONA_LIVRE_INICIO, '#3498DB', 'Clareira Início')
desenhar_cilindro(GOAL_LOC[0], GOAL_LOC[1], ZONA_LIVRE_POUSO, '#2ECC71', 'Clareira Pouso')

if len(caminho) > 0:
    fig.add_trace(go.Scatter3d(x=rota_final[:, 0], y=rota_final[:, 1], z=rota_final[:, 2], mode='lines',
                               line=dict(color='#00ffcc', width=4), name='Trajetória'))

    fig.add_trace(
        go.Scatter3d(x=[START_LOC[0]], y=[START_LOC[1]], z=[0.1], mode='markers', marker=dict(size=8, color='#3498DB'),
                     name='Base (H)'))

    if bateu:
        fig.add_trace(go.Scatter3d(x=[local_queda[0]], y=[local_queda[1]], z=[local_queda[2]], mode='markers',
                                   marker=dict(size=15, color='red', symbol='x'), name='Colisão'))
    else:
        fig.add_trace(go.Scatter3d(x=[GOAL_LOC[0]], y=[GOAL_LOC[1]], z=[0.1], mode='markers',
                                   marker=dict(size=8, color='#2ECC71'), name='Pouso (H)'))

status = '💥 CRASH' if (len(caminho) > 0 and bateu) else '✅ SUCESSO' if len(caminho) > 0 else '❌ BLOQUEADO'
fig.update_layout(template="plotly_dark",
                  title=f"Distância: {uav.distancia_voada:.1f}m | Bateria: {uav.energia_consumida_kwh:.4f} kWh | Status: {status}",
                  scene=dict(aspectmode='data', zaxis=dict(range=[0, 60])), margin=dict(l=0, r=0, b=0, t=40))
fig.show()

# =================================================================
# --- 6. VISUALIZAÇÃO 2D ---
# =================================================================
print("4. A abrir planta 2D...")
plt.figure(figsize=(10, 10), facecolor='#1a1a1a')
ax = plt.gca();
ax.set_facecolor('#1a1a1a')
lotes_gdf.plot(column='lote_id', cmap='viridis', ax=ax, edgecolor='white', linewidth=0.3)
c1 = plt.Circle(START_LOC, ZONA_LIVRE_INICIO, color='#3498DB', fill=False, linewidth=2, linestyle='--')
c2 = plt.Circle(GOAL_LOC, ZONA_LIVRE_POUSO, color='#2ECC71', fill=False, linewidth=2, linestyle='--')
ax.add_patch(c1);
ax.add_patch(c2)
ax.scatter([START_LOC[0], GOAL_LOC[0]], [START_LOC[1], GOAL_LOC[1]], color=['#3498DB', '#2ECC71'], s=100, zorder=5)
plt.axis('off');
plt.title(f"Planta 2D | Clareiras de Segurança ({ZONA_LIVRE_INICIO}m)", color='white')
plt.show()