# renderer.py
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import numpy as np
import math
import config as cfg


def plotar_frota_3d(max_x, max_y, lotes_gdf, centros_distribuicao, missoes_realizadas):
    fig = go.Figure()

    fig.add_trace(go.Mesh3d(x=[-20, max_x + 20, max_x + 20, -20], y=[-20, -20, max_y + 20, max_y + 20],
                            z=[-0.1, -0.1, -0.1, -0.1],
                            i=[0, 0], j=[1, 2], k=[2, 3], color='#1b2631', opacity=1, name='Chão', hoverinfo='skip'))

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

    for i, cd in enumerate(centros_distribuicao):
        fig.add_trace(go.Scatter3d(x=[cd[0]], y=[cd[1]], z=[0.5], mode='markers',
                                   marker=dict(size=12, color='#3498DB', symbol='square'), name=f'Base CD {i + 1}'))

    total_kwh = 0.0;
    total_dist = 0.0
    cores_drones = ['#00ffcc', '#ff00ff', '#ffff00', '#ff9900', '#00ccff']

    for missao in missoes_realizadas:
        rota = missao['rota']
        bateu = missao['bateu']

        total_kwh += missao['uav'].energia_consumida_kwh
        total_dist += missao['uav'].distancia_voada
        cor_linha = cores_drones[missao['id_drone'] % len(cores_drones)]

        fig.add_trace(go.Scatter3d(x=rota[:, 0], y=rota[:, 1], z=rota[:, 2], mode='lines',
                                   line=dict(color=cor_linha, width=5),
                                   name=f"Missão {missao['id_entrega']} (UAV {missao['id_drone']})"))

        if not bateu:
            fig.add_trace(go.Scatter3d(x=[missao['goal'][0]], y=[missao['goal'][1]], z=[0.1],
                                       mode='markers', marker=dict(size=6, color='#F1C40F'),
                                       name=f"Destino {missao['id_entrega']}"))
        else:
            fig.add_trace(go.Scatter3d(x=[missao['queda'][0]], y=[missao['queda'][1]], z=[missao['queda'][2]],
                                       mode='markers', marker=dict(size=12, color='red', symbol='x'),
                                       name=f"Crash M{missao['id_entrega']}"))

    fig.update_layout(template="plotly_dark",
                      title=f"Painel Operacional | Frota: {cfg.NUM_DRONES_DISPONIVEIS} Drones | Missões: {len(missoes_realizadas)} | Dist. Total: {total_dist:.1f}m | Bat. Total: {total_kwh:.4f} kWh",
                      scene=dict(aspectmode='data', zaxis=dict(range=[0, 60])), margin=dict(l=0, r=0, b=0, t=40))
    fig.show()


def plotar_frota_2d(lotes_gdf, centros_distribuicao, missoes_realizadas):
    plt.figure(figsize=(10, 10), facecolor='#1a1a1a')
    ax = plt.gca()
    ax.set_facecolor('#1a1a1a')

    lotes_gdf.plot(column='lote_id', cmap='viridis', ax=ax, edgecolor='white', linewidth=0.3, alpha=0.5)

    for i, cd in enumerate(centros_distribuicao):
        ax.add_patch(plt.Circle(cd, cfg.ZONA_LIVRE_CD, color='#3498DB', fill=False, linewidth=1.5, linestyle='--'))
        ax.scatter(cd[0], cd[1], color='#3498DB', s=150, marker='s', zorder=5)

    cores_drones = ['#00ffcc', '#ff00ff', '#ffff00', '#ff9900', '#00ccff']

    for missao in missoes_realizadas:
        rota = missao['rota']
        bateu = missao['bateu']
        goal = missao['goal']
        cor_linha = cores_drones[missao['id_drone'] % len(cores_drones)]

        ax.plot(rota[:, 0], rota[:, 1], color=cor_linha, linewidth=2, alpha=0.9)
        ax.add_patch(plt.Circle(goal, cfg.ZONA_LIVRE_ENTREGA, color='#F1C40F', fill=False, linewidth=1, linestyle=':'))

        if not bateu:
            ax.scatter(goal[0], goal[1], color='#F1C40F', s=80, marker='o', zorder=6)
        else:
            ax.scatter(missao['queda'][0], missao['queda'][1], color='red', s=150, marker='X', zorder=6)

    plt.axis('off')
    plt.title(f"Planta 2D | Mapa Global de Frota", color='white', fontsize=14)
    plt.tight_layout()
    plt.show()


def plotar_snapshots_entregas_2d(lotes_gdf, missoes_realizadas):
    n_missoes = len(missoes_realizadas)
    if n_missoes == 0: return

    cols = 3
    rows = math.ceil(n_missoes / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows), facecolor='#1a1a1a')

    if n_missoes == 1:
        axes = [axes]
    else:
        axes = axes.flatten()

    cores_drones = ['#00ffcc', '#ff00ff', '#ffff00', '#ff9900', '#00ccff']

    for i, missao in enumerate(missoes_realizadas):
        ax = axes[i]
        ax.set_facecolor('#1a1a1a')

        gx, gy = missao['goal']
        zoom = 35.0

        ax.set_xlim(gx - zoom, gx + zoom)
        ax.set_ylim(gy - zoom, gy + zoom)

        lotes_gdf.plot(column='lote_id', cmap='viridis', ax=ax, edgecolor='white', linewidth=1.0, alpha=0.6)

        rota = missao['rota']
        cor_linha = cores_drones[missao['id_drone'] % len(cores_drones)]
        ax.plot(rota[:, 0], rota[:, 1], color=cor_linha, linewidth=3, alpha=0.9)

        ax.add_patch(
            plt.Circle((gx, gy), cfg.ZONA_LIVRE_ENTREGA, color='#F1C40F', fill=False, linewidth=2, linestyle='--'))

        if not missao['bateu']:
            ax.scatter(gx, gy, color='#F1C40F', s=200, marker='*', zorder=5)
            # Emojis removidos para evitar erro de Glyph no Matplotlib
            titulo = f"[OK] Entrega {missao['id_entrega']} (Drone {missao['id_drone']})"
        else:
            qx, qy = missao['queda'][0], missao['queda'][1]
            ax.scatter(qx, qy, color='red', s=200, marker='X', zorder=5)
            titulo = f"[FALHA] Queda {missao['id_entrega']} (Drone {missao['id_drone']})"

        ax.set_title(titulo, color='white', fontsize=12, pad=10)
        ax.axis('off')

    for j in range(len(missoes_realizadas), len(axes)):
        axes[j].set_visible(False)

    plt.suptitle("Recortes Fotográficos de Operação (Zoom de Descida)", color='white', fontsize=16)
    plt.tight_layout()
    plt.show()