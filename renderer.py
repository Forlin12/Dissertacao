# renderer.py
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
import config as cfg


def plotar_frota_3d(max_x, max_y, lotes_gdf, centros_distribuicao, missoes_realizadas):
    fig = go.Figure()

    # Chão
    fig.add_trace(go.Mesh3d(x=[-20, max_x + 20, max_x + 20, -20], y=[-20, -20, max_y + 20, max_y + 20],
                            z=[-0.1, -0.1, -0.1, -0.1],
                            i=[0, 0], j=[1, 2], k=[2, 3], color='#1b2631', opacity=1, name='Chão', hoverinfo='skip'))

    # Edifícios
    all_x, all_y, all_z, all_i, all_j, all_k = [], [], [], [], [], []
    v_off = 0
    for _, l in lotes_gdf[lotes_gdf['altura_z'] > 0].iterrows():
        geometrias = [l.geometry] if l.geometry.geom_type == 'Polygon' else l.geometry.geoms
        for geom in geometrias:
            c = list(geom.exterior.coords)[:-1]
            n = len(c)
            h = l['altura_z']
            if n < 3: continue
            all_x.extend([p[0] for p in c] * 2)
            all_y.extend([p[1] for p in c] * 2)
            all_z.extend([0] * n + [h] * n)
            for s in range(1, n - 1):
                all_i.extend([v_off, v_off + n])
                all_j.extend([v_off + s, v_off + n + s])
                all_k.extend([v_off + s + 1, v_off + n + s + 1])
            for v in range(n):
                nxt = (v + 1) % n
                all_i.extend([v_off + v, v_off + nxt])
                all_j.extend([v_off + nxt, v_off + nxt + n])
                all_k.extend([v_off + v + n, v_off + v + n])
            v_off += 2 * n

    fig.add_trace(go.Mesh3d(x=all_x, y=all_y, z=all_z, i=all_i, j=all_j, k=all_k, color='#5d6d7e', flatshading=True,
                            name='Edifícios'))

    # Bases
    for i, cd in enumerate(centros_distribuicao):
        fig.add_trace(go.Scatter3d(x=[cd[0]], y=[cd[1]], z=[0.5], mode='markers',
                                   marker=dict(size=12, color='#3498DB', symbol='square'), name=f'Base CD {i + 1}'))

    total_kwh = 0.0
    total_dist = 0.0
    cores_drones = ['#00ffcc', '#ff00ff', '#ffff00', '#ff9900', '#00ccff']

    for missao in missoes_realizadas:
        rota = missao['rota']
        bateu = missao['bateu']

        total_kwh += missao['uav'].energia_consumida_kwh
        total_dist += missao['uav'].distancia_voada

        if missao.get('falha_planeamento', False): continue

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
                      title=f"Painel Operacional | Missões: {len(missoes_realizadas)} | Dist. Total: {total_dist:.1f}m | Bat. Total: {total_kwh:.4f} kWh",
                      scene=dict(aspectmode='data', zaxis=dict(range=[0, max(lotes_gdf['altura_z']) + 20])),
                      margin=dict(l=0, r=0, b=0, t=40))
    fig.show()


def plotar_frota_2d(lotes_gdf, centros_distribuicao, missoes_realizadas):
    fig, ax = plt.subplots(figsize=(10, 10), facecolor='#1a1a1a', dpi=300)
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

        ax.add_patch(plt.Circle(goal, cfg.ZONA_LIVRE_ENTREGA, color='#F1C40F', fill=False, linewidth=1, linestyle=':'))

        if missao.get('falha_planeamento', False):
            ax.scatter(missao['start'][0], missao['start'][1], color='#FF6600', s=150, marker='X', zorder=6)
            continue

        ax.plot(rota[:, 0], rota[:, 1], color=cor_linha, linewidth=2, alpha=0.9)

        if not bateu:
            ax.scatter(goal[0], goal[1], color='#F1C40F', s=80, marker='o', zorder=6)
        else:
            ax.scatter(missao['queda'][0], missao['queda'][1], color='red', s=150, marker='X', zorder=6)

    plt.axis('off')
    plt.title(f"Planta 2D | Mapa Global de Frota Clássico", color='white', fontsize=14)
    plt.tight_layout()
    plt.show()


def plotar_falhas_e_colisoes_2d(lotes_gdf, centros_distribuicao, missoes_realizadas):
    missoes_com_problema = [m for m in missoes_realizadas if m.get('falha_planeamento', False) or m.get('bateu', False)]

    if not missoes_com_problema:
        print("   ✅ Excelente! Nenhuma falha de planeamento ou colisão detetada para desenhar.")
        return

    fig, ax = plt.subplots(figsize=(10, 10), facecolor='#1a1a1a', dpi=300)
    ax.set_facecolor('#1a1a1a')

    lotes_gdf.plot(column='lote_id', cmap='viridis', ax=ax, edgecolor='white', linewidth=0.3, alpha=0.3)

    for cd in centros_distribuicao:
        ax.add_patch(plt.Circle(cd, cfg.ZONA_LIVRE_CD, color='#3498DB', fill=False, linewidth=1.5, linestyle='--'))
        ax.scatter(cd[0], cd[1], color='#3498DB', s=100, marker='s', zorder=5)

    for missao in missoes_com_problema:
        start = missao['start']
        goal = missao['goal']

        if missao.get('falha_planeamento', False):
            ax.plot([start[0], goal[0]], [start[1], goal[1]], color='#FF9900', linewidth=1.5, linestyle=':', alpha=0.8)
            ax.scatter(start[0], start[1], color='#FF9900', s=250, marker='X', zorder=6)
            ax.scatter(goal[0], goal[1], color='#FF9900', s=80, marker='o', facecolors='none', zorder=6)
            ax.text(start[0] + 3, start[1] + 3, f"M{missao['id_entrega']} Sem Rota", color='#FF9900', fontsize=9,
                    fontweight='bold')

        elif missao.get('bateu', False):
            rota = missao['rota']
            queda = missao['queda']
            ax.plot(rota[:, 0], rota[:, 1], color='#FF3333', linewidth=3, alpha=0.9)
            ax.scatter(queda[0], queda[1], color='#FF3333', s=300, marker='X', zorder=7)
            ax.scatter(goal[0], goal[1], color='#F1C40F', s=80, marker='o', facecolors='none', zorder=6)
            ax.text(queda[0] + 3, queda[1] + 3, f"M{missao['id_entrega']} Crash", color='#FF3333', fontsize=9,
                    fontweight='bold')

    plt.axis('off')
    plt.title(f"Relatório de Diagnóstico | Falhas de Planeamento e Colisões", color='#FF3333', fontsize=15,
              fontweight='bold')
    plt.tight_layout()
    plt.show()


def plotar_diagrama_espaco_tempo(missoes_realizadas):
    if len(missoes_realizadas) == 0: return

    fig, ax = plt.subplots(figsize=(12, 6), facecolor='#1a1a1a', dpi=300)
    ax.set_facecolor('#1a1a1a')

    cores_drones = ['#00ffcc', '#ff00ff', '#ffff00', '#ff9900', '#00ccff']

    for missao in missoes_realizadas:
        if missao.get('falha_planeamento', False): continue

        tempo_bruto = missao['tempo_global']
        rota = missao['rota']
        base_origem = missao['start']
        cor = cores_drones[(missao['id_drone'] - 1) % len(cores_drones)]

        tempo_corrigido = [tempo_bruto[0]]
        for t in tempo_bruto[1:]:
            if t <= tempo_corrigido[-1]:
                tempo_corrigido.append(tempo_corrigido[-1] + 1)
            else:
                tempo_corrigido.append(t)

        # NOVO CÁLCULO: Distância Euclidiana exata da Base Logística
        distancias_da_base = []
        for p in rota:
            dx = p[0] - base_origem[0]
            dy = p[1] - base_origem[1]
            dz = p[2] - base_origem[2] if len(p) > 2 and len(base_origem) > 2 else 0
            d = np.linalg.norm([dx, dy, dz])
            distancias_da_base.append(d)

        tamanho_plot = min(len(tempo_corrigido), len(distancias_da_base))

        ax.plot(tempo_corrigido[:tamanho_plot], distancias_da_base[:tamanho_plot],
                color=cor, linewidth=2.5, alpha=0.8,
                label=f"UAV {missao['id_drone']} (Missão {missao['id_entrega']})")

    ax.set_xlabel("Tempo Global da Simulação (Segundos)", color='white', fontsize=12, fontweight='bold')
    ax.set_ylabel("Distância da Base Logística (Metros)", color='white', fontsize=12, fontweight='bold')
    ax.set_title("Diagrama Espaço-Tempo UTM | Progressão Relativa à Base", color='white', fontsize=16, pad=15)

    ax.tick_params(axis='x', colors='white')
    ax.tick_params(axis='y', colors='white')
    ax.spines['bottom'].set_color('white')
    ax.spines['left'].set_color('white')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    ax.grid(color='#444444', linestyle='--', linewidth=0.5, alpha=0.7)

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(),
              facecolor='#1a1a1a', edgecolor='white', labelcolor='white',
              loc='upper left', bbox_to_anchor=(1, 1))

    plt.tight_layout()
    plt.show()