# renderer.py
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import numpy as np
import config as cfg
from matplotlib.colors import LinearSegmentedColormap


def plotar_frota_3d(max_x, max_y, lotes_gdf, centros_distribuicao, missoes_realizadas):
    fig = go.Figure()

    # --- CHÃO ---
    fig.add_trace(go.Mesh3d(x=[-20, max_x + 20, max_x + 20, -20], y=[-20, -20, max_y + 20, max_y + 20],
                            z=[-0.1, -0.1, -0.1, -0.1],
                            i=[0, 0], j=[1, 2], k=[2, 3], color='#1b2631', opacity=1, name='Chão', hoverinfo='skip'))

    # --- EDIFÍCIOS ---
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

    # --- BASES ---
    for i, cd in enumerate(centros_distribuicao):
        fig.add_trace(go.Scatter3d(x=[cd[0]], y=[cd[1]], z=[0.5], mode='markers',
                                   marker=dict(size=12, color='#3498DB', symbol='square'), name=f'Base CD {i + 1}'))

    total_kwh = 0.0
    total_dist = 0.0
    cores_drones = ['#00ffcc', '#ff00ff', '#ffff00', '#ff9900', '#00ccff']

    for missao in missoes_realizadas:
        rota = np.array(missao['rota'])
        bateu = missao['bateu']
        start_flt = missao['start']
        goal_flt = missao.get('goal', start_flt)

        total_kwh += missao['uav'].energia_consumida_kwh
        total_dist += missao['uav'].distancia_voada

        if missao.get('falha_planeamento', False): continue

        cor_linha = cores_drones[(missao['id_drone'] - 1) % len(cores_drones)]

        # --- LÓGICA DE PLOTAGEM CONTÍNUA (COM DESCIDA NO CLIENTE) ---
        partes_visuais = []

        partes_visuais.append(np.array([[start_flt[0], start_flt[1], 0.0]]))
        if len(rota) > 0:
            partes_visuais.append(np.array([[start_flt[0], start_flt[1], rota[0, 2]]]))

        if bateu:
            partes_visuais.append(rota)
            if 'queda' in missao:
                fig.add_trace(go.Scatter3d(x=[missao['queda'][0]], y=[missao['queda'][1]], z=[missao['queda'][2]],
                                           mode='markers', marker=dict(size=12, color='red', symbol='x'),
                                           name=f"Crash M{missao.get('id_entrega', 'X')}"))
        else:
            if len(rota) > 0:
                dists = np.hypot(rota[:, 0] - goal_flt[0], rota[:, 1] - goal_flt[1])
                idx_entrega = np.argmin(dists)

                partes_visuais.append(rota[:idx_entrega + 1])
                partes_visuais.append(np.array([[goal_flt[0], goal_flt[1], 0.0]]))

                if idx_entrega < len(rota) - 1:
                    partes_visuais.append(np.array([[goal_flt[0], goal_flt[1], rota[idx_entrega, 2]]]))
                    partes_visuais.append(rota[idx_entrega + 1:])
                    partes_visuais.append(np.array([[rota[-1, 0], rota[-1, 1], 0.0]]))

            fig.add_trace(go.Scatter3d(x=[goal_flt[0]], y=[goal_flt[1]], z=[0.1],
                                       mode='markers', marker=dict(size=6, color='#F1C40F'),
                                       name=f"Destino {missao.get('id_entrega', 'X')}"))

        if partes_visuais:
            rota_visual = np.concatenate(partes_visuais)
            fig.add_trace(go.Scatter3d(x=rota_visual[:, 0], y=rota_visual[:, 1], z=rota_visual[:, 2],
                                       mode='lines',
                                       line=dict(color=cor_linha, width=5),
                                       name=f"Missão {missao.get('id_entrega', 'X')} (UAV {missao['id_drone']})"))

    fig.update_layout(template="plotly_dark",
                      title=f"UTM 4D | Missões: {len(missoes_realizadas)} | Dist: {total_dist:.1f}m | Bat: {total_kwh:.4f} kWh",
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
        goal = missao.get('goal', (0, 0))
        cor_linha = cores_drones[(missao['id_drone'] - 1) % len(cores_drones)]

        ax.add_patch(plt.Circle(goal, cfg.ZONA_LIVRE_ENTREGA, color='#F1C40F', fill=False, linewidth=1, linestyle=':'))

        if missao.get('falha_planeamento', False):
            ax.scatter(missao['start'][0], missao['start'][1], color='#FF6600', s=150, marker='X', zorder=6)
            continue

        if len(rota) > 0:
            ax.plot(rota[:, 0], rota[:, 1], color=cor_linha, linewidth=2, alpha=0.9)

        if not bateu:
            ax.scatter(goal[0], goal[1], color='#F1C40F', s=80, marker='o', zorder=6)
        elif 'queda' in missao:
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
            ax.text(start[0] + 3, start[1] + 3, f"M{missao.get('id_entrega', 'X')} Sem Rota", color='#FF9900',
                    fontsize=9, fontweight='bold')

        elif missao.get('bateu', False):
            rota = missao['rota']
            queda = missao['queda']
            if len(rota) > 0:
                ax.plot(rota[:, 0], rota[:, 1], color='#FF3333', linewidth=3, alpha=0.9)
            ax.scatter(queda[0], queda[1], color='#FF3333', s=300, marker='X', zorder=7)
            ax.scatter(goal[0], goal[1], color='#F1C40F', s=80, marker='o', facecolors='none', zorder=6)
            ax.text(queda[0] + 3, queda[1] + 3, f"M{missao.get('id_entrega', 'X')} Crash", color='#FF3333', fontsize=9,
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
        if missao.get('falha_planeamento', False) or 'tempo_global' not in missao: continue

        tempo_bruto = missao['tempo_global']
        rota = missao['rota']
        base_origem = missao['start']
        cor = cores_drones[(missao['id_drone'] - 1) % len(cores_drones)]

        if len(tempo_bruto) == 0 or len(rota) == 0: continue

        tempo_corrigido = [tempo_bruto[0]]
        for t in tempo_bruto[1:]:
            if t <= tempo_corrigido[-1]:
                tempo_corrigido.append(tempo_corrigido[-1] + 1)
            else:
                tempo_corrigido.append(t)

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
                label=f"UAV {missao['id_drone']} (Missão {missao.get('id_entrega', 'X')})")

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
    if handles:
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(),
                  facecolor='#1a1a1a', edgecolor='white', labelcolor='white',
                  loc='upper left', bbox_to_anchor=(1, 1))

    plt.tight_layout()
    plt.show()


# ==========================================
# 1. CURVA DE DESCARGA DA BATERIA COM RECARGAS
# ==========================================
def plotar_curva_descarga(missoes_realizadas):
    fig, ax = plt.subplots(figsize=(12, 6), facecolor='#1a1a1a', dpi=150)
    ax.set_facecolor('#1a1a1a')

    cores_drones = ['#00ffcc', '#ff00ff', '#ffff00', '#ff9900', '#00ccff', '#ff3366', '#33ff33', '#ffffff']

    missoes_por_drone = {}
    for m in missoes_realizadas:
        d_id = m['id_drone']
        if d_id not in missoes_por_drone: missoes_por_drone[d_id] = []
        missoes_por_drone[d_id].append(m)

    for d_id, missoes in missoes_por_drone.items():
        tempo_acumulado = [0]
        consumo_pct = [100.0]
        cor = cores_drones[(d_id - 1) % len(cores_drones)]

        drone = missoes[0]['uav']
        capacidade_max = drone.bateria_capacidade_wh
        bateria_atual = capacidade_max
        ultimo_tempo = 0

        for m in missoes:
            rota = m['rota']
            tempos = m.get('tempo_global', [])
            if len(rota) == 0 or len(tempos) == 0: continue

            tempo_inicio = tempos[0]
            tempo_ocioso = tempo_inicio - ultimo_tempo

            # Recuperação de Bateria: Calcula o tempo na base e adiciona energia
            if tempo_ocioso > 0 and ultimo_tempo > 0:
                energia_recarregada = (drone.taxa_recarga_w / 3600.0) * tempo_ocioso
                bateria_atual = min(capacidade_max, bateria_atual + energia_recarregada)

                # Regista o nível de bateria reabastecido antes do voo
                tempo_acumulado.append(tempo_inicio)
                consumo_pct.append((bateria_atual / capacidade_max) * 100.0)

            for i in range(1, len(rota)):
                p_ant, p_atual = rota[i - 1], rota[i]
                dz = p_atual[2] - p_ant[2] if len(p_atual) == 3 else 0
                dxy = np.linalg.norm(p_atual[:2] - p_ant[:2])

                if dz > 0:
                    gasto = dz * drone.consumo_subida
                elif dz < 0:
                    gasto = abs(dz) * drone.consumo_descida
                elif dxy == 0:
                    gasto = drone.consumo_hover
                else:
                    gasto = dxy * drone.consumo_cruzeiro

                bateria_atual -= gasto
                bateria_atual = max(0, bateria_atual)

                t_atual = tempos[i] if i < len(tempos) else tempos[-1] + (i - len(tempos) + 1)
                tempo_acumulado.append(t_atual)
                consumo_pct.append((bateria_atual / capacidade_max) * 100.0)

            ultimo_tempo = tempo_acumulado[-1]

        ax.plot(tempo_acumulado, consumo_pct, color=cor, linewidth=2.5, alpha=0.8,
                label=f"UAV {d_id} ({drone.nome_modelo})")

    ax.axhline(y=20, color='#ff3333', linestyle='--', linewidth=1.5, alpha=0.8, label="Reserva Crítica (20%)")
    ax.axhline(y=100, color='#33ff33', linestyle=':', linewidth=1.0, alpha=0.5)

    ax.set_ylim(-5, 105)
    ax.set_xlabel("Tempo Global da Simulação (Frames/Segundos)", color='white', fontsize=12, fontweight='bold')
    ax.set_ylabel("Estado de Carga (SoC %)", color='white', fontsize=12, fontweight='bold')
    ax.set_title("Curva de Descarga e Recarga (Assinatura Energética da Frota)", color='white', fontsize=16, pad=15)

    ax.tick_params(colors='white')
    for spine in ['bottom', 'left']: ax.spines[spine].set_color('white')
    for spine in ['top', 'right']: ax.spines[spine].set_visible(False)

    ax.grid(color='#444444', linestyle=':', linewidth=1, alpha=0.7)

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), facecolor='#1a1a1a', edgecolor='white', labelcolor='white',
              loc='lower left')

    plt.tight_layout()
    plt.show()


# ==========================================
# 2. BOXPLOT DE ESPERAS COM JITTER
# ==========================================
def plotar_boxplot_esperas(missoes_realizadas, num_drones):
    fig, ax = plt.subplots(figsize=(10, 6), facecolor='#1a1a1a', dpi=150)
    ax.set_facecolor('#1a1a1a')

    dados_espera = [[] for _ in range(num_drones)]
    for m in missoes_realizadas:
        dados_espera[m['id_drone'] - 1].append(m.get('esperas_total', 0))

    labels_ativos = []
    dados_ativos = []
    max_espera = 0

    for i in range(num_drones):
        if dados_espera[i]:
            dados_ativos.append(dados_espera[i])
            labels_ativos.append(f"UAV {i + 1}")
            max_espera = max(max_espera, max(dados_espera[i]))

    if not dados_ativos:
        return

    # Desenha o Boxplot base
    ax.boxplot(dados_ativos, patch_artist=True,
               boxprops=dict(facecolor='#3498DB', color='white', alpha=0.7),
               capprops=dict(color='white', linewidth=1.5),
               whiskerprops=dict(color='white', linewidth=1.5),
               flierprops=dict(markerfacecolor='#e74c3c', marker='D', markersize=6),
               medianprops=dict(color='#F1C40F', linewidth=2.5))

    # Adiciona dispersão (Scatter) para que as ocorrências de 0 fiquem explícitas!
    for i, val_list in enumerate(dados_ativos):
        x = np.random.normal(i + 1, 0.05, size=len(val_list))
        ax.scatter(x, val_list, color='#e74c3c', alpha=0.7, s=25, zorder=3, label="Espera Individual" if i == 0 else "")

    ax.set_xticklabels(labels_ativos, color='white', fontweight='bold')
    ax.tick_params(axis='y', colors='white')
    ax.spines['bottom'].set_color('white')
    ax.spines['left'].set_color('white')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Previne que o eixo fique comprimido a zero caso não haja congestionamento
    ax.set_ylim(-1, max(5, max_espera + 5))

    plt.title("Boxplot e Dispersão: Atrasos de Congestionamento (TEA*)", color='white', fontsize=16, pad=15)
    plt.ylabel("Atraso Induzido (Frames em Hover Anti-Colisão)", color='white', fontsize=12, fontweight='bold')
    plt.grid(color='#444444', linestyle='--', linewidth=0.5, alpha=0.5, axis='y')

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), facecolor='#1a1a1a', edgecolor='white', labelcolor='white',
                  loc='upper right')

    plt.tight_layout()
    plt.show()


# ==========================================
# 3. MAPA DE CALOR (TRÁFEGO E QUEDAS)
# ==========================================
def plotar_mapa_calor(lotes_gdf, missoes_realizadas):
    fig, ax = plt.subplots(figsize=(10, 10), facecolor='#1a1a1a', dpi=150)
    ax.set_facecolor('#1a1a1a')

    lotes_gdf.plot(ax=ax, color='#2c3e50', edgecolor='#1a252f', linewidth=0.5, alpha=0.4)

    todos_x, todos_y = [], []
    querdas_x, quedas_y = [], []

    for m in missoes_realizadas:
        if not m.get('falha_planeamento', False):
            rota = m['rota']
            if len(rota) > 0:
                todos_x.extend(rota[:, 0])
                todos_y.extend(rota[:, 1])
        if m.get('bateu', False) and 'queda' in m:
            querdas_x.append(m['queda'][0])
            quedas_y.append(m['queda'][1])

    if not todos_x:
        return

    hb = ax.hexbin(todos_x, todos_y, gridsize=35, cmap='inferno', mincnt=1, alpha=0.85)

    if querdas_x:
        ax.scatter(querdas_x, quedas_y, color='#00ffff', s=120, marker='X', zorder=5, label='Acidente / Bateria')
        ax.legend(facecolor='#1a1a1a', edgecolor='white', labelcolor='white')

    cb = fig.colorbar(hb, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label('Intensidade de Tráfego Aéreo', color='white', fontweight='bold')
    cb.ax.yaxis.set_tick_params(color='white')
    cb.outline.set_edgecolor('white')
    plt.setp(plt.getp(cb.ax.axes, 'yticklabels'), color='white')

    plt.title("Heatmap: Gargalos Logísticos e Densidade 4D", color='white', fontsize=16, pad=15)
    plt.axis('off')
    plt.tight_layout()
    plt.show()