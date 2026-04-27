# renderer.py
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import numpy as np
import config as cfg

def plotar_3d(max_x, max_y, lotes_gdf, drone, rota_final, caminho, bateu, local_queda, start_loc, goal_loc):
    fig = go.Figure()

    fig.add_trace(go.Mesh3d(x=[-20, max_x+20, max_x+20, -20], y=[-20, -20, max_y+20, max_y+20], z=[-0.1, -0.1, -0.1, -0.1],
                            i=[0, 0], j=[1, 2], k=[2, 3], color='#1b2631', opacity=1, name='Chão'))

    all_x, all_y, all_z, all_i, all_j, all_k = [], [], [], [], [], []
    v_off = 0
    for _, l in lotes_gdf[lotes_gdf['altura_z'] > 0].iterrows():
        geometrias = [l.geometry] if l.geometry.geom_type == 'Polygon' else l.geometry.geoms
        for geom in geometrias:
            c = list(geom.exterior.coords)[:-1]; n = len(c); h = l['altura_z']
            if n < 3: continue
            all_x.extend([p[0] for p in c]*2); all_y.extend([p[1] for p in c]*2); all_z.extend([0]*n + [h]*n)
            for s in range(1, n-1):
                all_i.extend([v_off, v_off+n]); all_j.extend([v_off+s, v_off+n+s]); all_k.extend([v_off+s+1, v_off+n+s+1])
            for v in range(n):
                nxt = (v+1)%n
                all_i.extend([v_off+v, v_off+nxt]); all_j.extend([v_off+nxt, v_off+nxt+n]); all_k.extend([v_off+v+n, v_off+v+n])
            v_off += 2*n

    fig.add_trace(go.Mesh3d(x=all_x, y=all_y, z=all_z, i=all_i, j=all_j, k=all_k, color='#5d6d7e', flatshading=True, name='Edifícios'))

    def desenhar_cilindro(x, y, raio, cor, nome):
        theta = np.linspace(0, 2*np.pi, 20)
        cx = x + raio * np.cos(theta); cy = y + raio * np.sin(theta)
        fig.add_trace(go.Scatter3d(x=list(cx)+[cx[0]], y=list(cy)+[cy[0]], z=[0.5]*21, mode='lines', line=dict(color=cor, width=4), name=nome))

    desenhar_cilindro(start_loc[0], start_loc[1], cfg.ZONA_LIVRE_INICIO, '#3498DB', 'Clareira Início')
    desenhar_cilindro(goal_loc[0], goal_loc[1], cfg.ZONA_LIVRE_POUSO, '#2ECC71', 'Clareira Pouso')

    if len(caminho) > 0:
        fig.add_trace(go.Scatter3d(x=rota_final[:,0], y=rota_final[:,1], z=rota_final[:,2], mode='lines',
                                   line=dict(color='#00ffcc', width=6), name='Trajetória'))
        fig.add_trace(go.Scatter3d(x=[start_loc[0]], y=[start_loc[1]], z=[0.1], mode='markers', marker=dict(size=8, color='#3498DB'), name='Base (H)'))
        if bateu:
            fig.add_trace(go.Scatter3d(x=[local_queda[0]], y=[local_queda[1]], z=[local_queda[2]], mode='markers',
                                       marker=dict(size=15, color='red', symbol='x'), name='Colisão'))
        else:
            fig.add_trace(go.Scatter3d(x=[goal_loc[0]], y=[goal_loc[1]], z=[0.1], mode='markers', marker=dict(size=8, color='#2ECC71'), name='Pouso (H)'))

    status = '💥 CRASH' if (len(caminho)>0 and bateu) else '✅ SUCESSO' if len(caminho)>0 else '❌ BLOQUEADO'
    fig.update_layout(template="plotly_dark", title=f"Distância: {drone.distancia_voada:.1f}m | Bateria: {drone.energia_consumida_kwh:.4f} kWh | Status: {status}",
                      scene=dict(aspectmode='data', zaxis=dict(range=[0, 60])), margin=dict(l=0, r=0, b=0, t=40))
    fig.show()

def plotar_2d(lotes_gdf, start_loc, goal_loc):
    plt.figure(figsize=(10, 10), facecolor='#1a1a1a')
    ax = plt.gca(); ax.set_facecolor('#1a1a1a')
    lotes_gdf.plot(column='lote_id', cmap='viridis', ax=ax, edgecolor='white', linewidth=0.3)
    c1 = plt.Circle(start_loc, cfg.ZONA_LIVRE_INICIO, color='#3498DB', fill=False, linewidth=2, linestyle='--')
    c2 = plt.Circle(goal_loc, cfg.ZONA_LIVRE_POUSO, color='#2ECC71', fill=False, linewidth=2, linestyle='--')
    ax.add_patch(c1); ax.add_patch(c2)
    ax.scatter([start_loc[0], goal_loc[0]], [start_loc[1], goal_loc[1]], color=['#3498DB', '#2ECC71'], s=100, zorder=5)
    plt.axis('off'); plt.title(f"Planta 2D | Clareiras de Segurança ({cfg.ZONA_LIVRE_INICIO}m)", color='white')
    plt.show()