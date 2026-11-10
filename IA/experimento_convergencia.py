"""
Experimento comparativo:
1) Impacto de enxugar as features (Todas as 30 features geométricas vs. Apenas features físicas essenciais)
2) Velocidade de estabilização do GA (Geração 0 a Geração 8) comparando:
   - GA Cru (chute sequencial de fábrica)
   - GA + IA (30 features - modelo atual par-a-par)
   - GA + IA (Features Físicas Enxutas - treinada nos rótulos atuais do GA)
   - GA + IA/Warm Start Físico Enxuto (focado apenas nas grandezas físicas: dist_cd, peso, vel_ms, carga_kg, consumo)
"""
import sys
import math
import numpy as np
import pandas as pd
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial.distance import squareform
from scipy.stats import spearmanr

sys.stdout.reconfigure(encoding="utf-8")

CSV_PATH = r"C:\Users\Forlin\Dissertação\IA\dataset_treino.csv"


def calcular_custo_fisico_aprox(y, dist_m, peso_kg, vel_ms, carga_max_kg, consumo_wh_m, bateria_wh):
    """
    Replica exatamente a função objetivo física do ga_optimizer.py (tempo + 3.6 * energia + penalidades de carga/bateria)
    usando apenas as grandezas físicas reais do problema.
    """
    nd = len(vel_ms)
    tempo_drone = np.zeros(nd, dtype=float)
    energia_total = 0.0
    falhas = 0
    TEMPO_DESCARGA = 5.0

    for i, d in enumerate(y):
        d = int(d)
        v = max(vel_ms[d], 1.0)
        dist_i = dist_m[i]
        peso_i = min(peso_kg[i], carga_max_kg[d])

        # Tempo de ida + descarga + volta
        t_missao = (2.0 * dist_i / v) + (2.0 * TEMPO_DESCARGA)
        # Energia de ida + volta + hover descarga
        e_missao = (2.0 * dist_i * consumo_wh_m[d]) * (1.0 + 0.05 * peso_i)

        if e_missao * 1.5 > bateria_wh[d]:
            falhas += 1
            continue

        tempo_drone[d] += t_missao
        energia_total += e_missao

    makespan = float(np.max(tempo_drone))
    custo = makespan + 3.6 * energia_total + falhas * 50000.0
    return custo, makespan


def chute_sequencial(ne, nd):
    tb = math.ceil(ne / nd) if nd > 0 else 1
    return np.array([(i // tb) % nd for i in range(ne)], dtype=int)


def extrair_pares_completos(r, nd, ne):
    """30 features originais (geométricas + globais)."""
    ent = np.array([[r[f"ent_{i}_dist_cd"], r[f"ent_{i}_sin"], r[f"ent_{i}_cos"],
                     r[f"ent_{i}_peso"], r[f"ent_{i}_dist_od"]] for i in range(ne)], dtype=float)
    pos = np.column_stack([ent[:, 0] * ent[:, 2], ent[:, 0] * ent[:, 1]])
    D = np.hypot(pos[:, None, 0] - pos[None, :, 0], pos[:, None, 1] - pos[None, :, 1])
    ang = np.arctan2(ent[:, 1], ent[:, 2])
    dist, peso, dod = ent[:, 0], ent[:, 3], ent[:, 4]
    iu, ju = np.triu_indices(ne, 1)

    dij = D[iu, ju]
    med = float(np.median(dij)) or 1.0
    dang = np.abs(ang[iu] - ang[ju])
    dang = np.minimum(dang, 2 * np.pi - dang)
    dang_rel = dang / (2 * np.pi / max(nd, 1))

    Dm = D.copy()
    np.fill_diagonal(Dm, np.inf)
    rank = Dm.argsort(axis=1).argsort(axis=1) / max(ne - 2, 1)
    r_ij, r_ji = rank[iu, ju], rank[ju, iu]
    k3 = min(3, ne - 1)
    dens = np.sort(Dm, axis=1)[:, :k3].mean(axis=1)
    rank_d = dist.argsort().argsort() / max(ne - 1, 1)

    cols = [dij, dij / med, dang, dang_rel,
            np.minimum(dist[iu], dist[ju]), np.maximum(dist[iu], dist[ju]),
            np.abs(dist[iu] - dist[ju]),
            np.minimum(peso[iu], peso[ju]), np.maximum(peso[iu], peso[ju]), peso[iu] + peso[ju],
            np.minimum(dod[iu], dod[ju]), np.maximum(dod[iu], dod[ju]),
            np.minimum(r_ij, r_ji), np.maximum(r_ij, r_ji),
            np.minimum(dens[iu], dens[ju]), np.maximum(dens[iu], dens[ju]),
            np.minimum(rank_d[iu], rank_d[ju]), np.maximum(rank_d[iu], rank_d[ju])]
    return np.column_stack(cols), iu, ju


def extrair_pares_enxutos(r, nd, ne):
    """APENAS 4 features físicas essenciais do par (sem ângulos, sem vizinhança 2D, sem duplicatas)."""
    dist = np.array([r[f"ent_{i}_dist_cd"] for i in range(ne)], dtype=float)
    peso = np.array([r[f"ent_{i}_peso"] for i in range(ne)], dtype=float)
    iu, ju = np.triu_indices(ne, 1)
    rank_d = dist.argsort().argsort() / max(ne - 1, 1)
    cols = [
        np.minimum(dist[iu], dist[ju]),
        np.maximum(dist[iu], dist[ju]),
        np.abs(dist[iu] - dist[ju]),
        peso[iu] + peso[ju],
        np.abs(rank_d[iu] - rank_d[ju]),
    ]
    return np.column_stack(cols), iu, ju


def treinar_logreg_simples(X, y, l2=1.0):
    """Classificador logístico regularizado leve (via scipy/numpy) para testar features sem ruído."""
    mu = X.mean(axis=0)
    sd = X.std(axis=0) + 1e-8
    Xn = np.hstack([np.ones((len(X), 1)), (X - mu) / sd])
    # Aproximação por mínimos quadrados balanceados (Fisher LDA / Ridge) -> rápido e determinístico
    pos = (y == 1)
    w_sample = np.where(pos, 0.5 / max(pos.mean(), 1e-4), 0.5 / max((~pos).mean(), 1e-4))
    Xw = Xn * w_sample[:, None]
    A = Xn.T @ Xw + l2 * np.eye(Xn.shape[1])
    b = Xn.T @ (w_sample * (2 * y - 1))
    w = np.linalg.solve(A, b)
    return (mu, sd, w)


def prever_prob(modelo, X):
    mu, sd, w = modelo
    Xn = np.hstack([np.ones((len(X), 1)), (X - mu) / sd])
    z = np.clip(Xn @ w, -20, 20)
    return 1.0 / (1.0 + np.exp(-z))


def alocar_clusters_por_velocidade_e_carga(rot, dist, peso, vel_ms, carga_kg):
    """Corrige o bug de clusters_para_drones: atribui os grupos de maior distância total aos drones MAIS RÁPIDOS."""
    ncl = int(rot.max()) + 1
    esforco_cl = [float((dist[rot == c] * (1.0 + 0.1 * peso[rot == c])).sum()) for c in range(ncl)]
    ordem_cl = np.argsort(esforco_cl)[::-1]
    ordem_dr = sorted(range(len(vel_ms)), key=lambda d: (-vel_ms[d], -carga_kg[d], d))
    mapa = {int(c): ordem_dr[pos] for pos, c in enumerate(ordem_cl)}
    return np.array([mapa[int(c)] for c in rot], dtype=int)


def warm_start_fisico_enxuto(dist, peso, vel_ms, carga_kg, consumo_wh_m):
    """
    Usa APENAS as 4 features físicas (dist_cd, peso_kg, vel_ms, consumo_wh_m)
    para alocar cada entrega ao drone que minimiza o acréscimo de (Makespan + 3.6 * Energia).
    """
    nd = len(vel_ms)
    ne = len(dist)
    tempo_acum = np.zeros(nd, dtype=float)
    y = np.zeros(ne, dtype=int)

    # Ordena entregas da mais exigente (distância + peso) para a mais simples
    esforco_entrega = dist * (1.0 + 0.05 * peso)
    for i in np.argsort(-esforco_entrega):
        melhor_d = 0
        melhor_obj = math.inf
        for d in range(nd):
            sobrecarga_peso = 10000.0 if peso[i] > carga_kg[d] else 0.0
            dt = 2.0 * dist[i] / max(vel_ms[d], 1.0)
            de = 2.0 * dist[i] * consumo_wh_m[d] * (1.0 + 0.05 * peso[i])
            novo_makespan = max( float(tempo_acum.max()), tempo_acum[d] + dt )
            # Objetivo marginal: novo makespan + termo energético + equilíbrio
            obj = novo_makespan + 0.25 * (tempo_acum[d] + dt) + 3.6 * de + sobrecarga_peso
            if obj < melhor_obj:
                melhor_obj = obj
                melhor_d = d
        y[i] = melhor_d
        tempo_acum[melhor_d] += 2.0 * dist[i] / max(vel_ms[melhor_d], 1.0)
    return y


def simular_evolucao_ga(chute_ini, dist_m, peso_kg, vel_ms, carga_kg, consumo_wh_m, bateria_wh,
                        sol_pop=10, num_geracoes=8, seed=42):
    """
    Reproduz o ciclo evolutivo do GA (pop=10, ger=8, elitismo=5, mutação 1..3 na pop inicial + crossover 2 pontos)
    devolvendo o melhor custo em cada geração [Gen 0, Gen 1, ..., Gen 8] e a geração de estabilização.
    """
    rng = np.random.default_rng(seed)
    ne = len(dist_m)
    nd = len(vel_ms)

    # População inicial idêntica ao ga_optimizer.py: 1 chute + (SOL_POP-1) mutações de 1 a 3 genes
    pop = [chute_ini.copy()]
    for _ in range(sol_pop - 1):
        ind = chute_ini.copy()
        n_mut = int(rng.integers(1, 4))
        idx = rng.choice(ne, size=min(n_mut, ne), replace=False)
        ind[idx] = rng.integers(0, nd, size=len(idx))
        pop.append(ind)
    pop = np.array(pop, dtype=int)

    def avaliar_pop(P):
        return np.array([calcular_custo_fisico_aprox(ind, dist_m, peso_kg, vel_ms, carga_kg, consumo_wh_m, bateria_wh)[0]
                         for ind in P])

    custos = avaliar_pop(pop)
    historico = [float(custos.min())]

    elites = max(5, sol_pop // 10)
    n_mut_genes = max(1, int(round(0.10 * ne)))

    for g in range(1, num_geracoes + 1):
        ordem = np.argsort(custos)
        nova_pop = [pop[i].copy() for i in ordem[:elites]]
        while len(nova_pop) < sol_pop:
            # Torneio de 3 entre os melhores
            cand1 = rng.choice(sol_pop, size=min(5, sol_pop), replace=False)
            cand2 = rng.choice(sol_pop, size=min(5, sol_pop), replace=False)
            p1 = pop[cand1[np.argmin(custos[cand1])]]
            p2 = pop[cand2[np.argmin(custos[cand2])]]
            # Crossover de 2 pontos
            if ne > 2:
                pts = sorted(rng.choice(range(1, ne), size=2, replace=False))
                filho = p1.copy()
                filho[pts[0]:pts[1]] = p2[pts[0]:pts[1]]
            else:
                filho = p1.copy()
            # Mutação
            idx_m = rng.choice(ne, size=n_mut_genes, replace=False)
            filho[idx_m] = rng.integers(0, nd, size=n_mut_genes)
            nova_pop.append(filho)
        pop = np.array(nova_pop, dtype=int)
        custos = avaliar_pop(pop)
        historico.append(min(historico[-1], float(custos.min())))

    return np.array(historico)


def main():
    df = pd.read_csv(CSV_PATH)
    diag_mapa = math.hypot(3400.0, 3400.0)  # RAIO_M=1700 -> mapa 3400x3400m

    cenarios = []
    for _, r in df.iterrows():
        nd, ne = int(r["num_drones_ativos"]), int(r["num_entregas_reais"])
        y = np.array([int(r[f"drone_alocado_{i}"]) for i in range(ne)], dtype=int)
        if y.min() < 0 or y.max() >= nd:
            continue
        dist_rel = np.array([float(r[f"ent_{i}_dist_cd"]) for i in range(ne)])
        dist_m = dist_rel * diag_mapa
        peso_kg = np.array([float(r[f"ent_{i}_peso"]) for i in range(ne)]) * float(r["feat_peso_max"] or 1.0)
        vel_ms = np.array([float(r[f"uav_{d}_vel_ms"]) for d in range(nd)])
        carga_kg = np.array([float(r[f"uav_{d}_carga_kg"]) for d in range(nd)])
        bateria_wh = np.array([float(r[f"uav_{d}_bateria_wh"]) for d in range(nd)])
        consumo_wh_m = np.array([float(r[f"uav_{d}_consumo_wh_m"]) for d in range(nd)])

        X30, iu, ju = extrair_pares_completos(r, nd, ne)
        X5, _, _ = extrair_pares_enxutos(r, nd, ne)
        yp = (y[iu] == y[ju]).astype(int)

        cenarios.append({
            "id": int(r["id_amostra"]), "nd": nd, "ne": ne, "y_ga": y,
            "dist_m": dist_m, "peso_kg": peso_kg, "vel_ms": vel_ms,
            "carga_kg": carga_kg, "bateria_wh": bateria_wh, "consumo_wh_m": consumo_wh_m,
            "X30": X30, "X5": X5, "iu": iu, "ju": ju, "yp": yp,
            "makespan_real": float(r["makespan_final"]),
            "energia_real": float(r["energia_final"]),
        })

    # Validação cruzada 5-fold para comparar:
    # (A) Modelo com todas as features (X30) + alocação antiga por peso
    # (B) Modelo com features enxutas (X5) + alocação corrigida por velocidade
    # (C) Warm Start Físico Enxuto (usa diretamente as 4 variáveis físicas em cada entrega/drone)
    n = len(cenarios)
    idx_folds = np.array_split(np.arange(n), 5)

    curvas_cru = []
    curvas_ia30 = []
    curvas_ia5_vel = []
    curvas_ws_fisico = []

    custo_ga_csv = []
    custo_real_csv = []

    for f in range(5):
        te_idx = set(idx_folds[f])
        tr_cens = [cenarios[i] for i in range(n) if i not in te_idx]
        X30_tr = np.vstack([c["X30"] for c in tr_cens])
        X5_tr = np.vstack([c["X5"] for c in tr_cens])
        y_tr = np.concatenate([c["yp"] for c in tr_cens])

        mod30 = treinar_logreg_simples(X30_tr, y_tr)
        mod5 = treinar_logreg_simples(X5_tr, y_tr)

        for i in idx_folds[f]:
            c = cenarios[i]
            nd, ne = c["nd"], c["ne"]

            # 1) Chute sequencial (GA Cru)
            y_seq = chute_sequencial(ne, nd)

            # 2) IA com 30 features + alocação antiga por peso
            p30 = prever_prob(mod30, c["X30"])
            D30 = np.zeros((ne, ne))
            D30[c["iu"], c["ju"]] = 1.0 - p30
            D30[c["ju"], c["iu"]] = 1.0 - p30
            rot30 = fcluster(linkage(squareform(D30, checks=False), method="average"), nd, criterion="maxclust") - 1
            # alocação antiga por peso_kg
            ncl = int(rot30.max()) + 1
            peso_cl = [c["peso_kg"][rot30 == cl].sum() for cl in range(ncl)]
            ordem_cl = np.argsort(peso_cl)[::-1]
            ordem_dr = sorted(range(nd), key=lambda d: (-c["carga_kg"][d], d))
            mapa30 = {int(cl): ordem_dr[pos] for pos, cl in enumerate(ordem_cl)}
            y_ia30 = np.array([mapa30[int(cl)] for cl in rot30], dtype=int)

            # 3) IA com 5 features físicas enxutas + alocação corrigida pela velocidade do drone
            p5 = prever_prob(mod5, c["X5"])
            D5 = np.zeros((ne, ne))
            D5[c["iu"], c["ju"]] = 1.0 - p5
            D5[c["ju"], c["iu"]] = 1.0 - p5
            rot5 = fcluster(linkage(squareform(D5, checks=False), method="average"), nd, criterion="maxclust") - 1
            y_ia5 = alocar_clusters_por_velocidade_e_carga(rot5, c["dist_m"], c["peso_kg"], c["vel_ms"], c["carga_kg"])

            # 4) Warm Start Físico Direto (4 features físicas: dist_cd, peso_kg, vel_ms, consumo_wh_m)
            y_ws = warm_start_fisico_enxuto(c["dist_m"], c["peso_kg"], c["vel_ms"], c["carga_kg"], c["consumo_wh_m"])

            # Evolução geração a geração (Gen 0 a Gen 8)
            h_cru = simular_evolucao_ga(y_seq, c["dist_m"], c["peso_kg"], c["vel_ms"], c["carga_kg"], c["consumo_wh_m"], c["bateria_wh"], seed=1000 + i)
            h_ia30 = simular_evolucao_ga(y_ia30, c["dist_m"], c["peso_kg"], c["vel_ms"], c["carga_kg"], c["consumo_wh_m"], c["bateria_wh"], seed=1000 + i)
            h_ia5 = simular_evolucao_ga(y_ia5, c["dist_m"], c["peso_kg"], c["vel_ms"], c["carga_kg"], c["consumo_wh_m"], c["bateria_wh"], seed=1000 + i)
            h_ws = simular_evolucao_ga(y_ws, c["dist_m"], c["peso_kg"], c["vel_ms"], c["carga_kg"], c["consumo_wh_m"], c["bateria_wh"], seed=1000 + i)

            # Normaliza pelo custo inicial do GA Cru (Gen 0 = 100%)
            ref = h_cru[0]
            curvas_cru.append(100.0 * h_cru / ref)
            curvas_ia30.append(100.0 * h_ia30 / ref)
            curvas_ia5_vel.append(100.0 * h_ia5 / ref)
            curvas_ws_fisico.append(100.0 * h_ws / ref)

            c_ga, _ = calcular_custo_fisico_aprox(c["y_ga"], c["dist_m"], c["peso_kg"], c["vel_ms"], c["carga_kg"], c["consumo_wh_m"], c["bateria_wh"])
            custo_ga_csv.append(c_ga)
            custo_real_csv.append(c["makespan_real"] + 3.6 * c["energia_real"])

    rho_custo = spearmanr(custo_ga_csv, custo_real_csv)[0]
    print(f"Correlação de Spearman (Custo Físico Aprox vs Custo Simulador 4D nos 51 cenários): {rho_custo:.4f}\n")

    m_cru = np.mean(curvas_cru, axis=0)
    m_ia30 = np.mean(curvas_ia30, axis=0)
    m_ia5 = np.mean(curvas_ia5_vel, axis=0)
    m_ws = np.mean(curvas_ws_fisico, axis=0)

    print("CURVA DE CONVERGÊNCIA MÉDIA (51 cenários, % do Custo Inicial do GA Cru — menor é melhor):")
    print(f"{'Geração':<10} | {'GA Cru (Seq)':<15} | {'IA (30 Feats)':<15} | {'IA (5 Feats+Vel)':<18} | {'Warm Start Físico (4 Feats)':<28}")
    print("-" * 96)
    for g in range(9):
        print(f"Gen {g:<6d} | {m_cru[g]:>13.1f}% | {m_ia30[g]:>13.1f}% | {m_ia5[g]:>16.1f}% | {m_ws[g]:>26.1f}%")

    # Quantas gerações cada abordagem precisa para bater o resultado FINAL (Gen 8) do GA Cru?
    alvo_final_cru = m_cru[-1]
    print(f"\nMeta: atingir o custo final do GA Cru na Gen 8 ({alvo_final_cru:.1f}%):")
    for nome, curva in [
        ("GA Cru (chute sequencial)", m_cru),
        ("IA Antiga (30 features geométricas)", m_ia30),
        ("IA Enxuta (5 features físicas + velocidade)", m_ia5),
        ("Warm Start Físico Direto (4 features físicas)", m_ws),
    ]:
        atingiu = [g for g, val in enumerate(curva) if val <= alvo_final_cru + 1e-6]
        gen_atingiu = f"Gen {atingiu[0]}" if atingiu else "Não atingiu em 8 ger."
        print(f"  - {nome:<45}: atinge em {gen_atingiu:<8} (Gen 0 = {curva[0]:.1f}% -> Gen 8 = {curva[-1]:.1f}%)")


if __name__ == "__main__":
    main()
