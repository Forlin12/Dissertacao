"""
Pipeline de Warm Start para o GA de drones (v3 — alvo par-a-par):
  1) Gera dataset (retomável, escrita atómica)  [formato do CSV inalterado]
  2) Treina um modelo "estas duas entregas vão para o mesmo drone?" e valida
     por cenário, comparando com k-means geométrico e com o aleatório
  3) Benchmark GA a frio (CACHEADO em disco) vs warm start (ML e, opcional, geométrico)

Uso:  python dataset_generator.py
      python dataset_generator.py --amostras 500
      python dataset_generator.py --retreinar
      python dataset_generator.py --so-benchmark                # só compara (frio vem do cache)
      python dataset_generator.py --so-benchmark --com-geometrico
      python dataset_generator.py --so-benchmark --refazer-frio # recalcula o GA a frio
      python dataset_generator.py --consistencia                # teto realista: GA vs GA (seeds diferentes)
      python dataset_generator.py --auto
"""
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import argparse
import csv
import hashlib
import inspect
import json
import math
import random
import time
import traceback

import numpy as np
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial.distance import squareform

import config as cfg
from city_builder import gerar_cidade
import mission_control
from IA.teste import ga_optimizer
from IA.teste import executar_otimizacao_batch

STATE_FILE = "training_state.json"  # informativo; a retoma usa o CSV
DATASET_CSV = "dataset_treino.csv"
MODELO_PATH = "modelo_warm_start.joblib"
RELATORIO_PATH = "relatorio_validacao.json"
HISTORICO_PATH = "relatorio_historico.json"
BENCH_PATH = "relatorio_benchmark.json"
CACHE_FRIO_PATH = "benchmark_frio_cache.json"
SEED_BASE = 12345
SEED_BENCH = 900000
CIDADE_SEED = 42   # a cidade é SEMPRE a mesma (mesma seed do GA original)
N_BENCHMARK = 10
VERSAO_ALVO = 3    # 3 = modelo par-a-par (agrupamento), invariante à numeração dos drones

MATRIZ_TESTES_IA = [
    [5, 10, 'MOSP_CAMADAS'],
    [5, 20, 'MOSP_CAMADAS'],
    [5, 40, 'MOSP_CAMADAS'],
    [10, 20, 'MOSP_CAMADAS'],
    [10, 40, 'MOSP_CAMADAS'],
    # [10, 80, 'MOSP_CAMADAS'],
    # [20, 40, 'MOSP_CAMADAS'],
    # [20, 80, 'MOSP_CAMADAS'],
    # [20, 160, 'MOSP_CAMADAS'],
]


def _json_default(o):
    return o.item() if hasattr(o, "item") else str(o)


# =====================================================================
# FEATURES
# =====================================================================
def _pos_cd(cds, max_x, max_y):
    c = cds[0] if isinstance(cds, (list, tuple)) and cds and \
        isinstance(cds[0], (list, tuple, dict)) else cds
    if isinstance(c, dict):
        if 'x' in c and 'y' in c:
            return float(c['x']), float(c['y'])
        if 'pos' in c:
            return float(c['pos'][0]), float(c['pos'][1])
    elif isinstance(c, (list, tuple)) and len(c) >= 2:
        return float(c[0]), float(c[1])
    return max_x / 2.0, max_y / 2.0


def extrair_features(missoes, cd_xy, max_x, max_y, num_drones, max_entregas):
    """Devolve (agregadas[5], por_entrega[max_entregas][5]) relativas ao CD."""
    diag = math.hypot(max_x, max_y) or 1.0
    cx, cy = cd_xy
    dist, pesos, por_entrega = [], [], []
    peso_ref = max((m['peso_kg'] for m in missoes), default=1.0) or 1.0
    for m in missoes:
        dx, dy = m['destino'][0] - cx, m['destino'][1] - cy
        d = math.hypot(dx, dy)
        ang = math.atan2(dy, dx)
        orig = m.get('origem', (cx, cy))
        d_od = math.hypot(m['destino'][0] - orig[0], m['destino'][1] - orig[1])
        dist.append(d / diag)
        pesos.append(m['peso_kg'])
        por_entrega.append([d / diag, math.sin(ang), math.cos(ang),
                            m['peso_kg'] / peso_ref, d_od / diag])
    while len(por_entrega) < max_entregas:
        por_entrega.append([0.0] * 5)
    agregadas = [
        float(np.mean(dist)) if dist else 0.0,
        float(np.std(dist)) if dist else 0.0,
        float(np.mean(pesos)) if pesos else 0.0,
        float(np.max(pesos)) if pesos else 0.0,
        len(missoes) / max(num_drones, 1),
    ]
    return agregadas, por_entrega


def _stats_frota(perfis, num_drones):
    ps = [perfis[d % len(perfis)] for d in range(num_drones)]
    return [float(np.mean([p.get('carga_kg', 0.0) for p in ps])),
            float(np.mean([p.get('velocidade_cruzeiro_ms', 0.0) for p in ps])),
            float(np.mean([p.get('bateria_capacidade_wh', 0.0) for p in ps]))]


def _globais(num_drones, num_entregas, cd_rel, agregadas, frota):
    return [num_drones, num_entregas, cd_rel[0], cd_rel[1]] + list(agregadas) + list(frota)


N_FEAT_PAR = 18


def _posicoes(ent):
    """Posições (x, y) das entregas relativas ao CD, em unidades da diagonal."""
    ent = np.asarray(ent, dtype=float)
    return np.column_stack([ent[:, 0] * ent[:, 2], ent[:, 0] * ent[:, 1]])


def features_pares(globais, ent):
    """Features simétricas para cada par (i<j). ent: (n,5) = [dist, sin, cos, peso, d_od].
    Devolve X (n_pares, len(globais)+18), iu, ju."""
    ent = np.asarray(ent, dtype=float)
    n = len(ent)
    if n < 2:
        vazio = np.zeros(0, dtype=int)
        return np.zeros((0, len(globais) + N_FEAT_PAR)), vazio, vazio

    pos = _posicoes(ent)
    D = np.hypot(pos[:, None, 0] - pos[None, :, 0], pos[:, None, 1] - pos[None, :, 1])
    ang = np.arctan2(ent[:, 1], ent[:, 2])
    dist, peso, dod = ent[:, 0], ent[:, 3], ent[:, 4]
    iu, ju = np.triu_indices(n, 1)

    dij = D[iu, ju]
    med = float(np.median(dij)) or 1.0
    dang = np.abs(ang[iu] - ang[ju])
    dang = np.minimum(dang, 2 * np.pi - dang)
    nd = max(globais[0], 1)
    dang_rel = dang / (2 * np.pi / nd)               # largura angular em "fatias de drone"

    Dm = D.copy()
    np.fill_diagonal(Dm, np.inf)
    rank = Dm.argsort(axis=1).argsort(axis=1) / max(n - 2, 1)   # 0 = vizinho mais próximo
    r_ij, r_ji = rank[iu, ju], rank[ju, iu]
    k3 = min(3, n - 1)
    dens = np.sort(Dm, axis=1)[:, :k3].mean(axis=1)             # distância média aos 3 vizinhos
    rank_d = dist.argsort().argsort() / max(n - 1, 1)

    cols = [dij, dij / med, dang, dang_rel,
            np.minimum(dist[iu], dist[ju]), np.maximum(dist[iu], dist[ju]),
            np.abs(dist[iu] - dist[ju]),
            np.minimum(peso[iu], peso[ju]), np.maximum(peso[iu], peso[ju]), peso[iu] + peso[ju],
            np.minimum(dod[iu], dod[ju]), np.maximum(dod[iu], dod[ju]),
            np.minimum(r_ij, r_ji), np.maximum(r_ij, r_ji),
            np.minimum(dens[iu], dens[ju]), np.maximum(dens[iu], dens[ju]),
            np.minimum(rank_d[iu], rank_d[ju]), np.maximum(rank_d[iu], rank_d[ju])]
    par = np.column_stack(cols)
    g = np.tile(np.asarray(globais, dtype=float), (len(iu), 1))
    return np.hstack([g, par]), iu, ju


# =====================================================================
# AGRUPAMENTO (probabilidades de pares -> grupos -> drones)
# =====================================================================
def agrupar_por_probabilidade(p_pares, iu, ju, n, k):
    """Clustering aglomerativo (average linkage) com distância 1 - P(mesmo drone)."""
    if n <= 1:
        return np.zeros(n, dtype=int)
    k = max(1, min(int(k), n))
    D = np.zeros((n, n))
    D[iu, ju] = 1.0 - p_pares
    D[ju, iu] = 1.0 - p_pares
    Z = linkage(squareform(D, checks=False), method='average')
    return fcluster(Z, k, criterion='maxclust') - 1


def agrupar_geometrico(ent, k):
    """Baseline determinístico: k-means nas posições das entregas."""
    from sklearn.cluster import KMeans
    n = len(ent)
    k = max(1, min(int(k), n))
    if k == 1 or n <= 1:
        return np.zeros(n, dtype=int)
    return KMeans(n_clusters=k, n_init=10, random_state=0).fit_predict(_posicoes(ent))


def clusters_para_drones(rot, pesos, cargas):
    """Maior carga total de entregas -> drone de maior capacidade (desempate por índice)."""
    rot = np.asarray(rot, dtype=int)
    pesos = np.asarray(pesos, dtype=float)
    ncl = int(rot.max()) + 1
    peso_cl = [pesos[rot == c].sum() for c in range(ncl)]
    ordem_cl = np.argsort(peso_cl)[::-1]
    ordem_dr = sorted(range(len(cargas)), key=lambda d: (-cargas[d], d))
    mapa = {int(c): ordem_dr[pos] for pos, c in enumerate(ordem_cl)}
    return np.array([mapa[int(c)] for c in rot], dtype=int)


# =====================================================================
# PERSISTÊNCIA
# =====================================================================
def carregar_ultima_amostra():
    if not os.path.exists(DATASET_CSV):
        return -1
    ultimo = -1
    with open(DATASET_CSV, newline='') as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
            if row:
                try:
                    ultimo = max(ultimo, int(row[0]))
                except ValueError:
                    pass
    return ultimo


def salvar_estado(idx):
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"ultima_amostra_concluida": idx}, f)
    os.replace(tmp, STATE_FILE)


def anexar_linha_atomica(linha):
    tmp = DATASET_CSV + ".tmp"
    with open(tmp, 'w', newline='') as f_tmp:
        w = csv.writer(f_tmp)
        with open(DATASET_CSV, newline='') as f_orig:
            for r in csv.reader(f_orig):
                w.writerow(r)
        w.writerow(linha)
        f_tmp.flush()
        os.fsync(f_tmp.fileno())
    os.replace(tmp, DATASET_CSV)


# =====================================================================
# FASE 1 — GERAÇÃO DO DATASET (formato inalterado)
# =====================================================================
def _preparar_cenario(amostra_seed, num_drones, num_entregas, algoritmo, lotes_gdf, max_x, max_y):
    random.seed(amostra_seed)
    np.random.seed(amostra_seed)
    cfg.SEED_CENARIO = amostra_seed
    cfg.NUM_DRONES_DISPONIVEIS = num_drones
    cfg.NUM_ENTREGAS_TOTAL = num_entregas
    cfg.TIPO_ALGORITMO = algoritmo
    # Cópia: mission_control zera altura_z dos prédios e isso não pode acumular entre amostras.
    lotes_amostra = lotes_gdf.copy()
    cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_amostra)
    cenario = {'lotes_gdf': lotes_amostra, 'max_x': max_x, 'max_y': max_y,
               'cds': cds, 'missoes': missoes}
    return _pos_cd(cds, max_x, max_y), missoes, cenario


def gerar_dataset(total_amostras):
    max_entregas = getattr(cfg, 'MAX_ENTREGAS_IA', 160)
    max_drones = getattr(cfg, 'MAX_DRONES_IA', 20)

    mx_d = max(c[0] for c in MATRIZ_TESTES_IA)
    mx_e = max(c[1] for c in MATRIZ_TESTES_IA)
    if mx_d > max_drones or mx_e > max_entregas:
        raise ValueError(f"MATRIZ_TESTES_IA precisa de {mx_d} drones / {mx_e} entregas, mas "
                         f"cfg.MAX_DRONES_IA={max_drones} / cfg.MAX_ENTREGAS_IA={max_entregas}. "
                         "Aumente os limites no config (os dados seriam truncados).")

    colunas = ["id_amostra", "num_drones_ativos", "num_entregas_reais",
               "cd_x_rel", "cd_y_rel",
               "feat_dist_media", "feat_dist_std", "feat_peso_medio",
               "feat_peso_max", "feat_entregas_por_drone"]
    for d in range(max_drones):
        colunas += [f"uav_{d}_carga_kg", f"uav_{d}_vel_ms",
                    f"uav_{d}_bateria_wh", f"uav_{d}_consumo_wh_m"]
    for i in range(max_entregas):
        colunas += [f"ent_{i}_dist_cd", f"ent_{i}_sin", f"ent_{i}_cos",
                    f"ent_{i}_peso", f"ent_{i}_dist_od"]
    for i in range(max_entregas):
        colunas.append(f"drone_alocado_{i}")
    colunas += ["geracoes_convergencia", "makespan_final", "esperas_final",
                "falhas_final", "energia_final", "tempo_exec_s"]

    if os.path.exists(DATASET_CSV):
        with open(DATASET_CSV, newline='') as f:
            cab = next(csv.reader(f), [])
        if cab != colunas:
            bak = f"dataset_treino_antigo_{int(time.time())}.csv"
            os.replace(DATASET_CSV, bak)
            if os.path.exists(STATE_FILE):
                os.remove(STATE_FILE)
            print(f"⚠️ CSV existente tinha outro formato. Movido para {bak}; recomeçando do zero.")
    if not os.path.exists(DATASET_CSV):
        with open(DATASET_CSV, 'w', newline='') as f:
            csv.writer(f).writerow(colunas)

    inicio = carregar_ultima_amostra() + 1
    if inicio >= total_amostras:
        print("✅ Dataset já completo.")
        return
    print(f"🚀 Gerando dataset: amostras {inicio} → {total_amostras - 1}")

    cfg.CENARIO_SEMPRE_NOVO = False
    random.seed(CIDADE_SEED)
    np.random.seed(CIDADE_SEED)
    lotes_gdf, max_x, max_y = gerar_cidade()
    perfis = cfg.PERFIS_DRONES

    for amostra in range(inicio, total_amostras):
        try:
            nd, ne, alg = MATRIZ_TESTES_IA[amostra % len(MATRIZ_TESTES_IA)]
            print(f"\n{'=' * 50}\n📊 AMOSTRA {amostra}/{total_amostras} | 🚁 {nd} | 📦 {ne} | 🧠 {alg}\n{'=' * 50}")

            cd_xy, missoes, cenario = _preparar_cenario(SEED_BASE + amostra, nd, ne, alg, lotes_gdf, max_x, max_y)
            agreg, por_ent = extrair_features(missoes, cd_xy, max_x, max_y, nd, max_entregas)

            t0 = time.perf_counter()
            vetor, makespan, esperas, falhas, energia, geracoes = \
                executar_otimizacao_batch(amostra, nd, ne, alg, cenario=cenario,
                                          seed_ga=SEED_BASE + amostra)
            tempo = time.perf_counter() - t0

            linha = [amostra, nd, ne, cd_xy[0] / max_x, cd_xy[1] / max_y] + agreg
            for d in range(max_drones):
                if d < nd:
                    p = perfis[d % len(perfis)]
                    linha += [p.get('carga_kg', 0.0), p.get('velocidade_cruzeiro_ms', 0.0),
                              p.get('bateria_capacidade_wh', 0.0), p.get('consumo_cruzeiro_wh_m', 0.0)]
                else:
                    linha += [0.0] * 4
            for i in range(max_entregas):
                linha += por_ent[i]
            for i in range(max_entregas):
                linha.append(int(vetor[i]) if i < len(missoes) else -1)
            linha += [geracoes, makespan, esperas, falhas, round(energia, 2), round(tempo, 2)]

            anexar_linha_atomica(linha)
            salvar_estado(amostra)
            print(f"💾 Amostra {amostra} gravada ({geracoes} gerações, {tempo:.1f}s).")
        except Exception as e:
            print(f"\n⚠️ ERRO NA AMOSTRA {amostra}: {e}")
            traceback.print_exc()
            continue


# =====================================================================
# FASE 2 — TREINO + VALIDAÇÃO (par-a-par)
# =====================================================================
def _carregar_cenarios_do_csv():
    import pandas as pd
    df = pd.read_csv(DATASET_CSV)
    max_entregas = sum(1 for c in df.columns if c.startswith("drone_alocado_"))
    cenarios = []
    for _, r in df.iterrows():
        nd, ne = int(r["num_drones_ativos"]), int(r["num_entregas_reais"])
        if ne < 2:
            continue
        agreg = [r["feat_dist_media"], r["feat_dist_std"], r["feat_peso_medio"],
                 r["feat_peso_max"], r["feat_entregas_por_drone"]]
        frota = [np.mean([r[f"uav_{d}_carga_kg"] for d in range(nd)]),
                 np.mean([r[f"uav_{d}_vel_ms"] for d in range(nd)]),
                 np.mean([r[f"uav_{d}_bateria_wh"] for d in range(nd)])]
        ent = np.array([[r[f"ent_{i}_dist_cd"], r[f"ent_{i}_sin"], r[f"ent_{i}_cos"],
                         r[f"ent_{i}_peso"], r[f"ent_{i}_dist_od"]] for i in range(ne)])
        y = np.array([int(r[f"drone_alocado_{i}"]) for i in range(ne)])
        glob = _globais(nd, ne, (r["cd_x_rel"], r["cd_y_rel"]), agreg, frota)
        X, iu, ju = features_pares(glob, ent)
        cenarios.append({"id": int(r["id_amostra"]), "nd": nd, "ne": ne, "ent": ent, "y": y,
                         "X": X, "iu": iu, "ju": ju,
                         "yp": (y[iu] == y[ju]).astype(int)})
    return cenarios, max_entregas


def _p_mesmo(modelo, X):
    return modelo.predict_proba(X)[:, list(modelo.classes_).index(1)]


def treinar_e_validar():
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.model_selection import KFold
    from sklearn.metrics import adjusted_rand_score, roc_auc_score, average_precision_score
    import joblib

    cens, max_entregas = _carregar_cenarios_do_csv()
    n_cen = len(cens)
    if n_cen < 3:
        print(f"⚠️ Só {n_cen} cenários no dataset; mínimo 3 para validar. Treino ignorado.")
        return None
    n_pares = int(sum(len(c["yp"]) for c in cens))
    print(f"\n🧠 Treinando (par-a-par): {n_pares} pares de {n_cen} cenários.")

    def novo():
        return RandomForestClassifier(n_estimators=300, min_samples_leaf=3, n_jobs=-1,
                                      class_weight="balanced_subsample", random_state=42)

    def empilhar(lista):
        return np.vstack([c["X"] for c in lista]), np.concatenate([c["yp"] for c in lista])

    rng = np.random.default_rng(0)
    k = min(5, n_cen)
    ari_m, ari_geo, ari_ale, auc_l, ap_l, prev_l = [], [], [], [], [], []
    for tr, te in KFold(n_splits=k, shuffle=True, random_state=42).split(np.arange(n_cen)):
        Xtr, ytr = empilhar([cens[i] for i in tr])
        m = novo().fit(Xtr, ytr)
        yt_all, pt_all = [], []
        for i in te:
            c = cens[i]
            p = _p_mesmo(m, c["X"])
            yt_all.append(c["yp"])
            pt_all.append(p)
            rot = agrupar_por_probabilidade(p, c["iu"], c["ju"], c["ne"], c["nd"])
            ari_m.append(float(adjusted_rand_score(c["y"], rot)))
            ari_geo.append(float(adjusted_rand_score(c["y"], agrupar_geometrico(c["ent"], c["nd"]))))
            ari_ale.append(float(adjusted_rand_score(c["y"], rng.integers(0, c["nd"], c["ne"]))))
        yt, pt = np.concatenate(yt_all), np.concatenate(pt_all)
        prev_l.append(float(yt.mean()))
        if 0 < yt.sum() < len(yt):
            auc_l.append(float(roc_auc_score(yt, pt)))
            ap_l.append(float(average_precision_score(yt, pt)))

    relatorio = {
        "cenarios": n_cen, "pares": n_pares, "folds": k,
        "ari_agrupamento_medio": round(float(np.mean(ari_m)), 4),       # modelo
        "ari_baseline_kmeans": round(float(np.mean(ari_geo)), 4),       # geometria pura
        "ari_baseline_aleatorio": round(float(np.mean(ari_ale)), 4),
        "ari_modelo_std": round(float(np.std(ari_m)), 4),
        "auc_pares": round(float(np.mean(auc_l)), 4) if auc_l else None,
        "ap_pares": round(float(np.mean(ap_l)), 4) if ap_l else None,
        "prevalencia_pares": round(float(np.mean(prev_l)), 4),          # AP aleatório ≈ isto
    }
    relatorio["ganho_ari_vs_kmeans"] = round(
        relatorio["ari_agrupamento_medio"] - relatorio["ari_baseline_kmeans"], 4)
    print("\n📈 VALIDAÇÃO (por cenário, sem vazamento):")
    for kk, vv in relatorio.items():
        print(f"   {kk}: {vv}")
    if relatorio["ari_agrupamento_medio"] < 0.1:
        print("   ⚠️ ARI baixo: o GA pode ser pouco consistente (corra --consistencia para ver o teto).")
    if relatorio["ganho_ari_vs_kmeans"] < 0.02:
        print("   ℹ️ O modelo não bate o k-means geométrico: considere usá-lo como warm start "
              "(benchmark com --com-geometrico).")

    Xall, yall = empilhar(cens)
    modelo = novo().fit(Xall, yall)
    joblib.dump({"modelo": modelo, "max_entregas": max_entregas,
                 "versao_alvo": VERSAO_ALVO, "tipo": "pares"}, MODELO_PATH)
    with open(RELATORIO_PATH, "w") as f:
        json.dump(relatorio, f, indent=2)
    try:
        hist = json.load(open(HISTORICO_PATH)) if os.path.exists(HISTORICO_PATH) else []
    except Exception:
        hist = []
    hist.append({"quando": time.strftime("%Y-%m-%d %H:%M:%S"), **relatorio})
    with open(HISTORICO_PATH, "w") as f:
        json.dump(hist, f, indent=2)
    print(f"💾 Modelo → {MODELO_PATH} | Relatório → {RELATORIO_PATH}")
    return relatorio


# =====================================================================
# INFERÊNCIA + POPULAÇÃO INICIAL PARA O PYGAD
# =====================================================================
def _cargas(perfis, num_drones):
    return [perfis[d % len(perfis)].get('carga_kg', 0.0) for d in range(num_drones)]


def prever_alocacao(modelo, missoes, cd_xy, max_x, max_y, num_drones, perfis, max_entregas=160):
    n = len(missoes)
    if n == 0:
        return []
    if n == 1:
        return [0]
    agreg, ent = extrair_features(missoes, cd_xy, max_x, max_y, num_drones, max_entregas)
    ent = np.array(ent[:n])
    glob = _globais(num_drones, n, (cd_xy[0] / max_x, cd_xy[1] / max_y),
                    agreg, _stats_frota(perfis, num_drones))
    X, iu, ju = features_pares(glob, ent)
    rot = agrupar_por_probabilidade(_p_mesmo(modelo, X), iu, ju, n, num_drones)
    pesos = [m['peso_kg'] for m in missoes]
    return clusters_para_drones(rot, pesos, _cargas(perfis, num_drones)).tolist()


def prever_alocacao_geometrica(missoes, cd_xy, max_x, max_y, num_drones, perfis, max_entregas=160):
    n = len(missoes)
    if n == 0:
        return []
    if n == 1:
        return [0]
    _, ent = extrair_features(missoes, cd_xy, max_x, max_y, num_drones, max_entregas)
    rot = agrupar_geometrico(np.array(ent[:n]), num_drones)
    pesos = [m['peso_kg'] for m in missoes]
    return clusters_para_drones(rot, pesos, _cargas(perfis, num_drones)).tolist()


def gerar_populacao_inicial(pred, tam_pop, num_drones, taxa_mutacao=0.1, frac_aleatoria=0.3):
    """1 indivíduo = previsão exata; resto = mutações da previsão + aleatórios
    (mantém diversidade para o GA não estagnar)."""
    pred = np.array(pred, dtype=int)
    pop = [pred.copy()]
    n_alea = int(tam_pop * frac_aleatoria)
    while len(pop) < tam_pop - n_alea:
        ind = pred.copy()
        mask = np.random.rand(len(ind)) < taxa_mutacao
        ind[mask] = np.random.randint(0, num_drones, mask.sum())
        pop.append(ind)
    while len(pop) < tam_pop:
        pop.append(np.random.randint(0, num_drones, len(pred)))
    return np.array(pop)


# =====================================================================
# FASE 3 — BENCHMARK (frio CACHEADO vs warm start)
# =====================================================================
def _fingerprint():
    """Muda se mexer em qualquer cfg.GA_* ou nas seeds. Se mudar, o cache do frio é descartado.
    (Alterações no CÓDIGO do ga_optimizer não são detetadas: use --refazer-frio.)"""
    d = {k: repr(getattr(cfg, k)) for k in sorted(dir(cfg)) if k.startswith("GA_")}
    d.update(CIDADE_SEED=CIDADE_SEED, SEED_BENCH=SEED_BENCH)
    return hashlib.md5(json.dumps(d, sort_keys=True).encode()).hexdigest()


def _carregar_cache_frio(refazer):
    fp = _fingerprint()
    if not refazer and os.path.exists(CACHE_FRIO_PATH):
        try:
            c = json.load(open(CACHE_FRIO_PATH))
            if c.get("fingerprint") == fp:
                return c
            print("⚠️ Configuração do GA mudou desde o cache do frio: a descartar e recalcular.")
        except Exception:
            pass
    return {"fingerprint": fp, "cenarios": {}}


def _salvar_cache_frio(cache):
    tmp = CACHE_FRIO_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cache, f, indent=2, default=_json_default)
    os.replace(tmp, CACHE_FRIO_PATH)


def _correr_ga(idx, nd, ne, alg, seed, lotes_gdf, max_x, max_y, pop=None):
    """Cenário SEMPRE regenerado a partir da seed (determinístico), nunca reutilizado entre corridas."""
    _, _, cenario = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
    kw = dict(cenario=cenario, seed_ga=seed)
    if pop is not None:
        kw["populacao_inicial"] = pop
    t0 = time.perf_counter()
    vetor, mk, es, fa, en, ger = executar_otimizacao_batch(idx, nd, ne, alg, **kw)
    return {"geracoes": int(ger), "makespan": mk, "esperas": es, "falhas": fa,
            "energia": round(float(en), 2), "tempo_s": round(time.perf_counter() - t0, 2)}


def _pct(a, b):
    return round(100.0 * (a - b) / a, 1) if a else 0.0


def benchmark_convergencia(tam_pop=None, n_bench=N_BENCHMARK, refazer_frio=False, com_geo=False):
    import joblib
    params = inspect.signature(executar_otimizacao_batch).parameters
    if "populacao_inicial" not in params:
        print("\nℹ️ Benchmark ignorado: executar_otimizacao_batch ainda não aceita "
              "o argumento 'populacao_inicial'.")
        return None
    if not os.path.exists(MODELO_PATH):
        print("\nℹ️ Benchmark ignorado: não há modelo treinado.")
        return None
    pacote = joblib.load(MODELO_PATH)
    if pacote.get("versao_alvo") != VERSAO_ALVO:
        print("\nℹ️ Modelo de versão antiga: corra com --retreinar antes do benchmark.")
        return None
    tam_pop = tam_pop or getattr(cfg, 'GA_SOL_POP', 10)
    modelo, max_entregas = pacote["modelo"], pacote["max_entregas"]

    cfg.CENARIO_SEMPRE_NOVO = False
    random.seed(CIDADE_SEED)
    np.random.seed(CIDADE_SEED)
    lotes_gdf, max_x, max_y = gerar_cidade()
    perfis = cfg.PERFIS_DRONES
    cache = _carregar_cache_frio(refazer_frio)

    print(f"\n⏱️ Benchmark: {n_bench} cenários fixos (seeds {SEED_BENCH}…{SEED_BENCH + n_bench - 1}) "
          f"| frio em cache: {len(cache['cenarios'])}")
    res = []
    for k in range(n_bench):
        nd, ne, alg = MATRIZ_TESTES_IA[k % len(MATRIZ_TESTES_IA)]
        seed = SEED_BENCH + k
        chave = f"{seed}|{nd}|{ne}|{alg}"

        cd_xy, missoes, _ = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
        pred = prever_alocacao(modelo, missoes, cd_xy, max_x, max_y, nd, perfis, max_entregas)
        np.random.seed(seed + 1)
        pop_ml = gerar_populacao_inicial(pred, tam_pop, nd)

        if chave in cache["cenarios"]:
            frio, origem = cache["cenarios"][chave], "cache"
        else:
            frio = _correr_ga(100000 + k, nd, ne, alg, seed, lotes_gdf, max_x, max_y)
            cache["cenarios"][chave] = frio
            _salvar_cache_frio(cache)
            origem = "novo"
        quente = _correr_ga(200000 + k, nd, ne, alg, seed, lotes_gdf, max_x, max_y, pop=pop_ml)

        linha = {"cenario": [nd, ne, alg], "seed": seed, "frio": frio, "ml": quente}
        txt = (f"   {nd}d/{ne}e [frio:{origem}] gerações {frio['geracoes']} → ML {quente['geracoes']}"
               f" | makespan {frio['makespan']} → {quente['makespan']}")
        if com_geo:
            pg = prever_alocacao_geometrica(missoes, cd_xy, max_x, max_y, nd, perfis, max_entregas)
            np.random.seed(seed + 1)
            pop_g = gerar_populacao_inicial(pg, tam_pop, nd)
            geo = _correr_ga(300000 + k, nd, ne, alg, seed, lotes_gdf, max_x, max_y, pop=pop_g)
            linha["geo"] = geo
            txt += f" | geo {geo['geracoes']} ger / {geo['makespan']}"
        res.append(linha)
        print(txt)

    def media(chave, campo):
        return float(np.mean([r[chave][campo] for r in res]))

    resumo = {"n_cenarios": len(res),
              "geracoes_frio": media("frio", "geracoes"), "geracoes_ml": media("ml", "geracoes"),
              "reducao_geracoes_ml_pct": _pct(media("frio", "geracoes"), media("ml", "geracoes")),
              "makespan_frio": media("frio", "makespan"), "makespan_ml": media("ml", "makespan"),
              "ml_makespan_igual_ou_melhor": int(sum(r["ml"]["makespan"] <= r["frio"]["makespan"] for r in res)),
              "tempo_frio_s": media("frio", "tempo_s"), "tempo_ml_s": media("ml", "tempo_s")}
    if com_geo:
        resumo.update(geracoes_geo=media("geo", "geracoes"),
                      reducao_geracoes_geo_pct=_pct(media("frio", "geracoes"), media("geo", "geracoes")),
                      makespan_geo=media("geo", "makespan"),
                      geo_makespan_igual_ou_melhor=int(sum(r["geo"]["makespan"] <= r["frio"]["makespan"] for r in res)))
    resumo["detalhe"] = res

    print(f"\n🏁 Redução média de gerações (ML vs frio): {resumo['reducao_geracoes_ml_pct']}% "
          f"| makespan ML ≤ frio em {resumo['ml_makespan_igual_ou_melhor']}/{len(res)} cenários")
    if com_geo:
        print(f"🏁 Redução média de gerações (k-means vs frio): {resumo['reducao_geracoes_geo_pct']}% "
              f"| makespan ≤ frio em {resumo['geo_makespan_igual_ou_melhor']}/{len(res)}")
    print("   (menos gerações só conta como ganho se o makespan não piorar)")
    with open(BENCH_PATH, "w") as f:
        json.dump(resumo, f, indent=2, default=_json_default)
    return resumo


# =====================================================================
# DIAGNÓSTICO — consistência do próprio GA (teto realista do ARI)
# =====================================================================
def teste_consistencia_ga(n=5):
    from sklearn.metrics import adjusted_rand_score
    cfg.CENARIO_SEMPRE_NOVO = False
    random.seed(CIDADE_SEED)
    np.random.seed(CIDADE_SEED)
    lotes_gdf, max_x, max_y = gerar_cidade()
    aris = []
    print(f"\n🔬 Consistência do GA: {n} cenários, 2 seeds de GA cada")
    for k in range(n):
        nd, ne, alg = MATRIZ_TESTES_IA[k % len(MATRIZ_TESTES_IA)]
        seed = SEED_BENCH + 5000 + k
        vs = []
        for j, sg in enumerate((seed, seed + 7777)):
            _, _, cen = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
            v = executar_otimizacao_batch(400000 + 2 * k + j, nd, ne, alg, cenario=cen, seed_ga=sg)[0]
            vs.append(np.asarray(v[:ne], dtype=int))
        ari = float(adjusted_rand_score(vs[0], vs[1]))
        aris.append(ari)
        print(f"   {nd}d/{ne}e: ARI entre as duas soluções = {ari:.3f}")
    print(f"🔬 ARI médio GA vs GA: {np.mean(aris):.3f}  (teto realista para o modelo)")
    with open("relatorio_consistencia_ga.json", "w") as f:
        json.dump({"ari_medio": float(np.mean(aris)), "aris": aris}, f, indent=2)
    return aris


# =====================================================================
# ORQUESTRAÇÃO
# =====================================================================
def _modelo_desatualizado():
    if not os.path.exists(MODELO_PATH):
        return True
    import joblib
    try:
        if joblib.load(MODELO_PATH).get("versao_alvo") != VERSAO_ALVO:
            return True
    except Exception:
        return True
    return os.path.getmtime(DATASET_CSV) > os.path.getmtime(MODELO_PATH)


def modo_auto(lote, max_amostras, paciencia, melhoria_min, max_horas):
    """Gera em lotes, retreina e valida a cada lote; para quando o ARI estagna."""
    t_ini = time.time()
    melhor_ari, sem_melhoria, n_lote = -math.inf, 0, 0
    print(f"\n🔁 MODO AUTO: lotes de {lote} | máx {max_amostras} amostras | "
          f"paciência {paciencia} (melhoria mínima de ARI {melhoria_min})"
          + (f" | máx {max_horas}h" if max_horas else ""))
    try:
        while True:
            antes = carregar_ultima_amostra()
            if antes + 1 >= max_amostras:
                print(f"🛑 Limite de {max_amostras} amostras atingido.")
                break
            if max_horas and (time.time() - t_ini) / 3600 >= max_horas:
                print(f"🛑 Limite de tempo ({max_horas}h) atingido.")
                break

            n_lote += 1
            alvo = min(antes + 1 + lote, max_amostras)
            print(f"\n{'#' * 60}\n# LOTE {n_lote}: amostras {antes + 1} → {alvo - 1}\n{'#' * 60}")
            gerar_dataset(alvo)

            if carregar_ultima_amostra() == antes:
                print("🛑 O lote não produziu amostras novas (todas falharam). Verifique os erros acima.")
                break

            rel = treinar_e_validar()
            if rel is None:
                continue
            ari = rel["ari_agrupamento_medio"]
            if ari - melhor_ari > melhoria_min:
                ant = "primeiro lote" if melhor_ari == -math.inf else f"{melhor_ari:.4f}"
                print(f"📈 ARI {ant} → {ari:.4f}  (melhorou)")
                melhor_ari, sem_melhoria = ari, 0
            else:
                sem_melhoria += 1
                melhor_ari = max(melhor_ari, ari)
                print(f"➖ ARI {ari:.4f} sem melhoria relevante ({sem_melhoria}/{paciencia})")
                if sem_melhoria >= paciencia:
                    print("🛑 Estagnou: mais amostras já não melhoram a validação.")
                    break
    except KeyboardInterrupt:
        print("\n⏸️ Interrompido pelo utilizador. O dataset gravado está intacto.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--amostras", type=int, default=50)
    ap.add_argument("--retreinar", action="store_true")
    ap.add_argument("--sem-benchmark", action="store_true")
    ap.add_argument("--so-benchmark", action="store_true",
                    help="não gera nem treina; só corre o benchmark (frio vem do cache)")
    ap.add_argument("--n-bench", type=int, default=N_BENCHMARK)
    ap.add_argument("--refazer-frio", action="store_true",
                    help="ignora o cache e volta a correr o GA a frio")
    ap.add_argument("--com-geometrico", action="store_true",
                    help="inclui warm start por k-means como terceira referência")
    ap.add_argument("--consistencia", action="store_true",
                    help="mede o ARI do GA contra si próprio (seeds diferentes) e sai")
    ap.add_argument("--auto", action="store_true",
                    help="gera em lotes, retreina e valida até a métrica estagnar")
    ap.add_argument("--lote", type=int, default=50)
    ap.add_argument("--max-amostras", type=int, default=500)
    ap.add_argument("--paciencia", type=int, default=2)
    ap.add_argument("--melhoria-min", type=float, default=0.01)
    ap.add_argument("--max-horas", type=float, default=None)
    args = ap.parse_args()

    if args.consistencia:
        teste_consistencia_ga()
        return

    if args.so_benchmark:
        benchmark_convergencia(n_bench=args.n_bench, refazer_frio=args.refazer_frio,
                               com_geo=args.com_geometrico)
        return

    if args.auto:
        modo_auto(args.lote, args.max_amostras, args.paciencia,
                  args.melhoria_min, args.max_horas)
    else:
        gerar_dataset(args.amostras)

    if os.path.exists(DATASET_CSV) and (args.retreinar or _modelo_desatualizado()):
        treinar_e_validar()
    else:
        print("✅ Modelo já atualizado em relação ao dataset (use --retreinar para forçar).")

    if not args.sem_benchmark and os.path.exists(MODELO_PATH):
        benchmark_convergencia(n_bench=args.n_bench, refazer_frio=args.refazer_frio,
                               com_geo=args.com_geometrico)


if __name__ == "__main__":
    main()