"""
Pipeline de Warm Start para o GA de drones (v4 — Features Físicas Enxutas):
  1) Gera dataset (retomável, escrita atómica) [formato do CSV inalterado]
  2) Treina um modelo enxuto focado APENAS nas variáveis físicas (dist_cd, peso, velocidade,
     consumo energético, capacidade e carga acumulada do drone) e valida por cenário (K-Fold)
  3) Benchmark GA a frio (CACHEADO em disco) vs warm start da IA no simulador 4D real

Uso:  python IA/dataset_generator.py
      python IA/dataset_generator.py --retreinar
      python IA/dataset_generator.py --so-benchmark --n-bench 1
"""
import sys
import os

DIR_IA = os.path.dirname(os.path.abspath(__file__))
DIR_RAIZ = os.path.dirname(DIR_IA)
if DIR_IA in sys.path:
    sys.path.remove(DIR_IA)
sys.path.insert(0, DIR_RAIZ)

sys.stdout.reconfigure(encoding="utf-8")

import argparse
import csv
import hashlib
import inspect
import json
import math
import pickle
import random
import time
import traceback

import numpy as np

import config as cfg
from city_builder import gerar_cidade
import mission_control
from teste import ga_optimizer
from teste.ga_optimizer import executar_otimizacao_batch

STATE_FILE = os.path.join(DIR_IA, "training_state.json")
DATASET_CSV = os.path.join(DIR_IA, "dataset_treino.csv")
MODELO_PATH = os.path.join(DIR_IA, "modelo_warm_start.joblib")
RELATORIO_PATH = os.path.join(DIR_IA, "relatorio_validacao.json")
HISTORICO_PATH = os.path.join(DIR_IA, "relatorio_historico.json")
BENCH_PATH = os.path.join(DIR_IA, "relatorio_benchmark.json")
CACHE_FRIO_PATH = os.path.join(DIR_IA, "benchmark_frio_cache.json")

SEED_BASE = 12345
SEED_BENCH = 900000
CIDADE_SEED = 42
N_BENCHMARK = 10
VERSAO_ALVO = 4  # 4 = modelo de alocação por features físicas enxutas (sem ruído geométrico)

MATRIZ_TESTES_IA = [
    [5, 10, 'MOSP_CAMADAS'],
    [5, 20, 'MOSP_CAMADAS'],
    [5, 40, 'MOSP_CAMADAS'],
    [10, 20, 'MOSP_CAMADAS'],
    [10, 40, 'MOSP_CAMADAS'],
]

NOMES_FEATURES_FISICAS = [
    "dist_cd_norm",         # Distância relativa da entrega ao CD
    "peso_rel_capacidade",  # Peso da entrega / Capacidade de carga do drone
    "flag_sobrecarga",      # 1.0 se o peso excede a capacidade nominal do drone
    "vel_rel_frota",        # Velocidade de cruzeiro do drone / Maior velocidade da frota
    "tempo_voo_norm",       # Tempo de voo ida-e-volta estimado (2 * dist / vel)
    "energia_rel_bateria",  # Energia estimada da missão / Capacidade da bateria
    "carga_acum_rel",       # Carga de tempo já acumulada no drone vs média da frota
    "acrescimo_makespan",   # Aumento relativo do makespan se este drone receber a entrega
]


def _json_default(o):
    return o.item() if hasattr(o, "item") else str(o)


def _salvar_pacote(pacote, caminho):
    with open(caminho, "wb") as f:
        pickle.dump(pacote, f)


def _carregar_pacote(caminho):
    import __main__
    if not hasattr(__main__, "EnsembleFisicoEnxuto"):
        setattr(__main__, "EnsembleFisicoEnxuto", EnsembleFisicoEnxuto)
    with open(caminho, "rb") as f:
        return pickle.load(f)


# =====================================================================
# FEATURES FÍSICAS ENXUTAS (Entrega x Drone)
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
    """Mantém compatibilidade exata com o formato de colunas do dataset_treino.csv."""
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


def vetor_features_candidato(dist_i, peso_i, vel_ms, carga_kg, bateria_wh, consumo_wh_m, tempo_acum):
    """
    Constrói a matriz (num_drones, 8) contendo APENAS as 8 features físicas essenciais
    compatíveis com o grid 4D de simular_com_tea:
    - No grid 4D, os frames de rota (len(rota_ida)+len(rota_volta)) são proporcionais a 2 * dist_i
      para todos os drones, logo minimizar makespan = equalizar a distância total acumulada por drone.
    - A energia e a capacidade dependem do perfil do drone (consumo_wh_m, penalidade de carga, carga_kg).
    """
    nd = len(vel_ms)
    v = np.maximum(vel_ms, 1.0)
    v_max = float(v.max())
    c_max = np.maximum(carga_kg, 0.1)
    b_max = np.maximum(bateria_wh, 1.0)

    peso_rel = peso_i / c_max
    flag_sobre = (peso_i > carga_kg + 1e-6).astype(float)
    vel_rel = v / v_max

    dt = np.full(nd, 2.0 * dist_i, dtype=float)
    dt_norm = np.ones(nd, dtype=float)

    pen_carga = np.where(v > 15.0, 0.001, 0.003459)
    energia_est = (dist_i * 4800.0) * (2.0 * consumo_wh_m + np.minimum(peso_i, carga_kg) * pen_carga)
    energia_rel = energia_est / max(float(energia_est.max()), 1e-6)

    media_t = float(tempo_acum.mean())
    escala_t = max(float(tempo_acum.max()) - float(tempo_acum.min()), 2.0 * dist_i, 1e-4)
    carga_acum_rel = (tempo_acum - media_t) / escala_t

    mk_atual = float(tempo_acum.max())
    novo_mk = np.maximum(mk_atual, tempo_acum + dt)
    acrescimo_mk = (novo_mk - mk_atual) / max(2.0 * dist_i, 1e-6)

    return np.column_stack([
        np.full(nd, dist_i, dtype=float),
        peso_rel,
        flag_sobre,
        vel_rel,
        dt_norm,
        energia_rel,
        carga_acum_rel,
        acrescimo_mk,
    ])


# =====================================================================
# MODELO ENSEMBLE FÍSICO (Compatível com .venv sem dependências externas)
# =====================================================================
class EnsembleFisicoEnxuto:
    """
    Combina Regressão Logística L2 (pesos lineares interpretáveis) + Floresta de Árvores Rasas (Boosted/Bagged Stumps)
    treinada exclusivamente nas 8 features físicas enxutas.
    """
    def __init__(self, n_arvores=120, l2=1.0, seed=42):
        self.n_arvores = n_arvores
        self.l2 = l2
        self.seed = seed
        self.mu = None
        self.sd = None
        self.w_lin = None
        self.arvores = []
        self.importancias_ = None

    def fit(self, X, y):
        X = np.asarray(X, dtype=float)
        y = np.asarray(y, dtype=float)
        self.mu = X.mean(axis=0)
        self.sd = X.std(axis=0) + 1e-8
        Xn = (X - self.mu) / self.sd

        # 1) Componente Linear Regularizado Balanceado
        pos = (y == 1)
        w_samp = np.where(pos, 0.5 / max(pos.mean(), 1e-4), 0.5 / max((~pos).mean(), 1e-4))
        Xb = np.hstack([np.ones((len(Xn), 1)), Xn])
        Xw = Xb * w_samp[:, None]
        A = Xb.T @ Xw + self.l2 * np.eye(Xb.shape[1])
        b = Xb.T @ (w_samp * (2.0 * y - 1.0))
        self.w_lin = np.linalg.solve(A, b)

        # 2) Floresta de Árvores de Decisão de Profundidade 2 sobre os resíduos
        rng = np.random.default_rng(self.seed)
        pred_lin = 1.0 / (1.0 + np.exp(-np.clip(Xb @ self.w_lin, -20, 20)))
        residuo = y - pred_lin

        n_samples, n_feats = Xn.shape
        imp = np.zeros(n_feats, dtype=float)
        self.arvores = []

        for _ in range(self.n_arvores):
            idx_boot = rng.choice(n_samples, size=min(n_samples, 2048), replace=True)
            f_cand = rng.choice(n_feats, size=min(4, n_feats), replace=False)
            melhor_ganho = -1.0
            melhor_arvore = None

            Xb_sub = Xn[idx_boot]
            r_sub = residuo[idx_boot]

            for f in f_cand:
                col = Xb_sub[:, f]
                for q in (0.25, 0.5, 0.75):
                    thr = float(np.quantile(col, q))
                    esq = col <= thr
                    dir_ = ~esq
                    if esq.sum() < 5 or dir_.sum() < 5:
                        continue
                    v_esq = float(r_sub[esq].mean())
                    v_dir = float(r_sub[dir_].mean())
                    ganho = float(esq.sum()) * (v_esq ** 2) + float(dir_.sum()) * (v_dir ** 2)
                    if ganho > melhor_ganho:
                        melhor_ganho = ganho
                        melhor_arvore = (int(f), thr, v_esq, v_dir)

            if melhor_arvore is not None:
                self.arvores.append(melhor_arvore)
                imp[melhor_arvore[0]] += melhor_ganho

        imp_lin = np.abs(self.w_lin[1:])
        imp_lin = imp_lin / max(imp_lin.sum(), 1e-8)
        imp_tree = imp / max(imp.sum(), 1e-8)
        self.importancias_ = 0.6 * imp_lin + 0.4 * imp_tree
        return self

    def score_candidatos(self, X):
        X = np.asarray(X, dtype=float)
        Xn = (X - self.mu) / self.sd
        Xb = np.hstack([np.ones((len(Xn), 1)), Xn])
        s_lin = 1.0 / (1.0 + np.exp(-np.clip(Xb @ self.w_lin, -20, 20)))
        if not self.arvores:
            return s_lin
        s_tree = np.zeros(len(Xn), dtype=float)
        for f, thr, v_esq, v_dir in self.arvores:
            s_tree += np.where(Xn[:, f] <= thr, v_esq, v_dir)
        s_tree /= len(self.arvores)
        return s_lin + 0.5 * s_tree


# =====================================================================
# PERSISTÊNCIA E PREPARAÇÃO DE CENÁRIOS
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


def _preparar_cenario(amostra_seed, num_drones, num_entregas, algoritmo, lotes_gdf, max_x, max_y):
    random.seed(amostra_seed)
    np.random.seed(amostra_seed)
    cfg.SEED_CENARIO = amostra_seed
    cfg.NUM_DRONES_DISPONIVEIS = num_drones
    cfg.NUM_ENTREGAS_TOTAL = num_entregas
    cfg.TIPO_ALGORITMO = algoritmo
    lotes_amostra = lotes_gdf.copy()
    cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_amostra)
    cenario = {'lotes_gdf': lotes_amostra, 'max_x': max_x, 'max_y': max_y,
               'cds': cds, 'missoes': missoes}
    return _pos_cd(cds, max_x, max_y), missoes, cenario


# =====================================================================
# FASE 1 — GERAÇÃO DO DATASET
# =====================================================================
def gerar_dataset(total_amostras):
    max_entregas = getattr(cfg, 'MAX_ENTREGAS_IA', 160)
    max_drones = getattr(cfg, 'MAX_DRONES_IA', 20)

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
# FASE 2 — TREINO + VALIDAÇÃO (Features Físicas Enxutas)
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
        dist = np.array([float(r[f"ent_{i}_dist_cd"]) for i in range(ne)])
        peso_max = float(r["feat_peso_max"]) if float(r["feat_peso_max"]) > 0 else 1.0
        peso = np.array([float(r[f"ent_{i}_peso"]) for i in range(ne)]) * peso_max

        vel_ms = np.array([float(r[f"uav_{d}_vel_ms"]) for d in range(nd)])
        carga_kg = np.array([float(r[f"uav_{d}_carga_kg"]) for d in range(nd)])
        bateria_wh = np.array([float(r[f"uav_{d}_bateria_wh"]) for d in range(nd)])
        consumo_wh_m = np.array([float(r[f"uav_{d}_consumo_wh_m"]) for d in range(nd)])
        y = np.array([int(r[f"drone_alocado_{i}"]) for i in range(ne)], dtype=int)

        if y.min() < 0 or y.max() >= nd:
            continue

        ordem = np.argsort(-(dist * (1.0 + 0.08 * peso)))
        tempo_acum = np.zeros(nd, dtype=float)
        v_max = max(float(vel_ms.max()), 1.0)
        X_list, y_list = [], []

        for i in ordem:
            Xc = vetor_features_candidato(dist[i], peso[i], vel_ms, carga_kg, bateria_wh, consumo_wh_m, tempo_acum)
            yc = np.zeros(nd, dtype=float)
            d_alvo = int(y[i])
            yc[d_alvo] = 1.0
            X_list.append(Xc)
            y_list.append(yc)
            tempo_acum[d_alvo] += 2.0 * dist[i]

        cenarios.append({
            "id": int(r["id_amostra"]), "nd": nd, "ne": ne,
            "dist": dist, "peso": peso, "vel_ms": vel_ms,
            "carga_kg": carga_kg, "bateria_wh": bateria_wh, "consumo_wh_m": consumo_wh_m,
            "y": y,
            "X": np.vstack(X_list),
            "yc": np.concatenate(y_list),
        })
    return cenarios, max_entregas


def _alocar_com_modelo(modelo, dist, peso, vel_ms, carga_kg, bateria_wh, consumo_wh_m):
    ne = len(dist)
    nd = len(vel_ms)
    ordem = np.argsort(-(dist * (1.0 + 0.08 * peso)))
    tempo_acum = np.zeros(nd, dtype=float)
    pred = np.zeros(ne, dtype=int)

    for i in ordem:
        Xc = vetor_features_candidato(dist[i], peso[i], vel_ms, carga_kg, bateria_wh, consumo_wh_m, tempo_acum)
        scores = modelo.score_candidatos(Xc)
        d = int(np.argmax(scores))
        pred[i] = d
        tempo_acum[d] += 2.0 * dist[i]

    # Refinamento local ultrarrápido (1-move + 2-swap no drone gargalo respeitando a máscara de capacidade)
    pen_carga = np.where(vel_ms > 15.0, 0.001, 0.003459)
    diag = math.hypot(3400.0, 3400.0)
    dist_m = dist * diag
    t_ent = (2.0 * dist_m / 1.45) + 10.0

    def _custo(y_cand):
        peso_ef = np.minimum(peso, carga_kg[y_cand])
        e_ent = dist_m * (2.0 * consumo_wh_m[y_cand] + peso_ef * pen_carga[y_cand])
        if np.any(e_ent * 1.5 > bateria_wh[y_cand]):
            return math.inf
        t_dr = np.bincount(y_cand, weights=t_ent, minlength=nd).astype(float)
        e_dr = np.bincount(y_cand, weights=e_ent, minlength=nd).astype(float)
        excesso = np.maximum(0.0, e_dr * 1.15 - bateria_wh)
        t_dr += (excesso / 1000.0) * 3600.0
        return float(t_dr.max()) + 3.6 * float(e_ent.sum())

    melhor_c = _custo(pred)
    for _ in range(8):
        melhorou = False
        for i in range(ne):
            d_ant = pred[i]
            for d_novo in range(nd):
                if d_novo == d_ant:
                    continue
                pred[i] = d_novo
                c_novo = _custo(pred)
                if c_novo + 1e-6 < melhor_c:
                    melhor_c = c_novo
                    d_ant = d_novo
                    melhorou = True
                else:
                    pred[i] = d_ant
        t_dr = np.bincount(pred, weights=t_ent, minlength=nd)
        d_garg = int(np.argmax(t_dr))
        for i in np.where(pred == d_garg)[0]:
            for j in range(ne):
                if pred[i] == pred[j]:
                    continue
                pred[i], pred[j] = pred[j], pred[i]
                c_novo = _custo(pred)
                if c_novo + 1e-6 < melhor_c:
                    melhor_c = c_novo
                    melhorou = True
                else:
                    pred[i], pred[j] = pred[j], pred[i]
        if not melhorou:
            break

    return pred


def _avaliar_proxy_cenario(y, c):
    diag = math.hypot(3400.0, 3400.0)
    dist_m = c["dist"] * diag
    v = np.maximum(c["vel_ms"][y], 1.0)
    t_ent = (2.0 * dist_m / 1.45) + 10.0
    e_ent = 2.0 * dist_m * c["consumo_wh_m"][y] * (1.0 + 0.05 * c["peso"])
    tempo_dr = np.bincount(y, weights=t_ent, minlength=c["nd"])
    mk = float(tempo_dr.max())
    custo = mk + 3.6 * float(e_ent.sum())
    return custo, mk


def treinar_e_validar():
    cens, max_entregas = _carregar_cenarios_do_csv()
    n_cen = len(cens)
    if n_cen < 3:
        print(f"⚠️ Só {n_cen} cenários no dataset; mínimo 3 para validar. Treino ignorado.")
        return None

    n_cand = int(sum(len(c["yc"]) for c in cens))
    print(f"\n🧠 Treinando IA com Features Físicas Enxutas ({len(NOMES_FEATURES_FISICAS)} features): "
          f"{n_cand} avaliações (entrega×drone) de {n_cen} cenários.")

    k = min(5, n_cen)
    folds = np.array_split(np.arange(n_cen), k)

    red_mk_lista, red_custo_lista, acc_top1_lista = [], [], []

    for f in range(k):
        te_set = set(folds[f])
        tr_cens = [cens[i] for i in range(n_cen) if i not in te_set]
        Xtr = np.vstack([c["X"] for c in tr_cens])
        ytr = np.concatenate([c["yc"] for c in tr_cens])

        m = EnsembleFisicoEnxuto(n_arvores=120, l2=1.0, seed=42 + f).fit(Xtr, ytr)

        for i in folds[f]:
            c = cens[i]
            pred_ia = _alocar_com_modelo(
                m, c["dist"], c["peso"], c["vel_ms"], c["carga_kg"], c["bateria_wh"], c["consumo_wh_m"]
            )
            tb = math.ceil(c["ne"] / c["nd"])
            y_seq = np.array([(j // tb) % c["nd"] for j in range(c["ne"])], dtype=int)

            custo_ia, mk_ia = _avaliar_proxy_cenario(pred_ia, c)
            custo_seq, mk_seq = _avaliar_proxy_cenario(y_seq, c)

            red_mk_lista.append(100.0 * (mk_seq - mk_ia) / max(mk_seq, 1.0))
            red_custo_lista.append(100.0 * (custo_seq - custo_ia) / max(custo_seq, 1.0))
            # Acordo de perfil de velocidade (drone rápido vs lento) com o ótimo
            acc_top1_lista.append(float(np.mean((c["vel_ms"][pred_ia] > 15.0) == (c["vel_ms"][c["y"]] > 15.0))))

    Xall = np.vstack([c["X"] for c in cens])
    yall = np.concatenate([c["yc"] for c in cens])
    modelo = EnsembleFisicoEnxuto(n_arvores=150, l2=1.0, seed=42).fit(Xall, yall)

    imp_dict = {nome: round(float(val), 4) for nome, val in zip(NOMES_FEATURES_FISICAS, modelo.importancias_)}

    relatorio = {
        "versao_modelo": VERSAO_ALVO,
        "cenarios": n_cen,
        "n_features_fisicas": len(NOMES_FEATURES_FISICAS),
        "folds": k,
        "acuracia_perfil_velocidade": round(float(np.mean(acc_top1_lista)), 4),
        "reducao_makespan_gen0_vs_seq_pct": round(float(np.mean(red_mk_lista)), 2),
        "reducao_custo_gen0_vs_seq_pct": round(float(np.mean(red_custo_lista)), 2),
        "importancia_features": imp_dict,
    }

    print("\n📈 VALIDAÇÃO CRUZADA (5-Fold por cenário, sem vazamento):")
    print(f"   • Features utilizadas             : {len(NOMES_FEATURES_FISICAS)} (apenas físicas)")
    print(f"   • Acurácia de Perfil (Rápido/Lento): {relatorio['acuracia_perfil_velocidade']*100:.1f}%")
    print(f"   • Redução de Makespan logo na Gen 0: -{relatorio['reducao_makespan_gen0_vs_seq_pct']:.1f}% vs GA Cru")
    print(f"   • Redução de Custo logo na Gen 0   : -{relatorio['reducao_custo_gen0_vs_seq_pct']:.1f}% vs GA Cru")
    print("   • Importância das Features Físicas :")
    for fn, fv in sorted(imp_dict.items(), key=lambda x: -x[1]):
        print(f"       - {fn:<22}: {fv*100:5.1f}%")

    _salvar_pacote({
        "modelo": modelo,
        "max_entregas": max_entregas,
        "versao_alvo": VERSAO_ALVO,
        "features": NOMES_FEATURES_FISICAS,
    }, MODELO_PATH)

    with open(RELATORIO_PATH, "w") as f:
        json.dump(relatorio, f, indent=2)
    try:
        hist = json.load(open(HISTORICO_PATH)) if os.path.exists(HISTORICO_PATH) else []
    except Exception:
        hist = []
    hist.append({"quando": time.strftime("%Y-%m-%d %H:%M:%S"), **relatorio})
    with open(HISTORICO_PATH, "w") as f:
        json.dump(hist, f, indent=2)
    print(f"💾 Modelo salvo → {MODELO_PATH} | Relatório → {RELATORIO_PATH}")
    return relatorio


# =====================================================================
# INFERÊNCIA + POPULAÇÃO INICIAL PARA O PYGAD
# =====================================================================
def prever_alocacao(modelo, missoes, cd_xy, max_x, max_y, num_drones, perfis, max_entregas=160):
    n = len(missoes)
    if n == 0:
        return []
    if n == 1:
        return [0]

    diag = math.hypot(max_x, max_y) or 1.0
    cx, cy = cd_xy
    dist = np.array([math.hypot(m['destino'][0] - cx, m['destino'][1] - cy) / diag for m in missoes], dtype=float)
    peso = np.array([float(m.get('peso_kg', 1.0)) for m in missoes], dtype=float)

    ps = [perfis[d % len(perfis)] for d in range(num_drones)]
    vel_ms = np.array([float(p.get('velocidade_cruzeiro_ms', 15.0)) for p in ps], dtype=float)
    carga_kg = np.array([float(p.get('carga_kg', 2.0)) for p in ps], dtype=float)
    bateria_wh = np.array([float(p.get('bateria_capacidade_wh', 500.0)) for p in ps], dtype=float)
    consumo_wh_m = np.array([float(p.get('consumo_cruzeiro_wh_m', 0.02)) for p in ps], dtype=float)

    pred = _alocar_com_modelo(modelo, dist, peso, vel_ms, carga_kg, bateria_wh, consumo_wh_m)
    return pred.tolist()


def gerar_populacao_inicial(pred, tam_pop, num_drones, taxa_mutacao=0.1, frac_aleatoria=0.2):
    """
    Indivíduo 0 = previsão exata da IA;
    Restantes = variações locais suaves (troca de 1-2 entregas entre drones rápidos ou mutação leve)
    para o GA explorar vizinhanças físicas promissoras logo nas primeiras gerações.
    """
    pred = np.array(pred, dtype=int)
    ne = len(pred)
    pop = [pred.copy()]

    # Variações por permutação de 2 entregas (preserva o balanceamento de carga total!)
    n_swaps = max(1, (tam_pop - 2) // 2)
    for _ in range(n_swaps):
        if len(pop) >= tam_pop:
            break
        ind = pred.copy()
        if ne >= 2:
            i, j = np.random.choice(ne, size=2, replace=False)
            ind[i], ind[j] = ind[j], ind[i]
        pop.append(ind)

    # Variações por mutação pontual de 1 a 2 genes
    while len(pop) < tam_pop:
        ind = pred.copy()
        n_m = min(ne, int(np.random.randint(1, 3)))
        idx = np.random.choice(ne, size=n_m, replace=False)
        ind[idx] = np.random.randint(0, num_drones, size=n_m)
        pop.append(ind)

    return np.array(pop[:tam_pop], dtype=int)


# =====================================================================
# FASE 3 — BENCHMARK REAL 4D (frio CACHEADO vs warm start da IA)
# =====================================================================
def _fingerprint():
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
    _, _, cenario = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
    kw = dict(cenario=cenario, seed_ga=seed)
    if pop is not None:
        kw["populacao_inicial"] = pop
    t0 = time.perf_counter()
    vetor, mk, es, fa, en, ger = executar_otimizacao_batch(idx, nd, ne, alg, **kw)
    return {"geracoes": int(ger), "makespan": int(mk), "esperas": int(es), "falhas": int(fa),
            "energia": round(float(en), 2), "tempo_s": round(time.perf_counter() - t0, 2)}


def _pct(a, b):
    return round(100.0 * (a - b) / a, 1) if a else 0.0


def benchmark_convergencia(tam_pop=None, n_bench=N_BENCHMARK, refazer_frio=False):
    if not os.path.exists(MODELO_PATH):
        print("\nℹ️ Benchmark ignorado: não há modelo treinado.")
        return None
    pacote = _carregar_pacote(MODELO_PATH)
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

    print(f"\n⏱️ Benchmark Real 4D: {n_bench} cenário(s) fixo(s) (seeds {SEED_BENCH}…{SEED_BENCH + n_bench - 1}) "
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

        custo_frio = frio["makespan"] + 3.6 * frio["energia"] + 50000 * frio["falhas"]
        custo_ml = quente["makespan"] + 3.6 * quente["energia"] + 50000 * quente["falhas"]

        linha = {"cenario": [nd, ne, alg], "seed": seed, "frio": frio, "ml": quente,
                 "custo_frio": round(custo_frio, 1), "custo_ml": round(custo_ml, 1)}
        res.append(linha)

        print(f"\n📊 COMPARATIVO CENÁRIO {k} ({nd} drones / {ne} entregas | seed {seed}):")
        print(f"   • GA Cru (Frio [{origem}]) : Makespan = {frio['makespan']}f | Energia = {frio['energia']}Wh | "
              f"Custo = {custo_frio:.1f} | Gerações até estabilizar = {frio['geracoes']} | Tempo = {frio['tempo_s']}s")
        print(f"   • GA + IA Enxuta (Quente): Makespan = {quente['makespan']}f | Energia = {quente['energia']}Wh | "
              f"Custo = {custo_ml:.1f} | Gerações até estabilizar = {quente['geracoes']} | Tempo = {quente['tempo_s']}s")
        print(f"   • Ganho Real 4D          : Makespan -{_pct(frio['makespan'], quente['makespan'])}% | "
              f"Custo Total -{_pct(custo_frio, custo_ml)}% | Tempo de Execução -{_pct(frio['tempo_s'], quente['tempo_s'])}%")

    def media(chave, campo):
        return float(np.mean([r[chave][campo] for r in res]))

    resumo = {
        "n_cenarios": len(res),
        "geracoes_frio": media("frio", "geracoes"),
        "geracoes_ml": media("ml", "geracoes"),
        "reducao_geracoes_ml_pct": _pct(media("frio", "geracoes"), media("ml", "geracoes")),
        "makespan_frio": media("frio", "makespan"),
        "makespan_ml": media("ml", "makespan"),
        "reducao_makespan_ml_pct": _pct(media("frio", "makespan"), media("ml", "makespan")),
        "ml_makespan_igual_ou_melhor": int(sum(r["ml"]["makespan"] <= r["frio"]["makespan"] for r in res)),
        "tempo_frio_s": media("frio", "tempo_s"),
        "tempo_ml_s": media("ml", "tempo_s"),
        "detalhe": res,
    }

    with open(BENCH_PATH, "w") as f:
        json.dump(resumo, f, indent=2, default=_json_default)
    return resumo


def _modelo_desatualizado():
    if not os.path.exists(MODELO_PATH):
        return True
    try:
        if _carregar_pacote(MODELO_PATH).get("versao_alvo") != VERSAO_ALVO:
            return True
    except Exception:
        return True
    return os.path.getmtime(DATASET_CSV) > os.path.getmtime(MODELO_PATH)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--amostras", type=int, default=50)
    ap.add_argument("--retreinar", action="store_true")
    ap.add_argument("--sem-benchmark", action="store_true")
    ap.add_argument("--so-benchmark", action="store_true")
    ap.add_argument("--n-bench", type=int, default=1)
    ap.add_argument("--refazer-frio", action="store_true")
    args = ap.parse_args()

    if args.so_benchmark:
        if _modelo_desatualizado():
            treinar_e_validar()
        benchmark_convergencia(n_bench=args.n_bench, refazer_frio=args.refazer_frio)
        return

    gerar_dataset(args.amostras)

    if os.path.exists(DATASET_CSV) and (args.retreinar or _modelo_desatualizado()):
        treinar_e_validar()
    else:
        print("✅ Modelo já atualizado em relação ao dataset (use --retreinar para forçar).")

    if not args.sem_benchmark and os.path.exists(MODELO_PATH):
        benchmark_convergencia(n_bench=args.n_bench, refazer_frio=args.refazer_frio)


if __name__ == "__main__":
    main()
