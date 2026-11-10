"""
Reaproveita os 51 cenários reais já simulados em IA/dataset_treino.csv e corrige os rótulos
de alocação (drone_alocado_0 ... drone_alocado_159) eliminando o viés do chute sequencial [0,0,1,1...].
"""
import os
import sys
import shutil
import math
import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8")

DIR_IA = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(DIR_IA, "dataset_treino.csv")
BACKUP_ORIG_PATH = os.path.join(DIR_IA, "dataset_treino_original_ga.csv")


def precomputar_matrizes(dist_m, peso_kg, vel_ms, carga_kg, bateria_wh, consumo_wh_m, fator_t=1.0, fator_e=1.0):
    """
    Pré-calcula matrizes (ne, nd) de frames 4D, energia e penalidade exatamente como simular_com_tea:
    - No grid 4D (MOSP_CAMADAS), len(rota_ida) + len(rota_volta) depende da distância física em waypoints (2 * dist_m),
      enquanto a energia depende do consumo_wh_m e da penalidade de carga de cada perfil de drone.
    """
    TEMPO_DESCARGA = 5.0
    dist = dist_m[:, None]
    peso = np.minimum(peso_kg[:, None], carga_kg[None, :])
    pen_carga = np.where(vel_ms[None, :] > 15.0, 0.001, 0.003459)

    T_mat = ((2.0 * dist / 1.45) + (2.0 * TEMPO_DESCARGA)) * fator_t * np.ones((1, len(vel_ms)))
    E_mat = (dist * (2.0 * consumo_wh_m[None, :] + peso * pen_carga)) * fator_e
    F_mat = (E_mat * 1.5 > bateria_wh[None, :]).astype(int)
    return T_mat, E_mat, F_mat


def avaliar_rapido(y, T_mat, E_mat, F_mat, bateria_wh):
    ne, nd = T_mat.shape
    idx = np.arange(ne)
    t_ent = T_mat[idx, y]
    e_ent = E_mat[idx, y]
    f_ent = F_mat[idx, y]

    tempo_dr = np.bincount(y, weights=t_ent, minlength=nd).astype(float)
    energia_dr = np.bincount(y, weights=e_ent, minlength=nd).astype(float)
    # Tempo de recarga se o consumo acumulado do drone ultrapassar a bateria
    excesso_bat = np.maximum(0.0, energia_dr * 1.15 - bateria_wh)
    tempo_dr += (excesso_bat / 1000.0) * 3600.0

    makespan = float(tempo_dr.max())
    energia_total = float(e_ent.sum())
    falhas = int(f_ent.sum())
    custo = makespan + 3.6 * energia_total + falhas * 50000.0
    return custo, makespan, energia_total, falhas


def otimizar_cenario_fisico(dist_m, peso_kg, vel_ms, carga_kg, bateria_wh, consumo_wh_m,
                            fator_t=1.0, fator_e=1.0, seed=42):
    rng = np.random.default_rng(seed)
    ne = len(dist_m)
    nd = len(vel_ms)
    T_mat, E_mat, F_mat = precomputar_matrizes(
        dist_m, peso_kg, vel_ms, carga_kg, bateria_wh, consumo_wh_m, fator_t, fator_e
    )

    # 1. Ponto de partida construtivo ordenado por exigência física
    tempo_acum = np.zeros(nd, dtype=float)
    y = np.zeros(ne, dtype=int)
    ordem_ent = np.argsort(-(dist_m * (1.0 + 0.08 * peso_kg)))

    for i in ordem_ent:
        novo_mk = np.maximum(tempo_acum.max(), tempo_acum + T_mat[i])
        val = novo_mk + 0.15 * (tempo_acum + T_mat[i]) + 3.6 * E_mat[i] + 50000.0 * F_mat[i]
        d = int(np.argmin(val))
        y[i] = d
        tempo_acum[d] += T_mat[i, d]

    melhor_y = y.copy()
    melhor_custo, _, _, _ = avaliar_rapido(melhor_y, T_mat, E_mat, F_mat, bateria_wh)

    # 2. Busca Local (1-move + 2-swap restrito ao drone gargalo) com ILS
    for restart in range(5):
        atual_y = melhor_y.copy()
        if restart > 0:
            k_pert = min(ne, int(rng.integers(2, 4)))
            idx_p = rng.choice(ne, size=k_pert, replace=False)
            atual_y[idx_p] = rng.integers(0, nd, size=k_pert)

        atual_custo, _, _, _ = avaliar_rapido(atual_y, T_mat, E_mat, F_mat, bateria_wh)

        for _ in range(15):
            melhorou = False
            # 1-move em todas as entregas
            for i in range(ne):
                d_orig = atual_y[i]
                for d_novo in range(nd):
                    if d_novo == d_orig:
                        continue
                    atual_y[i] = d_novo
                    c_cand, _, _, _ = avaliar_rapido(atual_y, T_mat, E_mat, F_mat, bateria_wh)
                    if c_cand + 1e-6 < atual_custo:
                        atual_custo = c_cand
                        d_orig = d_novo
                        melhorou = True
                    else:
                        atual_y[i] = d_orig

            # 2-swap focado nas entregas do drone mais carregado
            idx_all = np.arange(ne)
            tempo_dr = np.bincount(atual_y, weights=T_mat[idx_all, atual_y], minlength=nd)
            d_gargalo = int(np.argmax(tempo_dr))
            ent_gargalo = np.where(atual_y == d_gargalo)[0]
            for i in ent_gargalo:
                for j in range(ne):
                    if atual_y[i] == atual_y[j]:
                        continue
                    atual_y[i], atual_y[j] = atual_y[j], atual_y[i]
                    c_cand, _, _, _ = avaliar_rapido(atual_y, T_mat, E_mat, F_mat, bateria_wh)
                    if c_cand + 1e-6 < atual_custo:
                        atual_custo = c_cand
                        melhorou = True
                    else:
                        atual_y[i], atual_y[j] = atual_y[j], atual_y[i]

            if not melhorou:
                break

        if atual_custo < melhor_custo:
            melhor_custo = atual_custo
            melhor_y = atual_y.copy()

    return avaliar_rapido(melhor_y, T_mat, E_mat, F_mat, bateria_wh), melhor_y


def corrigir_dataset():
    if not os.path.exists(BACKUP_ORIG_PATH):
        shutil.copy2(CSV_PATH, BACKUP_ORIG_PATH)
        print(f"📦 Backup do dataset original salvo em: {BACKUP_ORIG_PATH}")

    df = pd.read_csv(BACKUP_ORIG_PATH)
    diag_mapa = math.hypot(3400.0, 3400.0)

    ganhos_makespan = []
    ganhos_custo = []
    frac_rapidos_antes = []
    frac_rapidos_depois = []

    for idx, r in df.iterrows():
        nd, ne = int(r["num_drones_ativos"]), int(r["num_entregas_reais"])
        y_orig = np.array([int(r[f"drone_alocado_{i}"]) for i in range(ne)], dtype=int)

        dist_rel = np.array([float(r[f"ent_{i}_dist_cd"]) for i in range(ne)])
        dist_m = dist_rel * diag_mapa
        peso_max = float(r["feat_peso_max"]) if float(r["feat_peso_max"]) > 0 else 1.0
        peso_kg = np.array([float(r[f"ent_{i}_peso"]) for i in range(ne)]) * peso_max

        vel_ms = np.array([float(r[f"uav_{d}_vel_ms"]) for d in range(nd)])
        carga_kg = np.array([float(r[f"uav_{d}_carga_kg"]) for d in range(nd)])
        bateria_wh = np.array([float(r[f"uav_{d}_bateria_wh"]) for d in range(nd)])
        consumo_wh_m = np.array([float(r[f"uav_{d}_consumo_wh_m"]) for d in range(nd)])

        T0, E0, F0 = precomputar_matrizes(dist_m, peso_kg, vel_ms, carga_kg, bateria_wh, consumo_wh_m, 1.0, 1.0)
        _, mk_reta_orig, en_reta_orig, _ = avaliar_rapido(y_orig, T0, E0, F0, bateria_wh)
        mk_real_orig = float(r["makespan_final"])
        en_real_orig = float(r["energia_final"])

        fator_t = float(np.clip(mk_real_orig / max(mk_reta_orig, 1.0), 0.8, 3.0))
        fator_e = float(np.clip(en_real_orig / max(en_reta_orig, 1.0), 0.8, 3.0))

        custo_orig = mk_real_orig + 3.6 * en_real_orig + float(r["falhas_final"]) * 50000.0

        (custo_opt, mk_opt, en_opt, falhas_opt), y_opt = otimizar_cenario_fisico(
            dist_m, peso_kg, vel_ms, carga_kg, bateria_wh, consumo_wh_m,
            fator_t=fator_t, fator_e=fator_e, seed=100 + int(r["id_amostra"])
        )

        ganhos_makespan.append(100.0 * (mk_real_orig - mk_opt) / max(mk_real_orig, 1.0))
        ganhos_custo.append(100.0 * (custo_orig - custo_opt) / max(custo_orig, 1.0))
        frac_rapidos_antes.append(float(np.mean(vel_ms[y_orig] > 15.0)))
        frac_rapidos_depois.append(float(np.mean(vel_ms[y_opt] > 15.0)))

        for i in range(ne):
            df.at[idx, f"drone_alocado_{i}"] = int(y_opt[i])
        df.at[idx, "makespan_final"] = int(round(mk_opt))
        df.at[idx, "energia_final"] = round(float(en_opt), 2)
        df.at[idx, "falhas_final"] = int(falhas_opt)

    df.to_csv(CSV_PATH, index=False)
    print(f"✅ Dataset corrigido e salvo em: {CSV_PATH} ({len(df)} cenários reaproveitados)")
    print(f"   • Redução média de Makespan nas respostas-alvo : -{np.mean(ganhos_makespan):.1f}%")
    print(f"   • Redução média de Custo Total nas respostas   : -{np.mean(ganhos_custo):.1f}%")
    print(f"   • Uso dos drones rápidos (28 m/s) antes -> pós : {np.mean(frac_rapidos_antes)*100:.1f}% -> {np.mean(frac_rapidos_depois)*100:.1f}%")


if __name__ == "__main__":
    corrigir_dataset()
