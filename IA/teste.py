"""
Diagnóstico rápido (segundos, sem correr o GA): que estrutura há nas soluções do dataset?

Hipótese: com esperas=0 e falhas=0, o custo é dominado pelo makespan, que é a carga do
drone mais ocupado (soma dos tempos de ida e volta ao CD). Isto é balanceamento de carga,
não agrupamento espacial.

Por configuração (drones, entregas) compara o "makespan proxy" de:
  GA         solução gravada no dataset
  aleatório  média de 200 alocações aleatórias
  seq        blocos consecutivos (o "lance inicial sequencial" do GA)
  LPT        heurística gulosa (maior entrega primeiro, drone que terminaria mais cedo)
e a correlação de Spearman entre o proxy da solução do GA e o makespan_final real.

Uso: python diagnostico_proxy.py [dataset_treino.csv]
"""
import sys

import numpy as np
import pandas as pd
def spearmanr(a, b):
    ra = np.asarray(a).argsort().argsort().astype(float)
    rb = np.asarray(b).argsort().argsort().astype(float)
    return (float(np.corrcoef(ra, rb)[0, 1]), 0.0)


def proxy(y, dist, v):
    """Carga máxima por drone: soma de 2*dist/velocidade das entregas atribuídas."""
    cargas = np.bincount(y, weights=2.0 * dist / v[y], minlength=len(v))
    return float(cargas.max())


def lpt(dist, v):
    carga = np.zeros(len(v))
    y = np.zeros(len(dist), dtype=int)
    for i in np.argsort(-dist):
        d = int(np.argmin(carga + 2.0 * dist[i] / v))
        y[i] = d
        carga[d] += 2.0 * dist[i] / v[d]
    return y


def main(caminho):
    df = pd.read_csv(caminho)
    rng = np.random.default_rng(0)
    linhas = []
    for _, r in df.iterrows():
        nd, ne = int(r["num_drones_ativos"]), int(r["num_entregas_reais"])
        dist = np.array([r[f"ent_{i}_dist_cd"] for i in range(ne)], dtype=float)
        y = np.array([int(r[f"drone_alocado_{i}"]) for i in range(ne)])
        v = np.array([r[f"uav_{d}_vel_ms"] for d in range(nd)], dtype=float)
        v = np.where(v > 0, v, 1.0)
        if y.min() < 0 or y.max() >= nd:
            continue
        linhas.append({
            "nd": nd, "ne": ne,
            "ga": proxy(y, dist, v),
            "ale": float(np.mean([proxy(rng.integers(0, nd, ne), dist, v) for _ in range(200)])),
            "seq": proxy((np.arange(ne) * nd) // ne, dist, v),
            "lpt": proxy(lpt(dist, v), dist, v),
            "makespan": float(r["makespan_final"]),
        })
    res = pd.DataFrame(linhas)
    if res.empty:
        print("Sem amostras válidas no CSV.")
        return

    linhas_tab = []
    for (nd, ne), g in res.groupby(["nd", "ne"]):
        rho = spearmanr(g["ga"], g["makespan"])[0] if len(g) >= 4 else float("nan")
        linhas_tab.append({"drones": nd, "entregas": ne, "n": len(g),
                           "GA/aleat": (g.ga / g.ale).mean(),
                           "GA/seq": (g.ga / g.seq).mean(),
                           "LPT/GA": (g.lpt / g.ga).mean(),
                           "spearman": rho})
    tab = pd.DataFrame(linhas_tab)
    print("\nProxy de makespan (carga do drone mais ocupado), médias por configuração:\n")
    print(tab.to_string(index=False, float_format=lambda x: f"{x:.3f}"))
    print(f"\nGlobal: GA/aleat={np.mean(res.ga / res.ale):.3f} | GA/seq={np.mean(res.ga / res.seq):.3f} "
          f"| LPT/GA={np.mean(res.lpt / res.ga):.3f}")
    print("\nComo ler:")
    print("  GA/aleat ≈ 1      -> as soluções do GA são praticamente aleatórias (não otimizadas)")
    print("  LPT/GA < 1        -> a heurística de balanceamento já supera o GA no proxy")
    print("  spearman > 0.7    -> o proxy ordena bem o makespan real (proxy fiável)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "dataset_treino.csv")