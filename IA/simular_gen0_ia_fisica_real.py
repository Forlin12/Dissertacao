r"""
Simula com a FÍSICA 4D REAL (MOSP_CAMADAS + uav_physics.Drone) o novo chute inicial
da IA (Árvore de Decisão rasa n_arvores=12 + Regularização L2 Forte l2=150.0, sem busca local na Gen 0)
em todos os cenários e grava os resultados 100% reais em:
  - IA/relatorio_todos_cenarios_vm.json
  - IA/tabela_comparativa_completa.csv
  - IA/tabela_9_cenarios_completa.csv
"""
import os
import sys
import json
import csv
import time
import random
import numpy as np

DIR_IA = os.path.dirname(os.path.abspath(__file__))
DIR_RAIZ = os.path.dirname(DIR_IA)
_VENV_SP = os.path.join(DIR_RAIZ, ".venv", "Lib", "site-packages")
if os.path.isdir(_VENV_SP) and _VENV_SP not in sys.path:
    sys.path.insert(0, _VENV_SP)
if "" in sys.path:
    sys.path.remove("")
if DIR_IA in sys.path:
    sys.path.remove(DIR_IA)
if DIR_RAIZ in sys.path:
    sys.path.remove(DIR_RAIZ)
sys.path.insert(0, DIR_RAIZ)
try:
    os.chdir(DIR_RAIZ)
except Exception:
    pass

import config as cfg
from city_builder import gerar_cidade
from IA.dataset_generator import (
    _carregar_pacote,
    _carregar_cenarios_do_csv,
    _preparar_cenario,
    prever_alocacao,
    EnsembleFisicoEnxuto,
    MODELO_PATH,
    CIDADE_SEED,
)
from IA.teste_todos_cenarios_vm import (
    CENARIOS_TESTE,
    RELATORIO_TODOS_JSON,
    avaliar_vetor_4d_com_cache,
    exportar_tabela_completa,
    pct,
    salvar_json_atomico,
)


def main():
    print("=================================================================")
    print("🚀 SIMULADOR 4D FÍSICO REAL — AVALIANDO GEN(0) COM IA (DT + L2)")
    print("=================================================================", flush=True)

    pacote = _carregar_pacote(MODELO_PATH)
    modelo = pacote["modelo"]
    max_entregas = pacote.get("max_entregas", 160)

    _modelo_sem_grandes_cache = [None]
    def _obter_modelo_sem_grandes():
        if _modelo_sem_grandes_cache[0] is None:
            cens_csv, _ = _carregar_cenarios_do_csv()
            cens_menores = [c for c in cens_csv if c["id"] not in {5, 6, 7, 8, 14}]
            X_men = np.vstack([c["X"] for c in cens_menores])
            y_men = np.concatenate([c["yc"] for c in cens_menores])
            _modelo_sem_grandes_cache[0] = EnsembleFisicoEnxuto(n_arvores=12, l2=150.0, seed=42).fit(X_men, y_men)
        return _modelo_sem_grandes_cache[0]

    cfg.CENARIO_SEMPRE_NOVO = False
    random.seed(CIDADE_SEED)
    np.random.seed(CIDADE_SEED)
    lotes_gdf, max_x, max_y = gerar_cidade()
    perfis = cfg.PERFIS_DRONES

    with open(RELATORIO_TODOS_JSON, "r", encoding="utf-8") as f:
        relatorio = json.load(f)

    cenarios_ordenados = [x for x in CENARIOS_TESTE if not x["nome"].endswith("(B)")] + [x for x in CENARIOS_TESTE if x["nome"].endswith("(B)")]

    for idx, item in enumerate(cenarios_ordenados):
        seed, nd, ne, alg, nome = item["seed"], item["nd"], item["ne"], item["alg"], item["nome"]
        chave = f"{seed}|{nd}|{ne}|{alg}"
        reg_atual = relatorio["cenarios"].get(chave, {})

        cd_xy, missoes, cenario = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
        mod_usar = _obter_modelo_sem_grandes() if item.get("id_excluir_treino") else modelo
        vetor_ia = prever_alocacao(mod_usar, missoes, cd_xy, max_x, max_y, nd, perfis, max_entregas)

        print(f"\n[{idx+1}/{len(CENARIOS_TESTE)}] Simulando FÍSICA 4D REAL para {nome} (seed={seed})...", flush=True)
        print(f"   Vetor Gen(0) Com IA (DT+L2): {vetor_ia}", flush=True)

        res_ia = avaliar_vetor_4d_com_cache(chave, vetor_ia, cenario, nd)
        dist_km = reg_atual.get("gen0_sequencial", {}).get("distancia_km", 0.0)
        res_ia["distancia_km"] = dist_km
        res_ia["energia_kwh"] = round(res_ia["energia"] / 1000.0, 3)
        res_ia["potencia_kw"] = round((res_ia["energia"] * 3.6) / res_ia["makespan"], 3) if res_ia["makespan"] > 0 else 0.0

        reg_atual["vetor_ia"] = [int(x) for x in vetor_ia]
        reg_atual["gen0_ia"] = res_ia

        d_seq = reg_atual.get("gen0_sequencial", {})
        d_frio = reg_atual.get("ga_frio_8gen", {})
        d_ia8 = reg_atual.get("ga_ia_8gen", {})

        if d_seq:
            reg_atual["ganho_gen0_ia_vs_seq_makespan_pct"] = pct(d_seq["makespan"], res_ia["makespan"])
            reg_atual["ganho_gen0_ia_vs_seq_custo_pct"] = pct(d_seq["custo"], res_ia["custo"])
        if d_frio:
            reg_atual["ganho_gen0_ia_vs_frio8gen_makespan_pct"] = pct(d_frio["makespan"], res_ia["makespan"])
        if d_ia8:
            reg_atual["ganho_ga_sobre_gen0_ia_makespan_pct"] = pct(res_ia["makespan"], d_ia8["makespan"])

        relatorio["cenarios"][chave] = reg_atual
        salvar_json_atomico(RELATORIO_TODOS_JSON, relatorio)
        exportar_tabela_completa(relatorio)

        rows = list(csv.DictReader(open(os.path.join(DIR_IA, "tabela_comparativa_completa.csv"), encoding="utf-8")))
        rows_9 = [x for x in rows if not x["cenario"].endswith("(B)")]
        with open(os.path.join(DIR_IA, "tabela_9_cenarios_completa.csv"), "w", newline="", encoding="utf-8") as f9:
            w = csv.DictWriter(f9, fieldnames=list(rows_9[0].keys()))
            w.writeheader()
            w.writerows(rows_9)

        mk_s = d_seq.get("makespan", 0)
        mk_0 = res_ia["makespan"]
        mk_8 = d_ia8.get("makespan", 0)
        print(
            f"   ✅ [FÍSICA 4D REAL] {nome}: Gen0_Sem_IA={mk_s}s | Gen0_Com_IA(DT+L2)={mk_0}s ({res_ia['energia']}Wh, {res_ia['tempo_eval_s']}s) | Gen8_Com_IA(GA)={mk_8}s "
            f"[Ganho IA G0: -{pct(mk_s, mk_0)}% | Ganho GA sobre IA: +{pct(mk_0, mk_8)}%]",
            flush=True
        )

    print("\n🎉 SIMULAÇÃO 4D FÍSICA REAL CONCLUÍDA EM TODOS OS CENÁRIOS!", flush=True)


if __name__ == "__main__":
    main()
