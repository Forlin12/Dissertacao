r"""
Bateria dedicada de validacao real 4D (MOSP_CAMADAS) para o cenario 5 drones / 10 entregas
em diferentes configuracoes de pedidos, projetada para rodar na VM via pasta compartilhada
com telemetria ao vivo no PC principal.
"""
import os
import sys
import json
import math
import time
import random
import traceback
import numpy as np

DIR_IA = os.path.dirname(os.path.abspath(__file__))
DIR_RAIZ = os.path.dirname(DIR_IA)
if DIR_IA in sys.path:
    sys.path.remove(DIR_IA)
sys.path.insert(0, DIR_RAIZ)

sys.stdout.reconfigure(encoding="utf-8")

import config as cfg
from city_builder import gerar_cidade
from teste.ga_optimizer import executar_otimizacao_batch
from IA.dataset_generator import (
    _carregar_pacote, MODELO_PATH, CACHE_FRIO_PATH,
    _preparar_cenario, prever_alocacao, gerar_populacao_inicial, CIDADE_SEED
)

LOG_VM_PATH = os.path.join(DIR_IA, "vm_progresso.log")
STATUS_VM_PATH = os.path.join(DIR_IA, "vm_status.json")
RELATORIO_5_10_PATH = os.path.join(DIR_IA, "relatorio_5_10_vm.json")


def log_vm(msg):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    linha = f"[{ts}] {msg}"
    print(linha, flush=True)
    with open(LOG_VM_PATH, "a", encoding="utf-8") as f:
        f.write(linha + "\n")
        f.flush()


def atualizar_status(estado, detalhe=None):
    dados = {
        "estado": estado,
        "atualizado_em": time.strftime("%Y-%m-%d %H:%M:%S"),
        "maquina": os.environ.get("COMPUTERNAME", "desconhecida"),
        "detalhe": detalhe or {},
    }
    tmp = STATUS_VM_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(dados, f, indent=2, ensure_ascii=False)
    os.replace(tmp, STATUS_VM_PATH)


def main():
    with open(LOG_VM_PATH, "w", encoding="utf-8") as f:
        f.write("")

    log_vm(f"🚀 Iniciando Bateria 4D (5 drones / 10 entregas) na máquina: {os.environ.get('COMPUTERNAME')}")
    atualizar_status("RODANDO", {"etapa": "Gerando cidade 3D"})

    cfg.CENARIO_SEMPRE_NOVO = False
    random.seed(CIDADE_SEED)
    np.random.seed(CIDADE_SEED)
    lotes_gdf, max_x, max_y = gerar_cidade()

    pacote = _carregar_pacote(MODELO_PATH)
    modelo = pacote["modelo"]
    max_entregas = pacote.get("max_entregas", 160)
    perfis = cfg.PERFIS_DRONES
    tam_pop = getattr(cfg, "GA_SOL_POP", 10)

    cache_frio = {}
    if os.path.exists(CACHE_FRIO_PATH):
        try:
            cache_frio = json.load(open(CACHE_FRIO_PATH, encoding="utf-8")).get("cenarios", {})
        except Exception:
            cache_frio = {}

    seeds_5_10 = [900005, 700001, 700004]
    nd, ne, alg = 5, 10, "MOSP_CAMADAS"

    resultados = []
    if os.path.exists(RELATORIO_5_10_PATH):
        try:
            resultados = json.load(open(RELATORIO_5_10_PATH, encoding="utf-8")).get("cenarios", [])
        except Exception:
            resultados = []
    seeds_concluidas = {r["seed"] for r in resultados}

    for idx_s, seed in enumerate(seeds_5_10):
        if seed in seeds_concluidas:
            log_vm(f"⏭️ Seed {seed} já concluída anteriormente; pulando.")
            continue

        chave_cache = f"{seed}|{nd}|{ne}|{alg}"
        log_vm(f"\n{'='*65}\n📦 CENÁRIO {idx_s+1}/{len(seeds_5_10)} | 5 Drones / 10 Entregas | Seed {seed}\n{'='*65}")
        atualizar_status("RODANDO", {"cenario_atual": idx_s + 1, "total_cenarios": len(seeds_5_10), "seed": seed})

        cd_xy, missoes, _ = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
        pred_ia = prever_alocacao(modelo, missoes, cd_xy, max_x, max_y, nd, perfis, max_entregas)
        np.random.seed(seed + 1)
        pop_ia = gerar_populacao_inicial(pred_ia, tam_pop, nd)

        if chave_cache in cache_frio:
            frio = cache_frio[chave_cache]
            log_vm(f"❄️ [GA FRIO DO CACHE] Makespan: {frio['makespan']}f | Energia: {frio['energia']}Wh | Gerações: {frio['geracoes']}")
        else:
            log_vm(f"❄️ Rodando GA Cru (Frio) para Seed {seed}...")
            _, _, cen_frio = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
            t0 = time.perf_counter()
            _, mk_f, es_f, fa_f, en_f, ger_f = executar_otimizacao_batch(
                500000 + idx_s, nd, ne, alg, cenario=cen_frio, seed_ga=seed
            )
            frio = {
                "geracoes": int(ger_f), "makespan": int(mk_f), "esperas": int(es_f),
                "falhas": int(fa_f), "energia": round(float(en_f), 2),
                "tempo_s": round(time.perf_counter() - t0, 2)
            }
            log_vm(f"❄️ [GA FRIO CONCLUÍDO] Makespan: {frio['makespan']}f | Energia: {frio['energia']}Wh | Tempo: {frio['tempo_s']}s")

        log_vm(f"🔥 Rodando GA + Warm Start IA para Seed {seed} (Chute IA = {pred_ia})...")
        _, _, cen_ia = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
        t0 = time.perf_counter()
        vetor_q, mk_q, es_q, fa_q, en_q, ger_q = executar_otimizacao_batch(
            600000 + idx_s, nd, ne, alg, cenario=cen_ia, populacao_inicial=pop_ia, seed_ga=seed
        )
        quente = {
            "geracoes": int(ger_q), "makespan": int(mk_q), "esperas": int(es_q),
            "falhas": int(fa_q), "energia": round(float(en_q), 2),
            "tempo_s": round(time.perf_counter() - t0, 2),
            "chute_ia_gen0": pred_ia,
            "vetor_final": [int(x) for x in vetor_q],
        }

        ganho_mk = round(100.0 * (frio["makespan"] - quente["makespan"]) / max(frio["makespan"], 1), 1)
        custo_frio = round(frio["makespan"] + 3.6 * frio["energia"] + 50000 * frio["falhas"], 1)
        custo_ia = round(quente["makespan"] + 3.6 * quente["energia"] + 50000 * quente["falhas"], 1)
        ganho_custo = round(100.0 * (custo_frio - custo_ia) / max(custo_frio, 1.0), 1)

        item = {
            "seed": seed,
            "config": [nd, ne, alg],
            "frio": frio,
            "ia_warm_start": quente,
            "custo_frio": custo_frio,
            "custo_ia": custo_ia,
            "reducao_makespan_pct": ganho_mk,
            "reducao_custo_pct": ganho_custo,
        }
        resultados.append(item)

        with open(RELATORIO_5_10_PATH, "w", encoding="utf-8") as f:
            json.dump({"cenarios": resultados}, f, indent=2, ensure_ascii=False)

        log_vm(f"✅ SEED {seed} CONCLUÍDA: Frio={frio['makespan']}f ({custo_frio}) -> IA={quente['makespan']}f ({custo_ia}) | "
               f"Ganho Makespan: -{ganho_mk}% | Ganho Custo: -{ganho_custo}%")

    atualizar_status("CONCLUIDO", {"cenarios_concluidos": len(resultados), "relatorio": RELATORIO_5_10_PATH})
    log_vm("🏁 BATERIA 5/10 FINALIZADA COM SUCESSO NA VM!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        tb_str = traceback.format_exc()
        log_vm(f"❌ ERRO NA EXECUÇÃO DA VM: {e}\n{tb_str}")
        atualizar_status("ERRO", {"erro": str(e), "traceback": tb_str})
        raise
