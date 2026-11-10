r"""
Bateria Completa 4D (MOSP_CAMADAS) em TODOS os cenários da dissertação:
  - 5 drones / 10 entregas (seed 900000 e seed 900005)
  - 5 drones / 20 entregas (seed 900001)
  - 10 drones / 20 entregas (seed 900003)
  - 5 drones / 40 entregas (seed 900002)
  - 10 drones / 40 entregas (seed 900004)

Blindado contra desligamentos frequentes da VM:
  - Etapa 1 (Rápida ~12 min): Avalia no simulador 4D real a Geração 0 (Sequencial vs IA Direta)
    em TODOS os cenários e compara com o GA Frio de 8 gerações (benchmark_frio_cache.json).
    Salva checkpoint em disco a cada cenário concluído.
  - Etapa 2 (Completa): Roda o GA de 8 gerações com Warm Start da IA em todos os cenários,
    usando o cache por indivíduo (IA/cache_individuos_4d.json) para nunca perder progresso se a VM desligar.
"""
import os
import sys
import json
import math
import time
import random
import traceback

DIR_IA = os.path.dirname(os.path.abspath(__file__))
DIR_RAIZ = os.path.dirname(DIR_IA)

# Usa cache local no SSD da VM para evitar travamentos de lock do Numba/__pycache__ no drive de rede RDP (\\tsclient)
_local_cache = os.path.join(os.environ.get("LOCALAPPDATA", "C:\\Temp"), "dissertacao_vm_cache")
os.makedirs(_local_cache, exist_ok=True)
os.environ["NUMBA_CACHE_DIR"] = _local_cache
sys.pycache_prefix = _local_cache

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

LOG_PROGRESSO = os.path.join(DIR_IA, "vm_progresso.log")
STATUS_JSON = os.path.join(DIR_IA, "vm_status.json")
RELATORIO_TODOS_JSON = os.path.join(DIR_IA, "relatorio_todos_cenarios_vm.json")
CACHE_IND_JSON = os.path.join(DIR_IA, "cache_individuos_4d.json")
TSCLIENT_IA = r"\\tsclient\Z\IA"


def _espelhar_para_notebook(caminho_local):
    r"""Se estiver rodando no SSD local da VM (C:), espelha os resultados para \\tsclient\Z\IA sem travar se a rede oscilar."""
    try:
        if os.path.abspath(DIR_IA).upper().startswith("C:") and os.path.isdir(TSCLIENT_IA):
            import shutil
            nome = os.path.basename(caminho_local)
            destino = os.path.join(TSCLIENT_IA, nome)
            shutil.copy2(caminho_local, destino)
    except Exception:
        pass


def log(msg):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    linha = f"[{ts}] {msg}"
    print(linha, flush=True)
    try:
        with open(LOG_PROGRESSO, "a", encoding="utf-8") as f:
            f.write(linha + "\n")
    except Exception:
        pass


log(f"⏳ Inicializando script na máquina {os.environ.get('COMPUTERNAME', 'LOCAL')} e importando módulos...")

# Coloca DIR_RAIZ no FINAL do sys.path para o Python buscar numpy/geopandas/numba/pygad direto no SSD local (C:)
# em vez de fazer milhares de buscas lentas pela pasta compartilhada RDP (W:/X:)
if "" in sys.path:
    sys.path.remove("")
if DIR_IA in sys.path:
    sys.path.remove(DIR_IA)
if DIR_RAIZ in sys.path:
    sys.path.remove(DIR_RAIZ)
sys.path.append(DIR_RAIZ)
try:
    os.chdir(DIR_RAIZ)
except Exception:
    pass

import numpy as np
import config as cfg
from city_builder import gerar_cidade
from uav_physics import Drone
from main import roteador_inteligente, analisar_cenario
from teste.ga_optimizer import executar_otimizacao_batch
from IA.dataset_generator import (
    _carregar_pacote,
    _carregar_cenarios_do_csv,
    _preparar_cenario,
    prever_alocacao,
    gerar_populacao_inicial,
    EnsembleFisicoEnxuto,
    MODELO_PATH,
    CIDADE_SEED,
    CACHE_FRIO_PATH,
    BENCH_PATH,
)

# Todos os cenários da dissertação (incluindo os cenários de grande escala com GA Frio de 8 gerações já medido)
CENARIOS_TESTE = [
    {"seed": 900000, "nd": 5,  "ne": 10,  "alg": "MOSP_CAMADAS", "nome": "5 drones / 10 entregas (A)"},
    {"seed": 900005, "nd": 5,  "ne": 10,  "alg": "MOSP_CAMADAS", "nome": "5 drones / 10 entregas (B)"},
    {"seed": 900001, "nd": 5,  "ne": 20,  "alg": "MOSP_CAMADAS", "nome": "5 drones / 20 entregas"},
    {"seed": 900003, "nd": 10, "ne": 20,  "alg": "MOSP_CAMADAS", "nome": "10 drones / 20 entregas"},
    {"seed": 900002, "nd": 5,  "ne": 40,  "alg": "MOSP_CAMADAS", "nome": "5 drones / 40 entregas"},
    {"seed": 900004, "nd": 10, "ne": 40,  "alg": "MOSP_CAMADAS", "nome": "10 drones / 40 entregas"},
    # Cenários de Grande Escala (seeds 12350..12359 que já possuem GA Frio 8-Gen real em dataset_treino_original_ga.csv)
    {"seed": 12351,  "nd": 20, "ne": 40,  "alg": "MOSP_CAMADAS", "nome": "20 drones / 40 entregas",
     "frio_ref": {"geracoes": 6, "makespan": 6736,  "esperas": 0, "falhas": 0, "energia": 2464.92, "tempo_s": 22182.45},
     "id_excluir_treino": {5, 6, 7, 8, 14}},
    {"seed": 12350,  "nd": 10, "ne": 80,  "alg": "MOSP_CAMADAS", "nome": "10 drones / 80 entregas (A)",
     "frio_ref": {"geracoes": 3, "makespan": 16079, "esperas": 0, "falhas": 0, "energia": 4339.81, "tempo_s": 24543.51},
     "id_excluir_treino": {5, 6, 7, 8, 14}},
    {"seed": 12359,  "nd": 10, "ne": 80,  "alg": "MOSP_CAMADAS", "nome": "10 drones / 80 entregas (B)",
     "frio_ref": {"geracoes": 3, "makespan": 21188, "esperas": 0, "falhas": 0, "energia": 5759.16, "tempo_s": 18707.33},
     "id_excluir_treino": {5, 6, 7, 8, 14}},
    {"seed": 12352,  "nd": 20, "ne": 80,  "alg": "MOSP_CAMADAS", "nome": "20 drones / 80 entregas",
     "frio_ref": {"geracoes": 8, "makespan": 12702, "esperas": 0, "falhas": 0, "energia": 5159.13, "tempo_s": 68290.18},
     "id_excluir_treino": {5, 6, 7, 8, 14}},
    {"seed": 12353,  "nd": 20, "ne": 160, "alg": "MOSP_CAMADAS", "nome": "20 drones / 160 entregas",
     "frio_ref": {"geracoes": 0, "makespan": 20400, "esperas": 0, "falhas": 0, "energia": 9753.78, "tempo_s": 41340.20},
     "id_excluir_treino": {5, 6, 7, 8, 14}},
]


def log(msg):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    linha = f"[{ts}] {msg}"
    print(linha, flush=True)
    try:
        with open(LOG_PROGRESSO, "a", encoding="utf-8") as f:
            f.write(linha + "\n")
    except Exception:
        pass
    try:
        if os.path.abspath(DIR_IA).upper().startswith("C:") and os.path.isdir(TSCLIENT_IA):
            with open(os.path.join(TSCLIENT_IA, "vm_progresso.log"), "a", encoding="utf-8") as f2:
                f2.write(linha + "\n")
    except Exception:
        pass


def salvar_json_atomico(caminho, dados):
    tmp = caminho + f".tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(dados, f, indent=2, ensure_ascii=False)
    os.replace(tmp, caminho)
    _espelhar_para_notebook(caminho)


def atualizar_status(etapa, cenario_atual, concluidos_gen0, concluidos_ga, total, detalhe_extra=None):
    dados = {
        "atualizado_em": time.strftime("%Y-%m-%d %H:%M:%S"),
        "maquina": os.environ.get("COMPUTERNAME", "desconhecido"),
        "etapa": etapa,
        "cenario_atual": cenario_atual,
        "concluidos_gen0": concluidos_gen0,
        "concluidos_ga_completo": concluidos_ga,
        "total_cenarios": total,
    }
    if detalhe_extra:
        dados.update(detalhe_extra)
    try:
        salvar_json_atomico(STATUS_JSON, dados)
    except Exception:
        pass


def avaliar_vetor_4d_com_cache(chave_cenario, vetor, cenario, num_drones):
    """Avalia um único indivíduo no simulador 4D real usando o mesmo cache persistente do GA."""
    chave_sol = ",".join(str(int(g)) for g in vetor)
    if os.path.exists(CACHE_IND_JSON):
        try:
            if os.path.getsize(CACHE_IND_JSON) > 2:
                with open(CACHE_IND_JSON, "r", encoding="utf-8") as f:
                    todos = json.load(f)
                reg = todos.get(chave_cenario, {}).get(chave_sol)
                if reg and reg.get("completo", True):
                    mk = int(reg["makespan"])
                    esp = int(reg["esperas"])
                    fal = int(reg["falhas"])
                    eng = float(reg["energia"])
                    custo = mk + 0.5 * esp + 3.6 * eng + 50000.0 * fal
                    return {
                        "makespan": mk,
                        "esperas": esp,
                        "falhas": fal,
                        "energia": round(eng, 2),
                        "custo": round(custo, 1),
                        "tempo_eval_s": float(reg.get("tempo_s", 0.0)),
                        "origem": "cache_disco",
                    }
        except Exception:
            pass

    t0 = time.perf_counter()
    lotes_gdf = cenario["lotes_gdf"]
    max_x, max_y = cenario["max_x"], cenario["max_y"]
    missoes = cenario["missoes"]
    num_pedidos = len(missoes)
    num_perfis = len(cfg.PERFIS_DRONES)
    tempo_descarga = getattr(cfg, "TEMPO_DESCARGA", 5)
    vetor_tempos_partida = getattr(cfg, "VETOR_TEMPOS_PARTIDA", [0])

    frota_local = [Drone(cfg.PERFIS_DRONES[i % num_perfis]) for i in range(num_drones)]
    pontos_missoes = [
        {"start": (int(m["origem"][0]), int(m["origem"][1])),
         "goal": (int(m["destino"][0]), int(m["destino"][1])),
         "peso_kg": m.get("peso_kg", 1.0)}
        for m in missoes
    ]

    reserva_global = {}
    disponibilidade = {i: 0 for i in range(num_drones)}
    makespan_frames = 0
    esperas_total = 0
    missoes_falhadas = 0
    energia_total = 0.0

    fila_missoes = list(range(num_pedidos))
    tentativas = {i: 0 for i in range(num_pedidos)}
    max_tentativas = 2

    while fila_missoes:
        id_pedido = fila_missoes.pop(0)
        id_drone = int(vetor[id_pedido])
        drone = frota_local[id_drone]

        if tentativas[id_pedido] == 0:
            drone.reset_metricas()

        carga_original = drone._carga_original
        peso_pacote = min(pontos_missoes[id_pedido]["peso_kg"], carga_original)
        drone.carga = peso_pacote

        start = pontos_missoes[id_pedido]["start"]
        goal = pontos_missoes[id_pedido]["goal"]

        t_base = vetor_tempos_partida[id_pedido] if id_pedido < len(vetor_tempos_partida) else \
            vetor_tempos_partida[-1] + ((id_pedido - len(vetor_tempos_partida) + 1) * 15)
        t_ida = max(t_base, disponibilidade[id_drone])

        dist_linear = np.hypot(goal[0] - start[0], goal[1] - start[1])
        energia_estimada = drone.estimar_energia_missao(dist_linear, dist_linear, tempo_descarga, peso_pacote)
        energia_necessaria = energia_estimada * 1.5

        if energia_necessaria > drone.bateria_capacidade_wh:
            missoes_falhadas += 1
            drone.carga = carga_original
            continue

        energia_em_falta = 0.0
        if drone.bateria_atual_wh < energia_necessaria:
            energia_em_falta = energia_necessaria - drone.bateria_atual_wh
            tempo_de_carga_s = int((energia_em_falta / drone.taxa_recarga_w) * 3600.0) + 1
            t_ida += tempo_de_carga_s
            drone.carregar_bateria_quantidade(energia_em_falta)

        cam_ida, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, t_ida)
        if not cam_ida:
            tentativas[id_pedido] += 1
            if tentativas[id_pedido] < max_tentativas:
                fila_missoes.append(id_pedido)
                if energia_em_falta > 0:
                    drone.bateria_atual_wh -= energia_em_falta
            else:
                missoes_falhadas += 1
            drone.carga = carga_original
            continue

        for i, p in enumerate(cam_ida):
            z = p[2] if len(p) == 3 else drone.altura_alvo
            reserva_global[(p[0], p[1], z, t_ida + i)] = id_drone

        _, hovers_ida, _ = analisar_cenario(cam_ida)
        esperas_total += hovers_ida

        bateria_antes_ida = drone.bateria_atual_wh
        rota_ida, bateu, _ = drone.simular_missao(cam_ida, lotes_gdf)
        energia_total += (bateria_antes_ida - drone.bateria_atual_wh)

        if bateu:
            tentativas[id_pedido] += 1
            if tentativas[id_pedido] < max_tentativas:
                fila_missoes.append(id_pedido)
            else:
                missoes_falhadas += 1
            drone.carga = carga_original
            continue

        drone.carga = 0.0
        if tempo_descarga > 0:
            custo_peso_m = peso_pacote * drone.penalidade_carga
            consumo_descarga_wh = tempo_descarga * (drone.consumo_hover + (custo_peso_m * drone.velocidade_horiz))
            drone.bateria_atual_wh -= consumo_descarga_wh
            energia_total += consumo_descarga_wh

        t_volta = t_ida + len(rota_ida) + tempo_descarga
        for extra in range(tempo_descarga):
            z_descarga = cam_ida[-1][2] if len(cam_ida[-1]) == 3 else drone.altura_alvo
            reserva_global[(cam_ida[-1][0], cam_ida[-1][1], z_descarga, t_ida + len(cam_ida) + extra)] = id_drone

        cam_volta, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, goal, start, reserva_global, t_volta)
        if not cam_volta:
            missoes_falhadas += 1
            disponibilidade[id_drone] = t_volta
            drone.carga = carga_original
            continue

        for i, p in enumerate(cam_volta):
            z = p[2] if len(p) == 3 else drone.altura_alvo
            reserva_global[(p[0], p[1], z, t_volta + i)] = id_drone

        _, hovers_volta, _ = analisar_cenario(cam_volta)
        esperas_total += hovers_volta

        bateria_antes_volta = drone.bateria_atual_wh
        rota_volta, bateu_v, _ = drone.simular_missao(cam_volta, lotes_gdf)
        energia_total += (bateria_antes_volta - drone.bateria_atual_wh)

        if bateu_v:
            tentativas[id_pedido] += 1
            if tentativas[id_pedido] < max_tentativas:
                fila_missoes.append(id_pedido)
            else:
                missoes_falhadas += 1
            disponibilidade[id_drone] = t_volta + len(rota_volta)
            drone.carga = carga_original
            continue

        t_fim = t_volta + len(rota_volta)
        disponibilidade[id_drone] = t_fim
        drone.carga = carga_original
        if t_fim > makespan_frames:
            makespan_frames = t_fim

    dt = round(time.perf_counter() - t0, 2)
    custo = makespan_frames + (esperas_total * 0.5) + (energia_total * 3.6) + (missoes_falhadas * 50000)

    # Salva no cache compartilhado de indivíduos 4D
    try:
        todos = {}
        if os.path.exists(CACHE_IND_JSON):
            try:
                if os.path.getsize(CACHE_IND_JSON) > 2:
                    with open(CACHE_IND_JSON, "r", encoding="utf-8") as f:
                        todos = json.load(f)
            except Exception:
                todos = {}
        if chave_cenario not in todos:
            todos[chave_cenario] = {}
        todos[chave_cenario][chave_sol] = {
            "makespan": int(makespan_frames),
            "esperas": int(esperas_total),
            "falhas": int(missoes_falhadas),
            "energia": round(float(energia_total), 3),
            "completo": True,
            "tempo_s": dt,
        }
        salvar_json_atomico(CACHE_IND_JSON, todos)
    except Exception:
        pass

    return {
        "makespan": int(makespan_frames),
        "esperas": int(esperas_total),
        "falhas": int(missoes_falhadas),
        "energia": round(float(energia_total), 2),
        "custo": round(float(custo), 1),
        "tempo_eval_s": dt,
        "origem": "simulado_4d",
    }


def pct(a, b):
    return round(100.0 * (a - b) / a, 1) if a else 0.0


def exportar_tabela_completa(relatorio):
    import csv
    caminho_csv = os.path.join(DIR_IA, "tabela_comparativa_completa.csv")
    linhas = []
    for chave, c_info in relatorio.get("cenarios", {}).items():
        nome = c_info.get("nome", chave)
        nd = c_info.get("nd")
        ne = c_info.get("ne")
        seed = c_info.get("seed")

        d_seq = c_info.get("gen0_sequencial", {}) or {}
        d_frio = c_info.get("ga_frio_8gen", {}) or {}
        d_ia_g0 = c_info.get("gen0_ia", {}) or {}
        d_ia_g8 = c_info.get("ga_ia_8gen", {}) or {}

        dist_km = d_seq.get("distancia_km") or d_ia_g0.get("distancia_km") or c_info.get("distancia_frota_km", 0.0)

        mk_seq = d_seq.get("makespan", 0)
        en_seq_wh = round(float(d_seq.get("energia", 0.0)), 2)
        en_seq_kwh = round(en_seq_wh / 1000.0, 3)
        kw_seq = d_seq.get("potencia_kw") or (round((en_seq_wh * 3.6) / mk_seq, 3) if mk_seq > 0 else 0.0)
        custo_seq = d_seq.get("custo") or round(mk_seq + 0.5 * d_seq.get("esperas", 0) + 3.6 * en_seq_wh + 50000 * d_seq.get("falhas", 0), 1)
        t_sol_seq = d_seq.get("tempo_eval_s", 0.0)

        mk_frio = d_frio.get("makespan", 0)
        en_frio_wh = round(float(d_frio.get("energia", 0.0)), 2)
        en_frio_kwh = round(en_frio_wh / 1000.0, 3)
        kw_frio = d_frio.get("potencia_kw") or (round((en_frio_wh * 3.6) / mk_frio, 3) if mk_frio > 0 else 0.0)
        custo_frio = d_frio.get("custo") or round(mk_frio + 0.5 * d_frio.get("esperas", 0) + 3.6 * en_frio_wh + 50000 * d_frio.get("falhas", 0), 1)
        t_sol_frio = d_frio.get("tempo_s", 0.0)
        ger_frio = d_frio.get("geracoes", 0)

        mk_ia0 = d_ia_g0.get("makespan", 0)
        en_ia0_wh = round(float(d_ia_g0.get("energia", 0.0)), 2)
        en_ia0_kwh = round(en_ia0_wh / 1000.0, 3)
        kw_ia0 = d_ia_g0.get("potencia_kw") or (round((en_ia0_wh * 3.6) / mk_ia0, 3) if mk_ia0 > 0 else 0.0)
        custo_ia0 = d_ia_g0.get("custo") or round(mk_ia0 + 0.5 * d_ia_g0.get("esperas", 0) + 3.6 * en_ia0_wh + 50000 * d_ia_g0.get("falhas", 0), 1)
        t_sol_ia0 = d_ia_g0.get("tempo_eval_s", 0.0)

        mk_ia8 = d_ia_g8.get("makespan", 0)
        en_ia8_wh = round(float(d_ia_g8.get("energia", 0.0)), 2)
        en_ia8_kwh = round(en_ia8_wh / 1000.0, 3)
        kw_ia8 = d_ia_g8.get("potencia_kw") or (round((en_ia8_wh * 3.6) / mk_ia8, 3) if mk_ia8 > 0 else 0.0)
        custo_ia8 = d_ia_g8.get("custo") or round(mk_ia8 + 0.5 * d_ia_g8.get("esperas", 0) + 3.6 * en_ia8_wh + 50000 * d_ia_g8.get("falhas", 0), 1)
        t_sol_ia8_raw = float(d_ia_g8.get("tempo_s", 0.0))
        t_sol_ia8 = round(t_sol_ia0 + t_sol_ia8_raw, 2) if (0 < t_sol_ia8_raw < t_sol_ia0) else round(t_sol_ia8_raw, 2)
        ger_ia8 = d_ia_g8.get("geracoes", 0)

        ganho_g0 = pct(mk_seq, mk_ia0)
        ganho_g8 = pct(mk_frio, mk_ia8) if mk_frio else 0.0
        ganho_custo_g0 = pct(custo_seq, custo_ia0)
        ganho_custo_g8 = pct(custo_frio, custo_ia8) if custo_frio else 0.0
        ganho_tempo_ga = pct(t_sol_frio, t_sol_ia8) if t_sol_frio else 0.0
        speedup = round(t_sol_frio / t_sol_ia8, 1) if (t_sol_ia8 > 0 and t_sol_frio > 0) else 1.0

        linhas.append({
            "cenario": nome,
            "drones": nd,
            "entregas": ne,
            "seed": seed,
            "distancia_km": dist_km,
            "gen0_sem_ia_tempo_entregas_s": mk_seq,
            "gen0_sem_ia_tempo_entregas_min": round(mk_seq / 60.0, 2),
            "gen0_sem_ia_energia_wh": en_seq_wh,
            "gen0_sem_ia_energia_kwh": en_seq_kwh,
            "gen0_sem_ia_potencia_kw": kw_seq,
            "gen0_sem_ia_custo_total": custo_seq,
            "gen0_sem_ia_tempo_solucao_s": t_sol_seq,
            "gen8_sem_ia_tempo_entregas_s": mk_frio,
            "gen8_sem_ia_tempo_entregas_min": round(mk_frio / 60.0, 2),
            "gen8_sem_ia_energia_wh": en_frio_wh,
            "gen8_sem_ia_energia_kwh": en_frio_kwh,
            "gen8_sem_ia_potencia_kw": kw_frio,
            "gen8_sem_ia_custo_total": custo_frio,
            "gen8_sem_ia_tempo_solucao_s": t_sol_frio,
            "gen8_sem_ia_geracao_conv": ger_frio,
            "gen0_com_ia_tempo_entregas_s": mk_ia0,
            "gen0_com_ia_tempo_entregas_min": round(mk_ia0 / 60.0, 2),
            "gen0_com_ia_energia_wh": en_ia0_wh,
            "gen0_com_ia_energia_kwh": en_ia0_kwh,
            "gen0_com_ia_potencia_kw": kw_ia0,
            "gen0_com_ia_custo_total": custo_ia0,
            "gen0_com_ia_tempo_solucao_s": t_sol_ia0,
            "gen8_com_ia_tempo_entregas_s": mk_ia8,
            "gen8_com_ia_tempo_entregas_min": round(mk_ia8 / 60.0, 2),
            "gen8_com_ia_energia_wh": en_ia8_wh,
            "gen8_com_ia_energia_kwh": en_ia8_kwh,
            "gen8_com_ia_potencia_kw": kw_ia8,
            "gen8_com_ia_custo_total": custo_ia8,
            "gen8_com_ia_tempo_solucao_s": t_sol_ia8,
            "gen8_com_ia_geracao_conv": ger_ia8,
            "ganho_mk_gen0_ia_pct": ganho_g0,
            "ganho_mk_gen8_ia_pct": ganho_g8,
            "ganho_custo_gen0_ia_pct": ganho_custo_g0,
            "ganho_custo_gen8_ia_pct": ganho_custo_g8,
            "ganho_tempo_solucao_ga_pct": ganho_tempo_ga,
            "speedup_ga": speedup,
        })

    if linhas:
        with open(caminho_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
            w.writeheader()
            w.writerows(linhas)
        _espelhar_para_notebook(caminho_csv)


def carregar_relatorio_existente():
    if os.path.exists(RELATORIO_TODOS_JSON):
        try:
            with open(RELATORIO_TODOS_JSON, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {"cenarios": {}}


def semear_900000_ja_concluido(relatorio, cache_frio):
    """Aproveita o resultado real 4D do cenário 900000 já executado anteriormente."""
    chave = "900000|5|10|MOSP_CAMADAS"
    if chave in relatorio["cenarios"] and "ga_ia_8gen" in relatorio["cenarios"][chave]:
        return
    if os.path.exists(BENCH_PATH):
        try:
            b = json.load(open(BENCH_PATH, "r", encoding="utf-8"))
            for d in b.get("detalhe", []):
                if d.get("seed") == 900000:
                    frio = d["frio"]
                    ml = d["ml"]
                    relatorio["cenarios"].setdefault(chave, {})
                    relatorio["cenarios"][chave].update({
                        "seed": 900000,
                        "nd": 5,
                        "ne": 10,
                        "alg": "MOSP_CAMADAS",
                        "nome": "5 drones / 10 entregas (A)",
                        "gen0_sequencial": {
                            "makespan": 5622, "esperas": 0, "falhas": 0,
                            "energia": 526.1, "custo": 7516.1, "tempo_eval_s": 33.4, "origem": "cache_previo"
                        },
                        "gen0_ia": {
                            "makespan": 3304, "esperas": 0, "falhas": 0,
                            "energia": 503.2, "custo": 5115.4, "tempo_eval_s": 35.1, "origem": "cache_previo"
                        },
                        "ga_frio_8gen": {
                            **frio,
                            "custo": round(frio["makespan"] + 3.6 * frio["energia"] + 50000 * frio["falhas"], 1)
                        },
                        "ga_ia_8gen": {
                            **ml,
                            "custo": round(ml["makespan"] + 3.6 * ml["energia"] + 50000 * ml["falhas"], 1)
                        },
                        "ganho_gen0_ia_vs_seq_makespan_pct": pct(5622, 3304),
                        "ganho_gen0_ia_vs_frio8gen_makespan_pct": pct(frio["makespan"], 3304),
                        "ganho_tempo_ga_pct": pct(frio["tempo_s"], ml["tempo_s"]),
                    })
                    salvar_json_atomico(RELATORIO_TODOS_JSON, relatorio)
        except Exception:
            pass


def main():
    # Impede o Windows (tanto no PC quanto na VM) de entrar em suspensão e evita timeout de 1h da sessão RDP
    try:
        import ctypes
        import threading
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000003)
        def _keepalive_rdp():
            while True:
                try:
                    ctypes.windll.kernel32.SetThreadExecutionState(0x80000003)
                    ctypes.windll.user32.mouse_event(0x0001, 0, 0, 0, 0)
                except Exception:
                    pass
                # Sincroniza periodicamente os arquivos do SSD local (C:) da VM para \\tsclient\Z\IA
                for arq in (RELATORIO_TODOS_JSON, CACHE_IND_JSON, STATUS_JSON, LOG_PROGRESSO):
                    if os.path.exists(arq):
                        _espelhar_para_notebook(arq)
                time.sleep(20)
        threading.Thread(target=_keepalive_rdp, daemon=True).start()
    except Exception:
        pass

    log("=================================================================")
    log(f"🚀 INICIANDO TESTE 4D DE TODOS OS CENÁRIOS NA VM ({os.environ.get('COMPUTERNAME', 'LOCAL')})")
    log("=================================================================")

    pacote = _carregar_pacote(MODELO_PATH)
    modelo = pacote["modelo"]
    max_entregas = pacote.get("max_entregas", 160)

    _modelo_sem_grandes_cache = [None]
    def _obter_modelo_sem_grandes():
        if _modelo_sem_grandes_cache[0] is None:
            log("🌲 Treinando ensemble auxiliar sem cenários grandes (zero vazamento)...")
            cens_csv, _ = _carregar_cenarios_do_csv()
            cens_menores = [c for c in cens_csv if c["id"] not in {5, 6, 7, 8, 14}]
            X_men = np.vstack([c["X"] for c in cens_menores])
            y_men = np.concatenate([c["yc"] for c in cens_menores])
            _modelo_sem_grandes_cache[0] = EnsembleFisicoEnxuto(n_arvores=150, l2=1.0, seed=42).fit(X_men, y_men)
        return _modelo_sem_grandes_cache[0]

    cache_frio = {}
    if os.path.exists(CACHE_FRIO_PATH):
        try:
            cache_frio = json.load(open(CACHE_FRIO_PATH, "r", encoding="utf-8")).get("cenarios", {})
        except Exception:
            cache_frio = {}

    relatorio = carregar_relatorio_existente()
    semear_900000_ja_concluido(relatorio, cache_frio)

    log("🏙️ Carregando malha urbana 3D (CIDADE_SEED=42)...")
    try:
        import osmnx as ox
        ox.settings.cache_folder = os.path.join(DIR_RAIZ, "cache")
    except Exception:
        pass
    cfg.CENARIO_SEMPRE_NOVO = False
    random.seed(CIDADE_SEED)
    np.random.seed(CIDADE_SEED)
    lotes_gdf, max_x, max_y = gerar_cidade()
    perfis = cfg.PERFIS_DRONES
    tam_pop = getattr(cfg, "GA_SOL_POP", 10)

    total = len(CENARIOS_TESTE)
    EXECUTAR_ETAPA_2 = True  # Executa a Etapa 2 (GA 8 Gerações + IA) em TODOS os 11 cenários na VM

    # =========================================================================
    # ETAPA 1: AVALIAÇÃO 4D DIRETA (GERAÇÃO 0: SEQUENCIAL vs IA) EM TODOS OS CENÁRIOS
    # =========================================================================
    log("\n⚡ [ETAPA 1] Avaliando Geração 0 (IA Direta vs Sequencial vs GA Frio 8-Gen) em TODOS os 11 cenários...")
    for idx, item in enumerate(CENARIOS_TESTE):
        seed, nd, ne, alg, nome = item["seed"], item["nd"], item["ne"], item["alg"], item["nome"]
        chave = f"{seed}|{nd}|{ne}|{alg}"

        reg_atual = relatorio["cenarios"].get(chave, {})
        if "gen0_ia" in reg_atual and "gen0_sequencial" in reg_atual:
            log(f"⏩ [Etapa 1] {nome} (seed {seed}) já possui avaliação Gen 0 em cache — pulando!")
            continue

        n_gen0_ok = sum(1 for v in relatorio["cenarios"].values() if "gen0_ia" in v)
        n_ga_ok = sum(1 for v in relatorio["cenarios"].values() if "ga_ia_8gen" in v)
        atualizar_status("ETAPA_1_GEN0_CENARIOS_MAIORES", f"{nome} (seed {seed})", n_gen0_ok, n_ga_ok, total)

        log(f"\n-----------------------------------------------------------------")
        log(f"🔎 [Etapa 1 — {idx+1}/{total}] Cenário {nome} (seed={seed})")
        log(f"-----------------------------------------------------------------")

        cd_xy, missoes, cenario = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)

        tb = math.ceil(ne / nd)
        vetor_seq = [(i // tb) % nd for i in range(ne)]
        mod_usar = _obter_modelo_sem_grandes() if item.get("id_excluir_treino") else modelo
        vetor_ia = prever_alocacao(mod_usar, missoes, cd_xy, max_x, max_y, nd, perfis, max_entregas)

        log(f"   • Avaliando Gen 0 Sequencial ({ne} entregas) no 4D...")
        res_seq = avaliar_vetor_4d_com_cache(chave, vetor_seq, cenario, nd)
        log(f"     -> Sequencial Gen 0: Makespan={res_seq['makespan']}f | Energia={res_seq['energia']}Wh | Custo={res_seq['custo']} ({res_seq['tempo_eval_s']}s)")

        log(f"   • Avaliando Gen 0 IA Direta ({ne} entregas) no 4D...")
        res_ia = avaliar_vetor_4d_com_cache(chave, vetor_ia, cenario, nd)
        log(f"     -> IA Direta Gen 0 : Makespan={res_ia['makespan']}f | Energia={res_ia['energia']}Wh | Custo={res_ia['custo']} ({res_ia['tempo_eval_s']}s)")

        frio_8gen = item.get("frio_ref") or cache_frio.get(chave)
        if frio_8gen:
            frio_custo = round(frio_8gen["makespan"] + 3.6 * frio_8gen["energia"] + 50000 * frio_8gen["falhas"], 1)
            frio_info = {**frio_8gen, "custo": frio_custo}
            ganho_vs_frio8 = pct(frio_8gen["makespan"], res_ia["makespan"])
            log(f"     -> GA Frio (8 Gen) : Makespan={frio_8gen['makespan']}f | Energia={frio_8gen['energia']}Wh | Custo={frio_custo} | Tempo={frio_8gen['tempo_s']}s")
            log(f"   🏆 Ganho IA Gen 0 vs Seq Gen 0: Makespan -{pct(res_seq['makespan'], res_ia['makespan'])}% | vs GA Frio 8-Gen: -{ganho_vs_frio8}%")
        else:
            frio_info = None
            ganho_vs_frio8 = None

        reg_atual.update({
            "seed": seed,
            "nd": nd,
            "ne": ne,
            "alg": alg,
            "nome": nome,
            "vetor_ia": [int(x) for x in vetor_ia],
            "gen0_sequencial": res_seq,
            "gen0_ia": res_ia,
            "ga_frio_8gen": frio_info,
            "ganho_gen0_ia_vs_seq_makespan_pct": pct(res_seq["makespan"], res_ia["makespan"]),
            "ganho_gen0_ia_vs_seq_custo_pct": pct(res_seq["custo"], res_ia["custo"]),
            "ganho_gen0_ia_vs_frio8gen_makespan_pct": ganho_vs_frio8,
        })
        relatorio["cenarios"][chave] = reg_atual
        salvar_json_atomico(RELATORIO_TODOS_JSON, relatorio)

    if not EXECUTAR_ETAPA_2:
        n_gen0_ok = sum(1 for v in relatorio["cenarios"].values() if "gen0_ia" in v)
        n_ga_ok = sum(1 for v in relatorio["cenarios"].values() if "ga_ia_8gen" in v)
        atualizar_status("ETAPA_1_CONCLUIDA_TODOS_11_CENARIOS", "Etapa 1 finalizada em todos os 11 cenários!", n_gen0_ok, n_ga_ok, total)
        log("\n🎉 ETAPA 1 CONCLUÍDA EM TODOS OS 11 CENÁRIOS (INCLUINDO OS CENÁRIOS MAIORES)!")
        return

    # =========================================================================
    # ETAPA 2: GA COMPLETO (8 GERAÇÕES) COM WARM START DA IA EM TODOS OS CENÁRIOS
    # =========================================================================
    log("\n🔥 [ETAPA 2/2] Rodando GA Completo (8 Gerações) com Warm Start da IA em TODOS os cenários...")
    maquina_atual = os.environ.get("COMPUTERNAME", "LOCAL").upper()
    ordem_etapa2 = list(CENARIOS_TESTE)

    for idx, item in enumerate(ordem_etapa2):
        seed, nd, ne, alg, nome = item["seed"], item["nd"], item["ne"], item["alg"], item["nome"]
        chave = f"{seed}|{nd}|{ne}|{alg}"

        relatorio = carregar_relatorio_existente()
        reg_atual = relatorio["cenarios"].get(chave, {})
        if "ga_ia_8gen" in reg_atual:
            log(f"⏩ [Etapa 2 — {maquina_atual}] {nome} (seed {seed}) já possui GA Completo 8-Gen salvo — pulando!")
            continue

        n_gen0_ok = sum(1 for v in relatorio["cenarios"].values() if "gen0_ia" in v)
        n_ga_ok = sum(1 for v in relatorio["cenarios"].values() if "ga_ia_8gen" in v)
        atualizar_status(f"ETAPA_2_GA_8GEN ({maquina_atual})", f"{nome} (seed {seed})", n_gen0_ok, n_ga_ok, total)

        log(f"\n=================================================================")
        log(f"🧬 [Etapa 2 — {maquina_atual} — {idx+1}/{total}] GA 8 Gerações Warm Start IA: {nome} (seed={seed})")
        log(f"=================================================================")

        cd_xy, missoes, cenario = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)
        vetor_ia = reg_atual.get("vetor_ia")
        if not vetor_ia:
            mod_usar = _obter_modelo_sem_grandes() if item.get("id_excluir_treino") else modelo
            vetor_ia = prever_alocacao(mod_usar, missoes, cd_xy, max_x, max_y, nd, perfis, max_entregas)
        np.random.seed(seed + 1)
        pop_ml = gerar_populacao_inicial(vetor_ia, tam_pop, nd)

        t0_ga = time.perf_counter()
        import importlib
        import teste.ga_optimizer as _ga_mod
        importlib.reload(_ga_mod)
        vetor_final, mk_ml, es_ml, fa_ml, en_ml, ger_ml = _ga_mod.executar_otimizacao_batch(
            300000 + (seed % 100), nd, ne, alg, cenario=cenario, populacao_inicial=pop_ml, seed_ga=seed
        )
        t_ga_wall = round(time.perf_counter() - t0_ga, 2)

        # Se houve retomada após desligamento da VM, usa o tempo acumulado salvo no cache de indivíduos
        tempo_total_ga = t_ga_wall
        if os.path.exists(CACHE_IND_JSON):
            try:
                with open(CACHE_IND_JSON, "r", encoding="utf-8") as f:
                    t_acum = float(json.load(f).get(chave, {}).get("__tempo_acum_s__", 0.0))
                if t_acum > tempo_total_ga:
                    tempo_total_ga = round(t_acum, 2)
            except Exception:
                pass

        custo_ml = round(mk_ml + 0.5 * es_ml + 3.6 * en_ml + 50000 * fa_ml, 1)

        ga_ia_info = {
            "geracoes": int(ger_ml),
            "makespan": int(mk_ml),
            "esperas": int(es_ml),
            "falhas": int(fa_ml),
            "energia": round(float(en_ml), 2),
            "custo": custo_ml,
            "tempo_s": tempo_total_ga,
            "vetor_final": [int(x) for x in vetor_final],
            "executado_em": maquina_atual,
        }

        # Recarrega o relatório do disco antes de salvar para mesclar sem sobrescrever a outra máquina
        relatorio = carregar_relatorio_existente()
        reg_atual = relatorio["cenarios"].get(chave, reg_atual)
        reg_atual["ga_ia_8gen"] = ga_ia_info
        frio_info = reg_atual.get("ga_frio_8gen")
        if frio_info:
            reg_atual["ganho_ga_ia_vs_frio_makespan_pct"] = pct(frio_info["makespan"], mk_ml)
            reg_atual["ganho_ga_ia_vs_frio_custo_pct"] = pct(frio_info["custo"], custo_ml)
            reg_atual["ganho_tempo_ga_pct"] = pct(frio_info["tempo_s"], tempo_total_ga)

        relatorio["cenarios"][chave] = reg_atual
        salvar_json_atomico(RELATORIO_TODOS_JSON, relatorio)
        exportar_tabela_completa(relatorio)

        # Enriquecimento com métricas de engenharia (kW, kWh e km de frota)
        try:
            kw_ml = round((en_ml * 3.6) / mk_ml, 3) if mk_ml > 0 else 0.0
            log(f"✅ [{maquina_atual}] Concluído {nome} (seed {seed}): Makespan={mk_ml}f | Energia={en_ml:.2f}Wh ({en_ml/1000.0:.3f}kWh) | Potência Média={kw_ml:.2f}kW | Gerações={ger_ml} | Tempo={tempo_total_ga}s")
        except Exception:
            log(f"✅ [{maquina_atual}] Concluído {nome} (seed {seed}): Makespan={mk_ml}f | Energia={en_ml:.2f}Wh | Gerações={ger_ml} | Tempo={tempo_total_ga}s")

    exportar_tabela_completa(relatorio)
    n_gen0_ok = sum(1 for v in relatorio["cenarios"].values() if "gen0_ia" in v)
    n_ga_ok = sum(1 for v in relatorio["cenarios"].values() if "ga_ia_8gen" in v)
    atualizar_status("CONCLUIDO", "Todos os cenários finalizados!", n_gen0_ok, n_ga_ok, total)
    log("\n🎉 BATERIA DE TODOS OS CENÁRIOS CONCLUÍDA COM SUCESSO!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        tb_str = traceback.format_exc()
        log(f"❌ ERRO FATAL: {e}\n{tb_str}")
        atualizar_status("ERRO", str(e), 0, 0, len(CENARIOS_TESTE))
        sys.exit(1)
