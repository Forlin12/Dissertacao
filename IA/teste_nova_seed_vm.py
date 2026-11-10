r"""
Bateria Completa 4D em TODOS os 9 Cenários da Dissertação com SEED ÚNICA:
  - 5 drones / 10 entregas
  - 5 drones / 20 entregas
  - 10 drones / 20 entregas
  - 5 drones / 40 entregas
  - 10 drones / 40 entregas
  - 20 drones / 40 entregas
  - 10 drones / 80 entregas
  - 20 drones / 80 entregas
  - 20 drones / 160 entregas

Todas as tarefas utilizam a MESMA seed (default: 910000), garantindo reprodutibilidade uniforme.
Compara as 4 configurações para cada cenário:
  1. gen(0) sem IA (Round-Robin sequencial, 0 gerações)
  2. gen(8) sem IA (GA Frio convencional, 8 gerações)
  3. gen(0) com IA (Chute inicial direto por ML, 0 gerações)
  4. gen(8) com IA (GA inicializado com Warm Start da IA, 8 gerações)

Calcula automaticamente as grandezas físicas e elétricas:
  - Distância total percorrida pela frota (km)
  - Makespan (frames e segundos)
  - Energia elétrica total (kWh e Wh)
  - Potência média útil da frota (kW)
  - Speedup e percentual de redução do Makespan
"""
import os
import sys
import json
import math
import time
import random
import argparse
import traceback
import csv

DIR_IA = os.path.dirname(os.path.abspath(__file__))
DIR_RAIZ = os.path.dirname(DIR_IA)

_local_cache = os.path.join(os.environ.get("LOCALAPPDATA", "C:\\Temp"), "dissertacao_vm_cache")
os.makedirs(_local_cache, exist_ok=True)
os.environ["NUMBA_CACHE_DIR"] = _local_cache
sys.pycache_prefix = _local_cache

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

LOG_PROGRESSO = os.path.join(DIR_IA, "vm_nova_seed_progresso.log")
STATUS_JSON = os.path.join(DIR_IA, "vm_status.json")
RELATORIO_JSON = os.path.join(DIR_IA, "relatorio_nova_seed_vm.json")
TABELA_CSV = os.path.join(DIR_IA, "tabela_nova_seed_completa.csv")
CACHE_IND_JSON = os.path.join(DIR_IA, "cache_individuos_nova_seed.json")
TSCLIENT_IA = r"\\tsclient\Z\IA"

# Tortuosidade média da malha urbana de Bragança (A* 3D com desvio de quinas)
FATOR_ROTA_URBANA_3D = 1.285

# 9 Cenários Canônicos da Dissertação (1 instância por escala)
CENARIOS_CANONICOS = [
    {"nd": 5,  "ne": 10,  "alg": "MOSP_CAMADAS", "nome": "5 drones / 10 entregas"},
    {"nd": 5,  "ne": 20,  "alg": "MOSP_CAMADAS", "nome": "5 drones / 20 entregas"},
    {"nd": 10, "ne": 20,  "alg": "MOSP_CAMADAS", "nome": "10 drones / 20 entregas"},
    {"nd": 5,  "ne": 40,  "alg": "MOSP_CAMADAS", "nome": "5 drones / 40 entregas"},
    {"nd": 10, "ne": 40,  "alg": "MOSP_CAMADAS", "nome": "10 drones / 40 entregas"},
    {"nd": 20, "ne": 40,  "alg": "MOSP_CAMADAS", "nome": "20 drones / 40 entregas"},
    {"nd": 10, "ne": 80,  "alg": "MOSP_CAMADAS", "nome": "10 drones / 80 entregas"},
    {"nd": 20, "ne": 80,  "alg": "MOSP_CAMADAS", "nome": "20 drones / 80 entregas"},
    {"nd": 20, "ne": 160, "alg": "MOSP_CAMADAS", "nome": "20 drones / 160 entregas"},
]


def _espelhar_para_notebook(caminho_local):
    try:
        if os.path.abspath(DIR_IA).upper().startswith("C:") and os.path.isdir(TSCLIENT_IA):
            import shutil
            nome = os.path.basename(caminho_local)
            shutil.copy2(caminho_local, os.path.join(TSCLIENT_IA, nome))
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
    try:
        if os.path.abspath(DIR_IA).upper().startswith("C:") and os.path.isdir(TSCLIENT_IA):
            with open(os.path.join(TSCLIENT_IA, "vm_nova_seed_progresso.log"), "a", encoding="utf-8") as f2:
                f2.write(linha + "\n")
    except Exception:
        pass


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
    _preparar_cenario,
    prever_alocacao,
    gerar_populacao_inicial,
    EnsembleFisicoEnxuto,
    MODELO_PATH,
    CIDADE_SEED,
)


def salvar_json_atomico(caminho, dados):
    tmp = caminho + f".tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(dados, f, indent=2, ensure_ascii=False)
    os.replace(tmp, caminho)
    _espelhar_para_notebook(caminho)


def atualizar_status(etapa, cenario_atual, concluidos, total, seed_usada):
    dados = {
        "atualizado_em": time.strftime("%Y-%m-%d %H:%M:%S"),
        "maquina": os.environ.get("COMPUTERNAME", "desconhecido"),
        "seed_usada": seed_usada,
        "etapa": etapa,
        "cenario_atual": cenario_atual,
        "cenarios_concluidos": concluidos,
        "total_cenarios": total,
    }
    salvar_json_atomico(STATUS_JSON, dados)


def avaliar_vetor_4d_com_cache(chave_cenario, vetor, cenario, num_drones):
    chave_sol = "-".join(str(int(x)) for x in vetor)
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
            tentativas[id_pedido] += 1
            if tentativas[id_pedido] < max_tentativas:
                fila_missoes.append(id_pedido)
            else:
                missoes_falhadas += 1
            drone.carga = carga_original
            continue

        for i, p in enumerate(cam_volta):
            z = p[2] if len(p) == 3 else drone.altura_alvo
            reserva_global[(p[0], p[1], z, t_volta + i)] = id_drone

        _, hovers_volta, _ = analisar_cenario(cam_volta)
        esperas_total += hovers_volta

        bateria_antes_volta = drone.bateria_atual_wh
        rota_volta, bateu_volta, _ = drone.simular_missao(cam_volta, lotes_gdf)
        energia_total += (bateria_antes_volta - drone.bateria_atual_wh)

        if bateu_volta:
            tentativas[id_pedido] += 1
            if tentativas[id_pedido] < max_tentativas:
                fila_missoes.append(id_pedido)
            else:
                missoes_falhadas += 1
            drone.carga = carga_original
            continue

        t_fim = t_volta + len(rota_volta)
        disponibilidade[id_drone] = t_fim
        drone.carga = carga_original
        if t_fim > makespan_frames:
            makespan_frames = t_fim

    dt = round(time.perf_counter() - t0, 2)
    custo = makespan_frames + (esperas_total * 0.5) + (energia_total * 3.6) + (missoes_falhadas * 50000)

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


def exportar_csv(relatorio):
    linhas = []
    for chave, c_info in relatorio["cenarios"].items():
        nome = c_info.get("nome", chave)
        nd = c_info.get("nd")
        ne = c_info.get("ne")
        seed = c_info.get("seed")
        dist_km = c_info.get("distancia_frota_km", 0.0)

        d_seq = c_info.get("gen0_sequencial", {}) or {}
        d_frio = c_info.get("ga_frio_8gen", {}) or {}
        d_ia_g0 = c_info.get("gen0_ia", {}) or {}
        d_ia_g8 = c_info.get("ga_ia_8gen", {}) or {}

        mk_seq = d_seq.get("makespan", 0)
        en_seq_wh = d_seq.get("energia", 0.0)
        en_seq_kwh = d_seq.get("energia_kwh", round(en_seq_wh / 1000.0, 3))
        kw_seq = d_seq.get("potencia_kw", 0.0)
        custo_seq = d_seq.get("custo", 0.0)
        t_sol_seq = d_seq.get("tempo_achar_solucao_s", d_seq.get("tempo_eval_s", 0.0))

        mk_frio = d_frio.get("makespan", 0)
        en_frio_wh = d_frio.get("energia", 0.0)
        en_frio_kwh = d_frio.get("energia_kwh", round(en_frio_wh / 1000.0, 3))
        kw_frio = d_frio.get("potencia_kw", 0.0)
        custo_frio = d_frio.get("custo", 0.0)
        t_sol_frio = d_frio.get("tempo_achar_solucao_s", d_frio.get("tempo_s", 0.0))
        ger_frio = d_frio.get("geracoes", 0)

        mk_ia0 = d_ia_g0.get("makespan", 0)
        en_ia0_wh = d_ia_g0.get("energia", 0.0)
        en_ia0_kwh = d_ia_g0.get("energia_kwh", round(en_ia0_wh / 1000.0, 3))
        kw_ia0 = d_ia_g0.get("potencia_kw", 0.0)
        custo_ia0 = d_ia_g0.get("custo", 0.0)
        t_inf_ia0 = d_ia_g0.get("tempo_inferencia_ia_s", 0.0)
        t_sol_ia0 = d_ia_g0.get("tempo_achar_solucao_s", d_ia_g0.get("tempo_eval_s", 0.0))

        mk_ia8 = d_ia_g8.get("makespan", 0)
        en_ia8_wh = d_ia_g8.get("energia", 0.0)
        en_ia8_kwh = d_ia_g8.get("energia_kwh", round(en_ia8_wh / 1000.0, 3))
        kw_ia8 = d_ia_g8.get("potencia_kw", 0.0)
        custo_ia8 = d_ia_g8.get("custo", 0.0)
        t_sol_ia8 = d_ia_g8.get("tempo_achar_solucao_s", d_ia_g8.get("tempo_s", 0.0))
        ger_ia8 = d_ia_g8.get("geracoes", 0)

        ganho_g0 = pct(mk_seq, mk_ia0)
        ganho_g8 = pct(mk_frio, mk_ia8) if mk_frio else 0.0
        ganho_custo_g0 = pct(custo_seq, custo_ia0)
        ganho_custo_g8 = pct(custo_frio, custo_ia8) if custo_frio else 0.0
        ganho_tempo_ga = pct(t_sol_frio, t_sol_ia8) if t_sol_frio else 0.0
        speedup = round(t_sol_frio / t_sol_ia8, 2) if (t_sol_ia8 > 0 and t_sol_frio > 0) else 1.0

        linhas.append({
            "cenario": nome,
            "drones": nd,
            "entregas": ne,
            "seed": seed,
            "distancia_km": dist_km,
            # Gen(0) Sem IA
            "gen0_sem_ia_tempo_entregas_s": mk_seq,
            "gen0_sem_ia_tempo_entregas_min": round(mk_seq / 60.0, 2),
            "gen0_sem_ia_energia_wh": en_seq_wh,
            "gen0_sem_ia_energia_kwh": en_seq_kwh,
            "gen0_sem_ia_potencia_kw": kw_seq,
            "gen0_sem_ia_custo_total": custo_seq,
            "gen0_sem_ia_tempo_solucao_s": t_sol_seq,
            # Gen(8) Sem IA (GA Frio)
            "gen8_sem_ia_tempo_entregas_s": mk_frio,
            "gen8_sem_ia_tempo_entregas_min": round(mk_frio / 60.0, 2),
            "gen8_sem_ia_energia_wh": en_frio_wh,
            "gen8_sem_ia_energia_kwh": en_frio_kwh,
            "gen8_sem_ia_potencia_kw": kw_frio,
            "gen8_sem_ia_custo_total": custo_frio,
            "gen8_sem_ia_tempo_solucao_s": t_sol_frio,
            "gen8_sem_ia_geracao_conv": ger_frio,
            # Gen(0) Com IA
            "gen0_com_ia_tempo_entregas_s": mk_ia0,
            "gen0_com_ia_tempo_entregas_min": round(mk_ia0 / 60.0, 2),
            "gen0_com_ia_energia_wh": en_ia0_wh,
            "gen0_com_ia_energia_kwh": en_ia0_kwh,
            "gen0_com_ia_potencia_kw": kw_ia0,
            "gen0_com_ia_custo_total": custo_ia0,
            "gen0_com_ia_tempo_inferencia_s": t_inf_ia0,
            "gen0_com_ia_tempo_solucao_s": t_sol_ia0,
            # Gen(8) Com IA (GA Warm Start)
            "gen8_com_ia_tempo_entregas_s": mk_ia8,
            "gen8_com_ia_tempo_entregas_min": round(mk_ia8 / 60.0, 2),
            "gen8_com_ia_energia_wh": en_ia8_wh,
            "gen8_com_ia_energia_kwh": en_ia8_kwh,
            "gen8_com_ia_potencia_kw": kw_ia8,
            "gen8_com_ia_custo_total": custo_ia8,
            "gen8_com_ia_tempo_solucao_s": t_sol_ia8,
            "gen8_com_ia_geracao_conv": ger_ia8,
            # Ganhos e Speedup
            "ganho_mk_gen0_ia_pct": ganho_g0,
            "ganho_mk_gen8_ia_pct": ganho_g8,
            "ganho_custo_gen0_ia_pct": ganho_custo_g0,
            "ganho_custo_gen8_ia_pct": ganho_custo_g8,
            "ganho_tempo_solucao_ga_pct": ganho_tempo_ga,
            "speedup_ga": speedup,
        })

    if linhas:
        with open(TABELA_CSV, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
            w.writeheader()
            w.writerows(linhas)
        _espelhar_para_notebook(TABELA_CSV)


def _obter_tempo_acumulado_ga(chave_ga, t_wall):
    caminho_cache_ga = os.path.join(DIR_IA, "cache_individuos_4d.json")
    if os.path.exists(caminho_cache_ga):
        try:
            with open(caminho_cache_ga, "r", encoding="utf-8") as f:
                t_acum = float(json.load(f).get(chave_ga, {}).get("__tempo_acum_s__", 0.0))
            if t_acum > t_wall:
                return round(t_acum, 2)
        except Exception:
            pass
    return round(t_wall, 2)


def main():
    parser = argparse.ArgumentParser(description="Bateria 4D com Seed Unica nos 9 Cenarios")
    parser.add_argument("--seed", type=int, default=910000, help="Seed unica para todos os cenarios (default: 910000)")
    args, _ = parser.parse_known_args()
    seed_unica = args.seed

    # Impede suspensão do Windows e mantém sessão ativa na VM
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
                for arq in (RELATORIO_JSON, TABELA_CSV, CACHE_IND_JSON, STATUS_JSON, LOG_PROGRESSO):
                    if os.path.exists(arq):
                        _espelhar_para_notebook(arq)
                time.sleep(20)
        threading.Thread(target=_keepalive_rdp, daemon=True).start()
    except Exception:
        pass

    log("=================================================================")
    log(f"🚀 INICIANDO TESTE COM SEED ÚNICA ({seed_unica}) EM TODOS OS 9 CENÁRIOS NA VM ({os.environ.get('COMPUTERNAME', 'LOCAL')})")
    log("=================================================================")

    pacote = _carregar_pacote(MODELO_PATH)
    modelo = pacote["modelo"]
    max_entregas = pacote.get("max_entregas", 160)

    relatorio = {"cenarios": {}, "configuracao": {"seed_unica": seed_unica}}
    if os.path.exists(RELATORIO_JSON):
        try:
            with open(RELATORIO_JSON, "r", encoding="utf-8") as f:
                relatorio = json.load(f)
        except Exception:
            relatorio = {"cenarios": {}, "configuracao": {"seed_unica": seed_unica}}

    log("🏙️ Carregando malha urbana 3D (CIDADE_SEED=42) a partir do cache local...")
    try:
        import osmnx as ox
        ox.settings.cache_folder = os.path.join(DIR_RAIZ, "cache")
        ox.settings.use_cache = True
    except Exception:
        pass
    cfg.CENARIO_SEMPRE_NOVO = False
    random.seed(CIDADE_SEED)
    np.random.seed(CIDADE_SEED)
    lotes_gdf, max_x, max_y = gerar_cidade()
    perfis = cfg.PERFIS_DRONES
    tam_pop = getattr(cfg, "GA_SOL_POP", 10)

    total = len(CENARIOS_CANONICOS)

    for idx, item in enumerate(CENARIOS_CANONICOS):
        nd, ne, alg, nome = item["nd"], item["ne"], item["alg"], item["nome"]
        seed = seed_unica
        chave = f"{seed}|{nd}|{ne}|{alg}"

        reg_atual = relatorio["cenarios"].get(chave, {})
        if "ga_ia_8gen" in reg_atual and "ga_frio_8gen" in reg_atual and "gen0_ia" in reg_atual and "gen0_sequencial" in reg_atual:
            log(f"⏩ [Cenário {idx+1}/{total}] {nome} (seed {seed}) já concluído — pulando!")
            continue

        atualizar_status("EXECUTANDO", f"{nome} (seed {seed})", idx, total, seed)

        log(f"\n=================================================================")
        log(f"📍 [Cenário {idx+1}/{total}] {nome} | Seed Única = {seed}")
        log(f"=================================================================")

        cd_xy, missoes, cenario = _preparar_cenario(seed, nd, ne, alg, lotes_gdf, max_x, max_y)

        # Distância total 3D física percorrida pela frota
        dist_frota_metros = sum(2.0 * (math.hypot(m["destino"][0] - m["origem"][0], m["destino"][1] - m["origem"][1]) * FATOR_ROTA_URBANA_3D + 50.0) for m in missoes)
        dist_frota_km = round(dist_frota_metros / 1000.0, 2)
        log(f"   📏 Distância física total da frota: {dist_frota_km} km ({len(missoes)} entregas)")

        t0_seq_aloc = time.perf_counter()
        tb = math.ceil(ne / nd)
        vetor_seq = [(i // tb) % nd for i in range(ne)]
        t_aloc_seq = round(time.perf_counter() - t0_seq_aloc, 6)

        t0_ia_inf = time.perf_counter()
        vetor_ia = prever_alocacao(modelo, missoes, cd_xy, max_x, max_y, nd, perfis, max_entregas)
        t_inf_ia = round(time.perf_counter() - t0_ia_inf, 4)

        reg_atual["seed"] = seed
        reg_atual["nd"] = nd
        reg_atual["ne"] = ne
        reg_atual["alg"] = alg
        reg_atual["nome"] = nome
        reg_atual["distancia_frota_km"] = dist_frota_km
        reg_atual["vetor_sequencial"] = [int(x) for x in vetor_seq]
        reg_atual["vetor_ia"] = [int(x) for x in vetor_ia]

        # -------------------------------------------------------------
        # 1. Geração 0 Sequencial (Sem IA)
        # -------------------------------------------------------------
        if "gen0_sequencial" not in reg_atual:
            log(f"   • Avaliando [1/4] Geração 0 Sem IA (Sequencial)...")
            res_seq = avaliar_vetor_4d_com_cache(chave, vetor_seq, cenario, nd)
            kw_seq = round((res_seq["energia"] * 3.6) / res_seq["makespan"], 3) if res_seq["makespan"] > 0 else 0.0
            res_seq["geracoes"] = 0
            res_seq["tempo_entregas_s"] = int(res_seq["makespan"])
            res_seq["tempo_entregas_min"] = round(res_seq["makespan"] / 60.0, 2)
            res_seq["distancia_km"] = dist_frota_km
            res_seq["energia_wh"] = round(float(res_seq["energia"]), 2)
            res_seq["energia_kwh"] = round(res_seq["energia"] / 1000.0, 3)
            res_seq["potencia_kw"] = kw_seq
            res_seq["tempo_alocacao_s"] = t_aloc_seq
            res_seq["tempo_achar_solucao_s"] = round(t_aloc_seq + float(res_seq["tempo_eval_s"]), 2)
            res_seq["vetor_final"] = [int(x) for x in vetor_seq]
            reg_atual["gen0_sequencial"] = res_seq
            relatorio["cenarios"][chave] = reg_atual
            salvar_json_atomico(RELATORIO_JSON, relatorio)
            log(f"     -> Gen(0) Sem IA: Tempo Entregas={res_seq['makespan']}s ({res_seq['tempo_entregas_min']}min) | Energia={res_seq['energia_wh']}Wh ({res_seq['energia_kwh']}kWh) | Custo Total={res_seq['custo']} | Tempo Solução={res_seq['tempo_achar_solucao_s']}s")
        else:
            res_seq = reg_atual["gen0_sequencial"]

        # -------------------------------------------------------------
        # 2. Geração 0 com IA (Chute Inicial Direto ML)
        # -------------------------------------------------------------
        if "gen0_ia" not in reg_atual:
            log(f"   • Avaliando [2/4] Geração 0 Com IA (Alocação Direta ML)...")
            res_ia = avaliar_vetor_4d_com_cache(chave, vetor_ia, cenario, nd)
            kw_ia = round((res_ia["energia"] * 3.6) / res_ia["makespan"], 3) if res_ia["makespan"] > 0 else 0.0
            res_ia["geracoes"] = 0
            res_ia["tempo_entregas_s"] = int(res_ia["makespan"])
            res_ia["tempo_entregas_min"] = round(res_ia["makespan"] / 60.0, 2)
            res_ia["distancia_km"] = dist_frota_km
            res_ia["energia_wh"] = round(float(res_ia["energia"]), 2)
            res_ia["energia_kwh"] = round(res_ia["energia"] / 1000.0, 3)
            res_ia["potencia_kw"] = kw_ia
            res_ia["tempo_inferencia_ia_s"] = t_inf_ia
            res_ia["tempo_achar_solucao_s"] = round(t_inf_ia + float(res_ia["tempo_eval_s"]), 2)
            res_ia["vetor_final"] = [int(x) for x in vetor_ia]
            reg_atual["gen0_ia"] = res_ia
            relatorio["cenarios"][chave] = reg_atual
            salvar_json_atomico(RELATORIO_JSON, relatorio)
            log(f"     -> Gen(0) Com IA: Tempo Entregas={res_ia['makespan']}s ({res_ia['tempo_entregas_min']}min) | Energia={res_ia['energia_wh']}Wh ({res_ia['energia_kwh']}kWh) | Custo Total={res_ia['custo']} | Tempo Inferência={t_inf_ia}s | Tempo Solução+Sim={res_ia['tempo_achar_solucao_s']}s")
        else:
            res_ia = reg_atual["gen0_ia"]

        # -------------------------------------------------------------
        # 3. Geração 8 Sem IA (GA Frio Convencional)
        # -------------------------------------------------------------
        if "ga_frio_8gen" not in reg_atual:
            log(f"   • Otimizando [3/4] Gen(8) Sem IA — GA Frio Convencional (8 Gerações)...")
            cfg.USAR_ALOCACAO_IA = False
            t0_frio = time.perf_counter()
            vfrio, mk_frio, es_frio, fa_frio, en_frio, ger_frio = executar_otimizacao_batch(
                100000 + idx, nd, ne, alg, cenario=cenario, populacao_inicial=None, seed_ga=seed
            )
            t_frio_wall = round(time.perf_counter() - t0_frio, 2)
            t_frio = _obter_tempo_acumulado_ga(f"{seed}|{nd}|{ne}|{alg}|FRIO", t_frio_wall)
            kw_frio = round((en_frio * 3.6) / mk_frio, 3) if mk_frio > 0 else 0.0
            custo_frio = round(mk_frio + 0.5 * es_frio + 3.6 * en_frio + 50000 * fa_frio, 1)
            reg_atual["ga_frio_8gen"] = {
                "geracoes": int(ger_frio),
                "makespan": int(mk_frio),
                "tempo_entregas_s": int(mk_frio),
                "tempo_entregas_min": round(mk_frio / 60.0, 2),
                "esperas": int(es_frio),
                "falhas": int(fa_frio),
                "energia": round(float(en_frio), 2),
                "energia_wh": round(float(en_frio), 2),
                "energia_kwh": round(float(en_frio) / 1000.0, 3),
                "potencia_kw": kw_frio,
                "custo": custo_frio,
                "distancia_km": dist_frota_km,
                "tempo_s": t_frio,
                "tempo_achar_solucao_s": t_frio,
                "vetor_final": [int(x) for x in vfrio],
            }
            relatorio["cenarios"][chave] = reg_atual
            salvar_json_atomico(RELATORIO_JSON, relatorio)
            log(f"     -> Gen(8) Sem IA: Tempo Entregas={mk_frio}s ({mk_frio/60.0:.2f}min) | Energia={en_frio:.2f}Wh ({en_frio/1000.0:.3f}kWh) | Custo Total={custo_frio} | Tempo Achar Solução={t_frio}s")

        # -------------------------------------------------------------
        # 4. Geração 8 com IA (GA Warm Start da IA)
        # -------------------------------------------------------------
        if "ga_ia_8gen" not in reg_atual:
            log(f"   • Otimizando [4/4] Gen(8) Com IA — GA com Warm Start da IA (8 Gerações)...")
            cfg.USAR_ALOCACAO_IA = True
            np.random.seed(seed + 1)
            pop_ml = gerar_populacao_inicial(vetor_ia, tam_pop, nd)
            t0_ia8 = time.perf_counter()
            via8, mk_ia8, es_ia8, fa_ia8, en_ia8, ger_ia8 = executar_otimizacao_batch(
                200000 + idx, nd, ne, alg, cenario=cenario, populacao_inicial=pop_ml, seed_ga=seed
            )
            t_ia8_wall = round(time.perf_counter() - t0_ia8, 2)
            t_ia8_ga = _obter_tempo_acumulado_ga(f"{seed}|{nd}|{ne}|{alg}|IA", t_ia8_wall)
            t_eval_g0_ia = float(reg_atual.get("gen0_ia", {}).get("tempo_eval_s", 0.0))
            if t_ia8_ga < t_eval_g0_ia:
                t_ia8_ga = round(t_eval_g0_ia + t_ia8_ga, 2)
            t_ia8 = round(t_inf_ia + t_ia8_ga, 2)
            kw_ia8 = round((en_ia8 * 3.6) / mk_ia8, 3) if mk_ia8 > 0 else 0.0
            custo_ia8 = round(mk_ia8 + 0.5 * es_ia8 + 3.6 * en_ia8 + 50000 * fa_ia8, 1)
            reg_atual["ga_ia_8gen"] = {
                "geracoes": int(ger_ia8),
                "makespan": int(mk_ia8),
                "tempo_entregas_s": int(mk_ia8),
                "tempo_entregas_min": round(mk_ia8 / 60.0, 2),
                "esperas": int(es_ia8),
                "falhas": int(fa_ia8),
                "energia": round(float(en_ia8), 2),
                "energia_wh": round(float(en_ia8), 2),
                "energia_kwh": round(float(en_ia8) / 1000.0, 3),
                "potencia_kw": kw_ia8,
                "custo": custo_ia8,
                "distancia_km": dist_frota_km,
                "tempo_inferencia_ia_s": t_inf_ia,
                "tempo_s": t_ia8,
                "tempo_achar_solucao_s": t_ia8,
                "vetor_final": [int(x) for x in via8],
            }
            relatorio["cenarios"][chave] = reg_atual
            salvar_json_atomico(RELATORIO_JSON, relatorio)
            log(f"     -> Gen(8) Com IA: Tempo Entregas={mk_ia8}s ({mk_ia8/60.0:.2f}min) | Energia={en_ia8:.2f}Wh ({en_ia8/1000.0:.3f}kWh) | Custo Total={custo_ia8} | Tempo Achar Solução={t_ia8}s")

        # Métricas consolidadas do cenário
        mk_seq = reg_atual["gen0_sequencial"]["makespan"]
        mk_ia0 = reg_atual["gen0_ia"]["makespan"]
        mk_frio = reg_atual["ga_frio_8gen"]["makespan"]
        mk_ia8 = reg_atual["ga_ia_8gen"]["makespan"]

        custo_seq = reg_atual["gen0_sequencial"]["custo"]
        custo_ia0 = reg_atual["gen0_ia"]["custo"]
        custo_frio = reg_atual["ga_frio_8gen"]["custo"]
        custo_ia8 = reg_atual["ga_ia_8gen"]["custo"]

        reg_atual["ganho_gen0_ia_vs_seq_pct"] = pct(mk_seq, mk_ia0)
        reg_atual["ganho_gen8_ia_vs_frio_pct"] = pct(mk_frio, mk_ia8)
        reg_atual["ganho_custo_gen0_ia_pct"] = pct(custo_seq, custo_ia0)
        reg_atual["ganho_custo_gen8_ia_pct"] = pct(custo_frio, custo_ia8)
        t_frio = reg_atual["ga_frio_8gen"]["tempo_s"]
        t_ia8 = reg_atual["ga_ia_8gen"]["tempo_s"]
        reg_atual["ganho_tempo_solucao_ga_pct"] = pct(t_frio, t_ia8)
        reg_atual["speedup_ga"] = round(t_frio / t_ia8, 2) if (t_ia8 > 0 and t_frio > 0) else 1.0

        relatorio["cenarios"][chave] = reg_atual
        salvar_json_atomico(RELATORIO_JSON, relatorio)
        exportar_csv(relatorio)

        log(f"🏆 Resumo {nome}: Ganho Tempo Entregas G0 IA = -{reg_atual['ganho_gen0_ia_vs_seq_pct']}% | Ganho G8 IA = -{reg_atual['ganho_gen8_ia_vs_frio_pct']}% | Redução Tempo Solução = -{reg_atual['ganho_tempo_solucao_ga_pct']}% (Speedup {reg_atual['speedup_ga']}x)\n")

    atualizar_status("CONCLUIDO", "Todos os 9 cenarios canônicos finalizados!", total, total, seed_unica)
    log("\n🎉 BATERIA COMPLETA COM SEED ÚNICA FINALIZADA COM SUCESSO!")
    exportar_csv(relatorio)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        tb_str = traceback.format_exc()
        log(f"❌ ERRO FATAL: {e}\n{tb_str}")
        atualizar_status("ERRO", str(e), 0, len(CENARIOS_CANONICOS), 0)
        sys.exit(1)
