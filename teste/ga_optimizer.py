# ga_optimizer.py
import os
import math
import time
import threading
import numpy as np
import pygad
import config as cfg
from city_builder import gerar_cidade
import mission_control
from uav_physics import Drone

from main import roteador_inteligente, analisar_cenario
from teste.logger_system import TrainingLogger


# ==========================================
# UTILITÁRIOS DE LOG
# ==========================================
def log_secao(titulo):
    print(f"\n{'=' * 60}")
    print(f"  {titulo}")
    print(f"{'=' * 60}")


def log_ok(msg):   print(f"  ✅ {msg}")


def log_info(msg): print(f"  ℹ️  {msg}")


def log_warn(msg): print(f"  ⚠️  {msg}")


def log_tempo(msg, t0): print(f"  ⏱️  {msg}: {time.perf_counter() - t0:.2f}s")


# ==========================================
# MOTOR PRINCIPAL DO LOTE
# ==========================================
def executar_otimizacao_batch(teste_idx, num_drones, num_entregas, algoritmo,
                              cenario=None, populacao_inicial=None, seed_ga=None):
    """
    cenario: dict {'lotes_gdf','max_x','max_y','cds','missoes'}. Se fornecido, o GA usa
             EXATAMENTE este cenário (em vez de o regenerar com seed 42).
    populacao_inicial: lista de vetores (warm start da IA). None = chute sequencial (a frio).
    seed_ga: seed do numpy para o GA (comparações a frio/quente justas).
    Devolve: (vetor, makespan, esperas, falhas, energia, geracao_de_convergencia)
    """
    t_total_inicio = time.perf_counter()

    log_secao(f"INICIANDO BATCH {teste_idx + 1} | Drones: {num_drones} | Entregas: {num_entregas} | Alg: {algoritmo}")

    # Inicializa o Logger específico para este lote de testes (Live Tracking do GA)
    logger_ga = TrainingLogger(prefixo=f"otimizador_batch_{teste_idx + 1}")

    cfg.NUM_DRONES_DISPONIVEIS = num_drones
    cfg.NUM_ENTREGAS_TOTAL = num_entregas
    cfg.TIPO_ALGORITMO = algoritmo

    if cenario is None and not getattr(cfg, 'CENARIO_SEMPRE_NOVO', False):
        np.random.seed(42)

    NUM_CORES = os.cpu_count() or 4
    TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)
    VETOR_CAMADAS_VOO = getattr(cfg, 'VETOR_CAMADAS_VOO', [40, 50, 60, 70])
    VETOR_TEMPOS_PARTIDA = getattr(cfg, 'VETOR_TEMPOS_PARTIDA', [0])

    SOL_POP = getattr(cfg, 'GA_SOL_POP', 24)
    NUM_GERACOES = getattr(cfg, 'GA_NUM_GERACOES', 8)

    log_info(f"População GA: {SOL_POP} soluções × {NUM_GERACOES} gerações paralelas")

    # ==========================================
    # CARREGAR CENÁRIO
    # ==========================================
    if cenario is not None:
        lotes_gdf = cenario['lotes_gdf']
        max_x, max_y = cenario['max_x'], cenario['max_y']
        cds, missoes = cenario['cds'], cenario['missoes']
        NUM_PEDIDOS = len(missoes)
        log_info("Cenário externo recebido (missões e CD fornecidos pelo chamador)")
    else:
        t0 = time.perf_counter()
        lotes_gdf, max_x, max_y = gerar_cidade()
        log_tempo("Cidade gerada", t0)

        if not getattr(cfg, 'CENARIO_SEMPRE_NOVO', False):
            import random
            _seed_cen = getattr(cfg, 'SEED_CENARIO', 777777)
            random.seed(_seed_cen)
            np.random.seed(_seed_cen)

        t0 = time.perf_counter()
        cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)
        NUM_PEDIDOS = len(missoes)
        log_tempo("Missões geradas", t0)

    if seed_ga is not None:
        np.random.seed(seed_ga)

    num_perfis = len(cfg.PERFIS_DRONES)

    def criar_frota():
        return [Drone(cfg.PERFIS_DRONES[i % num_perfis]) for i in range(num_drones)]

    pontos_missoes = [
        {'start': (int(m['origem'][0]), int(m['origem'][1])),
         'goal': (int(m['destino'][0]), int(m['destino'][1])),
         'peso_kg': m.get('peso_kg', 1.0)}
        for m in missoes
    ]

    # ==========================================
    # CACHE DE ROTAS NOMINAIS E DESVIO 3D VIA MOSP (ALTERNATIVA 1)
    # ==========================================
    _cache_rotas_nominais = {}
    _cache_rotas_lock = threading.Lock()
    _rotas_nominais_usadas = [0]
    _desvios_mosp_executados = [0]

    def _obter_rota_nominal(drone_ref, p_start, p_goal, carga_kg):
        """Retorna rota estática nominal (sem tráfego) compartilhada entre todas as threads."""
        chave = (p_start, p_goal, drone_ref.nome_modelo, round(float(carga_kg), 3))
        with _cache_rotas_lock:
            if chave in _cache_rotas_nominais:
                return _cache_rotas_nominais[chave]

        # Calcula a rota estática nominal usando o roteador inteligente com reserva vazia
        drone_temp = Drone(drone_ref.perfil)
        drone_temp.carga = carga_kg
        cam_nom, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone_temp, p_start, p_goal, {}, 0)

        with _cache_rotas_lock:
            _cache_rotas_nominais[chave] = cam_nom
        return cam_nom

    def _verificar_conflito_4d(cam, t_inicio, id_drone_atual, reserva_map, altura_alvo_drone):
        """Detecta em O(comprimento_rota) se a rota nominal colide com reservas de outros drones."""
        if not cam or not reserva_map:
            return False
        for offset, p in enumerate(cam):
            z = p[2] if len(p) == 3 else altura_alvo_drone
            t_momento = t_inicio + offset
            dono = reserva_map.get((p[0], p[1], z, t_momento))
            if dono is not None and dono != id_drone_atual:
                return True
            # Checagem de colisão frontal cruzada (head-on swap)
            if offset > 0:
                p_ant = cam[offset - 1]
                z_ant = p_ant[2] if len(p_ant) == 3 else altura_alvo_drone
                dono_ant = reserva_map.get((p[0], p[1], z, t_momento - 1))
                dono_post = reserva_map.get((p_ant[0], p_ant[1], z_ant, t_momento))
                if dono_ant is not None and dono_ant == dono_post and dono_ant != id_drone_atual:
                    return True
        return False

    # ==========================================
    # SIMULAÇÃO FÍSICA ESTRITA
    # ==========================================
    def simular_com_tea(solution, teto_makespan=math.inf, verbose=False):
        frota_local = criar_frota()
        reserva_global = {}
        disponibilidade = {i: 0 for i in range(num_drones)}
        makespan_frames = 0
        esperas_total = 0
        missoes_falhadas = 0
        energia_total_da_simulacao = 0.0

        fila_missoes = list(range(NUM_PEDIDOS))
        tentativas = {i: 0 for i in range(NUM_PEDIDOS)}
        MAX_TENTATIVAS = 2

        while fila_missoes:
            id_pedido = fila_missoes.pop(0)
            id_drone = int(solution[id_pedido])
            drone = frota_local[id_drone]

            if tentativas[id_pedido] == 0:
                drone.reset_metricas()

            carga_original = drone._carga_original
            peso_pacote = pontos_missoes[id_pedido]['peso_kg']
            if peso_pacote > carga_original:
                peso_pacote = carga_original
            drone.carga = peso_pacote

            start = pontos_missoes[id_pedido]['start']
            goal = pontos_missoes[id_pedido]['goal']

            t_base = VETOR_TEMPOS_PARTIDA[id_pedido] if id_pedido < len(VETOR_TEMPOS_PARTIDA) else \
                VETOR_TEMPOS_PARTIDA[-1] + ((id_pedido - len(VETOR_TEMPOS_PARTIDA) + 1) * 15)

            t_ida = max(t_base, disponibilidade[id_drone])

            dist_linear = np.hypot(goal[0] - start[0], goal[1] - start[1])
            energia_estimada = drone.estimar_energia_missao(dist_linear, dist_linear, TEMPO_DESCARGA, peso_pacote)
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

            # ROTEAMENTO INTELIGENTE IDA: Tenta rota nominal rápida; se houver conflito 4D, invoca MOSP 3D
            cam_nom_ida = _obter_rota_nominal(drone, start, goal, peso_pacote)
            if cam_nom_ida and not _verificar_conflito_4d(cam_nom_ida, t_ida, id_drone, reserva_global, drone.altura_alvo):
                cam_ida = cam_nom_ida
                _rotas_nominais_usadas[0] += 1
            else:
                cam_ida, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, t_ida)
                _desvios_mosp_executados[0] += 1

            if not cam_ida:
                tentativas[id_pedido] += 1
                if tentativas[id_pedido] < MAX_TENTATIVAS:
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
            energia_total_da_simulacao += (bateria_antes_ida - drone.bateria_atual_wh)

            if bateu:
                tentativas[id_pedido] += 1
                if tentativas[id_pedido] < MAX_TENTATIVAS:
                    fila_missoes.append(id_pedido)
                else:
                    missoes_falhadas += 1
                drone.carga = carga_original
                continue

            drone.carga = 0.0

            if TEMPO_DESCARGA > 0:
                custo_peso_m = peso_pacote * drone.penalidade_carga
                consumo_descarga_wh = TEMPO_DESCARGA * (drone.consumo_hover + (custo_peso_m * drone.velocidade_horiz))
                drone.bateria_atual_wh -= consumo_descarga_wh
                energia_total_da_simulacao += consumo_descarga_wh

            t_volta = t_ida + len(rota_ida) + TEMPO_DESCARGA

            for extra in range(TEMPO_DESCARGA):
                z_descarga = cam_ida[-1][2] if len(cam_ida[-1]) == 3 else drone.altura_alvo
                reserva_global[(cam_ida[-1][0], cam_ida[-1][1], z_descarga, t_ida + len(cam_ida) + extra)] = id_drone

            # ROTEAMENTO INTELIGENTE VOLTA: Tenta rota nominal rápida; se houver conflito 4D, invoca MOSP 3D
            cam_nom_volta = _obter_rota_nominal(drone, goal, start, 0.0)
            if cam_nom_volta and not _verificar_conflito_4d(cam_nom_volta, t_volta, id_drone, reserva_global, drone.altura_alvo):
                cam_volta = cam_nom_volta
                _rotas_nominais_usadas[0] += 1
            else:
                cam_volta, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, goal, start, reserva_global, t_volta)
                _desvios_mosp_executados[0] += 1

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
            energia_total_da_simulacao += (bateria_antes_volta - drone.bateria_atual_wh)

            if bateu_v:
                tentativas[id_pedido] += 1
                if tentativas[id_pedido] < MAX_TENTATIVAS:
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

            if not verbose and makespan_frames > teto_makespan + 50:
                return makespan_frames, esperas_total, missoes_falhadas + len(fila_missoes), energia_total_da_simulacao, False

        return makespan_frames, esperas_total, missoes_falhadas, energia_total_da_simulacao, True

    # ==========================================
    # CACHE PERSISTENTE DE INDIVÍDUOS 4D (BLINDAGEM CONTRA DESLIGAMENTO DA VM)
    # ==========================================
    import json
    _dir_raiz = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _caminho_cache_ind = os.path.join(_dir_raiz, "IA", "cache_individuos_4d.json")
    _modo_tag = "IA" if (populacao_inicial is not None and len(populacao_inicial) > 0) else "FRIO"
    _chave_cenario_cache = f"{seed_ga or getattr(cfg, 'SEED_CENARIO', 42)}|{num_drones}|{NUM_PEDIDOS}|{algoritmo}|{_modo_tag}"
    _chave_cenario_legado = f"{seed_ga or getattr(cfg, 'SEED_CENARIO', 42)}|{num_drones}|{NUM_PEDIDOS}|{algoritmo}"
    _cache_disco_lock = threading.Lock()
    _cache_disco_mem = {}
    if os.path.exists(_caminho_cache_ind):
        try:
            with open(_caminho_cache_ind, "r", encoding="utf-8") as _f_c:
                _dados_c = json.load(_f_c)
                _cache_disco_mem = _dados_c.get(_chave_cenario_cache, _dados_c.get(_chave_cenario_legado, {})).copy()
        except Exception:
            _cache_disco_mem = {}
    _tempo_acum_anterior = float(_cache_disco_mem.get("__tempo_acum_s__", 0.0))

    def _salvar_ind_disco(chave_sol, mk, esp, fal, eng, completo, t_eval):
        with _cache_disco_lock:
            _cache_disco_mem[chave_sol] = {
                "makespan": int(mk),
                "esperas": int(esp),
                "falhas": int(fal),
                "energia": round(float(eng), 3),
                "completo": bool(completo),
                "tempo_s": round(float(t_eval), 2),
            }
            _cache_disco_mem["__tempo_acum_s__"] = round(_tempo_acum_anterior + (time.perf_counter() - t_total_inicio), 2)
            try:
                todos = {}
                if os.path.exists(_caminho_cache_ind):
                    with open(_caminho_cache_ind, "r", encoding="utf-8") as f_in:
                        todos = json.load(f_in)
                todos[_chave_cenario_cache] = _cache_disco_mem
                tmp_p = _caminho_cache_ind + f".tmp.{os.getpid()}.{threading.get_ident()}"
                with open(tmp_p, "w", encoding="utf-8") as f_out:
                    json.dump(todos, f_out)
                os.replace(tmp_p, _caminho_cache_ind)
            except Exception:
                pass

    _simular_com_tea_raw = simular_com_tea

    def simular_com_tea(solution, teto_makespan=math.inf, verbose=False):
        chave_sol = ",".join(str(int(g)) for g in solution)
        with _cache_disco_lock:
            reg = _cache_disco_mem.get(chave_sol)
        if reg is not None:
            if reg.get("completo", True):
                return reg["makespan"], reg["esperas"], reg["falhas"], reg["energia"]
            if not verbose and reg["makespan"] > teto_makespan + 50:
                return reg["makespan"], reg["esperas"], reg["falhas"], reg["energia"]

        t0_ev = time.perf_counter()
        mk, esp, fal, eng, completo = _simular_com_tea_raw(solution, teto_makespan=teto_makespan, verbose=verbose)
        t_ev = time.perf_counter() - t0_ev
        _salvar_ind_disco(chave_sol, mk, esp, fal, eng, completo, t_ev)
        return mk, esp, fal, eng

    # ==========================================
    # ESTRUTURAS HPC SEGURAS E TRACKING AO VIVO
    # ==========================================
    _simulacoes_reais = [0]

    # NOVO: Agora rastreamos o CUSTO TOTAL (Tempo + Energia), e não apenas o Tempo (Makespan)
    _melhor_custo_visual = [math.inf]

    _ga_lock = threading.Lock()
    _fitness_cache = {}
    FATOR_CONVERSAO_ENERGIA = 3.6  # 1 Wh = 3.6 segundos de penalização (recarga a 1000W)

    def fitness_func(ga_instance, solution, solution_idx):
        sol_tuple = tuple(int(g) for g in solution)

        with _ga_lock:
            if sol_tuple in _fitness_cache:
                return _fitness_cache[sol_tuple]
            recorde_atual_custo = _melhor_custo_visual[0]

        # PODA TEÓRICA FÍSICA: Estimativa conservadora de Tempo + Bateria usando o perfil REAL de cada drone
        tempo_ideal_drones = [0.0] * num_drones
        energia_ideal_drones = 0.0

        for id_pedido, id_drone in enumerate(sol_tuple):
            perfil_d = cfg.PERFIS_DRONES[id_drone % num_perfis]
            vel_max_d = max(float(perfil_d.get('velocidade_cruzeiro_ms', 15.0)), 1.0)
            cons_min_d = float(perfil_d.get('consumo_cruzeiro_wh_m', 0.015))

            start = pontos_missoes[id_pedido]['start']
            goal = pontos_missoes[id_pedido]['goal']
            dist_reta = math.hypot(goal[0] - start[0], goal[1] - start[1])

            frames_ideais = ((2.0 * dist_reta) / vel_max_d) + TEMPO_DESCARGA
            energia_ideal = (2.0 * dist_reta * cons_min_d)

            tempo_ideal_drones[id_drone] += frames_ideais
            energia_ideal_drones += energia_ideal

        makespan_teorico = max(tempo_ideal_drones)
        custo_teorico_perfeito = makespan_teorico + (energia_ideal_drones * FATOR_CONVERSAO_ENERGIA)

        if recorde_atual_custo != math.inf and custo_teorico_perfeito >= recorde_atual_custo:
            fitness_rejeitado = 1.0 / (custo_teorico_perfeito + 100000)
            with _ga_lock:
                _fitness_cache[sol_tuple] = fitness_rejeitado
            return fitness_rejeitado

        teto = math.inf
        if hasattr(ga_instance, "best_solutions_fitness") and ga_instance.best_solutions_fitness:
            melhor_fitness = np.max(ga_instance.best_solutions_fitness)
            teto = (1.0 / melhor_fitness)

        makespan, esperas, falhas, energia = simular_com_tea(sol_tuple, teto_makespan=teto)

        penalidade = falhas * 50000

        # O Custo agora reflete o esforço real do drone (Tempo de voo + Tempo equivalente de recarga)
        custo = makespan + (esperas * 0.5) + penalidade + (energia * FATOR_CONVERSAO_ENERGIA)
        fitness = 1.0 / (custo + 0.0001)

        with _ga_lock:
            _simulacoes_reais[0] += 1
            if falhas == 0 and custo < _melhor_custo_visual[0]:
                _melhor_custo_visual[0] = custo

                # Calcular tempo decorrido desde o início deste batch específico
                tempo_decorrido = time.perf_counter() - t_total_inicio

                print(
                    f"\n  ⭐ NOVO RECORDE: {makespan} frames! (Esperas: {esperas} | Falhas: 0 | Energia: {energia:.1f}Wh | Custo Físico: {custo:.1f})",
                    flush=True)

                # SALVA-VIDAS EM DISCO RÍGIDO IMEDIATO
                logger_ga.salvar_novo_recorde_ga(
                    tempo_execucao=tempo_decorrido,
                    makespan=makespan,
                    energia=energia,
                    esperas=esperas,
                    vetor_solucao=sol_tuple
                )
            else:
                print(".", end="", flush=True)
            _fitness_cache[sol_tuple] = fitness

        return fitness

    # Geração em que o melhor fitness melhorou pela última vez (0 = população inicial já era a melhor)
    _estado_conv = {"melhor": -math.inf, "gen": 0, "historico_custo": []}

    def on_start(ga_instance):
        try:
            f = ga_instance.cal_pop_fitness()
            best_f = float(np.max(f))
            _estado_conv["melhor"] = best_f
            _estado_conv["historico_custo"].append(round(1.0 / best_f, 1))
        except Exception:
            _estado_conv["melhor"] = -math.inf

    def on_generation(ga_instance):
        gen = ga_instance.generations_completed
        _, best_fit, _ = ga_instance.best_solution()
        if best_fit > _estado_conv["melhor"] * (1 + 1e-9):
            _estado_conv["melhor"] = float(best_fit)
            _estado_conv["gen"] = gen
        custo_aprox = int(1.0 / best_fit)
        _estado_conv["historico_custo"].append(round(1.0 / best_fit, 1))
        t_dec = time.perf_counter() - t_total_inicio
        pct = gen / NUM_GERACOES
        barra = "█" * int(pct * 20) + "░" * (20 - int(pct * 20))

        print(f"\n  [{barra}] Gen {gen:3d}/{NUM_GERACOES} | "
              f"Custo Físico Melhor: ~{custo_aprox} | ⏱ {t_dec:5.1f}s | Cache: {len(_fitness_cache)} rotas", flush=True)

    def gerar_chute_inicial_heuristico():
        chute = []
        # Calcula quantos pedidos cada drone deve pegar para dividir sequencialmente
        tamanho_bloco = math.ceil(NUM_PEDIDOS / num_drones) if num_drones > 0 else 1

        for id_pedido in range(NUM_PEDIDOS):
            # Atribuição sequencial em blocos (ex: 000, 111, 222)
            id_drone_alocado = int(id_pedido // tamanho_bloco) % num_drones
            chute.append(id_drone_alocado)

        return chute

    # ==========================================
    # CRIANDO A BASE DO GA A PARTIR DO CHUTE
    # ==========================================
    pop_externa = populacao_inicial
    if pop_externa is None and cenario is None and getattr(cfg, 'USAR_ALOCACAO_IA', False):
        try:
            from IA.dataset_generator import _carregar_pacote, MODELO_PATH, _pos_cd, prever_alocacao, gerar_populacao_inicial
            pacote_ia = _carregar_pacote(MODELO_PATH)
            cd_xy = _pos_cd(cds, max_x, max_y)
            pred_ia = prever_alocacao(
                pacote_ia["modelo"], missoes, cd_xy, max_x, max_y,
                num_drones, cfg.PERFIS_DRONES, pacote_ia.get("max_entregas", 160)
            )
            pop_externa = gerar_populacao_inicial(pred_ia, SOL_POP, num_drones)
        except Exception as e_ia:
            log_warn(f"Não foi possível carregar warm start da IA ({e_ia}); a usar chute sequencial.")
    populacao_inicial = []
    chute_sequencial = gerar_chute_inicial_heuristico()
    populacao_inicial.append(chute_sequencial)

    # Preenche TODO o resto da população (SOL_POP - 1) baseada no chute sequencial
    for _ in range(SOL_POP - 1):
        chute_mutado = chute_sequencial.copy()

        # Muta de 1 a 3 pedidos para criar diversidade (exploração),
        # mas mantendo o "esqueleto" da solução sequencial
        num_mutacoes = np.random.randint(1, 4)
        idx_mutar = np.random.choice(NUM_PEDIDOS, size=min(num_mutacoes, NUM_PEDIDOS), replace=False)

        for idx in idx_mutar:
            chute_mutado[idx] = int(np.random.randint(0, num_drones))

        populacao_inicial.append(chute_mutado)

    usando_warm = False
    if pop_externa is not None and len(pop_externa) > 0:
        warm = [[int(g) for g in ind] for ind in list(pop_externa)[:max(SOL_POP - 1, 1)]]
        for ind in warm:
            if len(ind) != NUM_PEDIDOS or max(ind) >= num_drones or min(ind) < 0:
                raise ValueError("populacao_inicial incompatível com o cenário "
                                 f"({len(ind)} genes vs {NUM_PEDIDOS} pedidos)")
        while len(warm) < SOL_POP - 1:                      # completa com permutações (swaps) preservando balanceamento
            ind = warm[0].copy()
            if NUM_PEDIDOS >= 2:
                i_sw, j_sw = np.random.choice(NUM_PEDIDOS, size=2, replace=False)
                ind[i_sw], ind[j_sw] = ind[j_sw], ind[i_sw]
            warm.append(ind)
        warm.append(chute_sequencial)                       # mantém o chute sequencial como rede de segurança
        populacao_inicial = warm[:SOL_POP]
        usando_warm = True
        log_info("🧠 Warm start: população inicial vem da IA")

    # Avalia e coloca em cache o lance líder da população inicial (IA se warm start, ou sequencial se a frio)
    lance_lider = tuple(int(g) for g in populacao_inicial[0])
    makespan_inicial, esperas_ini, falhas_ini, energia_ini = simular_com_tea(lance_lider, verbose=False)
    custo_inicial = makespan_inicial + (esperas_ini * 0.5) + (energia_ini * FATOR_CONVERSAO_ENERGIA) + (
                falhas_ini * 50000)
    _simulacoes_reais[0] += 1
    _fitness_cache[lance_lider] = 1.0 / (custo_inicial + 0.0001)
    if falhas_ini == 0:
        _melhor_custo_visual[0] = custo_inicial
        logger_ga.salvar_novo_recorde_ga(
            tempo_execucao=time.perf_counter() - t_total_inicio,
            makespan=makespan_inicial,
            energia=energia_ini,
            esperas=esperas_ini,
            vetor_solucao=lance_lider
        )

    tag_lance = "LANCE INICIAL WARM START IA" if usando_warm else "LANCE INICIAL SEQUENCIAL"
    print(
        f"\n  🎯 [{tag_lance}] Makespan: {makespan_inicial}f | Esperas: {esperas_ini} | Falhas: {falhas_ini} | Energia: {energia_ini:.1f}Wh | Custo: {custo_inicial:.1f}",
        flush=True)

    num_pais = max(2, min(SOL_POP - 1, max(10, SOL_POP // 4) if SOL_POP >= 20 else SOL_POP // 2))
    elitismo = max(1, min(num_pais, max(5, SOL_POP // 10) if SOL_POP >= 20 else max(1, SOL_POP // 5)))
    k_tourn = min(5, max(2, SOL_POP // 2))

    ga_instance = pygad.GA(
        num_generations=NUM_GERACOES,
        num_parents_mating=num_pais,
        fitness_func=fitness_func,
        sol_per_pop=SOL_POP,
        num_genes=NUM_PEDIDOS,
        gene_space=list(range(num_drones)),
        gene_type=int,
        mutation_type="adaptive",
        mutation_percent_genes=[10, 5],
        crossover_type="two_points",
        parent_selection_type="tournament",
        K_tournament=k_tourn,
        keep_elitism=elitismo,
        suppress_warnings=True,
        save_solutions=False,
        save_best_solutions=True,
        stop_criteria=["saturate_3"],
        initial_population=populacao_inicial,
        parallel_processing=["thread", NUM_CORES],
        on_start=on_start,
        on_generation=on_generation
    )

    t0_ga = time.perf_counter()
    ga_instance.run()
    t_ga = time.perf_counter() - t0_ga

    solution, solution_fitness, _ = ga_instance.best_solution()
    vetor_perfeito = [int(g) for g in solution]

    log_info("A validar solução final detalhada...")
    makespan_final, esperas_final, falhas_final, energia_final = simular_com_tea(solution, verbose=True)
    custo_final = makespan_final + (esperas_final * 0.5) + (energia_final * FATOR_CONVERSAO_ENERGIA) + (falhas_final * 50000)

    # ELITISMO ESTRITO: Garante que GA(8) nunca seja pior em makespan ou custo que o lance inicial GA(0)
    if falhas_ini == 0 and (custo_final > custo_inicial + 1e-6 or makespan_final > makespan_inicial):
        vetor_perfeito = list(lance_lider)
        makespan_final, esperas_final, falhas_final, energia_final = makespan_inicial, esperas_ini, falhas_ini, energia_ini
        _estado_conv["gen"] = 0

    log_secao(f"RELATÓRIO BATCH {teste_idx + 1}")
    gen_reais = ga_instance.generations_completed
    total_avaliacoes_teoricas = SOL_POP * gen_reais
    poupanca = total_avaliacoes_teoricas - _simulacoes_reais[0]

    log_ok(f"Algoritmo utilizado:      {algoritmo}")
    log_ok(f"Tempo total de treino:    {t_ga:.1f}s")
    log_ok(f"Cálculos Reais do Físicos:{_simulacoes_reais[0]:,} (Filtros pouparam ~{poupanca} chamadas!)")
    log_ok(f"Rotas Nominais (0ms):     {_rotas_nominais_usadas[0]:,} chamadas")
    log_ok(f"Desvios 3D via MOSP:      {_desvios_mosp_executados[0]:,} chamadas")
    log_ok(f"Makespan Inicial -> Final:{makespan_inicial}f -> {makespan_final}f")
    log_ok(f"Consumo Energético Total: {energia_final:.2f} Wh")
    log_ok(f"Histórico de Custo (G0..):{_estado_conv['historico_custo']}")

    print(f"\n{'=' * 60}")
    print(f"📋 RESULTADO DO BATCH {teste_idx + 1}:")
    print(f"VETOR_PEDIDO_DRONE = {vetor_perfeito}")
    print(f"{'=' * 60}\n")

    return (vetor_perfeito, makespan_final, esperas_final, falhas_final,
            energia_final, _estado_conv["gen"])


if __name__ == "__main__":
    print("\n" + "=" * 90)
    print("🚀 A INICIAR OTIMIZAÇÃO GENÉTICA EM BATCH")
    print("=" * 90)
    for idx, config_teste in enumerate(cfg.MATRIZ_TESTES):
        n_drones = config_teste[0]
        n_entregas = config_teste[1]
        algoritmo = config_teste[2] if len(config_teste) > 2 else cfg.TIPO_ALGORITMO

        executar_otimizacao_batch(idx, n_drones, n_entregas, algoritmo)
