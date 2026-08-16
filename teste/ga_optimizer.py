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
def executar_otimizacao_batch(teste_idx, num_drones, num_entregas, algoritmo):
    t_total_inicio = time.perf_counter()

    log_secao(f"INICIANDO BATCH {teste_idx + 1} | Drones: {num_drones} | Entregas: {num_entregas} | Alg: {algoritmo}")

    # Inicializa o Logger específico para este lote de testes (Live Tracking do GA)
    logger_ga = TrainingLogger(prefixo=f"otimizador_batch_{teste_idx + 1}")

    cfg.NUM_DRONES_DISPONIVEIS = num_drones
    cfg.NUM_ENTREGAS_TOTAL = num_entregas
    cfg.TIPO_ALGORITMO = algoritmo

    if not getattr(cfg, 'CENARIO_SEMPRE_NOVO', False):
        np.random.seed(42)

    NUM_CORES = os.cpu_count() or 4
    TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)
    VETOR_CAMADAS_VOO = getattr(cfg, 'VETOR_CAMADAS_VOO', [40, 50, 60, 70])
    VETOR_TEMPOS_PARTIDA = getattr(cfg, 'VETOR_TEMPOS_PARTIDA', [0])

    SOL_POP = 10
    NUM_GERACOES = 8

    log_info(f"População GA: {SOL_POP} soluções × {NUM_GERACOES} gerações paralelas")

    # ==========================================
    # CARREGAR CENÁRIO
    # ==========================================
    t0 = time.perf_counter()
    lotes_gdf, max_x, max_y = gerar_cidade()
    log_tempo("Cidade gerada", t0)

    t0 = time.perf_counter()
    cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)
    NUM_PEDIDOS = len(missoes)
    log_tempo("Missões geradas", t0)

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

            cam_ida, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, t_ida)

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
                return makespan_frames, esperas_total, missoes_falhadas + len(fila_missoes), energia_total_da_simulacao

        return makespan_frames, esperas_total, missoes_falhadas, energia_total_da_simulacao

    # ==========================================
    # ESTRUTURAS HPC SEGURAS E TRACKING AO VIVO
    # ==========================================
    _simulacoes_reais = [0]

    # NOVO: Agora rastreamos o CUSTO TOTAL (Tempo + Energia), e não apenas o Tempo (Makespan)
    _melhor_custo_visual = [math.inf]

    _ga_lock = threading.Lock()
    _fitness_cache = {}

    def fitness_func(ga_instance, solution, solution_idx):
        sol_tuple = tuple(solution)

        with _ga_lock:
            if sol_tuple in _fitness_cache:
                return _fitness_cache[sol_tuple]
            recorde_atual_custo = _melhor_custo_visual[0]

        # PODA TEÓRICA FÍSICA: Estimativa conservadora de Tempo + Bateria
        tempo_ideal_drones = [0.0] * num_drones
        energia_ideal_drones = 0.0

        for id_pedido, id_drone in enumerate(solution):
            start = pontos_missoes[id_pedido]['start']
            goal = pontos_missoes[id_pedido]['goal']
            dist_reta = math.hypot(goal[0] - start[0], goal[1] - start[1])

            VELOCIDADE_MAX_TEORICA = 15.0
            CONSUMO_CRUZEIRO_MINIMO = 0.015  # Wh/m otimista

            frames_ideais = (dist_reta / VELOCIDADE_MAX_TEORICA) + (TEMPO_DESCARGA * 2)
            energia_ideal = (dist_reta * CONSUMO_CRUZEIRO_MINIMO) * 2  # Ida e volta

            tempo_ideal_drones[id_drone] += frames_ideais
            energia_ideal_drones += energia_ideal

        makespan_teorico = max(tempo_ideal_drones)
        FATOR_CONVERSAO_ENERGIA = 3.6  # 1 Wh = 3.6 segundos de penalização (recarga a 1000W)

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

        makespan, esperas, falhas, energia = simular_com_tea(solution, teto_makespan=teto)

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
                    vetor_solucao=solution
                )
            else:
                print(".", end="", flush=True)
            _fitness_cache[sol_tuple] = fitness

        return fitness

    def on_generation(ga_instance):
        gen = ga_instance.generations_completed
        _, best_fit, _ = ga_instance.best_solution()
        custo_aprox = int(1.0 / best_fit)
        t_dec = time.perf_counter() - t_total_inicio
        pct = gen / NUM_GERACOES
        barra = "█" * int(pct * 20) + "░" * (20 - int(pct * 20))

        print(f"\n  [{barra}] Gen {gen:3d}/{NUM_GERACOES} | "
              f"Custo Físico Melhor: ~{custo_aprox} | ⏱ {t_dec:5.1f}s | Cache: {len(_fitness_cache)} rotas", flush=True)

    def gerar_chute_inicial_heuristico():
        chute = []
        for id_pedido in range(NUM_PEDIDOS):
            chute.append(id_pedido % num_drones)
        return chute

    populacao_inicial = []
    chute_guloso = gerar_chute_inicial_heuristico()
    populacao_inicial.append(chute_guloso)

    makespan_inicial, esperas_ini, falhas_ini, energia_ini = simular_com_tea(chute_guloso, verbose=False)
    FATOR_CONVERSAO_ENERGIA = 3.6
    custo_inicial = makespan_inicial + (esperas_ini * 0.5) + (energia_ini * FATOR_CONVERSAO_ENERGIA) + (
                falhas_ini * 50000)

    print(
        f"\n  🎯 [LANCE INICIAL] Makespan: {makespan_inicial}f | Esperas: {esperas_ini} | Falhas: {falhas_ini} | Energia: {energia_ini:.1f}Wh | Custo: {custo_inicial:.1f}",
        flush=True)

    for _ in range(3):
        chute_mutado = chute_guloso.copy()
        idx_mutar = np.random.choice(NUM_PEDIDOS, size=min(3, NUM_PEDIDOS), replace=False)
        for idx in idx_mutar:
            chute_mutado[idx] = int(np.random.randint(0, num_drones))
        populacao_inicial.append(chute_mutado)

    for _ in range(SOL_POP - 4):
        indiv_aleatorio = [int(np.random.randint(0, num_drones)) for _ in range(NUM_PEDIDOS)]
        populacao_inicial.append(indiv_aleatorio)

    ga_instance = pygad.GA(
        num_generations=NUM_GERACOES,
        num_parents_mating=max(10, SOL_POP // 4),
        fitness_func=fitness_func,
        sol_per_pop=SOL_POP,
        num_genes=NUM_PEDIDOS,
        gene_space=list(range(num_drones)),
        gene_type=int,
        mutation_type="adaptive",
        mutation_percent_genes=[10, 5],
        crossover_type="two_points",
        parent_selection_type="tournament",
        K_tournament=5,
        keep_elitism=max(5, SOL_POP // 10),
        suppress_warnings=True,
        save_solutions=False,
        save_best_solutions=True,
        stop_criteria=["saturate_3"],
        initial_population=populacao_inicial,
        parallel_processing=["thread", NUM_CORES],
        on_generation=on_generation
    )

    t0_ga = time.perf_counter()
    ga_instance.run()
    t_ga = time.perf_counter() - t0_ga

    solution, solution_fitness, _ = ga_instance.best_solution()
    vetor_perfeito = [int(g) for g in solution]

    log_info("A validar solução final detalhada...")
    makespan_final, esperas_final, falhas_final, energia_final = simular_com_tea(solution, verbose=True)

    log_secao(f"RELATÓRIO BATCH {teste_idx + 1}")
    gen_reais = ga_instance.generations_completed
    total_avaliacoes_teoricas = SOL_POP * gen_reais
    poupanca = total_avaliacoes_teoricas - _simulacoes_reais[0]

    log_ok(f"Algoritmo utilizado:      {algoritmo}")
    log_ok(f"Tempo total de treino:    {t_ga:.1f}s")
    log_ok(f"Cálculos Reais do Físicos:{_simulacoes_reais[0]:,} (Filtros pouparam ~{poupanca} chamadas!)")
    log_ok(f"Makespan final (FÍSICO):  {makespan_final} frames")
    log_ok(f"Consumo Energético Total: {energia_final:.2f} Wh")

    print(f"\n{'=' * 60}")
    print(f"📋 RESULTADO DO BATCH {teste_idx + 1}:")
    print(f"VETOR_PEDIDO_DRONE = {vetor_perfeito}")
    print(f"{'=' * 60}\n")


if __name__ == "__main__":
    print("\n" + "=" * 90)
    print("🚀 A INICIAR OTIMIZAÇÃO GENÉTICA EM BATCH")
    print("=" * 90)
    for idx, config_teste in enumerate(cfg.MATRIZ_TESTES):
        n_drones = config_teste[0]
        n_entregas = config_teste[1]
        algoritmo = config_teste[2] if len(config_teste) > 2 else cfg.TIPO_ALGORITMO

        executar_otimizacao_batch(idx, n_drones, n_entregas, algoritmo)