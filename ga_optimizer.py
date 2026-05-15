# ga_optimizer.py — VERSÃO FINAL 4D (PyGAD 3.x + Camadas + Threads)
import os
import math
import time
import numpy as np
import pygad
import config as cfg
from city_builder import gerar_cidade
import mission_control
from uav_physics import Drone

# IMPORTAÇÃO ATUALIZADA PARA O PATHFINDER 4D
from navegation.pathfinder_tea_camadas import calcular_rota_tea_camadas

t_total_inicio = time.perf_counter()


# ==========================================
# UTILITÁRIO DE LOG
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
# 1. ARRANQUE E CONFIGURAÇÃO
# ==========================================
log_secao("1/5  ARRANQUE DO SISTEMA")

NUM_CORES = os.cpu_count() or 4
TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)
VETOR_CAMADAS = getattr(cfg, 'VETOR_CAMADAS', [40, 50, 60, 70])  # Garante que as camadas existem

SOL_POP = 100
NUM_GERACOES = 8

log_info(f"CPU: {NUM_CORES} núcleos detectados → Threads ativadas")
log_info(f"Drones disponíveis:    {cfg.NUM_DRONES_DISPONIVEIS}")
log_info(f"Entregas a optimizar:  {cfg.NUM_ENTREGAS_TOTAL}")
log_info(f"Camadas de Voo (Z):    {VETOR_CAMADAS}")
log_info(f"População GA:          {SOL_POP} soluções × {NUM_GERACOES} gerações")

# ==========================================
# 2. CARREGAR CENÁRIO
# ==========================================
log_secao("2/5  A CARREGAR O CENÁRIO")

t0 = time.perf_counter()
if not getattr(cfg, 'CENARIO_SEMPRE_NOVO', False):
    np.random.seed(42)

print("  🌍 A gerar gémeo digital da cidade...")
lotes_gdf, max_x, max_y = gerar_cidade()
log_tempo("Cidade gerada", t0)

t0 = time.perf_counter()
print("  📦 A gerar tarefas logísticas...")
cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)
NUM_PEDIDOS = len(missoes)
NUM_DRONES = cfg.NUM_DRONES_DISPONIVEIS
log_tempo("Missões geradas", t0)

frota_base = [
    Drone(cfg.DRONE_RAIO_M, cfg.DRONE_ALTURA_VOO, cfg.DRONE_VELOCIDADE_MS, cfg.DRONE_CARGA_KG)
    for _ in range(NUM_DRONES)
]
pontos_missoes = [
    {'start': (int(m['origem'][0]), int(m['origem'][1])),
     'goal': (int(m['destino'][0]), int(m['destino'][1]))}
    for m in missoes
]


# ==========================================
# 3. SIMULAÇÃO TEA_STAR 4D COM EARLY-EXIT
# ==========================================
def simular_com_tea(solution, teto_makespan=math.inf, verbose=False):
    """
    Simula rotas em 4D. Aborta precocemente se ultrapassar o teto_makespan.
    """
    reserva_global = {}
    disponibilidade = {i: 0 for i in range(NUM_DRONES)}
    makespan_frames = 0
    esperas_total = 0
    missoes_falhadas = 0

    for id_pedido in range(NUM_PEDIDOS):
        id_drone = int(solution[id_pedido])
        drone = frota_base[id_drone]
        drone.reset_metricas()

        start = pontos_missoes[id_pedido]['start']
        goal = pontos_missoes[id_pedido]['goal']
        t_ida = disponibilidade[id_drone]

        # CHAMADA 4D (IDA)
        cam_ida = calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, VETOR_CAMADAS,
                                            t_ida)

        if not cam_ida:
            missoes_falhadas += 1
            continue

        # Registo UTM 4D dinâmico usando o Z real da rota (p[2])
        for i, p in enumerate(cam_ida):
            reserva_global[(p[0], p[1], p[2], t_ida + i)] = id_drone

        esperas_total += sum(1 for i in range(1, len(cam_ida)) if cam_ida[i] == cam_ida[i - 1])
        t_volta = t_ida + len(cam_ida) + TEMPO_DESCARGA

        # Regista o drone parado a descarregar (Mantém no Z de cruzeiro)
        for extra in range(TEMPO_DESCARGA):
            reserva_global[(cam_ida[-1][0], cam_ida[-1][1], cam_ida[-1][2], t_ida + len(cam_ida) + extra)] = id_drone

        # CHAMADA 4D (VOLTA)
        cam_volta = calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, goal, start, reserva_global,
                                              VETOR_CAMADAS, t_volta)

        if not cam_volta:
            missoes_falhadas += 1
            disponibilidade[id_drone] = t_volta
            continue

        # Registo UTM 4D dinâmico usando o Z real da rota (p[2])
        for i, p in enumerate(cam_volta):
            reserva_global[(p[0], p[1], p[2], t_volta + i)] = id_drone

        esperas_total += sum(1 for i in range(1, len(cam_volta)) if cam_volta[i] == cam_volta[i - 1])
        t_fim = t_volta + len(cam_volta)
        disponibilidade[id_drone] = t_fim

        if t_fim > makespan_frames:
            makespan_frames = t_fim

        # OTIMIZAÇÃO: EARLY-EXIT (PODA)
        if not verbose and makespan_frames > teto_makespan + 50:
            return makespan_frames, esperas_total, missoes_falhadas + (NUM_PEDIDOS - id_pedido - 1)

    return makespan_frames, esperas_total, missoes_falhadas


# ==========================================
# 4. FITNESS NATIVA DO PYGAD E FEEDBACK VISUAL
# ==========================================
_simulacoes_reais = [0]
_melhor_makespan_visual = [math.inf]


def fitness_func(ga_instance, solution, solution_idx):
    _simulacoes_reais[0] += 1

    teto = math.inf
    if hasattr(ga_instance, "best_solutions_fitness") and ga_instance.best_solutions_fitness:
        melhor_fitness = np.max(ga_instance.best_solutions_fitness)
        teto = (1.0 / melhor_fitness)

    makespan, esperas, falhas = simular_com_tea(solution, teto_makespan=teto)

    if falhas == 0 and makespan < _melhor_makespan_visual[0]:
        _melhor_makespan_visual[0] = makespan
        print(f"\n  ⭐ NOVO RECORDE: {makespan} frames! (Esperas: {esperas} | Falhas: 0)", flush=True)
    else:
        print(".", end="", flush=True)

    penalidade = falhas * 50000
    custo = makespan + (esperas * 0.5) + penalidade
    return 1.0 / (custo + 0.0001)


def on_generation(ga_instance):
    gen = ga_instance.generations_completed
    _, best_fit, _ = ga_instance.best_solution()
    makespan_aprox = int(1.0 / best_fit)

    t_dec = time.perf_counter() - t_total_inicio
    pct = gen / NUM_GERACOES
    barra = "█" * int(pct * 20) + "░" * (20 - int(pct * 20))

    print(f"\n  [{barra}] Gen {gen:3d}/{NUM_GERACOES} | "
          f"Melhor Estimado: ~{makespan_aprox}f | ⏱ {t_dec:5.1f}s")


# ==========================================
# 5. EXECUÇÃO DO GA
# ==========================================
log_secao("3/5  A EVOLUIR A FROTA")
print()

semente_logica = [i % NUM_DRONES for i in range(NUM_PEDIDOS)]
populacao_inicial = []

for _ in range(4):
    populacao_inicial.append(semente_logica)

for _ in range(SOL_POP - 4):
    indiv_aleatorio = [int(np.random.randint(0, NUM_DRONES)) for _ in range(NUM_PEDIDOS)]
    populacao_inicial.append(indiv_aleatorio)

ga_instance = pygad.GA(
    num_generations=NUM_GERACOES,
    num_parents_mating=max(10, SOL_POP // 4),
    fitness_func=fitness_func,
    sol_per_pop=SOL_POP,
    num_genes=NUM_PEDIDOS,
    gene_space=list(range(NUM_DRONES)),
    gene_type=int,

    mutation_type="adaptive",
    mutation_percent_genes=[10, 5],

    crossover_type="two_points",
    parent_selection_type="tournament",
    K_tournament=5,
    keep_elitism=max(5, SOL_POP // 10),
    suppress_warnings=True,

    save_solutions=True,
    save_best_solutions=True,
    stop_criteria=["saturate_3"],
    initial_population=populacao_inicial,

    parallel_processing=["thread", NUM_CORES],
    on_generation=on_generation
)

t0_ga = time.perf_counter()
ga_instance.run()
t_ga = time.perf_counter() - t0_ga

# ==========================================
# 6. VALIDAÇÃO FINAL
# ==========================================
log_secao("4/5  A VALIDAR SOLUÇÃO FINAL")

solution, solution_fitness, _ = ga_instance.best_solution()
vetor_perfeito = [int(g) for g in solution]

log_info("A correr simulação final detalhada com TEA_STAR 4D...")
makespan_final, esperas_final, falhas_final = simular_com_tea(solution, verbose=True)

# ==========================================
# 7. RELATÓRIO FINAL
# ==========================================
log_secao("5/5  RELATÓRIO FINAL")

t_total = time.perf_counter() - t_total_inicio

gen_reais = ga_instance.generations_completed
total_avaliacoes_teoricas = SOL_POP * gen_reais
poupanca = total_avaliacoes_teoricas - _simulacoes_reais[0]

log_ok(f"Tempo total de treino:    {t_ga:.1f}s")
if gen_reais < NUM_GERACOES:
    log_info(f"Paragem Antecipada:       O algoritmo estagnou na geração {gen_reais} e poupou tempo.")
log_ok(f"Avaliações GA Totais:     {total_avaliacoes_teoricas:,}")
log_ok(f"Cálculos Reais do TEA*:   {_simulacoes_reais[0]:,} (PyGAD poupou ~{poupanca} chamadas!)")
log_ok(f"Makespan final (TEA*):    {makespan_final} frames")

if falhas_final > 0:
    log_warn(f"Missões falhadas: {falhas_final} — considera aumentar a tolerância de iteracões no TEA")

print(f"\n{'=' * 60}")
print(f"  📋 Cola isto no teu config.py:")
print(f"{'=' * 60}")
print(f"VETOR_PEDIDO_DRONE = {vetor_perfeito}")
print(f"{'=' * 60}")