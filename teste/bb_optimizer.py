# bb_optimizer.py — BRANCH AND BOUND (Com Warm Start, Quebra de Simetria e Simulação Incremental)
import math
import time
import copy
import numpy as np
import config as cfg
from city_builder import gerar_cidade
import mission_control
from uav_physics import Drone
from navegation.pathfinder_tea_camadas import calcular_rota_tea_camadas
from teste.logger_system import TrainingLogger

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
log_secao("1/5  ARRANQUE DO SISTEMA (BRANCH & BOUND)")

TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)
VETOR_CAMADAS = getattr(cfg, 'VETOR_CAMADAS', [40, 50, 60, 70])

log_info(f"Drones disponíveis:    {cfg.NUM_DRONES_DISPONIVEIS}")
log_info(f"Entregas a optimizar:  {cfg.NUM_ENTREGAS_TOTAL}")
log_info("Ativada Otimização 1: Quebra de Simetria")
log_info("Ativada Otimização 2: Simulação Incremental (estado herdado do nó pai)")

# ==========================================
# 2. CARREGAR CENÁRIO
# ==========================================
log_secao("2/5  A CARREGAR O CENÁRIO")

t0 = time.perf_counter()
if not getattr(cfg, 'CENARIO_SEMPRE_NOVO', False):
    np.random.seed(42)

lotes_gdf, max_x, max_y = gerar_cidade()
cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)

NUM_PEDIDOS = len(missoes)
NUM_DRONES = cfg.NUM_DRONES_DISPONIVEIS

frota_base = [
    Drone(cfg.DRONE_RAIO_M, cfg.DRONE_ALTURA_VOO, cfg.DRONE_VELOCIDADE_MS, cfg.DRONE_CARGA_KG)
    for _ in range(NUM_DRONES)
]
pontos_missoes = [
    {'start': (int(m['origem'][0]), int(m['origem'][1])),
     'goal':  (int(m['destino'][0]), int(m['destino'][1]))}
    for m in missoes
]


# ==========================================
# 3. SIMULAÇÃO DE UMA ÚNICA TAREFA NOVA
#    Recebe o estado do nó-pai e só processa a última tarefa adicionada.
#    Devolve o estado actualizado (ou None se falhou / ultrapassou teto).
# ==========================================
def simular_tarefa_incremental(id_pedido, id_drone, estado_pai, teto_makespan=math.inf):
    """
    Simula apenas a missão `id_pedido` atribuída a `id_drone`,
    partindo do estado já computado pelo nó pai.

    `estado_pai` é um dict com:
        - reserva_global : dict  (espaço-tempo reservado por todos os drones)
        - disponibilidade: dict  {id_drone: t_livre}
        - makespan_frames: int
        - esperas_total  : int
        - missoes_falhadas: int

    Devolve (estado_filho, custo) ou (None, inf) se deve ser podado.
    O estado_filho é uma CÓPIA independente — o pai não é modificado.
    """
    # --- cópia rasa dos scalars, cópia profunda só do necessário ---
    reserva = dict(estado_pai['reserva_global'])          # cópia do dict (chaves são tuplos imutáveis)
    disponibilidade = dict(estado_pai['disponibilidade'])  # cópia
    makespan_frames  = estado_pai['makespan_frames']
    esperas_total    = estado_pai['esperas_total']
    missoes_falhadas = estado_pai['missoes_falhadas']

    drone_local = copy.deepcopy(frota_base[id_drone])
    drone_local.reset_metricas()

    start = pontos_missoes[id_pedido]['start']
    goal  = pontos_missoes[id_pedido]['goal']

    vetor_partida = getattr(cfg, 'VETOR_TEMPOS_PARTIDA', [0])
    t_base = (vetor_partida[id_pedido]
              if id_pedido < len(vetor_partida)
              else vetor_partida[-1] + ((id_pedido - len(vetor_partida) + 1) * 15))

    fila = [(id_pedido, id_drone, 0)]

    while fila:
        pid, did, tentativas = fila.pop(0)

        t_ida = (max(disponibilidade.values()) + 10
                 if tentativas > 0
                 else max(t_base, disponibilidade[did]))

        cam_ida = calcular_rota_tea_camadas(
            max_x, max_y, lotes_gdf, drone_local,
            start, goal, reserva, VETOR_CAMADAS, t_ida)

        if not cam_ida:
            if tentativas == 0:
                fila.append((pid, did, 1))
            else:
                missoes_falhadas += 1
            continue

        reservas_tmp = []
        for i, p in enumerate(cam_ida):
            chave = (p[0], p[1], p[2], t_ida + i)
            reserva[chave] = did
            reservas_tmp.append(chave)

        esperas_total += sum(1 for i in range(1, len(cam_ida)) if cam_ida[i] == cam_ida[i - 1])
        t_volta = t_ida + len(cam_ida) + TEMPO_DESCARGA

        for extra in range(TEMPO_DESCARGA):
            chave_d = (cam_ida[-1][0], cam_ida[-1][1], cam_ida[-1][2],
                       t_ida + len(cam_ida) + extra)
            reserva[chave_d] = did
            reservas_tmp.append(chave_d)

        cam_volta = calcular_rota_tea_camadas(
            max_x, max_y, lotes_gdf, drone_local,
            goal, start, reserva, VETOR_CAMADAS, t_volta)

        if not cam_volta:
            if tentativas == 0:
                for k in reservas_tmp:
                    reserva.pop(k, None)
                fila.append((pid, did, 1))
            else:
                missoes_falhadas += 1
                disponibilidade[did] = t_volta
            continue

        for i, p in enumerate(cam_volta):
            reserva[(p[0], p[1], p[2], t_volta + i)] = did

        esperas_total += sum(1 for i in range(1, len(cam_volta)) if cam_volta[i] == cam_volta[i - 1])
        t_fim = t_volta + len(cam_volta)
        disponibilidade[did] = t_fim
        if t_fim > makespan_frames:
            makespan_frames = t_fim

    custo = makespan_frames + (esperas_total * 0.5) + (missoes_falhadas * 50000)

    if custo >= teto_makespan:
        return None, math.inf

    estado_filho = {
        'reserva_global':  reserva,
        'disponibilidade': disponibilidade,
        'makespan_frames': makespan_frames,
        'esperas_total':   esperas_total,
        'missoes_falhadas': missoes_falhadas,
    }
    return estado_filho, custo


# Estado vazio — raiz da árvore
def estado_inicial():
    return {
        'reserva_global':   {},
        'disponibilidade':  {i: 0 for i in range(NUM_DRONES)},
        'makespan_frames':  0,
        'esperas_total':    0,
        'missoes_falhadas': 0,
    }


def calcular_custo_estado(estado):
    return (estado['makespan_frames']
            + estado['esperas_total'] * 0.5
            + estado['missoes_falhadas'] * 50000)


# ==========================================
# SIMULAÇÃO COMPLETA (usada só para Warm Start e validação final)
# ==========================================
def simular_atribuicao_completa(solucao):
    estado = estado_inicial()
    for id_pedido, id_drone in enumerate(solucao):
        estado_f, _ = simular_tarefa_incremental(id_pedido, int(id_drone), estado, math.inf)
        if estado_f is None:
            # missão falhou mas continua (não há teto aqui)
            estado_f2, _ = simular_tarefa_incremental(id_pedido, int(id_drone), estado, math.inf)
            estado = estado_f2 if estado_f2 else estado
        else:
            estado = estado_f
    return estado['makespan_frames'], estado['esperas_total'], estado['missoes_falhadas']


# ==========================================
# 4. MOTOR BRANCH AND BOUND
# ==========================================
log_secao("3/5  A EXPLORAR ÁRVORE DE DECISÃO (B&B)")

melhor_custo_global = math.inf
melhor_solucao_global = []
nos_explorados = 0
nos_podados = 0

INTERVALO_PRINT = 1000   # imprime uma linha nova a cada N iterações
LIMITE_TEMPO_S  = 6 * 3600  # 6 horas — para aqui e guarda o melhor encontrado

def imprimir_progresso():
    """Imprime uma nova linha a cada 100 iterações com barra de progresso visual."""
    t_decorrido = time.perf_counter() - t0_bb
    nos_por_seg = nos_explorados / t_decorrido if t_decorrido > 0 else 0

    # Barra de asteriscos que avança e reinicia
    LARGURA = 40
    preenchimento = (nos_explorados // INTERVALO_PRINT) % (LARGURA + 1)
    barra = "*" * preenchimento + "." * (LARGURA - preenchimento)

    print(
        f"  [{barra}]  iteração {nos_explorados:>6} "
        f"| podados {nos_podados:>6} "
        f"| teto {melhor_custo_global:>8.0f} "
        f"| {nos_por_seg:>5.0f} nós/s "
        f"| {t_decorrido:>6.1f}s"
    )

# --- WARM START ---
solucao_ga_conhecida = [
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4,

]
if solucao_ga_conhecida and len(solucao_ga_conhecida) == NUM_PEDIDOS:
    log_info("A simular solução Warm Start (Algoritmo Genético)...")
    mk_ini, esp_ini, falhas_ini = simular_atribuicao_completa(solucao_ga_conhecida)
    custo_ini = mk_ini + esp_ini * 0.5 + falhas_ini * 50000
    melhor_custo_global  = custo_ini
    melhor_solucao_global = list(solucao_ga_conhecida)
    log_ok(f"Teto inicial definido! Custo: {custo_ini:.1f} | Makespan: {mk_ini}f")
else:
    log_warn("Sem Warm Start válido. A iniciar com teto infinito.")


def branch_and_bound(tarefa_idx, solucao_atual, estado_atual):
    """
    estado_atual: resultado acumulado das tarefas 0..tarefa_idx-1.
    Só simula a tarefa `tarefa_idx` em cada chamada recursiva.
    """
    global melhor_custo_global, melhor_solucao_global, nos_explorados, nos_podados
    nos_explorados += 1

    # Critério de paragem por tempo
    if time.perf_counter() - t0_bb >= LIMITE_TEMPO_S:
        return

    # Base Case
    if tarefa_idx == NUM_PEDIDOS:
        custo_final = calcular_custo_estado(estado_atual)
        if custo_final < melhor_custo_global:
            melhor_custo_global  = custo_final
            melhor_solucao_global = list(solucao_atual)
            ms = estado_atual['makespan_frames']
            print(f"\n\n  ⭐ NOVO ÓTIMO ENCONTRADO: Custo {custo_final:.1f} | Makespan {ms}f")
            print(f"  Vetor: {melhor_solucao_global}\n")
        return

    # Quebra de Simetria Completa
    # Como todos os drones são idênticos, fixamos a ordem de primeiro uso:
    # drone 0 é sempre o 1º a ser usado, drone 1 o 2º, etc.
    # Só é permitido usar o drone K se 0..K-1 já foram usados antes.
    # Elimina todas as permutações equivalentes — reduz a árvore ~36.000x.
    drones_em_uso = set(solucao_atual)
    proximo_virgem = len(drones_em_uso)  # próximo ID ainda não usado

    drones_candidatos = list(drones_em_uso)        # drones já activos
    if proximo_virgem < NUM_DRONES:
        drones_candidatos.append(proximo_virgem)   # só 1 drone virgem permitido

    for drone_id in sorted(drones_candidatos):

        # Simula só a nova tarefa, herdando o estado do pai
        estado_filho, custo_filho = simular_tarefa_incremental(
            tarefa_idx, drone_id, estado_atual,
            teto_makespan=melhor_custo_global)

        if estado_filho is None:
            nos_podados += 1
            if nos_explorados % INTERVALO_PRINT == 0:
                imprimir_progresso()
            continue

        solucao_atual.append(drone_id)
        if nos_explorados % INTERVALO_PRINT == 0:
            imprimir_progresso()
        branch_and_bound(tarefa_idx + 1, solucao_atual, estado_filho)
        solucao_atual.pop()


# Iniciar B&B
t0_bb = time.perf_counter()
print("\n  A iniciar exploração... (pressione Ctrl+C para abortar)\n")
branch_and_bound(0, [], estado_inicial())
t_bb = time.perf_counter() - t0_bb

print(" " * 160, end="\r")

# ==========================================
# 5. VALIDAÇÃO FINAL
# ==========================================
log_secao("4/5  A VALIDAR SOLUÇÃO ÓTIMA")

if not melhor_solucao_global:
    log_warn("Nenhuma solução viável encontrada!")
else:
    makespan_final, esperas_final, falhas_final = simular_atribuicao_completa(melhor_solucao_global)

# ==========================================
# 6. RELATÓRIO FINAL
# ==========================================
log_secao("5/5  RELATÓRIO B&B")

t_total = time.perf_counter() - t_total_inicio

motivo_paragem = "Limite de tempo atingido (6h)" if t_bb >= LIMITE_TEMPO_S else "Árvore completamente explorada ✅"
log_ok(f"Paragem: {motivo_paragem}")
log_ok(f"Tempo total de exploração: {t_bb:.1f}s")
log_ok(f"Nós da árvore explorados:  {nos_explorados:,}")
log_info(f"Ramos podados (evitados):  {nos_podados:,}")

if melhor_solucao_global:
    log_ok(f"Makespan final garantido:  {makespan_final} frames")
    if falhas_final > 0:
        log_warn(f"Missões falhadas: {falhas_final}")

    print(f"\n{'=' * 60}")
    print(f"  📋 Vetor Perfeito (Solução Exata):")
    print(f"  VETOR_PEDIDO_DRONE = {melhor_solucao_global}")
    print(f"{'=' * 60}")

    try:
        logger_bb = TrainingLogger(prefixo="bb")
        kpis_bb = {
            "Tempo_Total_Treino_s":   round(t_bb, 2),
            "Nos_Explorados":         nos_explorados,
            "Nos_Podados":            nos_podados,
            "Makespan_Final_frames":  makespan_final,
            "Missoes_Falhadas":       falhas_final,
            "Vetor_Perfeito":         str(melhor_solucao_global)
        }
        logger_bb.salvar_resumo_ga(kpis_bb)
    except NameError:
        pass