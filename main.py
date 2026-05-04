# main.py
import numpy as np
import config as cfg
from logger_system import TrainingLogger
from uav_physics import Drone
from city_builder import gerar_cidade
import mission_control
import renderer

from navegation.pathfinder import calcular_rota_8way
from navegation.pathfindercamadas import calcular_rota_8way_camadas
from navegation.pathfinder_tea import calcular_rota_tea
from navegation.pathfinder_tea_camadas import calcular_rota_tea_camadas


def analisar_cenario(caminho_temporal):
    esperas, mudancas_camada = 0, 0
    movimentos = 0
    for i in range(1, len(caminho_temporal)):
        p1, p2 = caminho_temporal[i - 1], caminho_temporal[i]
        if p1 == p2:
            esperas += 1
        else:
            movimentos += 1
            if len(p1) == 3 and len(p2) == 3 and p1[2] != p2[2]:
                mudancas_camada += 1
    return movimentos, esperas, mudancas_camada


def roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, tempo_partida=0):
    if cfg.TIPO_ALGORITMO == 'A_STAR':
        return calcular_rota_8way(max_x, max_y, lotes_gdf, drone, start, goal)
    elif cfg.TIPO_ALGORITMO == 'A_STAR_CAMADAS':
        return calcular_rota_8way_camadas(max_x, max_y, lotes_gdf, drone, start, goal, cfg.VETOR_CAMADAS_VOO)
    elif cfg.TIPO_ALGORITMO == 'TEA_STAR':
        caminho = calcular_rota_tea(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, tempo_partida)
        return caminho, start, goal
    elif cfg.TIPO_ALGORITMO == 'TEA_STAR_CAMADAS':
        caminho = calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                            cfg.VETOR_CAMADAS_VOO, tempo_partida)
        return caminho, start, goal
    else:
        print("❌ ERRO: Algoritmo não reconhecido no config.py!")
        return [], start, goal


def executar_simulacao():
    print(f"🚀 A iniciar Gestão de Frota | Cérebro Ativo: {cfg.TIPO_ALGORITMO}...")
    logger = TrainingLogger()

    print("\n1/6: A gerar gémeo digital da cidade...")
    lotes_gdf, max_x, max_y = gerar_cidade()

    print("\n2/6: A gerar bases e tarefas logísticas...")
    cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)

    frota_resultados = []
    reserva_global = {}

    frota_de_drones = [
        Drone(cfg.DRONE_RAIO_M, cfg.DRONE_ALTURA_VOO, cfg.DRONE_VELOCIDADE_MS, cfg.DRONE_CARGA_KG)
        for _ in range(cfg.NUM_DRONES_DISPONIVEIS)
    ]

    TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)
    USAR_FANTASMA = getattr(cfg, 'ATIVAR_RELATORIO_FANTASMA', False)

    # Prepara o Dicionário de Atribuição Manual
    matriz_alocacao = getattr(cfg, 'MATRIZ_PEDIDO_DRONE', [])
    dict_alocacao = {pedido: drone for pedido, drone in matriz_alocacao}

    print(f"\n3/6: Despacho Logístico - Lendo Vetor de Descolagem...")

    # AGENDA DE DISPONIBILIDADE DA FROTA (Para evitar paradoxos de tempo)
    disponibilidade_drones = {i: 0 for i in range(cfg.NUM_DRONES_DISPONIVEIS)}

    for idx, missao in enumerate(missoes):

        # --- LÓGICA DE ATRIBUIÇÃO DRONE/PEDIDO ---
        id_drone_alocado = dict_alocacao.get(idx, idx % cfg.NUM_DRONES_DISPONIVEIS)

        # Alerta se o utilizador pedir na Matriz um ID de drone que não existe
        if id_drone_alocado >= cfg.NUM_DRONES_DISPONIVEIS:
            print(f"   ⚠️ Aviso: O Drone ID {id_drone_alocado} não existe na frota. A realocar para o Drone 0.")
            id_drone_alocado = 0

        drone_atual = frota_de_drones[id_drone_alocado]
        carga_original = drone_atual.carga

        # Limpa o consumo da viagem anterior
        drone_atual.reset_metricas()

        start = (int(missao['origem'][0]), int(missao['origem'][1]))
        goal = (int(missao['destino'][0]), int(missao['destino'][1]))

        # --- LÓGICA DE TEMPO DE PARTIDA E AGENDA ---
        vetor = getattr(cfg, 'VETOR_TEMPOS_PARTIDA', [0])
        if idx < len(vetor):
            tempo_descolagem_base = vetor[idx]
        else:
            tempo_descolagem_base = vetor[-1] + ((idx - len(vetor) + 1) * 15)

        # O drone sai na hora do Vetor OU quando estiver livre da missão anterior (o que for mais tarde)
        tempo_descolagem_ida = max(tempo_descolagem_base, disponibilidade_drones[id_drone_alocado])

        if 'TEA' in cfg.TIPO_ALGORITMO:
            print(
                f"   🔎 Missão {idx + 1} (UAV {id_drone_alocado + 1}): Planeando IDA com {cfg.TIPO_ALGORITMO} (Slot T+{tempo_descolagem_ida})...")
        else:
            tempo_descolagem_ida = 0
            print(f"   🔎 Missão {idx + 1} (UAV {id_drone_alocado + 1}): Planeando a IDA com {cfg.TIPO_ALGORITMO}...")

        caminho_ida, start_real, goal_real = roteador_inteligente(max_x, max_y, lotes_gdf, drone_atual, start, goal,
                                                                  reserva_global, tempo_descolagem_ida)

        movs_base_ida = 0
        if USAR_FANTASMA:
            if 'CAMADAS' in cfg.TIPO_ALGORITMO:
                caminho_base_ida = calcular_rota_8way_camadas(max_x, max_y, lotes_gdf, drone_atual, start, goal,
                                                              cfg.VETOR_CAMADAS_VOO)
            else:
                caminho_base_ida = calcular_rota_8way(max_x, max_y, lotes_gdf, drone_atual, start, goal)
            movs_base_ida = len(caminho_base_ida) - 1 if caminho_base_ida else 0

        if caminho_ida:
            if 'TEA' in cfg.TIPO_ALGORITMO:
                for i, p in enumerate(caminho_ida):
                    chave = (p[0], p[1], p[2], tempo_descolagem_ida + i) if len(p) == 3 else (
                    p[0], p[1], tempo_descolagem_ida + i)
                    reserva_global[chave] = id_drone_alocado

                movs, hovers, saltos_z = analisar_cenario(caminho_ida)

                acoes = []
                if hovers > 0: acoes.append(f"Espera: {hovers}f")

                if USAR_FANTASMA:
                    if movs_base_ida == 0: movs_base_ida = movs
                    desvio_xy = max(0, movs - movs_base_ida)
                    if desvio_xy > 0: acoes.append(f"Desvio XY: +{desvio_xy}m")

                if saltos_z > 0: acoes.append(f"Salto Altitude: {saltos_z}x")

                if not acoes:
                    msg_acao = "✅ ROTA LIVRE"
                else:
                    msg_acao = f"⚠️ TRÂNSITO EVITADO ({' | '.join(acoes)})"

                print(f"      📊 [Relatório IDA] Espaço: {movs} frames | {msg_acao}")
                print(f"   🛫 A aguardar o Slot e a Descolar...")
            else:
                print(f"   🛫 Indo entregar pacote (Altitude: {drone_atual.altura_alvo}m)...")

            rota_ida, bateu, queda = drone_atual.simular_missao(caminho_ida, lotes_gdf)
            tempo_global_ida = np.arange(tempo_descolagem_ida, tempo_descolagem_ida + len(rota_ida))

            logger.registrar(1, id_drone_alocado, idx + 1, "IDA", rota_ida[-1], drone_atual.energia_consumida_kwh,
                             drone_atual.carga, bateu)

            if not bateu:
                drone_atual.carga = 0.0

                if 'TEA' in cfg.TIPO_ALGORITMO:
                    print(f"   🛬 Entrega concluída! A descarregar...")
                    for extra_t in range(TEMPO_DESCARGA):
                        p = caminho_ida[-1]
                        chave = (p[0], p[1], p[2], tempo_descolagem_ida + len(caminho_ida) + extra_t) if len(
                            p) == 3 else (p[0], p[1], tempo_descolagem_ida + len(caminho_ida) + extra_t)
                        reserva_global[chave] = id_drone_alocado
                    tempo_descolagem_volta = tempo_descolagem_ida + len(caminho_ida) + TEMPO_DESCARGA
                    print(
                        f"   🔎 Missão {idx + 1} (UAV {id_drone_alocado + 1}): Planeando VOLTA com {cfg.TIPO_ALGORITMO} (Slot T+{tempo_descolagem_volta})...")
                else:
                    print(f"   🛬 Pacote entregue com sucesso! Planeando a VOLTA...")
                    tempo_descolagem_volta = 0

                caminho_volta, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone_atual, goal_real, start_real,
                                                           reserva_global, tempo_descolagem_volta)

                movs_base_volta = 0
                if USAR_FANTASMA:
                    if 'CAMADAS' in cfg.TIPO_ALGORITMO:
                        caminho_base_volta = calcular_rota_8way_camadas(max_x, max_y, lotes_gdf, drone_atual, goal_real,
                                                                        start_real, cfg.VETOR_CAMADAS_VOO)
                    else:
                        caminho_base_volta = calcular_rota_8way(max_x, max_y, lotes_gdf, drone_atual, goal_real,
                                                                start_real)
                    movs_base_volta = len(caminho_base_volta) - 1 if caminho_base_volta else 0

                if caminho_volta:
                    if 'TEA' in cfg.TIPO_ALGORITMO:
                        for i, p in enumerate(caminho_volta):
                            chave = (p[0], p[1], p[2], tempo_descolagem_volta + i) if len(p) == 3 else (
                            p[0], p[1], tempo_descolagem_volta + i)
                            reserva_global[chave] = id_drone_alocado

                        movs_v, hovers_v, saltos_z_v = analisar_cenario(caminho_volta)

                        acoes_v = []
                        if hovers_v > 0: acoes_v.append(f"Espera: {hovers_v}f")

                        if USAR_FANTASMA:
                            if movs_base_volta == 0: movs_base_volta = movs_v
                            desvio_xy_v = max(0, movs_v - movs_base_volta)
                            if desvio_xy_v > 0: acoes_v.append(f"Desvio XY: +{desvio_xy_v}m")

                        if saltos_z_v > 0: acoes_v.append(f"Salto Altitude: {saltos_z_v}x")

                        if not acoes_v:
                            msg_acao_v = "✅ ROTA LIVRE (Caminho Ótimo e Contínuo)"
                        else:
                            msg_acao_v = f"⚠️ TRÂNSITO EVITADO ({' | '.join(acoes_v)})"

                        print(f"      📊 [Relatório VOLTA] Espaço: {movs_v} frames | {msg_acao_v}")
                        print(f"   🛫 Retornando à Base...")
                    else:
                        print(f"   🛫 Retornando à Base (Altitude: {drone_atual.altura_alvo}m)...")

                    rota_volta, bateu_volta, queda_volta = drone_atual.simular_missao(caminho_volta, lotes_gdf)

                    # Vetor contínuo de descarga para manter a linha do gráfico contínua
                    rota_descarga = np.tile(rota_ida[-1], (TEMPO_DESCARGA, 1)) if TEMPO_DESCARGA > 0 else np.empty(
                        (0, rota_ida.shape[1]))
                    tempo_descarga = np.arange(tempo_descolagem_ida + len(rota_ida),
                                               tempo_descolagem_volta) if TEMPO_DESCARGA > 0 else []

                    tempo_global_volta = np.arange(tempo_descolagem_volta, tempo_descolagem_volta + len(rota_volta))
                    tempo_global_completo = np.concatenate([tempo_global_ida, tempo_descarga, tempo_global_volta])
                    rota_completa = np.vstack((rota_ida, rota_descarga, rota_volta))

                    logger.registrar(1, id_drone_alocado, idx + 1, "VOLTA", rota_volta[-1],
                                     drone_atual.energia_consumida_kwh, drone_atual.carga, bateu_volta)

                    # ATUALIZA A AGENDA DO DRONE PARA A PRÓXIMA MISSÃO
                    tempo_chegada_base = tempo_descolagem_volta + len(rota_volta)
                    disponibilidade_drones[id_drone_alocado] = tempo_chegada_base

                    frota_resultados.append({
                        'id_entrega': idx + 1, 'id_drone': id_drone_alocado + 1,
                        'uav': drone_atual, 'rota': rota_completa, 'bateu': bateu_volta,
                        'queda': queda_volta, 'start': start_real, 'goal': goal_real,
                        'tempo_global': tempo_global_completo
                    })
                else:
                    print(f"❌ Erro: Rota de volta bloqueada.")
                    frota_resultados.append({
                        'id_entrega': idx + 1, 'id_drone': id_drone_alocado + 1,
                        'uav': drone_atual, 'rota': rota_ida, 'bateu': True,
                        'queda': rota_ida[-1], 'start': start_real, 'goal': goal_real,
                        'tempo_global': tempo_global_ida
                    })
            else:
                frota_resultados.append({
                    'id_entrega': idx + 1, 'id_drone': id_drone_alocado + 1,
                    'uav': drone_atual, 'rota': rota_ida, 'bateu': bateu,
                    'queda': queda, 'start': start_real, 'goal': goal_real,
                    'tempo_global': tempo_global_ida
                })
        else:
            print(f"❌ Missão {idx + 1} Cancelada. Sem rota viável.")
            z_seguro = drone_atual.altura_alvo
            rota_falsa_3d = np.array([
                [start[0], start[1], z_seguro],
                [start[0], start[1], z_seguro]
            ])
            frota_resultados.append({
                'id_entrega': idx + 1, 'id_drone': id_drone_alocado + 1,
                'uav': drone_atual, 'rota': rota_falsa_3d, 'bateu': True,
                'queda': (start[0], start[1], z_seguro), 'start': start, 'goal': goal,
                'falha_planeamento': True
            })

        drone_atual.carga = carga_original

    # ==========================================
    # PAINEL DE VISUALIZAÇÕES ESTÁTICAS E RÁPIDAS
    # ==========================================
    print("\n4/6: A renderizar Painel 3D...")
    renderer.plotar_frota_3d(max_x, max_y, lotes_gdf, cds, frota_resultados)

    print("\n5/6: A renderizar Planta 2D Clássica...")
    renderer.plotar_frota_2d(lotes_gdf, cds, frota_resultados)

    print("\n6/6: A renderizar Diagrama de Espaço-Tempo...")
    renderer.plotar_diagrama_espaco_tempo(frota_resultados)

    print(f"\n✅ Logs CSV gravados em: {cfg.CAMINHO_LOG}")


if __name__ == "__main__":
    executar_simulacao()