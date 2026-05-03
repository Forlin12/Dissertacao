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


def analisar_cenario(caminho_temporal):
    esperas = 0
    movimentos = 0
    for i in range(1, len(caminho_temporal)):
        if caminho_temporal[i] == caminho_temporal[i - 1]:
            esperas += 1
        else:
            movimentos += 1
    return movimentos, esperas


def roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, tempo_partida=0):
    if cfg.TIPO_ALGORITMO == 'A_STAR':
        return calcular_rota_8way(max_x, max_y, lotes_gdf, drone, start, goal)
    elif cfg.TIPO_ALGORITMO == 'A_STAR_CAMADAS':
        return calcular_rota_8way_camadas(max_x, max_y, lotes_gdf, drone, start, goal, cfg.VETOR_CAMADAS_VOO)
    elif cfg.TIPO_ALGORITMO == 'TEA_STAR':
        caminho = calcular_rota_tea(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, tempo_partida)
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

    TEMPO_DESCARGA = 5

    print(f"\n3/6: Despacho Logístico - Lendo Vetor de Descolagem...")
    for idx, missao in enumerate(missoes):
        drone_atual = frota_de_drones[idx % cfg.NUM_DRONES_DISPONIVEIS]
        carga_original = drone_atual.carga

        start = (int(missao['origem'][0]), int(missao['origem'][1]))
        goal = (int(missao['destino'][0]), int(missao['destino'][1]))

        if cfg.TIPO_ALGORITMO == 'TEA_STAR':
            vetor = cfg.VETOR_TEMPOS_PARTIDA
            if idx < len(vetor):
                tempo_descolagem_ida = vetor[idx]
            else:
                tempo_descolagem_ida = vetor[-1] + ((idx - len(vetor) + 1) * 15)

            print(f"   🔎 Missão {idx + 1}: Planeando IDA com TEA* (Slot T+{tempo_descolagem_ida})...")
        else:
            tempo_descolagem_ida = 0
            print(f"   🔎 Missão {idx + 1}: Planeando a IDA com {cfg.TIPO_ALGORITMO}...")

        caminho_ida, start_real, goal_real = roteador_inteligente(max_x, max_y, lotes_gdf, drone_atual, start, goal,
                                                                  reserva_global, tempo_descolagem_ida)

        if caminho_ida:
            if cfg.TIPO_ALGORITMO == 'TEA_STAR':
                for i, (rx, ry) in enumerate(caminho_ida):
                    reserva_global[(rx, ry, tempo_descolagem_ida + i)] = idx
                movs, hovers = analisar_cenario(caminho_ida)

                if hovers > 0:
                    msg_acao = "⚠️ TRÂNSITO EVITADO (Realizou Espera Tática)"
                else:
                    msg_acao = "✅ ROTA LIVRE (Voo contínuo)"

                print(f"      📊 [Relatório IDA] Voo: {movs} frames | Espera: {hovers} frames | {msg_acao}")
                print(f"   🛫 A aguardar o Slot e a Descolar...")
            else:
                print(f"   🛫 Indo entregar pacote (Altitude: {drone_atual.altura_alvo}m)...")

            rota_ida, bateu, queda = drone_atual.simular_missao(caminho_ida, lotes_gdf)
            # --- NOVO: Regista o Tempo Global da IDA ---
            tempo_global_ida = np.arange(tempo_descolagem_ida, tempo_descolagem_ida + len(rota_ida))

            logger.registrar(1, idx % cfg.NUM_DRONES_DISPONIVEIS, idx + 1, "IDA", rota_ida[-1],
                             drone_atual.energia_consumida_kwh, drone_atual.carga, bateu)

            if not bateu:
                drone_atual.carga = 0.0

                if cfg.TIPO_ALGORITMO == 'TEA_STAR':
                    print(f"   🛬 Entrega concluída! A descarregar...")
                    for extra_t in range(TEMPO_DESCARGA):
                        reserva_global[
                            (goal_real[0], goal_real[1], tempo_descolagem_ida + len(caminho_ida) + extra_t)] = idx
                    tempo_descolagem_volta = tempo_descolagem_ida + len(caminho_ida) + TEMPO_DESCARGA
                    print(f"   🔎 Missão {idx + 1}: Planeando VOLTA com TEA* (Slot T+{tempo_descolagem_volta})...")
                else:
                    print(f"   🛬 Pacote entregue com sucesso! Planeando a VOLTA...")
                    tempo_descolagem_volta = 0

                caminho_volta, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone_atual, goal_real, start_real,
                                                           reserva_global, tempo_descolagem_volta)

                if caminho_volta:
                    if cfg.TIPO_ALGORITMO == 'TEA_STAR':
                        for i, (rx, ry) in enumerate(caminho_volta):
                            reserva_global[(rx, ry, tempo_descolagem_volta + i)] = idx
                        movs_v, hovers_v = analisar_cenario(caminho_volta)

                        if hovers_v > 0:
                            msg_acao_v = "⚠️ TRÂNSITO EVITADO (Realizou Espera Tática)"
                        else:
                            msg_acao_v = "✅ ROTA LIVRE (Voo contínuo)"

                        print(
                            f"      📊 [Relatório VOLTA] Voo: {movs_v} frames | Espera: {hovers_v} frames | {msg_acao_v}")
                        print(f"   🛫 Retornando à Base...")
                    else:
                        print(f"   🛫 Retornando à Base (Altitude: {drone_atual.altura_alvo}m)...")

                    rota_volta, bateu_volta, queda_volta = drone_atual.simular_missao(caminho_volta, lotes_gdf)

                    # --- NOVO: Regista o Tempo Global da VOLTA e Concatena ---
                    tempo_global_volta = np.arange(tempo_descolagem_volta, tempo_descolagem_volta + len(rota_volta))
                    tempo_global_completo = np.concatenate([tempo_global_ida, tempo_global_volta])

                    logger.registrar(1, idx % cfg.NUM_DRONES_DISPONIVEIS, idx + 1, "VOLTA", rota_volta[-1],
                                     drone_atual.energia_consumida_kwh, drone_atual.carga, bateu_volta)

                    rota_completa = np.vstack((rota_ida, rota_volta))
                    frota_resultados.append({
                        'id_entrega': idx + 1, 'id_drone': (idx % cfg.NUM_DRONES_DISPONIVEIS) + 1,
                        'uav': drone_atual, 'rota': rota_completa, 'bateu': bateu_volta,
                        'queda': queda_volta, 'start': start_real, 'goal': goal_real,
                        'tempo_global': tempo_global_completo  # O SEGREDO DO TEMPO
                    })
                else:
                    print(f"❌ Erro: Rota de volta bloqueada.")
            else:
                frota_resultados.append({
                    'id_entrega': idx + 1, 'id_drone': (idx % cfg.NUM_DRONES_DISPONIVEIS) + 1,
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
                'id_entrega': idx + 1, 'id_drone': (idx % cfg.NUM_DRONES_DISPONIVEIS) + 1,
                'uav': drone_atual, 'rota': rota_falsa_3d, 'bateu': True,
                'queda': (start[0], start[1], z_seguro), 'start': start, 'goal': goal,
                'falha_planeamento': True
            })

        drone_atual.carga = carga_original

    print("\n4/6: A renderizar Painel 3D...")
    renderer.plotar_frota_3d(max_x, max_y, lotes_gdf, cds, frota_resultados)

    print("\n5/6: A renderizar Planta 2D Global...")
    renderer.plotar_frota_2d(lotes_gdf, cds, frota_resultados)

    print("\n6/6: A renderizar Recortes 2D (Zoom nas entregas)...")
    renderer.plotar_snapshots_entregas_2d(lotes_gdf, frota_resultados)

    print(f"\n✅ Logs CSV gravados em: {cfg.CAMINHO_LOG}")


if __name__ == "__main__":
    executar_simulacao()