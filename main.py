# main.py
import numpy as np
import config as cfg
from logger_system import TrainingLogger
from uav_physics import Drone
from city_builder import gerar_cidade
from pathfinder import calcular_rota_8way
import mission_control
import renderer


def executar_simulacao():
    print("🚀 A iniciar Gestão de Frota (Ciclo Completo com Logs)...")
    logger = TrainingLogger()

    print("\n1/6: A gerar gémeo digital da cidade...")
    lotes_gdf, max_x, max_y = gerar_cidade()

    print("\n2/6: A gerar bases e tarefas logísticas...")
    cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)

    frota_resultados = []

    frota_de_drones = [
        Drone(cfg.DRONE_RAIO_M, cfg.DRONE_ALTURA_VOO, cfg.DRONE_VELOCIDADE_MS, cfg.DRONE_CARGA_KG)
        for _ in range(cfg.NUM_DRONES_DISPONIVEIS)
    ]

    print(f"\n3/6: Despacho Logístico - Executando Ida e Volta...")
    for idx, missao in enumerate(missoes):
        drone_atual = frota_de_drones[idx % cfg.NUM_DRONES_DISPONIVEIS]
        carga_original = drone_atual.carga

        start = (int(missao['origem'][0]), int(missao['origem'][1]))
        goal = (int(missao['destino'][0]), int(missao['destino'][1]))

        caminho_ida, start_real, goal_real = calcular_rota_8way(max_x, max_y, lotes_gdf, drone_atual, start, goal)

        if caminho_ida:
            print(f"   🛫 Missão {idx + 1}: Indo entregar pacote...")
            rota_ida, bateu, queda = drone_atual.simular_missao(caminho_ida, lotes_gdf)
            logger.registrar(idx % cfg.NUM_DRONES_DISPONIVEIS, idx + 1, "IDA", rota_ida[-1],
                             drone_atual.energia_consumida_kwh, drone_atual.carga, bateu)

            if not bateu:
                print(f"   🛬 Pacote entregue com sucesso! Retornando à Base...")
                drone_atual.carga = 0.0  # Tira o peso do pacote para economizar bateria na volta
                caminho_volta, _, _ = calcular_rota_8way(max_x, max_y, lotes_gdf, drone_atual, goal_real, start_real)

                if caminho_volta:
                    rota_volta, bateu_volta, queda_volta = drone_atual.simular_missao(caminho_volta, lotes_gdf)
                    logger.registrar(idx % cfg.NUM_DRONES_DISPONIVEIS, idx + 1, "VOLTA", rota_volta[-1],
                                     drone_atual.energia_consumida_kwh, drone_atual.carga, bateu_volta)

                    rota_completa = np.vstack((rota_ida, rota_volta))
                    frota_resultados.append({
                        'id_entrega': idx + 1,
                        'id_drone': (idx % cfg.NUM_DRONES_DISPONIVEIS) + 1,
                        'uav': drone_atual,
                        'rota': rota_completa,
                        'bateu': bateu_volta,
                        'queda': queda_volta,
                        'start': start_real, 'goal': goal_real
                    })
                else:
                    print(f"❌ Erro: Rota de volta bloqueada. Drone ficou preso no destino.")
            else:
                # Bateu na ida
                frota_resultados.append({
                    'id_entrega': idx + 1,
                    'id_drone': (idx % cfg.NUM_DRONES_DISPONIVEIS) + 1,
                    'uav': drone_atual,
                    'rota': rota_ida,
                    'bateu': bateu,
                    'queda': queda,
                    'start': start_real, 'goal': goal_real
                })
        else:
            print(f"❌ Missão {idx + 1} Cancelada: Rota de ida impossível.")

        drone_atual.carga = carga_original

    print("\n4/6: A renderizar Painel 3D (Abre no Navegador)...")
    renderer.plotar_frota_3d(max_x, max_y, lotes_gdf, cds, frota_resultados)

    print("\n5/6: A renderizar Planta 2D Global...")
    renderer.plotar_frota_2d(lotes_gdf, cds, frota_resultados)

    print("\n6/6: A renderizar Recortes 2D (Zoom nas entregas)...")
    renderer.plotar_snapshots_entregas_2d(lotes_gdf, frota_resultados)

    print(f"\n✅ Todos os Logs CSV foram gravados com sucesso em: {cfg.CAMINHO_LOG}")


if __name__ == "__main__":
    executar_simulacao()