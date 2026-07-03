# main.py
import numpy as np
import config as cfg
from logger_system import TrainingLogger
from uav_physics import Drone
from city_builder import gerar_cidade
import mission_control
import renderer
import warnings
import time

from navegation.pathfinder import calcular_rota_8way
from navegation.pathfindercamadas import calcular_rota_8way_camadas
from navegation.pathfinder_tea import calcular_rota_tea
from navegation.pathfinder_tea_camadas import calcular_rota_tea_camadas
# Importação corrigida com apelido (alias) para evitar conflitos
from navegation.pathfinder_mosp_camadas import calcular_rota_tea_camadas as calcular_rota_mosp_camadas


def analisar_cenario(caminho_temporal):
    esperas, mudancas_camada, movimentos = 0, 0, 0
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
        return calcular_rota_8way(max_x, max_y, lotes_gdf, drone, start, goal), start, goal
    elif cfg.TIPO_ALGORITMO == 'A_STAR_CAMADAS':
        return calcular_rota_8way_camadas(max_x, max_y, lotes_gdf, drone, start, goal,
                                          cfg.VETOR_CAMADAS_VOO), start, goal
    elif cfg.TIPO_ALGORITMO == 'TEA_STAR':
        caminho = calcular_rota_tea(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, tempo_partida)
        return caminho, start, goal
    elif cfg.TIPO_ALGORITMO == 'TEA_STAR_CAMADAS':
        caminho = calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                            cfg.VETOR_CAMADAS_VOO, tempo_partida)
        return caminho, start, goal
    # === NOVA CONDIÇÃO PARA O MOSP ===
    elif cfg.TIPO_ALGORITMO == 'MOSP_CAMADAS':
        caminho = calcular_rota_mosp_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                             cfg.VETOR_CAMADAS_VOO, tempo_partida)
        return caminho, start, goal
    else:
        print("❌ ERRO: Algoritmo não reconhecido no config.py!")
        return [], start, goal


def executar_simulacao():
    print(f"🚀 A iniciar Gestão de Frota | Cérebro Ativo: {cfg.TIPO_ALGORITMO}...")

    t_inicio_global = time.perf_counter()
    t_cpu_algoritmo = 0.0

    logger = TrainingLogger(prefixo="main")

    print("1/6: A gerar gémeo digital...")
    lotes_gdf, max_x, max_y = gerar_cidade()
    print("2/6: A gerar tarefas logísticas...")
    cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)

    frota_resultados = []
    reserva_global = {}
    frota_de_drones = [Drone(cfg.DRONE_RAIO_M, cfg.DRONE_ALTURA_VOO, cfg.DRONE_VELOCIDADE_MS, cfg.DRONE_CARGA_KG) for _
                       in range(cfg.NUM_DRONES_DISPONIVEIS)]

    TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)
    USAR_FANTASMA = getattr(cfg, 'ATIVAR_RELATORIO_FANTASMA', False)

    dict_alocacao = {idx: d_id for idx, d_id in enumerate(getattr(cfg, 'VETOR_PEDIDO_DRONE', []))}
    disponibilidade_drones = {i: 0 for i in range(cfg.NUM_DRONES_DISPONIVEIS)}

    print(f"\n3/6: Despacho Logístico...")

    # =======================================================
    # FILA DE MISSÕES (Permite Repescagem para Salvar CPU)
    # =======================================================
    fila_missoes = [(idx, missao, 0) for idx, missao in enumerate(missoes)]

    while fila_missoes:
        # Retira a primeira missão da fila
        idx, missao, tentativas = fila_missoes.pop(0)

        id_drone = dict_alocacao.get(idx, idx % cfg.NUM_DRONES_DISPONIVEIS)
        if id_drone >= cfg.NUM_DRONES_DISPONIVEIS: id_drone = 0

        drone = frota_de_drones[id_drone]
        carga_original = drone.carga
        drone.reset_metricas()

        start = (int(missao['origem'][0]), int(missao['origem'][1]))
        goal = (int(missao['destino'][0]), int(missao['destino'][1]))

        vetor = getattr(cfg, 'VETOR_TEMPOS_PARTIDA', [0])
        t_base = vetor[idx] if idx < len(vetor) else vetor[-1] + ((idx - len(vetor) + 1) * 15)

        # Se for repescagem (tentativa > 0), atira o tempo de partida para DEPOIS de todos os drones
        if tentativas > 0:
            t_ida = max(disponibilidade_drones.values()) + 10  # +10 frames de folga para ter o ar livre
        else:
            t_ida = max(t_base, disponibilidade_drones[id_drone])

        if tentativas == 0:
            print(f"   🔎 Missão {idx + 1} (UAV {id_drone + 1}): Planeando IDA...")
        else:
            print(f"   🔄 REPESCAGEM - Missão {idx + 1} (UAV {id_drone + 1}): Planeando IDA com espaço livre...")

        t0 = time.perf_counter()
        cam_ida, s_real, g_real = roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                                       t_ida)
        t_cpu_algoritmo += (time.perf_counter() - t0)

        hovers_ida, hovers_volta = 0, 0

        # Guardar as reservas desta missão para fazer ROLLBACK caso bata
        reservas_temporarias = []

        if cam_ida:
            # === ATUALIZADO: Reconhece o MOSP como 4D ===
            if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
                for i, p in enumerate(cam_ida):
                    chave = (p[0], p[1], p[2] if len(p) == 3 else drone.altura_alvo, t_ida + i)
                    reserva_global[chave] = id_drone
                    reservas_temporarias.append(chave)
                _, hovers_ida, _ = analisar_cenario(cam_ida)

            rota_ida, bateu, queda = drone.simular_missao(cam_ida, lotes_gdf)

            # --- CHECK COLISÃO NA IDA ---
            if bateu:
                print(f"   💥 ALERTA: O UAV {id_drone + 1} colidiu/caiu nas coordenadas {queda} durante a IDA!")
                if tentativas == 0:
                    print(f"   ⏳ Cancelando reservas e reagendando missão {idx + 1} para o fim da simulação...")
                    # Rollback das reservas para não prender os outros
                    for k in reservas_temporarias:
                        reserva_global.pop(k, None)
                    # Volta para o fim da fila
                    fila_missoes.append((idx, missao, tentativas + 1))
                    drone.carga = carga_original
                    continue  # Pula o registo desta tentativa
                else:
                    # Se falhou mesmo sozinho no fim, regista definitivamente
                    logger.registrar(1, id_drone, idx + 1, "IDA", rota_ida[-1], drone.energia_consumida_kwh,
                                     drone.carga, bateu)
                    frota_resultados.append({
                        'id_entrega': idx + 1, 'id_drone': id_drone + 1, 'uav': drone, 'rota': rota_ida, 'bateu': bateu,
                        'queda': queda, 'start': s_real, 'goal': g_real,
                        'tempo_global': np.arange(t_ida, t_ida + len(rota_ida)), 'esperas_total': hovers_ida
                    })
            else:
                drone.carga = 0.0
                # === ATUALIZADO: Reconhece o MOSP como 4D ===
                t_volta = t_ida + len(cam_ida) + TEMPO_DESCARGA if ('TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO) else 0

                # === ATUALIZADO: Reconhece o MOSP como 4D ===
                if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
                    for extra in range(TEMPO_DESCARGA):
                        chave = (
                        cam_ida[-1][0], cam_ida[-1][1], cam_ida[-1][2] if len(cam_ida[-1]) == 3 else drone.altura_alvo,
                        t_ida + len(cam_ida) + extra)
                        reserva_global[chave] = id_drone
                        reservas_temporarias.append(chave)

                print(f"   🔎 Missão {idx + 1} (UAV {id_drone + 1}): Planeando VOLTA...")

                t0 = time.perf_counter()
                cam_volta, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, g_real, s_real, reserva_global,
                                                       t_volta)
                t_cpu_algoritmo += (time.perf_counter() - t0)

                if cam_volta:
                    # === ATUALIZADO: Reconhece o MOSP como 4D ===
                    if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
                        for i, p in enumerate(cam_volta):
                            chave = (p[0], p[1], p[2] if len(p) == 3 else drone.altura_alvo, t_volta + i)
                            reserva_global[chave] = id_drone
                            reservas_temporarias.append(chave)
                        _, hovers_volta, _ = analisar_cenario(cam_volta)

                    rota_volta, bateu_v, queda_v = drone.simular_missao(cam_volta, lotes_gdf)

                    # --- CHECK COLISÃO NA VOLTA ---
                    if bateu_v:
                        from shapely.geometry import Point
                        ponto_queda = Point(queda_v[0], queda_v[1])
                        lote_atingido = lotes_gdf[lotes_gdf.geometry.contains(ponto_queda)]
                        altura_lote = lote_atingido['altura'].values[0] if not lote_atingido.empty else "Solo/Rua"

                        print(f"   💥 ALERTA: O UAV {id_drone + 1} bateu nas coordenadas {queda_v} durante a VOLTA!")
                        print(
                            f"   🔍 DEBUG LOTE: A altura do edifício em X={queda_v[0]}, Y={queda_v[1]} é: {altura_lote}m")
                        print(f"   🔍 DEBUG ROTA: Destino final era {g_real} e Origem {s_real}")
                        if tentativas == 0:
                            print(
                                f"   ⏳ Cancelando reservas completas e reagendando missão {idx + 1} para o fim da simulação...")
                            for k in reservas_temporarias:
                                reserva_global.pop(k, None)
                            fila_missoes.append((idx, missao, tentativas + 1))
                            drone.carga = carga_original
                            continue

                    # Se chegou aqui, ou correu bem ou é o registo de uma falha na 2ª tentativa
                    logger.registrar(1, id_drone, idx + 1, "IDA", rota_ida[-1], drone.energia_consumida_kwh,
                                     drone.carga, False)
                    logger.registrar(1, id_drone, idx + 1, "VOLTA", rota_volta[-1], drone.energia_consumida_kwh,
                                     drone.carga, bateu_v)

                    r_descarga = np.tile(rota_ida[-1], (TEMPO_DESCARGA, 1)) if TEMPO_DESCARGA > 0 else np.empty(
                        (0, rota_ida.shape[1]))
                    r_completa = np.vstack((rota_ida, r_descarga, rota_volta))
                    t_global = np.arange(t_ida, t_volta + len(rota_volta))

                    disponibilidade_drones[id_drone] = t_volta + len(rota_volta)

                    frota_resultados.append({
                        'id_entrega': idx + 1, 'id_drone': id_drone + 1, 'uav': drone, 'rota': r_completa,
                        'bateu': bateu_v, 'queda': queda_v, 'start': s_real, 'goal': g_real, 'tempo_global': t_global,
                        'esperas_total': hovers_ida + hovers_volta
                    })
                else:
                    logger.registrar(1, id_drone, idx + 1, "IDA", rota_ida[-1], drone.energia_consumida_kwh,
                                     drone.carga, False)
                    frota_resultados.append({
                        'id_entrega': idx + 1, 'id_drone': id_drone + 1, 'uav': drone, 'rota': rota_ida, 'bateu': True,
                        'queda': rota_ida[-1], 'start': s_real, 'goal': g_real,
                        'tempo_global': np.arange(t_ida, t_ida + len(rota_ida)), 'esperas_total': hovers_ida
                    })
        else:
            print(f"❌ Missão {idx + 1} Cancelada.")

        drone.carga = carga_original

    # ==========================================
    # MÉTRICAS SILENCIOSAS E GRAVAÇÃO GLOBAL
    # ==========================================
    t_cpu_total = time.perf_counter() - t_inicio_global

    sucessos = [r for r in frota_resultados if not r.get('bateu', True)]
    kpis_simulacao = {
        "makespan_frames": max([r['tempo_global'][-1] for r in sucessos]) if sucessos else 0,
        "taxa_congestionamento_frames": sum([r.get('esperas_total', 0) for r in frota_resultados]),
        "cpu_tempo_total_s": round(t_cpu_total, 4),
        "cpu_tempo_algoritmo_s": round(t_cpu_algoritmo, 4)
    }

    logger.salvar_kpis_globais(kpis_simulacao)

    print(
        f"\n📊 Resumo Rápido: Makespan: {kpis_simulacao['makespan_frames']}f | Esperas: {kpis_simulacao['taxa_congestionamento_frames']}f | CPU Algoritmo: {kpis_simulacao['cpu_tempo_algoritmo_s']}s")

    # ==========================================
    # RENDERIZAÇÃO
    # ==========================================
    print("\n4/6 a 6/6: A renderizar Gráficos...")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        renderer.plotar_frota_3d(max_x, max_y, lotes_gdf, cds, frota_resultados)
        renderer.plotar_frota_2d(lotes_gdf, cds, frota_resultados)
        renderer.plotar_diagrama_espaco_tempo(frota_resultados)

if __name__ == "__main__":
    executar_simulacao()