# main.py
import numpy as np
import config as cfg
from teste.logger_system import TrainingLogger
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
from navegation.pathfinder_mosp_camadas import calcular_rota_tea_camadas as calcular_rota_mosp_camadas
# Nova importação para a versão 3D de alta fidelidade física[cite: 4]


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
    # === CONDIÇÃO PARA O MOSP 4D (Label Setting) ===[cite: 4]
    elif cfg.TIPO_ALGORITMO == 'MOSP_CAMADAS':
        caminho = calcular_rota_mosp_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                             cfg.VETOR_CAMADAS_VOO, tempo_partida)
        return caminho, start, goal
    # === CONDIÇÃO PARA O NOVO MOSP 3D (Bateria Real) ===[cite: 4]


    else:
        print("❌ ERRO: Algoritmo não reconhecido no config.py!")
        return [], start, goal


# =======================================================
# RELATÓRIO TÉCNICO DE DESEMPENHO E IMPACTO ATMOSFÉRICO
# =======================================================
def imprimir_relatorio_tecnico_detalhado(frota_de_drones, consumo_acumulado_wh):
    print("\n" + "=" * 95)
    print("📈 RELATÓRIO TÉCNICO DE DESEMPENHO E IMPACTO ATMOSFÉRICO")
    print("=" * 95)

    # --- Análise de Vento ---
    if getattr(cfg, 'USAR_VENTO', False):
        print(f"Condição de Vento Ativa: {cfg.VETOR_VENTO} m/s")
    else:
        print("Condição de Vento: Desativada")

    # A distância voada já é acumulada dentro de cada objeto Drone individual[cite: 4]
    total_distancia_km = sum([d.distancia_voada for d in frota_de_drones]) / 1000.0
    total_consumido_wh = sum(consumo_acumulado_wh.values())

    if total_distancia_km > 0:
        eficiencia_media = total_consumido_wh / total_distancia_km
        print(f"Eficiência Média Real da Frota: {eficiencia_media:.2f} Wh/km")

    print("-" * 95)
    print(
        f"{'ID UAV':<8} | {'Dist. (km)':<12} | {'Cons. (Wh)':<12} | {'Efic. Ideal (Wh/km)':<20} | {'Efic. Real (Wh/km)':<18} | {'Penalidade de Missão (%)'}")
    print("-" * 95)

    for i, drone in enumerate(frota_de_drones):
        dist_uav_km = drone.distancia_voada / 1000.0
        cons_uav_wh = consumo_acumulado_wh[i]

        if dist_uav_km > 0:
            eficiencia_real = cons_uav_wh / dist_uav_km
            # Eficiência teórica ideal: Consumo de cruzeiro (Wh/m) * 1000 sem vento e sem subidas[cite: 4]
            eficiencia_ideal = drone.consumo_cruzeiro * 1000

            # Cálculo do Impacto Percentual (Vento + Subidas + Hover - Ideal)[cite: 4]
            impacto_pct = ((eficiencia_real / eficiencia_ideal) - 1) * 100
            sinal = "+" if impacto_pct > 0 else ""

            print(
                f"UAV {i + 1:<4} | {dist_uav_km:<12.2f} | {cons_uav_wh:<12.2f} | {eficiencia_ideal:<20.2f} | {eficiencia_real:<18.2f} | {sinal}{impacto_pct:.1f}%")

    print("=" * 95 + "\n")


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

    def montar_frota():
        perfis = cfg.PERFIS_DRONES
        frota = []
        for i in range(cfg.NUM_DRONES_DISPONIVEIS):
            perfil = perfis[i % len(perfis)]
            d = Drone(perfil)
            frota.append(d)
            print(f"   🔋 UAV {i + 1}: {d.nome_modelo} | Bateria: {d.bateria_capacidade_wh:.0f}Wh "
                  f"| Subida: {d.velocidade_subida:.1f}m/s | Descida: {d.velocidade_descida:.1f}m/s "
                  f"| Cruzeiro: {d.velocidade_horiz:.1f}m/s")
        return frota

    print("\n🛠️  A montar a frota (perfis customizados)...")
    frota_de_drones = montar_frota()

    # Dicionários para rastrear consumo e recarga acumulados de cada drone[cite: 4]
    consumo_acumulado_wh = {i: 0.0 for i in range(cfg.NUM_DRONES_DISPONIVEIS)}
    recarga_acumulada_wh = {i: 0.0 for i in range(cfg.NUM_DRONES_DISPONIVEIS)}

    TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)

    dict_alocacao = {idx: d_id for idx, d_id in enumerate(getattr(cfg, 'VETOR_PEDIDO_DRONE', []))}
    disponibilidade_drones = {i: 0 for i in range(cfg.NUM_DRONES_DISPONIVEIS)}

    print(f"\n3/6: Despacho Logístico...")

    # =======================================================
    # FILA DE MISSÕES (Permite Repescagem para Salvar CPU)
    # =======================================================
    fila_missoes = [(idx, missao, 0) for idx, missao in enumerate(missoes)]

    while fila_missoes:
        # Retira a primeira missão da fila[cite: 4]
        idx, missao, tentativas = fila_missoes.pop(0)

        id_drone = dict_alocacao.get(idx, idx % cfg.NUM_DRONES_DISPONIVEIS)
        if id_drone >= cfg.NUM_DRONES_DISPONIVEIS: id_drone = 0

        drone = frota_de_drones[id_drone]

        # Guarda a capacidade máxima do drone definida no perfil físico[cite: 4]
        carga_original = drone._carga_original
        drone.reset_metricas()

        # Vai buscar o peso gerado no mission_control (assume 1.0kg se não encontrar)[cite: 4]
        peso_pacote = missao.get('peso_kg', 1.0)

        # Segurança: Se o pacote for mais pesado que a capacidade do drone, limita à capacidade máxima[cite: 4]
        if peso_pacote > carga_original:
            print(
                f"   ⚠️ ALERTA: Pacote de {peso_pacote}kg excede a capacidade do UAV {id_drone + 1}! A ajustar para {carga_original}kg.")
            peso_pacote = carga_original

        # O drone assume o peso do produto para a viagem de IDA[cite: 4]
        drone.carga = peso_pacote

        start = (int(missao['origem'][0]), int(missao['origem'][1]))
        goal = (int(missao['destino'][0]), int(missao['destino'][1]))

        vetor = getattr(cfg, 'VETOR_TEMPOS_PARTIDA', [0])
        t_base = vetor[idx] if idx < len(vetor) else vetor[-1] + ((idx - len(vetor) + 1) * 15)

        # Se for repescagem (tentativa > 0), atira o tempo de partida para DEPOIS de todos os drones[cite: 4]
        if tentativas > 0:
            t_ida = max(disponibilidade_drones.values()) + 10  # +10 frames de folga para ter o ar livre
        else:
            t_ida = max(t_base, disponibilidade_drones[id_drone])

        # ================================================================
        # SISTEMA DE RECARGA ON-DEMAND NA BASE (CENTRO DISTRIBUIÇÃO)
        # ================================================================
        dist_linear = np.hypot(goal[0] - start[0], goal[1] - start[1])
        energia_estimada = drone.estimar_energia_missao(dist_linear, dist_linear, TEMPO_DESCARGA, peso_pacote)

        # Fator de segurança (1.5x) para cobrir subidas, descidas, desvios nos prédios e tempos de espera[cite: 4]
        energia_necessaria = energia_estimada * 1.5

        # 1. Verifica se a missão é sequer possível para a bateria deste drone[cite: 4]
        if energia_necessaria > drone.bateria_capacidade_wh:
            print(
                f"   ❌ ALERTA: Missão {idx + 1} requer ~{energia_necessaria:.1f}Wh, mas o máximo do UAV {id_drone + 1} é {drone.bateria_capacidade_wh}Wh. Cancelada.")
            drone.carga = carga_original
            continue  # Pula esta missão

        # 2. Verifica se precisa recarregar antes de sair[cite: 4]
        if drone.bateria_atual_wh < energia_necessaria:
            energia_em_falta = energia_necessaria - drone.bateria_atual_wh
            # Tempo (segundos) = Energia(Wh) / Potencia(W) * 3600[cite: 4]
            tempo_de_carga_s = int((energia_em_falta / drone.taxa_recarga_w) * 3600.0) + 1

            print(
                f"   🔌 UAV {id_drone + 1} no CD a carregar... (Bateria: {drone.bateria_atual_wh:.1f}Wh | Requerido: {energia_necessaria:.1f}Wh) -> Carregando +{energia_em_falta:.1f}Wh em {tempo_de_carga_s}s")

            # Regista a energia recarregada neste drone[cite: 4]
            recarga_acumulada_wh[id_drone] += energia_em_falta

            # Atrasamos a partida no simulador para dar tempo físico de carregar[cite: 4]
            t_ida += tempo_de_carga_s
            drone.carregar_bateria_quantidade(energia_em_falta)
        # ================================================================

        if tentativas == 0:
            print(f"   🔎 Missão {idx + 1} (UAV {id_drone + 1}): Planeando IDA...")
        else:
            print(f"   🔄 REPESCAGEM - Missão {idx + 1} (UAV {id_drone + 1}): Planeando IDA com espaço livre...")

        t0 = time.perf_counter()
        cam_ida, s_real, g_real = roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                                       t_ida)
        t_cpu_algoritmo += (time.perf_counter() - t0)

        hovers_ida, hovers_volta = 0, 0

        # Guardar as reservas desta missão para fazer ROLLBACK caso bata[cite: 4]
        reservas_temporarias = []

        if cam_ida:
            # === ATUALIZADO: Reconhece o MOSP como 4D ===[cite: 4]
            if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
                for i, p in enumerate(cam_ida):
                    chave = (p[0], p[1], p[2] if len(p) == 3 else drone.altura_alvo, t_ida + i)
                    reserva_global[chave] = id_drone
                    reservas_temporarias.append(chave)
                _, hovers_ida, _ = analisar_cenario(cam_ida)

            # Salva a bateria antes do voo para calcular o consumo real desta perna[cite: 4]
            bateria_antes_ida = drone.bateria_atual_wh
            rota_ida, bateu, queda = drone.simular_missao(cam_ida, lotes_gdf)

            # Acumula o consumo real da viagem de ida[cite: 4]
            consumo_acumulado_wh[id_drone] += (bateria_antes_ida - drone.bateria_atual_wh)

            # --- CHECK COLISÃO NA IDA ---[cite: 4]
            if bateu:
                if drone.motivo_falha == 'bateria':
                    print(f"   🔋 ALERTA: O UAV {id_drone + 1} ficou sem bateria em {queda} durante a IDA!")
                else:
                    print(f"   💥 ALERTA: O UAV {id_drone + 1} colidiu/caiu nas coordenadas {queda} durante a IDA!")
                if tentativas == 0:
                    print(f"   ⏳ Cancelando reservas e reagendando missão {idx + 1} para o fim da simulação...")
                    # Rollback das reservas para não prender os outros[cite: 4]
                    for k in reservas_temporarias:
                        reserva_global.pop(k, None)
                    # Volta para o fim da fila[cite: 4]
                    fila_missoes.append((idx, missao, tentativas + 1))
                    drone.carga = carga_original
                    continue  # Pula o registo desta tentativa[cite: 4]
                else:
                    # Se falhou mesmo sozinho no fim, regista definitivamente[cite: 4]
                    logger.registrar(1, id_drone, idx + 1, "IDA", rota_ida[-1], drone.energia_consumida_kwh,
                                     drone.carga, bateu)
                    frota_resultados.append({
                        'id_entrega': idx + 1, 'id_drone': id_drone + 1, 'uav': drone, 'rota': rota_ida, 'bateu': bateu,
                        'queda': queda, 'start': s_real, 'goal': g_real,
                        'tempo_global': np.arange(t_ida, t_ida + len(rota_ida)), 'esperas_total': hovers_ida
                    })
            else:
                drone.carga = 0.0

                # --- CORREÇÃO: Consumo físico de Hover no Cliente ---[cite: 4]
                if TEMPO_DESCARGA > 0:
                    custo_peso_m = peso_pacote * drone.penalidade_carga
                    consumo_descarga_wh = TEMPO_DESCARGA * (
                                drone.consumo_hover + (custo_peso_m * drone.velocidade_horiz))

                    drone.bateria_atual_wh -= consumo_descarga_wh
                    consumo_acumulado_wh[id_drone] += consumo_descarga_wh
                    drone.energia_consumida_kwh += (consumo_descarga_wh / 1000.0)

                    if drone.bateria_atual_wh <= 0:
                        drone.bateria_atual_wh = 0.0
                        drone.bateria_baixa = True
                        print(f"   🔋 ALERTA: O UAV {id_drone + 1} esgotou a bateria em Hover no cliente!")
                # ----------------------------------------------------

                # === ATUALIZADO: Reconhece o MOSP como 4D ===[cite: 4]
                t_volta = t_ida + len(cam_ida) + TEMPO_DESCARGA if (
                        'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO) else 0

                # === ATUALIZADO: Reconhece o MOSP como 4D ===[cite: 4]
                if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
                    for extra in range(TEMPO_DESCARGA):
                        chave = (
                            cam_ida[-1][0], cam_ida[-1][1],
                            cam_ida[-1][2] if len(cam_ida[-1]) == 3 else drone.altura_alvo,
                            t_ida + len(cam_ida) + extra)
                        reserva_global[chave] = id_drone
                        reservas_temporarias.append(chave)

                print(f"   🔎 Missão {idx + 1} (UAV {id_drone + 1}): Planeando VOLTA...")

                t0 = time.perf_counter()
                cam_volta, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, g_real, s_real, reserva_global,
                                                       t_volta)
                t_cpu_algoritmo += (time.perf_counter() - t0)

                if cam_volta:
                    # === ATUALIZADO: Reconhece o MOSP como 4D ===[cite: 4]
                    if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
                        for i, p in enumerate(cam_volta):
                            chave = (p[0], p[1], p[2] if len(p) == 3 else drone.altura_alvo, t_volta + i)
                            reserva_global[chave] = id_drone
                            reservas_temporarias.append(chave)
                        _, hovers_volta, _ = analisar_cenario(cam_volta)

                    # Salva a bateria antes do voo de regresso (incluindo o consumo do tempo de descarga)[cite: 4]
                    bateria_antes_volta = drone.bateria_atual_wh
                    rota_volta, bateu_v, queda_v = drone.simular_missao(cam_volta, lotes_gdf)

                    # Acumula o consumo real da viagem de volta e descarga[cite: 4]
                    consumo_acumulado_wh[id_drone] += (bateria_antes_volta - drone.bateria_atual_wh)

                    # --- CHECK COLISÃO NA VOLTA ---[cite: 4]
                    if bateu_v:
                        if drone.motivo_falha == 'bateria':
                            print(f"   🔋 ALERTA: O UAV {id_drone + 1} ficou sem bateria em {queda_v} durante a VOLTA!")
                        else:
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

                    # Se chegou aqui, ou correu bem ou é o registo de uma falha na 2ª tentativa[cite: 4]
                    logger.registrar(1, id_drone, idx + 1, "IDA", rota_ida[-1], drone.energia_consumida_kwh,
                                     drone.carga, False)
                    logger.registrar(1, id_drone, idx + 1, "VOLTA", rota_volta[-1], drone.energia_consumida_kwh,
                                     drone.carga, bateu_v)

                    r_descarga = np.tile(rota_ida[-1], (TEMPO_DESCARGA, 1)) if TEMPO_DESCARGA > 0 else np.empty(
                        (0, rota_ida.shape[1]))
                    r_completa = np.vstack((rota_ida, r_descarga, rota_volta))
                    t_global = np.arange(t_ida, t_volta + len(rota_volta))

                    disponibilidade_drones[id_drone] = t_volta + len(rota_volta)

                    # UAV pousou de volta no CD sem falhas: desativada a recarga instantânea[cite: 4]
                    if not bateu_v:
                        pass

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
    # NOVO: TELEMETRIA AVANÇADA DE BATERIAS
    # ==========================================
    print("\n🔋 RELATÓRIO ENERGÉTICO DA FROTA (Centro de Distribuição):")
    print("-" * 100)
    print(
        f"{'UAV':<5} | {'Modelo':<24} | {'Bat. Inicial':<14} | {'Consumido':<14} | {'Recarregado':<14} | {'Bat. Final'}")
    print("-" * 100)

    for i, d in enumerate(frota_de_drones):
        aviso = " ⚠️ BAIXA" if d.bateria_baixa else ""

        inicial_wh = d.bateria_capacidade_wh
        consumo_wh = consumo_acumulado_wh[i]
        recarga_wh = recarga_acumulada_wh[i]
        final_wh = d.bateria_atual_wh
        final_pct = d.bateria_percentual()

        print(
            f"UAV {i + 1:<2} | {d.nome_modelo:<24} | {inicial_wh:>10.1f} Wh | {consumo_wh:>10.1f} Wh | {recarga_wh:>10.1f} Wh | {final_wh:>6.1f} Wh ({final_pct:>5.1f}%){aviso}")
    print("-" * 100)

    # Chamada atualizada com a lista de drones original[cite: 4]
    imprimir_relatorio_tecnico_detalhado(frota_de_drones, consumo_acumulado_wh)

    # ==========================================
    # RENDERIZAÇÃO
    # ==========================================
    print("\n4/6 a 6/6: A renderizar Gráficos...")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        renderer.plotar_frota_3d(max_x, max_y, lotes_gdf, cds, frota_resultados)
        renderer.plotar_frota_2d(lotes_gdf, cds, frota_resultados)
        renderer.plotar_diagrama_espaco_tempo(frota_resultados)

        # NOVOS GRÁFICOS ANALÍTICOS (Dissertação)[cite: 4]
        renderer.plotar_curva_descarga(frota_resultados)
        renderer.plotar_boxplot_esperas(frota_resultados, cfg.NUM_DRONES_DISPONIVEIS)
        renderer.plotar_mapa_calor(lotes_gdf, frota_resultados)


if __name__ == "__main__":
    executar_simulacao()