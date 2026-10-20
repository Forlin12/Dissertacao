# main.py
import sys
from pathlib import Path

# Garante que a raiz do projeto está no path do Python
raiz_projeto = Path(__file__).resolve().parent
if str(raiz_projeto) not in sys.path:
    sys.path.insert(0, str(raiz_projeto))

import numpy as np
import config as cfg
from teste.logger_system import TrainingLogger
from uav_physics import Drone
from city_builder import gerar_cidade
import mission_control
import renderer
import warnings
import time

from navegation.pathfindercamadas import calcular_rota_8way_camadas
from navegation.pathfinder_tea_camadas import calcular_rota_tea_camadas as calcular_rota_tea_camadas
from navegation.pathfinder_tea_4d import calcular_rota_tea_camadas_4d as calcular_rota_tea_4d_camadas
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
    if cfg.TIPO_ALGORITMO == 'A_STAR_CAMADAS':
        return calcular_rota_8way_camadas(max_x, max_y, lotes_gdf, drone, start, goal,
                                          cfg.VETOR_CAMADAS_VOO), start, goal
    elif cfg.TIPO_ALGORITMO == 'TEA_STAR_CAMADAS':
        caminho = calcular_rota_tea_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                            cfg.VETOR_CAMADAS_VOO, tempo_partida)
        return caminho, start, goal
    elif cfg.TIPO_ALGORITMO == 'TEA_STAR_CAMADAS_4d':
        caminho = calcular_rota_tea_4d_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                              cfg.VETOR_CAMADAS_VOO, tempo_partida)
        return caminho, start, goal
    elif cfg.TIPO_ALGORITMO == 'MOSP_CAMADAS':
        caminho = calcular_rota_mosp_camadas(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global,
                                             cfg.VETOR_CAMADAS_VOO, tempo_partida)
        return caminho, start, goal
    else:
        print(f"❌ ERRO: Algoritmo '{cfg.TIPO_ALGORITMO}' não reconhecido no config.py!")
        return [], start, goal


def montar_frota(modo_silencioso=False):
    perfis = cfg.PERFIS_DRONES
    frota = []
    for i in range(cfg.NUM_DRONES_DISPONIVEIS):
        perfil = perfis[i % len(perfis)]
        d = Drone(perfil)
        frota.append(d)
        if not modo_silencioso:
            print(f"   🔋 UAV {i + 1}: {d.nome_modelo} | Bateria: {d.bateria_capacidade_wh:.0f}Wh "
                  f"| Subida: {d.velocidade_subida:.1f}m/s | Descida: {d.velocidade_descida:.1f}m/s "
                  f"| Cruzeiro: {d.velocidade_horiz:.1f}m/s")
    return frota


# =======================================================
# MOTOR UNIFICADO (DRY) - USADO PELO MAIN E PELO GA
# =======================================================
def simular_frota_completo(max_x, max_y, lotes_gdf, missoes, vetor_alocacao=None, modo_silencioso=False, logger=None):
    t_cpu_algoritmo = 0.0
    reserva_global = {}
    frota_resultados = []

    if not modo_silencioso:
        print("\n🛠️  A montar a frota (perfis customizados)...")
    frota_de_drones = montar_frota(modo_silencioso=modo_silencioso)

    num_drones = cfg.NUM_DRONES_DISPONIVEIS
    consumo_acumulado_wh = {i: 0.0 for i in range(num_drones)}
    recarga_acumulada_wh = {i: 0.0 for i in range(num_drones)}
    disponibilidade_drones = {i: 0 for i in range(num_drones)}

    TEMPO_DESCARGA = getattr(cfg, 'TEMPO_DESCARGA', 5)
    vetor_partida = getattr(cfg, 'VETOR_TEMPOS_PARTIDA', [0])

    if vetor_alocacao is None:
        vetor_alocacao = getattr(cfg, 'VETOR_PEDIDO_DRONE', [])
    dict_alocacao = {idx: int(d_id) for idx, d_id in enumerate(vetor_alocacao)}

    if not modo_silencioso:
        print(f"\n3/6: Despacho Logístico...")

    fila_missoes = [(idx, missao, 0) for idx, missao in enumerate(missoes)]
    makespan_frames = 0
    esperas_total_acumulado = 0
    missoes_falhadas = 0

    while fila_missoes:
        idx, missao, tentativas = fila_missoes.pop(0)

        id_drone = dict_alocacao.get(idx, idx % num_drones)
        if id_drone >= num_drones:
            id_drone = 0

        drone = frota_de_drones[id_drone]
        carga_original = drone._carga_original
        drone.reset_metricas()

        bateria_checkpoint = drone.bateria_atual_wh
        consumo_checkpoint = consumo_acumulado_wh[id_drone]
        recarga_checkpoint = recarga_acumulada_wh[id_drone]

        peso_pacote = missao.get('peso_kg', 1.0)
        if peso_pacote > carga_original:
            peso_pacote = carga_original

        drone.carga = peso_pacote

        start = (int(missao['origem'][0]), int(missao['origem'][1]))
        goal = (int(missao['destino'][0]), int(missao['destino'][1]))

        t_base = vetor_partida[idx] if idx < len(vetor_partida) else vetor_partida[-1] + ((idx - len(vetor_partida) + 1) * 15)
        t_ida = max(t_base, disponibilidade_drones[id_drone])

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
            recarga_acumulada_wh[id_drone] += energia_em_falta
            t_ida += tempo_de_carga_s
            drone.carregar_bateria_quantidade(energia_em_falta)

        t0 = time.perf_counter()
        cam_ida, s_real, g_real = roteador_inteligente(max_x, max_y, lotes_gdf, drone, start, goal, reserva_global, t_ida)
        t_cpu_algoritmo += (time.perf_counter() - t0)

        if not cam_ida:
            if tentativas == 0:
                fila_missoes.append((idx, missao, tentativas + 1))
                drone.bateria_atual_wh = bateria_checkpoint
                consumo_acumulado_wh[id_drone] = consumo_checkpoint
                recarga_acumulada_wh[id_drone] = recarga_checkpoint
                drone.carga = carga_original
                continue
            else:
                missoes_falhadas += 1
                drone.carga = carga_original
                continue

        hovers_ida, hovers_volta = 0, 0
        reservas_temporarias = []

        if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
            for i, p in enumerate(cam_ida):
                chave = (p[0], p[1], p[2] if len(p) == 3 else drone.altura_alvo, t_ida + i)
                reserva_global[chave] = id_drone
                reservas_temporarias.append(chave)
            _, hovers_ida, _ = analisar_cenario(cam_ida)

        bateria_antes_ida = drone.bateria_atual_wh
        rota_ida, bateu, queda = drone.simular_missao(cam_ida, lotes_gdf)
        consumo_acumulado_wh[id_drone] += (bateria_antes_ida - drone.bateria_atual_wh)

        if bateu:
            if tentativas == 0:
                for k in reservas_temporarias:
                    reserva_global.pop(k, None)
                fila_missoes.append((idx, missao, tentativas + 1))
                drone.bateria_atual_wh = bateria_checkpoint
                consumo_acumulado_wh[id_drone] = consumo_checkpoint
                recarga_acumulada_wh[id_drone] = recarga_checkpoint
                drone.carga = carga_original
                continue
            else:
                missoes_falhadas += 1
                esperas_total_acumulado += hovers_ida
                drone.carga = carga_original
                continue

        drone.carga = 0.0

        if TEMPO_DESCARGA > 0:
            custo_peso_m = peso_pacote * drone.penalidade_carga
            consumo_descarga_wh = TEMPO_DESCARGA * (drone.consumo_hover + (custo_peso_m * drone.velocidade_horiz))
            drone.bateria_atual_wh -= consumo_descarga_wh
            consumo_acumulado_wh[id_drone] += consumo_descarga_wh
            drone.energia_consumida_kwh += (consumo_descarga_wh / 1000.0)

            if drone.bateria_atual_wh <= 0:
                drone.bateria_atual_wh = 0.0
                drone.bateria_baixa = True

        t_volta = t_ida + len(cam_ida) + TEMPO_DESCARGA if (
                    'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO) else 0

        if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
            for extra in range(TEMPO_DESCARGA):
                chave = (
                    cam_ida[-1][0], cam_ida[-1][1],
                    cam_ida[-1][2] if len(cam_ida[-1]) == 3 else drone.altura_alvo,
                    t_ida + len(cam_ida) + extra)
                reserva_global[chave] = id_drone
                reservas_temporarias.append(chave)

        t0 = time.perf_counter()
        cam_volta, _, _ = roteador_inteligente(max_x, max_y, lotes_gdf, drone, g_real, s_real, reserva_global, t_volta)
        t_cpu_algoritmo += (time.perf_counter() - t0)

        if not cam_volta:
            missoes_falhadas += 1
            drone.carga = carga_original
            continue

        if 'TEA' in cfg.TIPO_ALGORITMO or 'MOSP' in cfg.TIPO_ALGORITMO:
            for i, p in enumerate(cam_volta):
                chave = (p[0], p[1], p[2] if len(p) == 3 else drone.altura_alvo, t_volta + i)
                reserva_global[chave] = id_drone
                reservas_temporarias.append(chave)
            _, hovers_volta, _ = analisar_cenario(cam_volta)

        bateria_antes_volta = drone.bateria_atual_wh
        rota_volta, bateu_v, queda_v = drone.simular_missao(cam_volta, lotes_gdf)
        consumo_acumulado_wh[id_drone] += (bateria_antes_volta - drone.bateria_atual_wh)

        if bateu_v:
            if tentativas == 0:
                for k in reservas_temporarias:
                    reserva_global.pop(k, None)
                fila_missoes.append((idx, missao, tentativas + 1))
                drone.bateria_atual_wh = bateria_checkpoint
                consumo_acumulado_wh[id_drone] = consumo_checkpoint
                recarga_acumulada_wh[id_drone] = recarga_checkpoint
                drone.carga = carga_original
                continue
            else:
                missoes_falhadas += 1

        t_fim = t_volta + len(rota_volta)
        disponibilidade_drones[id_drone] = t_fim
        esperas_total_acumulado += (hovers_ida + hovers_volta)

        if not bateu_v and (t_fim - 1) > makespan_frames:
            makespan_frames = t_fim - 1

        if not modo_silencioso:
            if logger:
                logger.registrar(1, id_drone, idx + 1, "IDA", rota_ida[-1], drone.energia_consumida_kwh, drone.carga, False)
                logger.registrar(1, id_drone, idx + 1, "VOLTA", rota_volta[-1], drone.energia_consumida_kwh, drone.carga, bateu_v)

            r_descarga = np.tile(rota_ida[-1], (TEMPO_DESCARGA, 1)) if TEMPO_DESCARGA > 0 else np.empty((0, rota_ida.shape[1]))
            r_completa = np.vstack((rota_ida, r_descarga, rota_volta))
            t_global = np.arange(t_ida, t_volta + len(rota_volta))

            frota_resultados.append({
                'id_entrega': idx + 1, 'id_drone': id_drone + 1, 'uav': drone, 'rota': r_completa,
                'bateu': bateu_v, 'queda': queda_v, 'start': s_real, 'goal': g_real, 'tempo_global': t_global,
                'esperas_total': hovers_ida + hovers_volta
            })

        drone.carga = carga_original

    energia_total_wh = sum(consumo_acumulado_wh.values())

    return {
        'makespan_frames': makespan_frames,
        'esperas_total': esperas_total_acumulado,
        'missoes_falhadas': missoes_falhadas,
        'energia_total_wh': energia_total_wh,
        'frota_de_drones': frota_de_drones,
        'frota_resultados': frota_resultados,
        'consumo_acumulado_wh': consumo_acumulado_wh,
        'recarga_acumulada_wh': recarga_acumulada_wh,
        't_cpu_algoritmo': t_cpu_algoritmo
    }


def imprimir_relatorio_tecnico_detalhado(frota_de_drones, consumo_acumulado_wh):
    print("\n" + "=" * 95)
    print("📈 RELATÓRIO TÉCNICO DE DESEMPENHO E IMPACTO ATMOSFÉRICO")
    print("=" * 95)
    total_distancia_km = sum([d.distancia_voada for d in frota_de_drones]) / 1000.0
    total_consumido_wh = sum(consumo_acumulado_wh.values())
    if total_distancia_km > 0:
        eficiencia_media = total_consumido_wh / total_distancia_km
        print(f"Eficiência Média Real da Frota: {eficiencia_media:.2f} Wh/km")
    print("=" * 95 + "\n")


def executar_simulacao():
    print(f"🚀 A iniciar Gestão de Frota | Cérebro Ativo: {cfg.TIPO_ALGORITMO}...")
    t_inicio_global = time.perf_counter()
    logger = TrainingLogger(prefixo="main")
    lotes_gdf, max_x, max_y = gerar_cidade()
    cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)

    res = simular_frota_completo(max_x, max_y, lotes_gdf, missoes, getattr(cfg, 'VETOR_PEDIDO_DRONE', []), False, logger)
    t_cpu_total = time.perf_counter() - t_inicio_global

    print(f"\n📊 Resumo Rápido: Makespan: {res['makespan_frames']}f | Esperas: {res['esperas_total']}f | Falhas: {res['missoes_falhadas']}")
    imprimir_relatorio_tecnico_detalhado(res['frota_de_drones'], res['consumo_acumulado_wh'])

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        renderer.plotar_frota_3d(max_x, max_y, lotes_gdf, cds, res['frota_resultados'])
        renderer.plotar_frota_2d(lotes_gdf, cds, res['frota_resultados'])
        renderer.plotar_diagrama_espaco_tempo(res['frota_resultados'])
        renderer.plotar_curva_descarga(res['frota_resultados'])
        renderer.plotar_boxplot_esperas(res['frota_resultados'], cfg.NUM_DRONES_DISPONIVEIS)
        renderer.plotar_mapa_calor(lotes_gdf, res['frota_resultados'])


def executar_bateria_testes():
    for teste_idx, config_teste in enumerate(cfg.MATRIZ_TESTES):
        cfg.NUM_DRONES_DISPONIVEIS = config_teste[0]
        cfg.NUM_ENTREGAS_TOTAL = config_teste[1]
        cfg.TIPO_ALGORITMO = config_teste[2] if len(config_teste) > 2 else cfg.TIPO_ALGORITMO
        if not getattr(cfg, 'CENARIO_SEMPRE_NOVO', False):
            np.random.seed(42)
        executar_simulacao()


if __name__ == "__main__":
    executar_bateria_testes()