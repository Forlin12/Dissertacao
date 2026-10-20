import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import csv
import json
import math
import traceback
import numpy as np
import time
import config as cfg
from city_builder import gerar_cidade
import mission_control

# Importação correta puxando da pasta 'teste'
from teste.ga_optimizer import executar_otimizacao_batch

STATE_FILE = "training_state.json"
DATASET_CSV = "dataset_treino.csv"

MATRIZ_TESTES_IA = [
    [5, 10, 'MOSP_CAMADAS'],
    [5, 20, 'TEA_STAR_CAMADAS'],
    [5, 40, 'TEA_STAR_CAMADAS'],
    [10, 20, 'TEA_STAR_CAMADAS'],
    [10, 40, 'TEA_STAR_CAMADAS'],
    [10, 80, 'TEA_STAR_CAMADAS'],
    [20, 40, 'TEA_STAR_CAMADAS'],
    [20, 80, 'TEA_STAR_CAMADAS'],
    [20, 160, 'TEA_STAR_CAMADAS']
]


def carregar_estado():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r") as f:
            return json.load(f)
    return {"ultima_amostra_concluida": -1}


def salvar_estado(amostra_idx):
    with open(STATE_FILE, "w") as f:
        json.dump({"ultima_amostra_concluida": amostra_idx}, f)


def extrair_features_relativas(missoes, max_x, max_y, num_drones):
    """
    Extrai features relativas e normalizadas para evitar overfit espacial.
    Em vez de coordenadas X/Y absolutas, usa distâncias relativas ao centro,
    cargas médias e densidade da frota.
    """
    cd_x, cd_y = max_x / 2.0, max_y / 2.0
    distancias = []
    pesos = []
    for m in missoes:
        d = math.hypot(m['destino'][0] - cd_x, m['destino'][1] - cd_y)
        distancias.append(d)
        pesos.append(m['peso_kg'])

    features = [
        num_drones,
        len(missoes),
        np.mean(distancias) if distancias else 0.0,
        np.std(distancias) if distancias else 0.0,
        np.mean(pesos) if pesos else 0.0,
        np.max(pesos) if pesos else 0.0
    ]
    return features


def iniciar_geracao_dataset():
    max_entregas = getattr(cfg, 'MAX_ENTREGAS_IA', 160)
    max_drones = getattr(cfg, 'MAX_DRONES_IA', 20)
    TOTAL_AMOSTRAS_DESEJADAS = 20

    # Estrutura do Dataset focada em features relativas e métrica de convergência
    colunas = [
        "id_amostra", "num_drones_ativos", "num_entregas_reais",
        "feat_dist_media", "feat_dist_std", "feat_peso_medio", "feat_peso_max"
    ]

    for d in range(max_drones):
        colunas.extend([f"uav_{d}_carga_kg", f"uav_{d}_vel_ms", f"uav_{d}_bateria_wh", f"uav_{d}_consumo_wh_m"])

    for i in range(max_entregas):
        colunas.extend([f"drone_alocado_{i}"])

    # Métricas de otimização e tempo de convergência
    colunas.extend(["geracoes_convergencia", "makespan_final", "esperas_final", "falhas_final", "energia_final"])

    if not os.path.exists(DATASET_CSV):
        with open(DATASET_CSV, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(colunas)

    estado = carregar_estado()
    amostra_inicial = estado["ultima_amostra_concluida"] + 1

    if amostra_inicial >= TOTAL_AMOSTRAS_DESEJADAS:
        print("✅ Dataset de treino já está completo!")
        return

    print(f"🚀 Iniciando/Retomando geração de dataset resiliente a partir da amostra {amostra_inicial}...")

    cfg.CENARIO_SEMPRE_NOVO = False
    lotes_gdf, max_x, max_y = gerar_cidade()
    perfis_disponiveis = cfg.PERFIS_DRONES
    num_perfis = len(perfis_disponiveis)

    for amostra in range(amostra_inicial, TOTAL_AMOSTRAS_DESEJADAS):
        try:
            cenario_atual = MATRIZ_TESTES_IA[amostra % len(MATRIZ_TESTES_IA)]
            num_drones = cenario_atual[0]
            num_entregas = cenario_atual[1]
            algoritmo_atual = cenario_atual[2]

            print(f"\n{'=' * 50}")
            print(f"📊 GERANDO AMOSTRA {amostra} / {TOTAL_AMOSTRAS_DESEJADAS}")
            print(f"🚁 Drones: {num_drones} | 📦 Entregas: {num_entregas} | 🧠 Alg: {algoritmo_atual}")
            print(f"{'=' * 50}")

            cfg.NUM_DRONES_DISPONIVEIS = num_drones
            cfg.NUM_ENTREGAS_TOTAL = num_entregas
            cfg.TIPO_ALGORITMO = algoritmo_atual

            cds, missoes = mission_control.gerar_tarefas_logisticas(max_x, max_y, lotes_gdf)
            features_cenario = extrair_features_relativas(missoes, max_x, max_y, num_drones)

            t0_exec = time.perf_counter()
            # Executa o AG e obtém a solução ótima e o número de gerações necessárias para convergir
            vetor_perfeito, makespan, esperas, falhas, energia, geracoes_gastas = executar_otimizacao_batch(
                amostra, num_drones, num_entregas, algoritmo_atual
            )
            tempo_total_exec = time.perf_counter() - t0_exec

            linha = [amostra, num_drones, num_entregas] + features_cenario

            # Adiciona dados dos drones
            for d in range(max_drones):
                if d < num_drones:
                    p = perfis_disponiveis[d % num_perfis]
                    linha.extend([
                        p.get('carga_kg', 0.0),
                        p.get('velocidade_cruzeiro_ms', 0.0),
                        p.get('bateria_capacidade_wh', 0.0),
                        p.get('consumo_cruzeiro_wh_m', 0.0)
                    ])
                else:
                    linha.extend([0.0, 0.0, 0.0, 0.0])

            # Adiciona alocações de cada pedido
            for i in range(max_entregas):
                if i < len(missoes):
                    linha.append(vetor_perfeito[i])
                else:
                    linha.append(-1)  # Padding para entregas inexistentes no cenário

            linha.extend([geracoes_gastas, makespan, esperas, falhas, round(energia, 2)])

            # Escrita atômica segura contra quedas de servidor
            temp_file = DATASET_CSV + ".tmp"
            with open(temp_file, mode='w', newline='') as f_tmp:
                writer_tmp = csv.writer(f_tmp)
                # Lê o arquivo original e reescreve com a nova linha para manter integridade
                if os.path.exists(DATASET_CSV):
                    with open(DATASET_CSV, mode='r') as f_orig:
                        for r in csv.reader(f_orig):
                            writer_tmp.writerow(r)
                writer_tmp.writerow(linha)

            os.replace(temp_file, DATASET_CSV)
            salvar_estado(amostra)
            print(f"💾 Amostra {amostra} salva com sucesso no disco com redundância atômica!")

        except Exception as e:
            print(f"\n⚠️ ERRO NA AMOSTRA {amostra}! O script guardou o progresso anterior. Detalhe: {e}")
            traceback.print_exc()
            continue


if __name__ == "__main__":
    iniciar_geracao_dataset()