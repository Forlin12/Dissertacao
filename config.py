# config.py
from sympy import false

# [Controlo de Simulação]
CENARIO_SEMPRE_NOVO = False

# [Geografia e Topologia]
COORDENADAS = (41.8058, -6.7572) # Bragança
RAIO_M = 500
LARGURA_RUA = 10
AREA_MEDIA_LOTE = 400
ESPACO_ENTRE_LOTES = 1.2
DENSIDADE_PREDIOS = 0.6
ALTURA_MIN = 20
ALTURA_MAX = 45
USAR_MINI_MAPA = False
TAMANHO_MINI_MAPA = 500

# ==========================================
# MATRIZ DE TESTES AUTOMATIZADOS (BATCH RUN)
# ==========================================
# Formato: [ [Nº Drones, Nº Entregas, 'ALGORITMO'], ... ]
MATRIZ_TESTES = [
    [8, 10, 'TEA_STAR_CAMADAS'],  # Teste 1: Otimizado para Tempo
    [8, 10, 'MOSP_CAMADAS'],      # Teste 2: O mesmo cenário, focado em Tempo+Energia
    [5, 8, 'TEA_STAR_CAMADAS'],  # Teste 3: Cenário de alta pressão (poucos drones, muita carga)
    # [5, 8, 'MOSP_CAMADAS']       # Teste 4: Comparação de alta pressão
]

# (Valores de fallback caso o loop não sobrescreva)
NUM_DRONES_DISPONIVEIS = 8
NUM_ENTREGAS_TOTAL = 10
TIPO_ALGORITMO = 'TEA_STAR_CAMADAS'


DISTANCIA_MIN_ENTREGA = 70.0
MARGEM_SEGURANCA_MAPA = 15.0
ZONA_LIVRE_CD = 7
ZONA_LIVRE_ENTREGA = 0.2

# [Parâmetros Físicos do Drone]
CUSTO_ESPERA = 4000
VETOR_CAMADAS_VOO = [25, 28, 30, 35, 40, 50]


VETOR_PEDIDO_DRONE = [4, 5, 3, 5, 6, 5, 2, 6, 0, 7]
# ==========================================
# PERFIS DE DRONES (Frota Heterogénea)
# ==========================================
# Cada drone da frota é montado a partir de um destes perfis, distribuídos
# de forma cíclica pelos NUM_DRONES_DISPONIVEIS (perfis[i % len(perfis)]).
# Ou seja: com 5 drones e 3 perfis, o padrão é 0,1,2,0,1 — customiza como quiseres.
#
# Campos de cada perfil:
#   raio_m                        -> raio físico do drone (usado na colisão), em metros
#   altura_voo                    -> altitude base de cruzeiro, em metros
#   carga_kg                      -> carga transportada, em kg
#   velocidade_cruzeiro_ms        -> velocidade horizontal de cruzeiro, em m/s
#   velocidade_subida_ms          -> velocidade vertical de SUBIDA, em m/s
#   velocidade_descida_ms         -> velocidade vertical de DESCIDA, em m/s
#   bateria_capacidade_wh         -> capacidade total da bateria, em Wh
#   bateria_reserva_seguranca_pct -> % mínima de bateria antes de acender o alerta "bateria_baixa"
#   potencia_hover_w              -> potência base para se manter parado no ar, em W
#   potencia_subida_w             -> potência ADICIONAL ao hover durante a subida, em W
#   potencia_descida_w            -> potência ADICIONAL ao hover durante a descida, em W
#                                     (pode ser negativa — descida costuma "poupar" energia)
#   coef_arrasto_cruzeiro         -> arrasto: W extra em cruzeiro = coef * velocidade^2
#   coef_carga_w_por_kg           -> W extra por cada kg de carga transportada
#
# Se a bateria de um drone chegar a 0 Wh a meio de uma rota, a missão é
# marcada como falha com motivo 'bateria' (distinto de 'colisao'), e o
# ponto exato da queda/pouso forçado é registado — ver uav_physics.Drone.


TEMPO_DESCARGA = 5
# [Inteligência de Navegação (Cérebro do Drone)]
# Escolha uma das opções abaixo:
# 'A_STAR'         -> 2D Clássico (Mais rápido, mas bate se não houver caminho na altitude base)
# 'A_STAR_CAMADAS' -> 2.5D Multi-Tier (Tenta subir para alturas maiores se o caminho estiver bloqueado)
# 'TEA_STAR'       -> 3D Espaço-Tempo (Desvia de prédios e de outros drones, com penalidade de espera)
# 'TEA_STAR_CAMADAS'-> 4D Espaço-Tempo Multi-Tier Clássico
# 'MOSP_CAMADAS'   -> 4D Espaço-Tempo Multi-Objetivo (Minimiza tempo total e consumo de energia)

TIPO_ALGORITMO = 'MOSP_CAMADAS'

# [Caminho de Salvamento de Logs]
CAMINHO_LOG = r"C:\Users\Forlin\Dissertação\log"

# [Configurações de Tráfego e Visualização UTM]
# Lista com o atraso de saída de cada missão (se houver mais missões, ele repete o ciclo)
VETOR_TEMPOS_PARTIDA = [0]






# USAR_VENTO = True
# Vetor 3D do vento (Wx, Wy, Wz) em m/s.
# Exemplo: (5.0, 0.0, 0.0) significa vento a soprar 5 m/s na direção X positivo (Este).
# VETOR_VENTO = (5.0, -2.0, 0.0)


PERFIS_DRONES = [
    # {
    #     'id_modelo': 'Standard_M300',
    #     'raio_m': 2.0,
    #     'altura_voo': 25,
    #     'carga_kg': 3.0,
    #     'velocidade_cruzeiro_ms': 15.0,
    #     'velocidade_subida_ms': 5.0,
    #     'velocidade_descida_ms': 3.0,
    #     'bateria_capacidade_wh': 200.0,
    #     'bateria_reserva_seguranca_pct': 20.0,
    #     'taxa_recarga_w': 3000.0,
    #     'consumo_cruzeiro_wh_m': 0.15,
    #     'consumo_subida_wh_m': 0.25,
    #     'consumo_descida_wh_m': 0.05,
    #     'consumo_hover_wh_s': 0.12,
    #     'penalidade_carga_wh_kg_m': 0.02
    # },
    {
        'id_modelo': 'DJI_Matrice_300_RTK',
        'raio_m': 2,
        'altura_voo': 100,
        'carga_kg': 2.7,
        'velocidade_cruzeiro_ms': 8.0,
        'velocidade_subida_ms': 3.0,
        'velocidade_descida_ms': 2.5,
        'bateria_capacidade_wh': 548.0,
        'bateria_reserva_seguranca_pct': 20.0,
        'taxa_recarga_w': 992.0,
        'consumo_cruzeiro_wh_m': 0.02076,
        'consumo_subida_wh_m': 0.07988,
        'consumo_descida_wh_m': 0.06642,
        'consumo_hover_wh_s': 0.1661,
        'penalidade_carga_wh_kg_m': 0.003459
    },
    {
        'id_modelo': 'Wingcopter_198',
        'raio_m': 2,
        'altura_voo': 100,
        'carga_kg': 5.0,
        'velocidade_cruzeiro_ms': 28.0,
        'velocidade_subida_ms': 3.0,
        'velocidade_descida_ms': 3.0,
        'bateria_capacidade_wh': 1628.0,
        'bateria_reserva_seguranca_pct': 20.0,
        'taxa_recarga_w': 1085.3,
        'consumo_cruzeiro_wh_m': 0.0173,
        'consumo_subida_wh_m': 0.045,
        'consumo_descida_wh_m': 0.050,
        'consumo_hover_wh_s': 1.44,
        'penalidade_carga_wh_kg_m': 0.001
    },
    # {
    # 'id_modelo': 'Matternet_M2',
    # 'raio_m': 10000.0,                    # [OFICIAL] metade dos 20km de alcance (matternet.com)
    # 'altura_voo': 120,                    # [OFICIAL] datasheet técnico 2017
    # 'carga_kg': 2.0,                      # [OFICIAL] matternet.com
    # 'velocidade_cruzeiro_ms': 10.0,       # [OFICIAL] datasheet técnico 2017
    # 'velocidade_subida_ms': 3.0,          # [ESTIMADO] padrão de transição VTOL, sem dado publicado
    # 'velocidade_descida_ms': 2.5,         # [ESTIMADO] idem
    # 'bateria_capacidade_wh': 826.5,       # [ESTIMADO] 9,5kg (massa vazia c/bateria) x 87 Wh/kg
    # 'bateria_reserva_seguranca_pct': 20.0,# [ESTIMADO] premissa consistente com os outros perfis
    # 'taxa_recarga_w': 1496.0,             # [ESTIMADO] 826,5 x 1,81
    # 'consumo_cruzeiro_wh_m': 0.03306,     # [ESTIMADO] 0,8x826,5Wh / 20.000m (ida+volta)
    # 'consumo_subida_wh_m': 0.1416,        # [ESTIMADO] física m·g·v aplicada
    # 'consumo_descida_wh_m': 0.1322,       # [ESTIMADO] idem
    # 'consumo_hover_wh_s': 0.3306,         # [ESTIMADO] potência de cruzeiro / 3600
    # 'penalidade_carga_wh_kg_m': 0.004311  # [ESTIMADO] escalonamento P∝W^1,5
    # },
    # {
    # 'id_modelo': 'Wing_Alphabet',
    # 'raio_m': 9650.0,                     # [OFICIAL] metade de 19,3km round-trip declarado
    # 'altura_voo': 45,                     # [FONTE SECUNDÁRIA] ~45m, fontes independentes (Osinto/Medium)
    # 'carga_kg': 1.2,                      # [FONTE SECUNDÁRIA] engenharia reversa (Medium), modelo padrão
    # 'velocidade_cruzeiro_ms': 29.0,       # [OFICIAL] 104,4 km/h declarado pela Alphabet
    # 'velocidade_subida_ms': 3.0,          # [ESTIMADO] sem dado publicado
    # 'velocidade_descida_ms': 2.5,         # [ESTIMADO] idem
    # 'bateria_capacidade_wh': 452.4,       # [ESTIMADO] 5,2kg x 87 Wh/kg — CAUTELA: asa fixa é mais eficiente, provável superestimativa
    # 'bateria_reserva_seguranca_pct': 20.0,# [ESTIMADO]
    # 'taxa_recarga_w': 818.8,              # [ESTIMADO] 452,4 x 1,81
    # 'consumo_cruzeiro_wh_m': 0.01875,     # [ESTIMADO] 0,8x452,4Wh / 19.300m
    # 'consumo_subida_wh_m': 0.1988,        # [ESTIMADO] física m·g·v
    # 'consumo_descida_wh_m': 0.2175,       # [ESTIMADO] idem
    # 'consumo_hover_wh_s': 0.5438,         # [ESTIMADO] relevância limitada: Wing só paira na transição VTOL
    # 'penalidade_carga_wh_kg_m': 0.004396  # [ESTIMADO]
    # },
    # {
    # 'id_modelo': 'Zipline_P1_Sparrow',
    # 'raio_m': 80000.0,                    # [OFICIAL] MDPI Encyclopedia, dado citado pela Zipline
    # 'altura_voo': 100,                    # [OFICIAL] faixa 80-120m declarada; usei o ponto médio
    # 'carga_kg': 1.75,                     # [OFICIAL] consistente em múltiplas fontes
    # 'velocidade_cruzeiro_ms': 28.06,      # [OFICIAL] 101 km/h declarado
    # 'velocidade_subida_ms': 3.0,          # [ESTIMADO] sem dado publicado
    # 'velocidade_descida_ms': 2.5,         # [ESTIMADO] idem
    # 'bateria_capacidade_wh': 1815.7,      # [ESTIMADO] 20,87kg (peso vazio, museu Sullenberger) x 87 Wh/kg
    # 'bateria_reserva_seguranca_pct': 20.0,# [ESTIMADO]
    # 'taxa_recarga_w': 3286.4,             # [ESTIMADO] 1815,7 x 1,81
    # 'consumo_cruzeiro_wh_m': 0.009079,    # [ESTIMADO] 0,8x1815,7Wh / 160.000m (bate com "160km round-trip" citado por fonte independente)
    # 'consumo_subida_wh_m': 0.1466,        # [ESTIMADO] física m·g·v
    # 'consumo_descida_wh_m': 0.1019,       # [ESTIMADO] idem
    # 'consumo_hover_wh_s': 0.2548,         # [ESTIMADO] relevância limitada: Zipline P1 não paira em voo normal
    # 'penalidade_carga_wh_kg_m': 0.000602  # [ESTIMADO] menor por ser asa fixa (mais consistente com sustentação por planeio)
    # }

]