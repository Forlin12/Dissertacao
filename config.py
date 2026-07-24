# config.py

# [Controlo de Simulação]
CENARIO_SEMPRE_NOVO = False

# [Geografia e Topologia]
COORDENADAS = (41.8058, -6.7572) # Bragança
RAIO_M = 250
LARGURA_RUA = 10
AREA_MEDIA_LOTE = 350
ESPACO_ENTRE_LOTES = 1.2
DENSIDADE_PREDIOS = 0.6
ALTURA_MIN = 20
ALTURA_MAX = 45
USAR_MINI_MAPA = True
TAMANHO_MINI_MAPA = 500

NUM_DRONES_DISPONIVEIS = 8
NUM_ENTREGAS_TOTAL = 20
DISTANCIA_MIN_ENTREGA = 70.0
MARGEM_SEGURANCA_MAPA = 15.0
ZONA_LIVRE_CD = 7
ZONA_LIVRE_ENTREGA = 0.2

# [Parâmetros Físicos do Drone]
CUSTO_ESPERA = 4
VETOR_CAMADAS_VOO = [25, 28, 30, 35, 40, 50]

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

TIPO_ALGORITMO = 'TEA_STAR_CAMADAS'

# [Caminho de Salvamento de Logs]
CAMINHO_LOG = r"C:\Users\Forlin\Dissertação\log"

# [Configurações de Tráfego e Visualização UTM]
# Lista com o atraso de saída de cada missão (se houver mais missões, ele repete o ciclo)
VETOR_TEMPOS_PARTIDA = [0]



VETOR_PEDIDO_DRONE = [0]


# USAR_VENTO = True
# Vetor 3D do vento (Wx, Wy, Wz) em m/s.
# Exemplo: (5.0, 0.0, 0.0) significa vento a soprar 5 m/s na direção X positivo (Este).
# VETOR_VENTO = (5.0, -2.0, 0.0)


PERFIS_DRONES = [
    {
        'id_modelo': 'Standard_M300',
        'raio_m': 2.0,
        'altura_voo': 25,
        'carga_kg': 3.0,
        'velocidade_cruzeiro_ms': 15.0,
        'velocidade_subida_ms': 5.0,
        'velocidade_descida_ms': 3.0,
        'bateria_capacidade_wh': 200.0,
        'bateria_reserva_seguranca_pct': 20.0,
        'taxa_recarga_w': 3000.0,
        'consumo_cruzeiro_wh_m': 0.15,
        'consumo_subida_wh_m': 0.25,
        'consumo_descida_wh_m': 0.05,
        'consumo_hover_wh_s': 0.12,
        'penalidade_carga_wh_kg_m': 0.02
    },
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
]