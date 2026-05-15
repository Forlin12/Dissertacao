# config.py

# [Controlo de Simulação]
CENARIO_SEMPRE_NOVO = False

# [Geografia e Topologia]
COORDENADAS = (41.8058, -6.7572) # Bragança
RAIO_M = 200
LARGURA_RUA = 10
AREA_MEDIA_LOTE = 350
ESPACO_ENTRE_LOTES = 1.2
DENSIDADE_PREDIOS = 0.6
ALTURA_MIN = 20
ALTURA_MAX = 45
USAR_MINI_MAPA = True
TAMANHO_MINI_MAPA = 400

NUM_DRONES_DISPONIVEIS = 5
NUM_ENTREGAS_TOTAL = 10
DISTANCIA_MIN_ENTREGA = 70.0
MARGEM_SEGURANCA_MAPA = 15.0
ZONA_LIVRE_CD = 7
ZONA_LIVRE_ENTREGA = 0.2

# [Parâmetros Físicos do Drone]
CUSTO_ESPERA = 1.2
VETOR_CAMADAS_VOO = [25, 30, 35, 40, 50]
DRONE_RAIO_M = 2.0
DRONE_ALTURA_VOO = 25
DRONE_VELOCIDADE_MS = 25
DRONE_CARGA_KG = 3.0
TEMPO_DESCARGA = 5
# [Inteligência de Navegação (Cérebro do Drone)]
# Escolha uma das 3 opções abaixo:
# 'A_STAR'         -> 2D Clássico (Mais rápido, mas bate se não houver caminho na altitude base)
# 'A_STAR_CAMADAS' -> 2.5D Multi-Tier (Tenta subir para alturas maiores se o caminho estiver bloqueado)
# 'TEA_STAR'       -> 3D Espaço-Tempo (Desvia de prédios e de outros drones, com penalidade de espera)
TIPO_ALGORITMO = 'TEA_STAR_CAMADAS'

# [Caminho de Salvamento de Logs]
CAMINHO_LOG = r"C:\Users\Forlin\Dissertação\log"

# [Configurações de Tráfego e Visualização UTM]
# Lista com o atraso de saída de cada missão (se houver mais missões, ele repete o ciclo)
VETOR_TEMPOS_PARTIDA = [0]


ATIVAR_RELATORIO_FANTASMA = False

# ==========================================
# ATRIBUIÇÃO MANUAL DE MISSÕES
# ==========================================
# Formato: [ID_do_Pedido, ID_do_Drone]
# (Lembrando que em Python a contagem começa em 0)
# ==========================================
# ATRIBUIÇÃO MANUAL DE MISSÕES
# ==========================================
# O índice (posição) na lista é o ID da Encomenda.
# O valor é o ID do Drone que vai fazer a entrega.
VETOR_PEDIDO_DRONE = [0]
