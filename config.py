# config.py

# [Controlo de Simulação]
CENARIO_SEMPRE_NOVO = False

# [Geografia e Topologia]
COORDENADAS = (41.8058, -6.7572) # Bragança
RAIO_M = 200
LARGURA_RUA = 10
AREA_MEDIA_LOTE = 350
ESPACO_ENTRE_LOTES = 0.6
DENSIDADE_PREDIOS = 0.8
ALTURA_MIN = 20
ALTURA_MAX = 45

# [Gestão de Frota e Logística]
NUM_DRONES_DISPONIVEIS = 20
NUM_ENTREGAS_TOTAL = 20
DISTANCIA_MIN_ENTREGA = 70.0
MARGEM_SEGURANCA_MAPA = 15.0
ZONA_LIVRE_CD = 15.0
ZONA_LIVRE_ENTREGA = 5.0

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
TIPO_ALGORITMO = 'TEA_STAR'

# [Caminho de Salvamento de Logs]
CAMINHO_LOG = r"C:\Users\Forlin\Dissertação\log"

# [Configurações de Tráfego e Visualização UTM]
# Lista com o atraso de saída de cada missão (se houver mais missões, ele repete o ciclo)
VETOR_TEMPOS_PARTIDA = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


ATIVAR_RELATORIO_FANTASMA = False

# ==========================================
# ATRIBUIÇÃO MANUAL DE MISSÕES
# ==========================================
# Formato: [ID_do_Pedido, ID_do_Drone]
# (Lembrando que em Python a contagem começa em 0)
MATRIZ_PEDIDO_DRONE = [
    [1, 1],  # Pedido 1 vai para o Drone 1
    [2, 2],  # Pedido 2 vai para o Drone 0 (O Drone 0 faz duas viagens)
    [3, 2]   # Pedido 3 vai para o Drone 2
]