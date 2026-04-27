# main.py
import config as cfg
from uav_physics import Drone
from city_builder import gerar_cidade
from pathfinder import calcular_rota_8way
import renderer

def executar_simulacao():
    print("🚀 A iniciar Simulador Digital Twin UAV...")

    # 1. Instanciar o Drone
    uav = Drone(
        raio_m=cfg.DRONE_RAIO_M,
        altura_voo=cfg.DRONE_ALTURA_VOO,
        velocidade_ms=cfg.DRONE_VELOCIDADE_MS,
        carga_kg=cfg.DRONE_CARGA_KG
    )

    # 2. Gerar Mundo
    print("\n1/4: A gerar gémeo digital da cidade...")
    lotes_gdf, max_x, max_y = gerar_cidade()

    # 3. Planear Rota (Pathfinding)
    print("2/4: A calcular rotas de fuga (A* 8-Way)...")
    caminho, start_real, goal_real = calcular_rota_8way(max_x, max_y, lotes_gdf, uav)

    # 4. Executar Física
    if len(caminho) > 0:
        print("3/4: A simular aerodinâmica e colisões no C-Space...")
        rota_final, bateu, local_queda = uav.simular_missao(caminho, lotes_gdf)
    else:
        print("❌ Erro: Cidade impenetrável para as dimensões deste drone.")
        rota_final, bateu, local_queda = [], False, None

    # 5. Renderizar Relatórios
    print("4/4: A gerar relatórios visuais...")
    renderer.plotar_3d(max_x, max_y, lotes_gdf, uav, rota_final, caminho, bateu, local_queda, start_real, goal_real)
    renderer.plotar_2d(lotes_gdf, start_real, goal_real)

if __name__ == "__main__":
    executar_simulacao()