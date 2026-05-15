# mission_control.py
import numpy as np
import math
from shapely.geometry import Point
import config as cfg


def gerar_tarefas_logisticas(max_x, max_y, lotes_gdf):
    centros_distribuicao = []
    lista_missoes = []

    def esculpir_ponto_livre(x_min, x_max, y_min, y_max, zona_livre):
        # Arredonda para inteiro — garante conformidade exata com o grid do A*
        px = float(int(round(np.random.uniform(x_min, x_max))))
        py = float(int(round(np.random.uniform(y_min, y_max))))
        p = Point(px, py)

        # OTIMIZAÇÃO DE SEGURANÇA: Aumentamos ligeiramente o buffer de limpeza (zona_livre + 1.5)
        # para engolir qualquer micro-fragmento geométrico gerado pelo clipping do mini-mapa.
        area_seguranca = p.buffer(zona_livre + 1.5)

        # Força brutal corrigida para indexação pós-clip/explode
        for idx in lotes_gdf.index:
            if lotes_gdf.loc[idx, 'geometry'].intersects(area_seguranca):
                lotes_gdf.at[idx, 'altura_z'] = 0

        return (px, py)

    print("\n📦 A processar Sistema de Logística (A preparar zonas de aterragem)...")

    cd_principal = esculpir_ponto_livre(cfg.MARGEM_SEGURANCA_MAPA, max_x - cfg.MARGEM_SEGURANCA_MAPA,
                                        cfg.MARGEM_SEGURANCA_MAPA, max_y - cfg.MARGEM_SEGURANCA_MAPA, cfg.ZONA_LIVRE_CD)
    centros_distribuicao.append(cd_principal)
    print(f"   🏠 Base Operacional estabelecida em: {int(cd_principal[0])}, {int(cd_principal[1])}")

    for i in range(cfg.NUM_ENTREGAS_TOTAL):
        for _ in range(100):
            angulo = np.random.uniform(0, 2 * math.pi)
            distancia = np.random.uniform(cfg.DISTANCIA_MIN_ENTREGA, min(max_x / 1.5, max_y / 1.5))

            ex = cd_principal[0] + math.cos(angulo) * distancia
            ey = cd_principal[1] + math.sin(angulo) * distancia

            if (cfg.MARGEM_SEGURANCA_MAPA < ex < max_x - cfg.MARGEM_SEGURANCA_MAPA) and \
                    (cfg.MARGEM_SEGURANCA_MAPA < ey < max_y - cfg.MARGEM_SEGURANCA_MAPA):
                destino = esculpir_ponto_livre(ex - 1, ex + 1, ey - 1, ey + 1, cfg.ZONA_LIVRE_ENTREGA)
                lista_missoes.append({'origem': cd_principal, 'destino': destino})
                print(f"   📦 Pedido {i + 1} agendado. Destino: {int(destino[0])}, {int(destino[1])}")
                break

    return centros_distribuicao, lista_missoes