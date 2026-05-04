import numpy as np
import math
from shapely.geometry import Point


class Drone:
    def __init__(self, raio_m, altura_voo, velocidade_ms, carga_kg):
        self.raio = raio_m
        self.altura_alvo = altura_voo
        self.velocidade_horiz = velocidade_ms
        self.velocidade_vert = 3.0
        self.carga = carga_kg

        self.distancia_voada = 0.0
        self.tempo_voo_s = 0.0
        self.energia_consumida_kwh = 0.0

    def reset_metricas(self):
        """[CORREÇÃO P4/P12] Limpa a telemetria antes de uma nova missão para os logs ficarem precisos."""
        self.distancia_voada = 0.0
        self.tempo_voo_s = 0.0
        self.energia_consumida_kwh = 0.0

    def calcular_potencia_kw(self, estado='cruzeiro'):
        """[CORREÇÃO P5] Uso real da velocidade vertical na física de subida."""
        p_estatica = 0.5 + (0.15 * self.carga)
        if estado == 'subida':
            return p_estatica * (1.0 + 0.15 * self.velocidade_vert)  # Física mais realista
        elif estado == 'cruzeiro':
            return p_estatica + (0.005 * self.velocidade_horiz ** 2)
        elif estado == 'descida':
            return p_estatica * 0.7
        elif estado == 'hover':
            return p_estatica * 1.2
        return p_estatica

    def checar_impacto(self, alvo_xyz, lotes_gdf):
        """[CORREÇÃO P3] Substituição do iterrows O(N) por Spatial Indexing O(log N)"""
        hitbox = Point(alvo_xyz[0], alvo_xyz[1]).buffer(self.raio)

        # O sindex encontra rapidamente os índices dos polígonos na vizinhança geométrica
        idx_possiveis = list(lotes_gdf.sindex.intersection(hitbox.bounds))
        if not idx_possiveis:
            return False

        predios_proximos = lotes_gdf.iloc[idx_possiveis]
        for _, predio in predios_proximos.iterrows():
            if alvo_xyz[2] <= predio['altura_z']:
                if hitbox.intersects(predio.geometry):
                    return True
        return False

    def simular_missao(self, caminho_a_star, lotes_gdf, tempo_por_passo=1.0):
        rota_xyz = []
        colisao = False
        ponto_queda = None

        z_inicial = caminho_a_star[0][2] if len(caminho_a_star[0]) == 3 else self.altura_alvo
        pos_atual = np.array([caminho_a_star[0][0], caminho_a_star[0][1], z_inicial], dtype=float)
        rota_xyz.append(pos_atual.copy())

        for i in range(1, len(caminho_a_star)):
            p_alvo = caminho_a_star[i]
            alvo_z = p_alvo[2] if len(p_alvo) == 3 else self.altura_alvo
            alvo = np.array([p_alvo[0], p_alvo[1], alvo_z], dtype=float)

            # [CORREÇÃO P1] A colisão é testada PREDITIVAMENTE no alvo, antes de o drone se mover para lá
            if self.checar_impacto(alvo, lotes_gdf):
                colisao = True
                ponto_queda = alvo.copy()
                break

            dist_xy = np.linalg.norm(alvo[:2] - pos_atual[:2])
            dist_z = alvo[2] - pos_atual[2]

            estado_motor = 'cruzeiro'
            if dist_xy == 0 and dist_z == 0:
                estado_motor = 'hover'
            elif dist_z > 0:
                estado_motor = 'subida'
            elif dist_z < 0:
                estado_motor = 'descida'

            self.tempo_voo_s += tempo_por_passo
            self.distancia_voada += math.hypot(dist_xy, dist_z)
            self.energia_consumida_kwh += self.calcular_potencia_kw(estado_motor) * (tempo_por_passo / 3600)

            # Movimento consumado com segurança
            pos_atual = alvo.copy()
            rota_xyz.append(pos_atual.copy())

        return np.array(rota_xyz), colisao, ponto_queda