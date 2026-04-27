# uav_physics.py
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

    def calcular_potencia_kw(self, estado='cruzeiro'):
        p_estatica = 0.5 + (0.15 * self.carga)
        if estado == 'subida':
            return p_estatica * 1.5 + 0.1
        elif estado == 'cruzeiro':
            return p_estatica + (0.005 * self.velocidade_horiz ** 2)
        elif estado == 'descida':
            return p_estatica * 0.7
        return p_estatica

    def checar_impacto(self, pos_xyz, obstaculos_gdf):
        hitbox = Point(pos_xyz[0], pos_xyz[1]).buffer(self.raio)
        for _, predio in obstaculos_gdf.iterrows():
            if pos_xyz[2] <= predio['altura_z']:
                if hitbox.intersects(predio.geometry):
                    return True
        return False

    def simular_missao(self, caminho_a_star, lotes_gdf, dt=0.1):
        pos = np.array([caminho_a_star[0][0], caminho_a_star[0][1], 0.1], dtype=float)
        rota_xyz = [pos.copy()]
        colisao = False;
        ponto_queda = None

        # FASE 1: DESCOLAGEM VERTICAL
        while pos[2] < self.altura_alvo:
            pos[2] += self.velocidade_vert * dt
            self.tempo_voo_s += dt
            self.energia_consumida_kwh += self.calcular_potencia_kw('subida') * (dt / 3600)
            rota_xyz.append(pos.copy())
            if self.checar_impacto(pos, lotes_gdf): colisao = True; ponto_queda = pos.copy(); break

        # FASE 2: VOO DE CRUZEIRO
        if not colisao:
            for i in range(1, len(caminho_a_star)):
                alvo = np.array([caminho_a_star[i][0], caminho_a_star[i][1], self.altura_alvo], dtype=float)
                while np.linalg.norm(alvo[:2] - pos[:2]) > (self.velocidade_horiz * dt):
                    dir_vec = (alvo - pos)[:2]
                    pos[:2] += (dir_vec / np.linalg.norm(dir_vec)) * self.velocidade_horiz * dt

                    self.tempo_voo_s += dt
                    self.distancia_voada += self.velocidade_horiz * dt
                    self.energia_consumida_kwh += self.calcular_potencia_kw('cruzeiro') * (dt / 3600)
                    rota_xyz.append(pos.copy())

                    if self.checar_impacto(pos, lotes_gdf): colisao = True; ponto_queda = pos.copy(); break
                if colisao: break
                pos[:2] = alvo[:2]

                # FASE 3: POUSO VERTICAL
        if not colisao:
            while pos[2] > 0.15:
                pos[2] -= self.velocidade_vert * dt
                self.tempo_voo_s += dt
                self.energia_consumida_kwh += self.calcular_potencia_kw('descida') * (dt / 3600)
                rota_xyz.append(pos.copy())

        return np.array(rota_xyz), colisao, ponto_queda