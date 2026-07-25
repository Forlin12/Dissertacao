# uav_physics.py
import numpy as np
import math
from shapely.geometry import Point
import config as cfg  # Necessário para ler o vento


class Drone:
    """
    UAV com perfil físico individual e customizável.
    Energia baseada em dados diretos do fabricante, com integração de vento.
    """

    def __init__(self, perfil: dict):
        self.perfil = perfil
        self.nome_modelo = perfil.get('id_modelo', 'Drone_Generico')

        self.raio = perfil.get('raio_m', 2.0)
        self.altura_alvo = perfil.get('altura_voo', 25.0)
        self.carga = perfil.get('carga_kg', 0.0)
        self._carga_original = self.carga

        self.velocidade_horiz = perfil.get('velocidade_cruzeiro_ms', 15.0)
        self.velocidade_subida = perfil.get('velocidade_subida_ms', 3.0)
        self.velocidade_descida = perfil.get('velocidade_descida_ms', 3.0)
        self.velocidade_vert = self.velocidade_subida

        self.consumo_cruzeiro = perfil.get('consumo_cruzeiro_wh_m', 0.15)
        self.consumo_subida = perfil.get('consumo_subida_wh_m', 0.25)
        self.consumo_descida = perfil.get('consumo_descida_wh_m', 0.05)
        self.consumo_hover = perfil.get('consumo_hover_wh_s', 0.12)
        self.penalidade_carga = perfil.get('penalidade_carga_wh_kg_m', 0.02)

        self.bateria_capacidade_wh = perfil.get('bateria_capacidade_wh', 300.0)
        self.bateria_reserva_pct = perfil.get('bateria_reserva_seguranca_pct', 20.0)
        self.bateria_atual_wh = self.bateria_capacidade_wh
        self.bateria_baixa = False

        self.taxa_recarga_w = perfil.get('taxa_recarga_w', 3000.0)

        self.distancia_voada = 0.0
        self.tempo_voo_s = 0.0
        self.energia_consumida_kwh = 0.0
        self.motivo_falha = None

    def reset_metricas(self):
        # NOTA: 'bateria_baixa' propositalmente NÃO é resetado aqui.
        # Este método é chamado a cada nova missão do mesmo drone (ver main.py).
        # Se resetássemos a flag, o relatório final só refletiria a última
        # missão de cada UAV, escondendo alertas de bateria baixa ocorridos
        # em missões anteriores da mesma simulação.
        self.distancia_voada = 0.0
        self.tempo_voo_s = 0.0
        self.energia_consumida_kwh = 0.0
        self.motivo_falha = None

    def recarregar_bateria(self):
        self.bateria_atual_wh = self.bateria_capacidade_wh

    def bateria_percentual(self):
        if self.bateria_capacidade_wh <= 0:
            return 0.0
        return max(0.0, (self.bateria_atual_wh / self.bateria_capacidade_wh) * 100.0)

    def calcular_fator_vento(self, dx, dy, dz, velocidade_fase):
        """
        Gera um multiplicador de consumo baseado no produto escalar do vento.
        Fator > 1.0 (Headwind - Vento Contra)
        Fator < 1.0 (Tailwind - Vento a Favor)
        """
        if not getattr(cfg, 'USAR_VENTO', False):
            return 1.0

        dist = np.linalg.norm([dx, dy, dz])
        if dist == 0:
            return 1.0

        wx, wy, wz = getattr(cfg, 'VETOR_VENTO', (0.0, 0.0, 0.0))

        # Produto escalar para achar a componente de vento na direção do movimento
        vento_a_favor = ((dx / dist) * wx + (dy / dist) * wy + (dz / dist) * wz)
        vento_contra = -vento_a_favor

        # O impacto no consumo é proporcional à força do vento vs velocidade do drone
        fator = 1.0 + (vento_contra / velocidade_fase)

        # Limita fisicamente (O drone nunca vira um "gerador eólico" infinito, mínimo de consumo garantido)
        return max(0.4, fator)

    def estimar_energia_missao(self, dist_ida, dist_volta, tempo_descarga_s, peso_carga):
        """Estimativa de despacho pré-voo (Ignora o vento por segurança)"""
        custo_ida = dist_ida * (self.consumo_cruzeiro + (peso_carga * self.penalidade_carga))
        custo_volta = dist_volta * self.consumo_cruzeiro
        custo_hover = tempo_descarga_s * (
                    self.consumo_hover + (peso_carga * (self.penalidade_carga * self.velocidade_horiz)))
        return custo_ida + custo_volta + custo_hover

    def carregar_bateria_quantidade(self, energia_wh):
        self.bateria_atual_wh = min(self.bateria_capacidade_wh, self.bateria_atual_wh + energia_wh)

    def checar_impacto(self, alvo_xyz, lotes_gdf):
        hitbox = Point(alvo_xyz[0], alvo_xyz[1]).buffer(self.raio)
        idx_possiveis = list(lotes_gdf.sindex.intersection(hitbox.bounds))
        if not idx_possiveis:
            return False
        predios_proximos = lotes_gdf.iloc[idx_possiveis]
        for _, predio in predios_proximos.iterrows():
            if alvo_xyz[2] <= predio['altura_z']:
                if hitbox.intersects(predio.geometry):
                    return True
        return False

    def simular_missao(self, caminho_a_star, lotes_gdf):
        rota_xyz = []
        colisao = False
        ponto_falha = None
        self.motivo_falha = None

        z_inicial = caminho_a_star[0][2] if len(caminho_a_star[0]) == 3 else self.altura_alvo
        pos_atual = np.array([caminho_a_star[0][0], caminho_a_star[0][1], z_inicial], dtype=float)
        rota_xyz.append(pos_atual.copy())

        for i in range(1, len(caminho_a_star)):
            p_alvo = caminho_a_star[i]
            alvo_z = p_alvo[2] if len(p_alvo) == 3 else self.altura_alvo
            alvo = np.array([p_alvo[0], p_alvo[1], alvo_z], dtype=float)

            if self.checar_impacto(alvo, lotes_gdf):
                colisao = True
                ponto_falha = alvo.copy()
                self.motivo_falha = 'colisao'
                break

            dist_xy = np.linalg.norm(alvo[:2] - pos_atual[:2])
            dist_z = alvo[2] - pos_atual[2]

            energia_wh = 0.0
            dt = 0.0
            custo_peso_m = self.carga * self.penalidade_carga

            if dist_xy == 0 and dist_z == 0:
                dt = 1.0
                energia_wh = self.consumo_hover * dt
            else:
                if dist_z > 1e-6:
                    fator_v = self.calcular_fator_vento(0, 0, dist_z, self.velocidade_subida)
                    dt = dist_z / self.velocidade_subida if self.velocidade_subida > 0 else 0.0
                    energia_wh += dist_z * (self.consumo_subida + custo_peso_m) * fator_v

                elif dist_z < -1e-6:
                    fator_v = self.calcular_fator_vento(0, 0, dist_z, self.velocidade_descida)
                    dt = abs(dist_z) / self.velocidade_descida if self.velocidade_descida > 0 else 0.0
                    energia_wh += abs(dist_z) * (self.consumo_descida + custo_peso_m) * fator_v

                if dist_xy > 0:
                    dx, dy = alvo[0] - pos_atual[0], alvo[1] - pos_atual[1]
                    fator_v = self.calcular_fator_vento(dx, dy, 0, self.velocidade_horiz)

                    t_horiz = dist_xy / self.velocidade_horiz if self.velocidade_horiz > 0 else 0.0
                    dt = max(dt, t_horiz)
                    energia_wh += dist_xy * (self.consumo_cruzeiro + custo_peso_m) * fator_v

            self.tempo_voo_s += dt
            self.distancia_voada += math.hypot(dist_xy, dist_z)
            self.energia_consumida_kwh += energia_wh / 1000.0
            self.bateria_atual_wh -= energia_wh

            if self.bateria_percentual() < self.bateria_reserva_pct:
                self.bateria_baixa = True

            if self.bateria_atual_wh <= 0:
                self.bateria_atual_wh = 0.0
                colisao = True
                pos_atual = alvo.copy()
                rota_xyz.append(pos_atual.copy())
                ponto_falha = alvo.copy()
                self.motivo_falha = 'bateria'
                break

            pos_atual = alvo.copy()
            rota_xyz.append(pos_atual.copy())

        return np.array(rota_xyz), colisao, ponto_falha