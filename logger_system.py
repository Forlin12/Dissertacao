# logger_system.py
import os
import csv
from datetime import datetime
import config as cfg


class TrainingLogger:
    def __init__(self):
        if not os.path.exists(cfg.CAMINHO_LOG):
            os.makedirs(cfg.CAMINHO_LOG)

        self.arquivo_csv = os.path.join(cfg.CAMINHO_LOG, f"treino_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.inicializar_csv()

    def inicializar_csv(self):
        cabecalho = [
            "Timestamp", "Drone_ID", "Missao_ID", "Estado",
            "PosX", "PosY", "PosZ", "Bateria_kWh", "Carga_kg", "Impacto"
        ]
        with open(self.arquivo_csv, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(cabecalho)

    def registrar(self, drone_id, missao_id, estado, pos, bateria, carga, impacto=False):
        with open(self.arquivo_csv, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().strftime("%H:%M:%S.%f"),
                drone_id, missao_id, estado,
                round(pos[0], 2), round(pos[1], 2), round(pos[2], 2),
                round(bateria, 6), carga, int(impacto)
            ])