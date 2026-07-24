# logger_system.py
import os
import csv
from datetime import datetime
import config as cfg


class TrainingLogger:
    def __init__(self, prefixo="simulacao"):
        """Inicializa o sistema de logs com ficheiros únicos baseados em data/hora."""
        self.log_dir = getattr(cfg, 'CAMINHO_LOG', 'logs')
        os.makedirs(self.log_dir, exist_ok=True)

        # Gera um timestamp único para a execução atual
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Ficheiro de rotas com nome único
        self.arquivo_log = os.path.join(self.log_dir, f"{prefixo}_rotas_{self.timestamp}.csv")
        self._inicializar_arquivo()

    def _inicializar_arquivo(self):
        """Cria o cabeçalho do ficheiro de missões."""
        with open(self.arquivo_log, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Cenario_ID", "ID_Drone", "ID_Missao", "Fase",
                "Pos_Final_X", "Pos_Final_Y", "Pos_Final_Z",
                "Energia_kWh", "Carga_Kg", "Colisao"
            ])

    def registrar(self, cenario_id, id_drone, id_missao, fase, pos_final, energia, carga, bateu):
        """Regista os dados físicos e operacionais individuais de cada missão."""
        with open(self.arquivo_log, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            pos_x, pos_y = pos_final[0], pos_final[1]
            pos_z = pos_final[2] if len(pos_final) == 3 else getattr(cfg, 'DRONE_ALTURA_VOO', 40)

            writer.writerow([
                cenario_id, id_drone, id_missao, fase,
                round(pos_x, 2), round(pos_y, 2), round(pos_z, 2),
                round(energia, 6), round(carga, 2), bateu
            ])

    def salvar_resumo_ga(self, kpis_ga):
        """Salva o resumo do treino do Algoritmo Genético num CSV único."""
        arquivo_ga = os.path.join(self.log_dir, f"ga_resumo_treino_{self.timestamp}.csv")

        with open(arquivo_ga, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=kpis_ga.keys())
            writer.writeheader()
            writer.writerow(kpis_ga)
        print(f"  💾 [LOG] Resumo do GA guardado em: {arquivo_ga}")

    def salvar_kpis_globais(self, kpis_dict):
        """Salva as métricas globais da simulação principal num CSV único."""
        arquivo_kpis = os.path.join(self.log_dir, f"main_kpis_globais_{self.timestamp}.csv")

        with open(arquivo_kpis, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=kpis_dict.keys())
            writer.writeheader()
            writer.writerow(kpis_dict)
        print(f"  💾 [LOG] KPIs globais guardados em: {arquivo_kpis}")