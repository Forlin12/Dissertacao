# logger_system.py
import os
import csv
import config as cfg


class TrainingLogger:
    def __init__(self):
        """Inicializa o sistema de logs e garante que a pasta e os ficheiros base existem."""
        self.log_dir = cfg.CAMINHO_LOG
        os.makedirs(self.log_dir, exist_ok=True)

        self.arquivo_log = os.path.join(self.log_dir, "simulacao_log.csv")
        self._inicializar_arquivo()

    def _inicializar_arquivo(self):
        """Cria o cabeçalho do ficheiro de missões caso este não exista."""
        if not os.path.isfile(self.arquivo_log):
            with open(self.arquivo_log, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Cenario_ID", "ID_Drone", "ID_Missao", "Fase",
                    "Pos_Final_X", "Pos_Final_Y", "Pos_Final_Z",
                    "Energia_kWh", "Carga_Kg", "Colisao"
                ])

    def registrar(self, cenario_id, id_drone, id_missao, fase, pos_final, energia, carga, bateu):
        """Regista os dados físicos e operacionais individuais de cada missão (Ida ou Volta)."""
        with open(self.arquivo_log, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            pos_x, pos_y = pos_final[0], pos_final[1]
            pos_z = pos_final[2] if len(pos_final) == 3 else cfg.DRONE_ALTURA_VOO

            writer.writerow([
                cenario_id, id_drone, id_missao, fase,
                round(pos_x, 2), round(pos_y, 2), round(pos_z, 2),
                round(energia, 6), round(carga, 2), bateu
            ])

    def salvar_kpis_globais(self, kpis_dict, caminho_pasta):
        """
        NOVO: Salva as métricas globais da simulação inteira num ficheiro CSV separado
        para facilitar a geração de gráficos para a dissertação.
        """
        os.makedirs(caminho_pasta, exist_ok=True)
        caminho_arquivo = os.path.join(caminho_pasta, "kpis_resumo_simulacao.csv")

        arquivo_ja_existe = os.path.isfile(caminho_arquivo)

        with open(caminho_arquivo, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=kpis_dict.keys())

            if not arquivo_ja_existe:
                writer.writeheader()

            writer.writerow(kpis_dict)