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

        # Formatação legível para encontrar os testes (ex: 16-08-2026_13h00)
        self.timestamp = datetime.now().strftime("%d-%m-%Y_%Hh%M")

        # Garante exatamente a extração dos 5 caracteres iniciais
        prefix = str(prefixo)[:5]

        # Ficheiro ÚNICO dedicado para o Live Tracking dos Recordes do GA
        self.arquivo_recordes = os.path.join(self.log_dir, f"{prefix}_recordes_ga_{self.timestamp}.csv")
        self._inicializar_arquivo_recordes()

    def _inicializar_arquivo_recordes(self):
        """Cria o cabeçalho do ficheiro de tracking de recordes."""
        with open(self.arquivo_recordes, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Tempo_Ate_Solucao_Segundos",
                "Makespan_Frames",
                "Energia_Total_Wh",
                "Total_Esperas_Hover",
                "Vetor_Combinacao"
            ])

    def salvar_novo_recorde_ga(self, tempo_execucao, makespan, energia, esperas, vetor_solucao):
        """Guarda imediatamente um novo recorde do Algoritmo Genético no disco."""
        with open(self.arquivo_recordes, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            vetor_limpo = [int(v) for v in vetor_solucao]
            writer.writerow([
                round(tempo_execucao, 2),
                makespan,
                round(energia, 2),
                esperas,
                str(vetor_limpo)
            ])

    # =====================================================================
    # FUNÇÕES FANTASMA - Evitam erros no main.py, mas não criam ficheiros
    # =====================================================================
    def _inicializar_arquivo(self):
        pass

    def registrar(self, cenario_id, id_drone, id_missao, fase, pos_final, energia, carga, bateu):
        pass

    def salvar_resumo_ga(self, kpis_ga):
        pass

    def salvar_kpis_globais(self, kpis_dict):
        pass