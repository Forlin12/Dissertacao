import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import math


def mascara_nao_dominados(pontos):
    """
    pontos: array (N, 2) com [makespan, energia] (minimizar ambos).
    Retorna máscara booleana: True = solução não-dominada.
    """
    n = len(pontos)
    mascara = np.ones(n, dtype=bool)
    for i in range(n):
        # j domina i se é <= em tudo e < em pelo menos um objetivo
        domina = np.all(pontos <= pontos[i], axis=1) & np.any(pontos < pontos[i], axis=1)
        if domina.any():
            mascara[i] = False
    return mascara


# 1. Diretório onde este script está salvo
diretorio = os.path.dirname(os.path.abspath(__file__))

# 2. Busca e ordena todos os .csv do diretório
arquivos = sorted([f for f in os.listdir(diretorio) if f.endswith('.csv')])
num_arquivos = len(arquivos)

if num_arquivos == 0:
    print(f"Nenhum arquivo .csv encontrado na pasta: {diretorio}")
else:
    # 3. Grid de gráficos
    colunas = 2
    linhas = math.ceil(num_arquivos / colunas)

    fig, axes = plt.subplots(linhas, colunas, figsize=(14, 5 * linhas))
    axes = np.atleast_1d(axes).flatten()

    for i, nome_arquivo in enumerate(arquivos):
        caminho_completo = os.path.join(diretorio, nome_arquivo)

        try:
            df = pd.read_csv(caminho_completo)

            # Usa os nomes das colunas se existirem; senão, cai no fallback por posição
            if 'Makespan_Frames' in df.columns and 'Energia_Total_Wh' in df.columns:
                df_otimizacao = df[['Makespan_Frames', 'Energia_Total_Wh']]
            else:
                df_otimizacao = df.iloc[:, 1:3]

            # Remove pontos duplicados (mesmo makespan e energia)
            df_otimizacao = df_otimizacao.drop_duplicates().reset_index(drop=True)

            valores = df_otimizacao.values.astype(float)
            makespan_todos = valores[:, 0]
            energia_todos = valores[:, 1]

            # ---------------------------------------------------
            # FILTRO DE DOMINÂNCIA
            # ---------------------------------------------------
            mascara = mascara_nao_dominados(valores)
            pareto = valores[mascara]
            dominados = valores[~mascara]

            # Ordena a fronteira por makespan (para desenhar a linha)
            pareto = pareto[np.argsort(pareto[:, 0])]
            makespan = pareto[:, 0]
            energia = pareto[:, 1]

            # ---------------------------------------------------
            # Ponto ótimo (mais próximo do ideal) — só na fronteira real
            # ---------------------------------------------------
            col_min = pareto.min(axis=0)
            col_max = pareto.max(axis=0)
            range_vals = col_max - col_min
            range_vals[range_vals == 0] = 1

            pareto_normalized = (pareto - col_min) / range_vals
            distancias = np.linalg.norm(pareto_normalized - np.zeros(2), axis=1)
            indice_otimo = np.argmin(distancias)

            melhor_makespan = makespan[indice_otimo]
            melhor_energia = energia[indice_otimo]

            # ---------------------------------------------------
            # Plotagem
            # ---------------------------------------------------
            ax = axes[i]

            ax.plot(makespan, energia, c='#1f77b4', alpha=0.4, linestyle='--')
            ax.scatter(makespan, energia, c='#1f77b4', label=f'Fronteira de Pareto ({len(pareto)})',
                       alpha=0.8, edgecolors='w', s=60)

            label_otima = f'Ótima: Makespan {melhor_makespan:.0f} | {melhor_energia:.2f} Wh'
            ax.scatter(melhor_makespan, melhor_energia, c='red', marker='*', s=250, label=label_otima,
                       edgecolors='black')

            ax.set_title(f'Análise: {nome_arquivo}')
            ax.set_xlabel('Makespan')
            ax.set_ylabel('Energia (Wh)')
            ax.grid(True, linestyle='--', alpha=0.5)
            ax.legend(fontsize=9)

            print(f"{nome_arquivo}: {len(valores)} pontos -> {len(pareto)} não-dominados, "
                  f"{len(dominados)} dominados")

        except Exception as e:
            axes[i].set_title(f'Erro ao processar {nome_arquivo}')
            axes[i].axis('off')
            print(f"Erro em {nome_arquivo}: {e}")

    # 4. Oculta subplots vazios
    for j in range(num_arquivos, len(axes)):
        axes[j].axis('off')

    plt.tight_layout()
    plt.show()