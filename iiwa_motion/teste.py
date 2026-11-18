import sys
import statistics
import matplotlib.pyplot as plt

def ler_arquivo(nome_arquivo):
    """
    Lê o arquivo e extrai posição (y), gap e velocidade.
    Retorna três listas: posicoes, gaps, velocidades.
    """
    posicoes = []
    gaps = []
    velocidades = []
    tempo = []

    with open(nome_arquivo, 'r', encoding='utf-8') as f:
        for linha in f:
            linha = linha.strip()
            if not linha or linha.startswith('#'):  # ignora linhas vazias ou comentários
                continue

            time = float(linha.strip())

            tempo.append(time)
    return tempo

def gerar_graficos(posicoes, gaps, velocidades):
    """Cria os dois gráficos solicitados."""
    # Gráfico 1: Gap (y) vs Posição (x)
    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.plot(posicoes, gaps, 'b.-', linewidth=1, markersize=4)
    plt.xlabel('Y Position (m)')
    plt.ylabel('Gap (m)')
    plt.title('Gap measurements')
    plt.grid(True, linestyle='--', alpha=0.7)

    # Gráfico 2: Velocidade (y) vs Posição (x)
    plt.subplot(1, 2, 2)
    plt.plot(posicoes, velocidades, 'r.-', linewidth=1, markersize=4)
    plt.xlabel('Y Position (m)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity variation')
    plt.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.show()

def main():
    # Obtém o nome do arquivo da linha de comando ou solicita ao usuário
    if len(sys.argv) > 1:
        arquivo = sys.argv[1]
    else:
        arquivo = input("Informe o caminho do arquivo de dados: ")

    try:
        tempo = ler_arquivo(arquivo)
    except FileNotFoundError:
        print(f"Erro: arquivo '{arquivo}' não encontrado.")
        return
    except Exception as e:
        print(f"Erro inesperado ao ler o arquivo: {e}")
        return



    print([statistics.mean(tempo), statistics.stdev(tempo)])


if __name__ == "__main__":
    main()
