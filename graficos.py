import pandas as pd
import matplotlib.pyplot as plt

ARQUIVO = "coleta.csv"
COLUNAS  = ["fumaca","umidade","temperatura","gas"]

df = pd.read_csv(ARQUIVO, sep=',')

cols = [c for c in COLUNAS if c in df.columns]
if len(cols) != 4:
    raise ValueError(f"Colunas não encontradas: esperadas {COLUNAS}, encontradas {cols}")

x = df.index 

for c in cols:
    plt.figure()
    plt.plot(x, df[c].values)
    plt.xlabel("Tempo (amostras)")
    plt.ylabel(c)
    plt.title(f"{c} vs tempo (sequencial)")
    plt.grid(True)

    plt.show() 
