import pandas as pd

t1 = pd.read_csv('fumaca.csv')
t2 = pd.read_csv('umidade.csv')
t3 = pd.read_csv('temperatura.csv')
t4 = pd.read_csv('gas.csv')

t1 = t1.add_prefix('fumaca')
t2 = t2.add_prefix('umidade')
t3 = t3.add_prefix('temperatura')
t4 = t4.add_prefix('gas')

t1 = t1.reset_index(drop=True)
t2 = t2.reset_index(drop=True)
t3 = t3.reset_index(drop=True)
t4 = t4.reset_index(drop=True)

n = len(t1)
assert len(t2) == n and len(t3) == n and len(t4) == n, "Os CSVs precisam ter o mesmo número de linhas."

out = pd.concat([t1, t2, t3,t4], axis=1)

out.to_csv('coleta.csv', index=False) 