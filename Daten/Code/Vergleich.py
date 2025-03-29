import pandas as pd
import os
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib
matplotlib.use("TkAgg")  # Alternativ: "QtAgg", je nach Betriebssystem



df_down_Felix = pd.read_csv('Daten Felix/zusammengefasst/df_NDB.csv')
df_down_Pia = pd.read_csv('Daten Pia/zusammengefasst/df_NDB.csv')
df_down = pd.concat([df_down_Pia, df_down_Felix], axis = 0)
df_down = df_down.drop(columns=['Unnamed: 0.1', 'Unnamed: 0'])
df_down['position'] = 'down'

df_neutra_Felix = pd.read_csv('Daten Felix/zusammengefasst/df_NNB.csv')
df_neutra_Pia = pd.read_csv('Daten Pia/zusammengefasst/df_NNB.csv')
df_neutral = pd.concat([df_neutra_Felix, df_neutra_Pia], axis = 0)
df_neutral = df_neutral.drop(columns=['Unnamed: 0.1', 'Unnamed: 0'])
df_neutral['position'] = 'neutral'
df = pd.concat([df_neutral, df_down], axis=0)


#sensor_cols = ['accX', 'accY', 'accZ', 'gyrX', 'gyrY', 'gyrZ', 'magX', 'magY', 'magZ']
sensor_cols = ['magX', 'magY', 'magZ']
df_long = df.melt(id_vars='position', value_vars=sensor_cols,
                  var_name='Achse', value_name='Wert')

plt.figure(figsize=(10, 6))
sns.boxplot(x='Achse', y='Wert', data=df_long, hue="position")
plt.title("Boxplot der Sensorachse Magnetometer in uT – Sensorposition 'Back'")
plt.ylabel("Messwert")
plt.show()

