import cbor2
import pandas as pd
import os

# Transformieren der Daten
#input_folder = "./Daten Felix"
#for filename in os.listdir(input_folder):
#    if filename.endswith(".cbor"):
#        file_path = os.path.join(input_folder, filename)
#        with open(file_path, "rb") as f:
#            data = cbor2.load(f)
#            name = file_path[14:26]
#            cols = data["payload"]["sensors"]
#            cols = [item['name'] for item in cols]
#            rows = [item for item in data["payload"]["values"]]
#            sensor_df = pd.DataFrame(columns=cols, data=rows)
#            sensor_df.to_csv('Daten Felix/transformiert/' + name + '.csv')


df_NDB = pd.DataFrame()
# Transformieren der Daten
input_folder = "./Daten Felix/transformiert"
for filename in os.listdir(input_folder):
    if filename.startswith("NDB"):
        file_path = os.path.join(input_folder, filename)
        df = pd.read_csv(file_path)
        df_NDB = pd.concat([df, df_NDB], axis=0)

df_NDB = df_NDB.reset_index(drop=True)
df_NDB.to_csv('./Daten Felix/zusammengefasst/df_NDB.csv')


