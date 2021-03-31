import pandas as pd
import struct
import matplotlib.pyplot as plt

data = pd.read_csv('test3.txt', header = None, sep=" ")
data.columns=['Timestamp','Time_Diff','Acc_Data']

data[['Counter', 'x_value', 'y_value', 'z_value','lol']] = data.Acc_Data.str.split(",", expand=True)

data = data.drop(data.columns[[1, 2, 3, 7]], axis=1)
data['Timestamp'] = data['Timestamp'].str.slice_replace(0,1,'')
#print(data)
print(data.loc[0][2])
print(struct.unpack('>i', bytes.fromhex(data.loc[0][2]))[0])

for i in range(1,4):
	for j in range(len(data)):
		if len(data.loc[j][i]) < 8:
			data.loc[j][i]="0000"+data.loc[j][i]

for j in range(1,4):
	for i in range(len(data)):
		data.loc[i][j] = (struct.unpack('>i', bytes.fromhex(data.loc[i][j]))[0])

#print(data)

plt.plot(data['Timestamp'], data['x_value'], label = "x_value")
plt.plot(data['Timestamp'], data['y_value'], label = "y_label")
plt.plot(data['Timestamp'], data['z_value'], label = "z_value")
plt.legend()
plt.savefig("test_fig.png")