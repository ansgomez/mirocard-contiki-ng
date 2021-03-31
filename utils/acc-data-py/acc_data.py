import pandas as pd
import struct

data = pd.read_csv('test3.txt', header = None, sep=" ")
data.columns=['Timestamp','Time_Diff','Acc_Data']

data[['Counter', 'x_value', 'y_value', 'z_value','lol']] = data.Acc_Data.str.split(",", expand=True)

data = data.drop(data.columns[[1, 2, 3, 7]], axis=1)
data['Timestamp'] = data['Timestamp'].str.slice_replace(0,1,'')
print(data)


print(struct.unpack('!f', bytes.fromhex(data.loc[2][2]))[0])

#for i in range(len(data)-1):
#	data[2][i] = float(struct.unpack('!f', bytes.fromhex(data[2][i]))[0])

#for i in range(len(data)):
#	print(struct.unpack('!d', bytes.fromhex(data.loc[i][2]))[0])