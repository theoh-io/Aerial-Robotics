import matplotlib.pyplot as plt
import pandas as pd

# import data in pandas dataframe (change the file name to your local file)
data = pd.read_csv('logs/2022_05_24_19_14_58.csv', header = None)
print(data)

# remove all-zeros rows
data = data[(data.T != 0).any()]
print(data)

# plot all timeseries
plt.figure()
for i in data:
    data[i].plot()
    print(i)
plt.show()
#plt.savefig('Take off - Test velocity fast')