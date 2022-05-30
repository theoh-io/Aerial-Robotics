import matplotlib.pyplot as plt
import pandas as pd

# import data in pandas dataframe (change the file name to your local file)
data = pd.read_csv('logs/2022_05_30_11_31_05.csv', header = None)
print(data)

# remove all-zeros rows
data = data[(data.T != 0).any()]
print(data)

# plot all timeseries
plt.figure()
for i in data:
    ax=data[i].plot()
ax.legend(["x_axis", "y_axis", "z_axis", "z_ranger", "yaw"])
plt.show()
#plt.savefig('Take off - Test velocity fast')