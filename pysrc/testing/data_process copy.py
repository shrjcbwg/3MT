from mpl_toolkits.mplot3d import axes3d
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

dataset = pd.read_csv('~/Desktop/data/cmd_1.txt')

x_data = dataset['x']
y_data = dataset['y']
z_data = dataset['z']

# data.to_csv('~/Desktop/odo1.txt')

# new a figure and set it into 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

# set figure information
ax.set_title("3D_Curve")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

# draw the figure, the color is r = read
figure = ax.plot(x_data, y_data, z_data, c='r')

plt.show()