from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas

points = pandas.read_csv('gps.csv')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = points['Longitude'].values
y = points['Latitude'].values
z = points['Altitude'].values

ax.scatter(x, y, z, c='r', marker='*')

plt.show()

