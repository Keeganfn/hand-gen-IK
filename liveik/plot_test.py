from skspatial.objects import Line, Point
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon


points = [[1,2], [2,1], [3,4], [5,6]]
line = Line(point=[0, 0], direction=[1, 1])
projected_points = []
for i in points:
    point = Point(i)
    point_projected = line.project_point(point)
    projected_points.append(point_projected)

comparison = 0
# plt.plot([0,5], [0,5])
# print(projected_points)
# print(np.array(projected_points)[:,0])
# plt.plot(np.array(projected_points)[:,0],np.array(projected_points)[:,1],'ro')

x = [0, .45, .7, .45, 0, -.45, -.7, -.45]
y = [.7, .45, 0, -.45, -.7, -.45, 0, .45]
pgon = Polygon(zip(x, y)) # Assuming the OP's x,y coordinates
print(pgon.area)
plt.fill(x, y, alpha=.2)
# plt.show()



plt.plot([0,0], [0,1], "--")
plt.plot([0,.707],[0,.707], "--")
plt.plot([0,1],[0,0], "--")
plt.plot([0,.707],[0,-.707], "--")
plt.plot([0,0], [0,-1], "--")
plt.plot([0,-.707],[0,-.707], "--")
plt.plot([0,-1],[0,0], "--")
plt.plot([0,-.707],[0,.707], "--")
plt.axis('square')
plt.show()

