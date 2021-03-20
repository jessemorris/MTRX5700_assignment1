import matplotlib.pyplot as plt

plt.style.use('seaborn-whitegrid')
import numpy as np
import math

length1 = 300
length2 = 280
length3 = 350

iterator = 10
maxS = 150
startingHeight = np.linspace(0, maxS, iterator)

Angle2 = np.linspace(-(math.pi / 3), (math.pi / 3), 50)
Angle3 = np.linspace(-(math.pi * 2 / 3), (math.pi * 2 / 3), 70)
a2 = Angle2
a3 = Angle3

x1 = 0
y1 = 0

for m in startingHeight:

    y1 = m

    for i in a2:
        for j in a3:
            x2 = x1 + length2 * math.cos(i)
            y2 = y1 + length2 * math.sin(i)

            x3 = x2 + length3 * math.cos(j + i)
            y3 = y2 + length3 * math.sin(j + i)

            # Shows all possible end effector poses (workspace)
            plt.figure(1)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.plot(x3, y3, 'co')


            # Shows all possible positions of each link (configuration space)
            plt.figure(2)
            plt.plot([x1, abs(x2)], [y1, y2], color='green', linewidth=1)
            plt.plot([abs(x2), x3], [y2, y3], color='red', linewidth=1)
            plt.gca().set_aspect('equal', adjustable='box')

plt.show()
