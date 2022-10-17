from matplotlib import pyplot as plt
import numpy as np

t = np.arange(0,2*np.pi,0.1)
y = np.cos(t)

plt.plot(t,y)
for i in range(len(t)):
    temp, = plt.plot(t[i],y[i],color='green',marker = 'o', markersize=10)
    plt.pause(0.05)
    temp.remove()

plt.show()
