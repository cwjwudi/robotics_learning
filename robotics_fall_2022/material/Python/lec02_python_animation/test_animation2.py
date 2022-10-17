import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

# matplotlib.patches.Rectangle(xy, width, height, angle=0.0, **kwargs)
rect = plt.Rectangle((0, 0),2, 1,color='brown',alpha=1)

#class matplotlib.patches.Circle(xy, radius=5, **kwargs)[source]
# circ = plt.Circle((1,0.5),0.25,color="yellow")

#class matplotlib.lines.Line2D(xdata, ydata, linewidth=None, linestyle=None, color=None, marker=None, markersize=None, markeredgewidth=None, markeredgecolor=None, markerfacecolor=None, markerfacecoloralt='none', fillstyle=None, antialiased=None, dash_capstyle=None, solid_capstyle=None, dash_joinstyle=None, solid_joinstyle=None, pickradius=5, drawstyle=None, markevery=None, **kwargs)[source]
line = plt.Line2D([0, 2],[1, 1],color='black',linewidth=5)

y1 = np.linspace(0.5,1.5,20)
y2 = np.linspace(1.5,0.5,40)
y = np.concatenate((y1,y2))
for j in range(3):
    for i in range(len(y)):
        circ = plt.Circle((1,y[i]),0.25,color="yellow")
        ax.add_patch(circ)
        ax.add_patch(rect)
        ax.add_line(line)
        plt.xlim(0,2)
        plt.ylim(0,2)
        plt.gca().set_aspect('equal')
        plt.pause(0.05)
        circ.remove()

# plt.show()
plt.close()
