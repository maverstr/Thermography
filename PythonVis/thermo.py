import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import time


fname = "raw.txt"
frames = []
with open(fname) as file:
    values = []
    for line in file:
        line = line.rstrip()
        if line == '':
            frame = np.array(values, np.int16).reshape((24, 32))
            print(frame.shape)
            frames.append(frame)
            values=[]
            continue
        v = line.split(" ")
        values.extend(v)
        


frames = np.array(frames)
print(frames.shape)

print(frames[0])

min = frames.min()
max = frames.max()

print("min : %d"%min)
print("max : %d"%max)

dev = np.std(frames, axis=0)
print(dev)

diff = np.diff(frames, axis=0)
print(diff.shape)

sum = np.cumsum(diff, axis=0)  
print(sum.shape) 
   
fig = plt.figure()
im = plt.imshow(sum[0], cmap='hot', interpolation='nearest')


print(sum[-1])

def animate(i):
    im.set_data(sum[i])
    return im
    
anim = animation.FuncAnimation(fig, animate, frames=sum.shape[0],
                               interval=50, repeat=False)
plt.show()