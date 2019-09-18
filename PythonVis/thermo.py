import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import time
import threading


fname = "raw11.txt"
frames = []
frames2 = []
column = 0
row = 0
lineNumber = 1
previousLineNumber = 0

totalFrames = []
previousShape = (0,0)
anim = []
sumVal = []


with open(fname) as file:
    values = []
    for lineNumber,line in enumerate(file):
        line = line.rstrip()
        if line == '':
            row = lineNumber - previousLineNumber
            frame = np.array(values, np.int16).reshape((row, column))
            if(frame.shape != previousShape):
                if(doonce):
                    totalFrames.append(frames)
                    frames = []
                else:
                    doonce = True
            print(frame.shape)
            frames.append(frame)
            values=[]
            previousLineNumber = lineNumber +1
            previousShape = frame.shape
            continue
        v = line.split(" ")
        column = len(v);
        values.extend(v)
    
totalFrames.append(frames)

fig = plt.figure()

def plotGraph(i):
    def animate(i):
        im.set_data(sumVal[i])
        return 
    
    frames = np.array(totalFrames[i])
    print("in")
    print(i)
    diff = np.diff(frames, axis=0)
    sumVal = np.cumsum(diff, axis=0)
    im = plt.imshow(sumVal[0], cmap='hot', interpolation='none', animated=True)
    anim.append(animation.FuncAnimation(fig, animate, frames=sumVal.shape[0],
                           interval=1, repeat=False))

for i in range(len(totalFrames)):
    while (not plt.waitforbuttonpress()):
    
        p = threading.Thread(target = plotGraph, args = (i,) )
        p.start()
        plt.show(block = False) 
        p.join()
        break

while (not plt.waitforbuttonpress()):
    plt.close('all')
    break
