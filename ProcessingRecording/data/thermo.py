import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import time
import threading
import argparse             # for command line argument parsing
from os.path import isfile  # for checking if a file exists

from matplotlib.widgets import LassoSelector
from matplotlib import path




#parser = argparse.ArgumentParser(description='display raw data recorded')
#parser.add_argument('inputFile', metavar='input file name', help='the input txt file')
#        
#args = parser.parse_args()
#
## print args just for debugging
#print(args)
#    
#    
## check if input file exists, otherwise display help and quit
#if not isfile(args.inputFile):
#    print("!!!! input file %s not found"%args.inputFile)
#    parser.print_help()
#    exit()

fname = input("enter the name of the file to analyze : ")
if fname == "":
    fname = "rawR5.txt"

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
doonce = False


ROIArray = np.zeros((24,32))
totalROIArray = []
totalCounter = 0

def updateArray(array, indices):
    lin = np.arange(array.size)
    newArray = array.flatten()
    newArray[lin[indices]] = 1
    return newArray.reshape(array.shape)

def onselect(verts):
    global ROIArray, pix, totalCounter, totalROIArray
    ROIArray = totalROIArray[totalCounter]#resets the lasso selection
    print(ROIArray.shape)
    p = path.Path(verts)
    ind = p.contains_points(pix, radius=1)
    print(ind.shape)
    ROIArray = updateArray(ROIArray, ind)
    msk.set_data(ROIArray)
    fig.canvas.draw_idle()
    
def setPixelCoordinates(ROIArray):
    global pix
    # Pixel coordinates
    pixX = np.arange(ROIArray.shape[1])
    pixY = np.arange(ROIArray.shape[0])
    xv, yv = np.meshgrid(pixX, pixY)
    pix = np.vstack( (xv.flatten(), yv.flatten()) ).T



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
                    totalROIArray.append(ROIArray)
                    ROIArray = np.zeros(frame.shape)
                    frames = []
                else:
                    ROIArray = np.zeros(frame.shape)
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
totalROIArray.append(ROIArray)
ROIArray = totalROIArray[0] #to reset the initial roi array
setPixelCoordinates(ROIArray)


# prepare the figure to hold 2 plots
fig = plt.figure(num = "Thermography Visualization")
ax1 = fig.add_subplot(2,2,1)  #  2 x 1 grid, first plot (top)
ax2 = fig.add_subplot(2,1,2)  #  2 x 1 grid, second plot (bottom)
ax3 = fig.add_subplot(2,2,2)
ax1.set_title("Data:")
ax2.set_title("Mean of ROI:")
ax3.set_title("ROI Visualization:")
msk = ax3.imshow(ROIArray,vmax=1, interpolation='nearest')

analogSignalYData = []
analogSignalXData = []
analogMin = 0
analogMax = 0

def plotGraph(i):
    def animate(i):
        global analogMin, analogMax
        im.set_data(sumVal[i])
        #uncomment next line if you want a "live" update of the average plot
        sumValROI = np.cumsum(diff[i] * ROIArray)
        ROIMean = np.mean(sumValROI)
        analogSignalXData.append(i)
        analogSignalYData.append(ROIMean)
        if ROIMean < analogMin:
            analogMin = ROIMean
            ax2.set_ylim(analogMin, analogMax)
        elif ROIMean > analogMax:
            analogMax = ROIMean
            ax2.set_ylim(analogMin, analogMax)
#        im2.set_data(range(i),analogOutput[:i])
        im2.set_data(analogSignalXData, analogSignalYData)
        return 
    
    frames = np.array(totalFrames[i])
    diff = np.diff(frames, axis=0)
    sumVal = np.cumsum(diff, axis=0)
    # compute the analog signal output, must be an array of #(sumVal.shape[0]) values (currently it is set to the mean of each integral frame)
    analogOutput = [np.mean(sumVal[i]) for i in range(sumVal.shape[0])]
    
    # prepare image on ax1
    im = ax1.imshow(sumVal[0], cmap='hot', interpolation='none', animated=True)
    
    #prepare plot on ax2
    im2, = ax2.plot([],[])   # plot returns a *list* of line2D objects, we are interested only in the first (and single) one, hence the comma after im2 
    # we must set the axis start/stop beforehand
    ax2.set_xlim([0,sumVal.shape[0]])                       # number of elements we will have along x axis
#    ax2.set_ylim(np.min(analogOutput), np.max(analogOutput))                # min, max values of the computed output for y axis
    
    # uncomment this to print the whole "average" plot at once (this goes with the comment in the animate() function above)
    #ax2.plot(analogOutput)
    
    anim.append(animation.FuncAnimation(fig, animate, frames=sumVal.shape[0],
                          interval=1, repeat=False))
   
    
lasso = LassoSelector(ax1, onselect)

for i in range(len(totalFrames)):
    while (True):
        if(plt.waitforbuttonpress()):
            ax2.clear()
            analogSignalYData = []
            analogSignalXData = []
            analogMin = 0
            analogMax = 0
            ROIArray = totalROIArray[i]
            totalCounter = i
            setPixelCoordinates(ROIArray)
            p = threading.Thread(target = plotGraph, args = (i,) )
            p.start()
            plt.show(block = False) 
            p.join()
            break


while(True):
    if(plt.waitforbuttonpress()):
        plt.close('all')
        break

