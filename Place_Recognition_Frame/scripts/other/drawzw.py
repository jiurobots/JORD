import matplotlib.pyplot as plt
import matplotlib.image  as mpimg
import numpy as np
from numpy.core.numeric import normalize_axis_tuple
import pylab
from scipy.interpolate import make_interp_spline

readDir1 = "/home/jlurobot/catkin_ws/src/JM_LCD/results/JORD/CSSC_JORD_0621.txt"



Curve_one=[]
Curve_two=[]
Curve_three=[]
Curve_four=[]
Curve_five=[]

drawMat=[]
PRscore=[]
gtMat=[]
dataMat=[]
scoreMat=[]

#save data and core
def savescore(readDir):
    f = open(readDir,"r")
    line = f.readline()
    while line:
        txt = line.split(',')
        dataMat.append(int(txt[0]))
        score = float(txt[2])
        scoreMat.append(score)
        line = f.readline()
    # print ('dataMat:',scoreMat)
    
    f.close()
    
# get ground truth
def generateGT():
    for i in range(0 ,len(dataMat), 1):
    # for num in dataMat:
        num = dataMat[i]
        # if (num >= 1303 and num <= 1553) or (num >= 2581 and num <= 2627) or (num >= 2431 and num <= 2512):   #05 gtdata
        # if (num >= 1304 and num <= 1551) or (num >= 2578 and num <= 2619) or (num >= 2425 and num <= 2516) :   #05 gtdata good use
        # if (num >= 1570 and num <= 1636) or (num >= 2447 and num <= 2463) or (num >= 3398 and num <= 3847) or (num >= 3288 and num <= 3418) or (num >= 4447 and num <= 4531):   #00 gtdata good
        # if (num >= 1570 and num <= 1635) or (num >= 2446 and num <= 2460) or (num >= 3398 and num <= 3844) or (num >= 3295 and num <= 3418) or (num >= 4451 and num <= 4528):   #00 gtdata
        # if (num >= 4205 and num <= 4267) or (num >= 4403 and num <= 4567) or (num >= 4578 and num <= 4615) :    #02 gtdata good
        # if (num >= 4205 and num <= 4266) or (num >= 4404 and num <= 4569) or (num >= 4566 and num <= 4620) :    #02 gtdata
        # if (num >= 834 and num <= 1086) :    #06 gtdata good
        # if (num >= 2381 and num <= 3781) :    # JORD_051
        #if (num >= 4149 and num <= 7742) :    # JORD_052
        if (num >= 1419 and num <= 1500) or (num >= 1621 and num <= 1796): #08 gtdata good
        # if(num >= 1576 and num <= 1593): # 09 gtdata
        # if (num >= 3296 and num <= 3663) or (num >= 3688 and num <= 3715) or (num >= 7366 and num <= 7764):   # 0501_01
       # if (num >= 4219 and num <= 4292) or (num >= 7379 and num <= 7555) or (num >= 7794 and num <= 8148) #or(num >= 8228 and num <= 8299) or (num >= 8482 and num <= 8627) or(num >= 8791 and num <= 9104) or(num >= #10019 and num <= 10164) :   # 1226 gtdata
        # if(num >= 50 and num <= 56) or (num >= 2612 and num <= 2745) or (num >= 3733 and num <= 3741):#1121_1
        # if(num >= 2990 and num <= 3025) or (num >= 3242 and num <= 3258) or(num >= 3392 and num <= 3560) or(num >= 3392 and num <= 3560):#1121_3
            gtMat.append(1)
        else:
            gtMat.append(0)

# calculate tp fp tn fn
def calculatetruefalse(i,drawMat,gtMat):
    A = 0.0
    B = 0.0
    C = 0.0
    for num in range(0, len(dataMat), 1):
        if float(scoreMat[num]) <= i:
            drawMat.append(1)
        else :
            drawMat.append(0)
        # calculate A B C
        if (gtMat[num] == 1) and (drawMat[num] == 1) :
            A=A+1
        if (gtMat[num] == 0) and (drawMat[num] == 1) :
            B=B+1
        if (gtMat[num] == 1) and (drawMat[num] == 0) :
            C=C+1
    if(A+B != 0):
        precious = A/(A+B)
    else :
        precious = 0
    if A+C != 0 :
        recall = A/(A+C)
    else :
        recall = 0
    PRscore.append([precious,recall])
    

def draw(Curve_one):
 
    plt.figure()
    
    plot1 = plt.plot(Curve_one[1], Curve_one[0],     'r.-', linewidth=2.5, markersize=1.0)
   
    # set X axis
    plt.xlim( [0, 1] )
    plt.xticks( np.linspace(0, 1.0, 11) )
    plt.xlabel("Recall", fontsize="x-large")
    
    # set Y axis
    plt.ylim( [0, 1] )
    plt.yticks( np.linspace(0, 1.0, 11) )
    plt.ylabel("Precious", fontsize="x-large")
    
    # set figure information
    # plt.title("Precision --- Recall", fontsize="x-large")
    #plt.legend([plot1, plot2, plot3, plot4, plot5], ("ours", "SC", "ISC", "ESF", "M2DP"), loc="lower left", #numpoints=1)
    plt.grid(False)
 
    # draw the chart
    plt.show()

def save_curve(readDir,drawMat,gtMat,PRscore) :
    savescore(readDir)
    generateGT()
    for i in range(20, 1000, 10):
        calculatetruefalse(i/1000.0,drawMat,gtMat)
        drawMat=[]
    precious = [1.0]
    recall = [0.0]
    F1score = 0
    for j in range(0,len(PRscore)) :
        if(PRscore[j][0] != 0.0 and PRscore[j][1] != 0.0) :
            precious.append(PRscore[j][0])
            recall.append(PRscore[j][1])
            F1scoreTemp = 2 * PRscore[j][0] * PRscore[j][1] / ( PRscore[j][0] + PRscore[j][1] )
            if F1scoreTemp >= F1score :
                F1score = F1scoreTemp
           
    CurveX=[]
    CurveX.append(precious)
    CurveX.append(recall)
    print(F1score)
    F1score = 0
    precious=[]
    recall=[]
    return CurveX

def resetmat() :
    global drawMat 
    global PRscore
    global gtMat
    global dataMat
    global scoreMat
    drawMat=[]
    PRscore=[]
    gtMat=[]
    dataMat=[]
    scoreMat=[]

if __name__ == '__main__':
    
    Curve_one = save_curve(readDir1,drawMat,gtMat,PRscore)
    resetmat()
    
   

    # drawtest(precious,recall)
    draw (Curve_one)
