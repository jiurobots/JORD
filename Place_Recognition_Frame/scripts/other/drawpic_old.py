import matplotlib.pyplot as plt
import matplotlib.image  as mpimg
import numpy as np
import pylab
import sys

def file2array(path, delimiter=' '):     # delimiter是数据分隔符
    fp = open(path, 'r', encoding='utf-8')
    string = fp.read()              # string是一行字符串，该字符串包含文件所有内容
    fp.close()
    row_list = string.splitlines()  # splitlines默认参数是‘\n’
    data_list = [[float(i) for i in row.strip().split(delimiter)] for row in row_list]
    return np.array(data_list)

# data = file2array('./scds.scd')
# print(data)
# print("data's shape", data.shape)

def plotData(data):
    plt.imshow(data, cmap=plt.cm.rainbow, interpolation='nearest', vmin=0, vmax=6)   #  vmax  5 - 25  #jet   #Greys

    plt.colorbar(fraction=0.015)
    # plt.colorbar(mappable=None, cax=None, ax=None,)
    #fig,ax = plt.subplots(1,1,figsize=(15,10))
    
    #fig = plt.figure(figsize=(15,10),dpi=30)
   # ax=fig.add_subplot(111) 
    #ax.scatter()
    ax = plt.gca()
    ax.invert_yaxis()  # 反转坐标轴 
    plt.xlim((-1,61))
    plt.ylim((-1,21))
    #plt.yticks([])   # 去除纵坐标刻度
    #plt.xticks([])   #去除横坐标刻度
    plt.xlabel("Sector") 
    plt.ylabel("Ring")
    plt.show() 

# plotData(data)

if __name__ == "__main__":
    data = file2array(sys.argv[1])
    plotData(data)