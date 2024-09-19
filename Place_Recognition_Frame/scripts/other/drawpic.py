import matplotlib.pyplot as plt
import matplotlib.image  as mpimg
import numpy as np
import pylab
import sys


# 读取一个文件，将其内容转换为一个二维的NumPy数组，其中每行的数据用delimiter指定的分隔符进行分割，并将每个元素转换为float类型。
def file2array(path, delimiter=' '):     # delimiter是数据分隔符（默认为空格）
    # 打开文件path，以只读模式打开，使用UTF-8编码
    fp = open(path, 'r', encoding='utf-8')
    # 读取整个文件内容到string字符串，string是一行字符串，该字符串包含文件所有内容
    string = fp.read()
    fp.close()  # 关闭文件
    # 将string字符串按行分割，得到行的列表，splitlines默认参数是'\n'（换行符）
    row_list = string.splitlines()
    # 对每一行进行循环，strip()去除每行首尾空白字符，然后按照delimiter分隔字符串并转换为float类型，组成列表的列表data_list
    data_list = [[float(i) for i in row.strip().split(delimiter)] for row in row_list]
    return np.array(data_list)

# data = file2array('./scds.scd')
# print(data)
# print("data's shape", data.shape)


# 绘制一个热图，其中数据使用彩虹色彩映射进行表示。
def plotData(data):
    # 使用imshow绘制热图，将data以彩虹色彩映射进行显示
    # cmap=plt.cm.rainbow：使用彩虹色彩映射
    # interpolation='nearest'：使用最近邻插值进行图像的绘制
    # vmin=0, vmax=6：设置颜色映射的最小值和最大值
    # 可以根据需要替换彩虹色彩映射为其他预定义的颜色映射，如viridis、plasma等
    plt.imshow(data, cmap=plt.cm.rainbow, interpolation='nearest', vmin=0, vmax=6)   #  vmax  5 - 25  #jet   #Greys

    plt.colorbar(fraction=0.015)  # 添加颜色条（颜色映射的标尺），fraction参数控制颜色条的长度
    # plt.colorbar(mappable=None, cax=None, ax=None,)
    #fig,ax = plt.subplots(1,1,figsize=(15,10))
    
    #fig = plt.figure(figsize=(15,10),dpi=30)
   # ax=fig.add_subplot(111) 
    #ax.scatter()
    ax = plt.gca()  # 获取当前的坐标轴对象
    ax.invert_yaxis()  # 反转坐标轴，使y轴的方向从上到下
    plt.xlim((-1, 61))  # 设置x轴坐标范围为-1到60
    plt.ylim((-1, 21))  # 设置y轴坐标范围为-1到20
    #plt.yticks([])   # 去除纵坐标刻度
    #plt.xticks([])   #去除横坐标刻度
    plt.xlabel("Sector")  # 设置x轴标签为"Sector"
    plt.ylabel("Ring")  # 设置y轴标签为"Ring"
    plt.show()  # 显示绘制的图像

# plotData(data)

if __name__ == "__main__":
    data = file2array(sys.argv[1])
    plotData(data)