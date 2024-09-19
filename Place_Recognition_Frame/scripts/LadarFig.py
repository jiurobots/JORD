# -*- coding: utf-8 -*-

print("*****纯小白的学习python，高手绕行*****")
print("*****欢迎收藏点赞*****")
print("\n")
print("本文内容是雷达 极轴图和常见螺线图的简单示例")


import numpy as np
import matplotlib.pyplot as plt

#先准备数据
results = [{"力量": 7, "防御": 10, "速度": 7, "智力": 2, "暴击": 5,"格挡": 8},
           {"力量": 2, "防御": 5, "速度": 10, "智力": 8, "暴击": 9,"格挡": 4}]

propname = ["力量","防御","速度","智力","暴击","格挡"]
length = len(propname)
propvalue1 = [7,10,7,2,5,8]
propvalue2 = [2,5,10,8,9,4]

# 将极坐标的角度值根据数据属性数量进行等分
angles = np.linspace(0, 2*np.pi, length, endpoint=False)
#linspace和arange函数很像，都是生成结构和numpy数组类似的均匀分布数值序列
#linspace(start,stop,num,endpoint dtype)
#start 参数数值范围的起始点。如果设置为0，则结果的第一个数为0.该参数必须提供。
#stop 参数数值范围的终止点。通常其为结果的最后一个值，但如果修改endpoint = False, 则结果中不包括该值。
#num (可选) 参数控制结果中共有多少个元素。该参数缺省为50。
#endpoint (可选) 参数决定终止值(stop参数指定)是否被包含在结果数组中。如果 endpoint = True, 结果中包括终止值，反之不包括。缺省为True。
#dtype (可选) 和其他的 NumPy 一样, np.linspace中的dtype 参数决定输出数组的数据类型。如果不指定，python基于其他参数值推断数据类型。如果需要可以显示指定，参数值为NumPy 和 Python支持的任意数据类型。

#print(angles) #最终得到六个角度值，用来在极坐标下绘制折线图用，可以打印一下看看
#[0.         1.04719755 2.0943951  3.14159265 4.1887902  5.23598776]

#这里感觉有必要复习一下极坐标的知识。
#平面直接坐标体系下，决定一个点的位置，有(横坐标x，纵坐标y)就够了
#极坐标体系下，决定一个点的位置，用的是(弧度，距离圆心的长度)，诸如平面直接的（0,1) 换算成极坐标，就是(pi/2,1)
#雷达图直角坐标倒也不是不能画，只是太不方便，所以用极坐标

#将第一个元素添加到最后的位置，使雷达图封闭。不然后面折线图画到最后一个点就不画了，会缺个口子
angles_a = np.concatenate((angles, [angles[0]]))
#np.concatenate是numpy库中用于数组拼接的函数。
#它可以按照指定的轴将多个数组拼接在一起，生成一个新的数组。
#在numpy中，数组可以有不同的维度，因此np.concatenate实质上是按照维度来将数组拼接。
#需要注意的是，在拼接数组之前，这些数组的维度需要一致。
#np.concatenate((a1, a2, ...), axis=0, out=None)
#a1、a2、...表示要拼接的数组序列，可以传入一个含有数组的元祖或列表。
#axis表示要拼接的轴的方向，如axis=0表示沿着第一维进行拼接（行方向），axis=1表示沿着第二维进行拼接（列方向），默认值为0。
#out表示输出结果的数组，可以不在函数中指定，而是在外部定义输出数组对象，以节省计算空间。

propvalue1_a = propvalue1 + [propvalue1[0]]
#也可以用concatenate()来处理，但因为这个只是普通列表，所以可以直接+

propvalue2_a = propvalue2 + [propvalue2[0]]

# 设置图形的大小
fig = plt.figure(figsize=(10, 12.5), dpi=100)

##利用网格来创建子图，这样可以单独调整子图大小
import matplotlib.gridspec as gridspec
grid = gridspec.GridSpec(4, 3)  # 设定4行*3列的网格

# 新建一个子图
ax = fig.add_subplot(grid[0,0:2],polar=True) #第1行前2列都给第1个子图
#这个子图是本文其余子图的2倍大，不然图例显示别扭

#参数111是简写，等价于1,1,1，polar=True表示坐标系是极坐标系

#显示中文和负号
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 绘制折线图，在极坐标下仍然使用plot()，只需参数是极坐标参数就可以
ax.plot(angles_a, propvalue1_a, color='c')
ax.plot(angles_a, propvalue2_a, color='b')

# 填充区域
ax.fill(angles_a, propvalue1_a, color='r',alpha=0.2)
ax.fill(angles_a, propvalue2_a, color='y',alpha=0.2)

#把具体的值显示在图上，text在极坐标上一样用，只是前两个参数是极坐标
for item in range(len(propname)):
    ax.text(angles_a[item],propvalue1_a[item],s=propvalue1_a[item])
for item in range(len(propname)):
    ax.text(angles_a[item],propvalue2_a[item],s=propvalue2_a[item])


#设置网格标签，第一个参数的单位需由弧度转化成度数
plt.thetagrids(angles*180/np.pi, propname,color = "r",fontsize = 10)
#thetagrids(angles, labels=None,)
#angles:将角度设置为网格的位置(这些网格线沿维度相等)
#labels:如果不是None，那么就是len(angles)或者每个角度下要使用的标签字符串列表。
#注意：极坐标下的各种函数中输入的坐标,几乎全是弧度,但thetagrids是少有的需要输入角度的函数

#设置网格样式
ax.tick_params(pad=10, grid_color='k', grid_alpha=0.2,grid_linewidth=1, \
               grid_linestyle=(0, (5, 5)),\
               labelsize = 10,labelcolor = "r"
               )
#pad 刻度线和刻度值之间的距离
#labelsize 刻度值的字体大小。注意这个地方的labelsize会把上面thetagrids的fontsize覆盖
#labelcolor 刻度值的文字颜色
#grid_alpha 网格线透明度
#grid_color 网格线颜色
#grid_linewidth 网格线宽度
#grid_linestyle 网格线型
#还有好多参数，看着眼晕，不写了

# 设置0度起始位置
ax.set_theta_zero_location('N')

# 设置坐标值显示角度相对于起始角度的偏移量
ax.set_rlabel_position(270)

# 设置显示的极径范围
ax.set_rlim(0, 10)

#设置显示的极径最大值
ax.set_rmax(10)

# 设置极径网格线的显示范围
ax.set_rticks(np.arange(5,10,1))

#设置坐标轴正方向，默认逆时针
ax.set_theta_direction(-1)#本例其实无所谓

#显示图例
plt.legend(["战士", "弓手"], loc='center',bbox_to_anchor=(1.5,0.5),fontsize = "8")
#没有给折线图添加label值，所以这个地方要手动设置

#设置标题
ax.set_title("属性",y = -0.5)
#y=-0.5是把标题的位置往下挪一下，不然会重叠

#以上就是雷达图了
###########################
#下面是极轴图的示例，也即极坐标下的柱状图

axa =  plt.subplot(433, polar=True)
axa.set_title("极轴图",y = -0.5)
axa.set_rlim(0,10)
bar1 = axa.bar(angles_a, propvalue1_a,alpha=0.5)

#换颜色填充
for r,bar1 in zip(propvalue1_a,bar1):
    bar1.set_facecolor(plt.cm.jet(r/10.))

###########################
#既然用到了极坐标，就来画点螺线
###########################

#等速螺线 极坐标方程式为r = a+b*theta
ax2 = plt.subplot(434, polar=True)

#定义参数
a,b=0,1
#设置角度范围和精度
theta = np.linspace(0,5*np.pi,1000)
#计算等速螺线坐标
r = a+b*theta

ax2.plot(theta, r, color='g')
ax2.set_title("等速螺线",y = -0.5)
###########################
#双曲螺线
# 极坐标方程式为r = a/theta
ax3 = plt.subplot(435, polar=True)

#定义参数
a=1
#设置角度范围和精度，注意到过来取值，并且不要0，不然计算r会出除0的警告
theta = np.linspace(100*np.pi,0,1000,endpoint = False)
#计算坐标
r = a/theta

ax3.plot(theta, r, color='b')
ax3.set_title("双曲螺线",y = -0.5)
###########################
#费马螺线
# 极坐标方程式为r = a*theta**(1/2)
ax4 = plt.subplot(436, polar=True)

#定义参数
a=1
#设置角度范围和精度
theta = np.linspace(0,2*np.pi,1000)
#计算坐标
r = a*theta**(1/2)

ax4.plot(theta, r, color='y')
ax4.set_title("费马螺线",y = -0.5)


###########################
#等角螺线
# 极坐标方程式为r = a*b**(c*theta)
ax5 = plt.subplot(437, polar=True)

#定义参数
a,b,c=1,np.e,1/5
#设置角度范围和精度
theta = np.linspace(0,5*np.pi,1000)
#计算坐标
r = a*b**(c*theta)

ax5.plot(theta, r, color='c')
ax5.set_title("等角螺线",y = -0.5)

###########################
#玫瑰线
# 极坐标方程式为r = a*sin(b*theta) r = a*cos(b*theta)
ax6 = plt.subplot(438, polar=True)

#定义参数
a,b=1,5.5
#设置角度范围和精度
theta = np.linspace(0,5*np.pi,1000)
#计算坐标
r = a*np.sin(b*theta)

ax6.plot(theta, r, color='g')
ax6.set_title("玫瑰线",y = -0.5)

###########################
#心脏线
# 极坐标方程式为r = a*(1-sin(theta)) r = a*(1+cos(theta))
ax7 = plt.subplot(439, polar=True)

#定义参数
a=1
#设置角度范围和精度
theta = np.linspace(0,5*np.pi,1000)
#计算坐标
r = a*(1-np.sin(theta))

ax7.plot(theta, r, color='r')
ax7.set_title("心脏线",y = -0.5)

###########################
#双纽线
# 极坐标方程式为 r** = 2*(a**2)*(cos(2*theta)  r= +/-(2*(a**2)*(cos(2*theta))**(1/2))
ax8 = plt.subplot(4,3,10, polar=True)

#定义参数
a=1
#设置角度范围和精度，也要倒过来取，不包括0
theta = np.linspace(5*np.pi,0,1000,endpoint = False)
#计算坐标
r = (2*(a**2)*(np.cos(2*theta))**(1/2))

ax8.plot(theta, r, color='b')
ax8.set_title("双纽线",y = -0.5)

###########################
#连锁螺线
# 极坐标方程式为 r = a*(theta**1/2)
ax9 = plt.subplot(4,3,11, polar=True)

#定义参数
a=1
#设置角度范围和精度
theta = np.linspace(0,5*np.pi,1000)
#计算坐标
r = a*(theta**1/2)

ax9.plot(theta, r, color='y')
ax9.set_title("连锁螺线",y = -0.5)

###########################
#线和圆
ax10 = plt.subplot(4,3,12, polar=True)

#圆，必须这么算一下r，不然参数维度对不上
theta = np.linspace(0,5*np.pi,1000)
r = theta*0 + 0.5
ax10.plot(theta,r, color='r')

#画线
r1 = np.linspace(0,1,1000)
theta1 = r1*0 + np.pi/4
ax10.plot(theta1,r1,color='r')

ax10.set_title("线和圆",y = -0.5)

#plt.axis('off')     #可以关闭坐标尺寸

###########################
#设置间距
plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=0.3, hspace=0.9)
plt.savefig("jizuobiao")