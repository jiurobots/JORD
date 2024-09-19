# !/usr/bin/python3
# -*- coding: utf-8 -*-            
# @Author : jiamian22
# @Time : 2023/8/14 20:03

import sys
import os
import numpy as np
import matplotlib.pyplot as plt


# 从给定的 ground truth 文件中读取数据并解析为一个字典，其中键是帧索引，值是一个列表，表示与该帧有关的 loop_id。
def read_gt(gt_filename):
    # frame_index loop_id1 loop_id2 ....
    gt = []  # 用于存储解析数据的列表
    # 使用 'with open' 语句打开文件，并将文件对象命名为 'f'
    with open(gt_filename, 'r') as f:
        # 逐行遍历文件内容
        for line in f:
            line = line.strip()  # 去除行首尾的空白字符
            # 跳过空行
            if line == '':
                continue

            if line.find(' ') != -1:
                objs = line.split(' ')  # 根据空格分隔数据
                frame_index = int(objs[0])  # 提取帧索引
                loop_ids = [int(i) for i in objs[1:] if int(i) <= frame_index - 50]  # 提取 loop_id，要求值小于帧索引 - 50
                gt.append((frame_index, loop_ids))  # 将解析后的数据作为元组添加到结果列表中
    # 创建一个字典，键是帧索引，值是与该帧有关的 loop_id 列表
    return {k: v for k, v in gt if len(v) > 0}


def read_result(path):
    result = []  # 用于存储读取数据的列表

    # 使用 'with open' 语句打开文件，并将文件对象命名为 'f'
    with open(path, 'r') as f:
        # 逐行遍历文件内容
        for line in f:
            line = line.strip()  # 去除行首尾的空白字符
            # 跳过空行
            if line == '':
                continue

            if line.count(',') == 2:
                (a, b, d) = line.split(',')  # 根据逗号分隔数据
            else:
                (a, b, d) = line.split(' ')  # 根据空格分隔数据

            # if float(d) > 1.0:  # add
            #     d = -float(d)
            result.append((int(a), int(b), float(d)))  # 将解析后的数据作为元组添加到结果列表中
    # 返回包含解析数据的列表
    return result


def getName(file_path):
    (filepath, tempfilename) = os.path.split(file_path)  # 将文件路径分割为目录路径和文件名
    (filename, extension) = os.path.splitext(tempfilename)  # 将文件名分割为名称部分和扩展名部分
    # 新增
    parts = filename.split("_")
    if len(parts) > 0:
        name = parts[0]
    else:
        name = filename
    return name, filename


def is_true(a, b, gt):
    if a in gt and b in gt[a]:
        return True
    return False


def is_positive(a, gt):
    return a in gt


def calculate_values(result, thr, gt):
    TP = 0  # 初始化真正例（True Positives）计数为 0
    TN = 0  # 初始化真负例（True Negatives）计数为 0
    FP = 0  # 初始化假正例（False Positives）计数为 0
    FN = 0  # 初始化假负例（False Negatives）计数为 0

    # 遍历 result 中的每个元素
    for p in result:
        # 如果满足条件，判断为正例
        if p[2] <= thr and p[1] >= 0:
            # 如果判断为正例的结果在 ground truth 中也为正例，增加真正例个数
            if is_true(p[0], p[1], gt):
                TP += 1
            else:
                FP += 1  # 否则，增加假正例计数
        else:  # 否则，判断为负例
            # 如果判断为负例的结果在 ground truth 中也为正例，增加假负例计数
            if is_positive(p[0], gt):
                FN += 1
            else:
                TN += 1  # 否则，增加真负例计数

    # 如果存在以下任一情况，返回 (0.0, 1.0, False)
    if (TP + FP == 0) or (TP + FN == 0) or TP == 0:
        return (0.0, 1.0, False)

    # 计算精度和召回率，并返回 (precision, recall, True)
    precision = TP / (TP + FP)
    recall = TP / (TP + FN)
    return (precision, recall, True)


def calculate_PR(name, gt, result):
    F1 = 0.0  # 初始化 F1 分数为 0.0
    minRecall = 1.0  # 初始化最小召回率为 1.0
    maxRecall = 0.0  # 初始化最大召回率为 0.0，100%准确率下的最大召回率
    precious_minRecall = 0.0  # 最小召回率时的准确率 P_R0
    EP = 0.0  # 初始化 EP（召回率）为 0.0
    pr_values = []  # 存储准确率和召回率值的列表

    # 仅选择 p[1] 不为 -1 的结果进行计算
    filtered_result = [p for p in result if p[1] != -1]
    min_distance = min([p[2] for p in filtered_result])  # 计算 result 中距离的最小值
    max_distance = max([p[2] for p in filtered_result])  # 计算 result 中距离的最大值

    step = 1000  # 步长
    # step = (max_distance - min_distance) / 1000  # 计算步长，将距离范围分成 1000 个区间
    # 循环遍历 1000 个区间
    # for i in range(1000):
    #     thr = min_distance + i * step  # 计算当前阈值
    for thr in np.linspace(min_distance, max_distance, step):
        # 调用 calculate_values 函数计算精度、召回率和 ok 标志
        (P, R, ok) = calculate_values(result, thr, gt)
        # 如果计算成功
        if ok:
            pr_values.append((P, R))  # 将精度和召回率值添加到 pr_values 列表中
            F1 = max(F1, 2 * P * R / (P + R))  # 更新 F1 分数
            if R < minRecall:
                minRecall = R  # 更新最小召回率
                precious_minRecall = P  # 更新最小召回率时的准确率 P_R0
            if R > maxRecall and P == 1:
                maxRecall = R  # 更新100%准确率下的最大召回率
    EP = (precious_minRecall + maxRecall) / 2  # 计算拓展精度

    # 打印 F1 分数和 EP 值
    print("==============================")
    print("Name:", name)
    print('F1score:', F1)
    print('EP:', EP)
    return pr_values  # 返回精度和召回率值的列表

"""
def draw_points(points, names):
    plt.figure()  # 创建一个新的图形窗口
    plots = []  # 用于存储绘制的曲线对象
    # colors = ['purple', 'pink', 'b', 'g', 'orange', 'r', 'yellow']  # 颜色列表
    cmap = plt.get_cmap('rainbow')  # 使用彩虹颜色映射（add）

    # 循环绘制多个精度-召回率曲线
    for i in range(len(points)):
        d = sorted(points[i], key=lambda x: x[1])  # 按召回率排序数据点
        colors = [cmap(j / len(d)) for j in range(len(d))]  # 生成颜色数组（add）
        # p, = plt.plot([p[1] for p in d], [p[0] for p in d], colors[i], ls="-", linewidth=1.8, markersize=1.0)
        plot_objects = plt.plot([p[1] for p in d], [p[0] for p in d], colors[i], ls="-", linewidth=1.8, markersize=1.0)
        p = plot_objects[0]  # 获取列表中的第一个元素
        plots.append(p)  # 添加绘制的曲线对象到列表中

    plt.title("Precision --- Recall", fontsize="x-large")  # 设置标题
    plt.xlabel("Recall", fontsize=14)  # 设置x轴标签
    plt.ylabel("Precision", fontsize=14)  # 设置y轴标签
    plt.xlim(0, 1.02)  # 设置x轴范围
    plt.ylim(0, 1.02)  # 设置y轴范围
    plt.legend(plots, names, loc="lower left", numpoints=1, fontsize=12)  # 添加图例
    plt.show()  # 显示绘制的图形
"""


"""
def draw_points(points, names):
	# 定义颜色列表和标记风格列表
    colors = [
        [0.87, 0.49, 0],
        [0.6, 0.2, 0],
        [0.75, 0, 0.75],
        [0, 0.5, 0],
        [0, 0.75, 0.75],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0]
    ]
    marker_style = ['+', 'o', 'x', 's', 'd', '^', 'v', 'p']
    plots = []

	# 创建绘图窗口
    fig, ax1 = plt.subplots()
    # 遍历每个点集并绘制
    for i, point_set in enumerate(points):
        cols = colors[i]  # 选择颜色
        d = np.array(point_set)  # 将点集转换为NumPy数组
        d = d[d[:, 1].argsort()]  # 根据第2列的值对数组进行排序
        
        # 对点集进行下采样，以便绘制
        dx, dy = downsample(d[:, 1], d[:, 0])
        # 绘制点集
        pm, = ax1.plot(dx, dy, color=cols, linestyle='-', marker=marker_style[i], linewidth=0.5, markersize=2.0)
        plots.append(pm)

	# 设置图形的标题、横坐标和纵坐标标签等属性
    ax1.set_title('JORD05', fontsize=14)
    ax1.set_xlabel('Recall', fontsize=12)
    ax1.set_ylabel('Precision', fontsize=12)
    ax1.set_xlim(0, 1.02)
    ax1.set_ylim(0, 1.02)

	# 在图形中添加图例
    ax1.legend(plots, names, loc='lower left')
    # 关闭网格线
    ax1.grid(False)
    # 设置纵横坐标一致
    ax1.set_aspect('equal')
    # 调整图形布局
    plt.tight_layout()

    # 保存图形为SVG文件
    plt.savefig('./output.svg', format='svg', dpi=300)
"""



def downsample(X, Y):
	min_dis = 0.02
	newX = []
	newY = []
	lastX = -20.0
	for i in range(1, len(X)):
		if X[i] - lastX > min_dis:
			newX.append(X[i])
			newY.append(Y[i])
			lastX = X[i]
	return newX,newY
	


def draw_points(points, names):
	plt.figure()  # 创建一个新的图形窗口
	colors_old = ['gold', 'purple', 'pink', 'b', 'g', 'orange', 'r', 'c', 'm', 'yellow', 'k']  # 颜色列表
	colors = [(42, 47, 128), (153, 51, 250), (67, 120, 188), (111, 204, 222), (153, 203, 111), (246, 235, 20), (244, 164, 96), (246, 127, 33), (238, 32, 36), (125, 20, 21)]  # 颜色列表
	marker_style = ['+', 'o', '*', 'x', 's', 'd', '^', 'v', 'p', '1','2','3','4','8']
	plots = []  # 用于存储绘制的曲线对象
    # 循环绘制多个精度-召回率曲线
	for i in range(len(points)):
		d = sorted(points[i], key=lambda x: x[1])  # 按召回率排序数据点
		p, =plt.plot([p[1] for p in d], [p[0] for p in d], color=[c / 255.0 for c in colors[i]], ls="-" , linewidth=1.8, markersize=1.0)
		plots.append(p)  # 添加绘制的曲线对象到列表中

        # 添加图示
		# dx, dy = downsample([p[1] for p in d], [p[0] for p in d])
		# pm, = plt.plot(dx, dy, colors_old[i], ls='' ,marker=marker_style[i], markersize=6.0)
		# plots.append(pm)

	plt.title("Precision --- Recall", fontsize="x-large")  # 设置标题
	plt.xlabel("Recall", fontsize=14)  # 设置x轴标签
	plt.ylabel("Precision", fontsize=14)  # 设置y轴标签
	#set xrange and yrange
	plt.xlim(0, 1.02)  # 设置x轴范围
	plt.ylim(0, 1.02)  # 设置y轴范围
	plt.legend(plots, names, loc="lower left", numpoints=1, fontsize= 12)  # 添加图例
	plt.savefig('./output.svg', format='svg', dpi=300)  # 保存图形为SVG文件
	plt.show()  # 显示绘制的图形


def main(args):
    prs = []  # 用于存储 precision-recall 曲线数据的列表
    names = []  # 用于存储名称的列表

    # 对于参数列表中从第二个参数开始的每个参数（跳过第一个参数）
    for i in range(2, len(args), 1):
        gt = read_gt(args[1])  # 读取真值数据
        result = read_result(args[i])  # 读取结果数据
        name, filename = getName(args[i])  # 获取名称
        pr_values = calculate_PR(filename, gt, result)  # 计算 precision-recall 数据
        names.append(name)  # 将名称添加到列表中
        prs.append(pr_values)  # 将 precision-recall 数据添加到列表中

    draw_points(prs, names)  # 调用函数来绘制 precision-recall 曲线数据点


if __name__ == '__main__':
    main(sys.argv)

