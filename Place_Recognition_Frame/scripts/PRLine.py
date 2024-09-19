import matplotlib.pyplot as ply                    
import sys
import os
from mpl_toolkits.axes_grid1 import make_axes_locatable


# 读取ground truth文件，返回一个字典，键为帧索引，值为对应的回环帧索引列表
def read_gt(gt_filename):
	# frame_index loop_id1 loop_id2 ....
	gt = []
	with open(gt_filename, 'r') as f:
		for line in f:
			line = line.strip()
			if line == '':
				continue
			if line.find(' ') != -1:
				objs = line.split(' ')
				gt.append((int(objs[0]), [int(i) for i in objs[1:] if int(i) < int(objs[0]) - 50]))
	return { k:v for k, v in gt if len(v) > 0 }


# 绘制精确率-召回率曲线图
def draw_points(points, names):
	ply.figure()
	colors = ['gold','dodgerblue','yellowgreen','purple','red','darkorange','purple']
	markers = ['s','^','d','p','*','o','x']
	plots = []
	for i in range(len(points)):
		d = sorted(points[i], key=lambda x: x[1])
		#p, = ply.plot([p[1] for p in d[::10]], [p[0] for p in d[::10], colors[i], markevery=20, markersize = 4, linewidth=1)
		p, = ply.plot([p[1] for p in d[::20]], [p[0] for p in d[::20]], colors[i], marker = markers[i], linewidth=1)
		plots.append(p)
	ax = ply.gca()
	ply.xlabel("recall", fontsize = 15)
	ply.ylabel("precision", fontsize =15)
	#set xrange and yrange
	ply.xlim(0, 1.02)  # 设置x轴范围
	ply.ylim(0, 1.02)  # 设置y轴范围
	ply.xticks(fontsize=15)
	ply.yticks(fontsize=15)
	names=['$R_{1}=5$','$R_{1}=10$','$R_{1}=15$','$R_{1}=20$']
	ply.legend(plots, names, loc="best", numpoints=1, fontsize =15)	
	dpi = 300
	ply.savefig("kitti00.png",dpi = dpi)  # 保存图像
	ply.show()
	

# 判断帧索引a和回环帧索引b是否在ground truth中
def is_true(a, b, gt):
	if a in gt and b in gt[a]:
		return True
	return False


# 判断帧索引a是否在ground truth中
def is_positive(a, gt):
	return a in gt


# 根据给定的距离阈值thr，计算对应的精确率和召回率
def calculate_values(result, thr, gt):
	TP = 0
	TN = 0
	FP = 0
	FN = 0

	for p in result:
		if p[2] <= thr and p[1] >= 0:#positive
			if is_true(p[0], p[1], gt):
				TP += 1
			else:
				FP += 1
		else: #negative
			if is_positive(p[0], gt):
				FN += 1
			else:
				TN += 1
			
	if (TP + FP == 0) or (TP + FN == 0) or TP == 0:
		return (0.0, 1.0, False)
	precision = TP / (TP + FP)
	recall = TP / (TP + FN)
	return (precision, recall, True)


# 根据不同的距离阈值，计算对应的精确率和召回率，并返回这些精确率和召回率的列表
def calculate_PR(result, gt):
	f1 = 0.0
	pr_values = []  # 创建一个空列表用于保存计算得到的精确率和召回率
	min_distance = min([p[2] for p in result])  # 在结果列表中找到距离的最小值
	max_distance = max([p[2] for p in result])  # 在结果列表中找到距离的最大值
	step = (max_distance - min_distance) / 1000  # 计算步长，用于划分距离阈值的区间
	# 循环1000次，每次增加一个步长，计算不同距离阈值下的精确率和召回率
	for i in range(1000):
		thr = min_distance + i * step  # 当前的距离阈值
		(p, r, ok) = calculate_values(result, thr, gt)  # 调用calculate_values函数，计算当前阈值下的精确率和召回率
		if ok:
			pr_values.append((p, r))  # 将计算得到的精确率和召回率添加到pr_values列表中
			f1 = max(f1,2*p*r/(p+r))  # 计算当前阈值下的F1-score，用于后续比较
		
	print(f1)  # 打印最大的F1-score，即最优的F1-score
	return pr_values  # 返回保存了各个阈值下的精确率和召回率的列表


# 读取结果文件，将文件中的帧索引和回环帧索引以及距离保存为一个列表，并返回该列表
def read_result(path):
	result = []
	with open(path, 'r') as f:
		for line in f:
			line = line.strip()
			if line == '':
				continue
			if line.count(',') == 2:
				(a, b, d) = line.split(',')
			else:
				(a, b, d) = line.split(' ')
			result.append((int(a), int(b), float(d)))
	return result


# 获取文件的名称（去除路径和后缀）
def getName(file_path):
	(filepath,tempfilename) = os.path.split(file_path)
	(filename,extension) = os.path.splitext(tempfilename)
	return filename


# 主函数，根据传入的命令行参数，读取结果文件并计算精确率-召回率曲线的数据，最后绘制曲线。
def main(args):
	prs = []
	names = []

	for i in range(2,len(args),1):
		result = read_result(args[i])
		pr_values = calculate_PR(result, read_gt(args[1]))
		name = getName(args[i])
		names.append(name)
		prs.append(pr_values)

	draw_points(prs,names)

	# result = read_result(args[1])
	# pr_values = calculate_PR(result, read_gt(args[1]))
	# title = os.path.basename(args[1])
	# draw_points(pr_values, title)

if __name__ == '__main__':
	# if(len(sys.argv) < 2):
	# 	main(["", "C:\\Users\\jlurobot\\Desktop\\result\\sc\\res_08.txt"])
	# else:
	# 	main(sys.argv) 
	main(sys.argv) 
