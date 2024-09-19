import matplotlib.pyplot as ply                    
import sys
import os

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
				gt.append((int(objs[0]), [int(i) for i in objs[1:] if int(i) <= int(objs[0]) - 50]))
	return { k:v for k, v in gt if len(v) > 0 }

def draw_points(points, names):
	ply.figure()
	# colors = ['purple','pink','b','g','orange','r' ,'yellow']
	colors = ['purple', 'pink', 'b', 'g', 'orange', 'r', 'c', 'm', 'yellow', 'k']  # 颜色列表
	plots = []
	for i in range(len(points)):
		d = sorted(points[i], key=lambda x: x[1])
		p, =ply.plot([p[1] for p in d], [p[0] for p in d], colors[i], ls="-" , linewidth=1.8, markersize=1.0)
		plots.append(p)

	ply.title("Precision --- Recall", fontsize="x-large")
	ply.xlabel("Recall", fontsize=14)
	ply.ylabel("Precision", fontsize=14)
	#set xrange and yrange
	ply.xlim(0, 1.02)
	ply.ylim(0, 1.02)
	ply.legend(plots, names, loc="lower left", numpoints=1, fontsize= 12)	
	ply.show()

def is_true(a, b, gt):
	if a in gt and b in gt[a]:
		return True
	return False

def is_positive(a, gt):
	return a in gt

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

def calculate_PR(result, gt):
	f1 = 0.0
	minRecall = 1.0
	maxRecall = 0.0
	precious_minRecall = 0.0
	EP = 0.0
	pr_values = []
	min_distance = min([p[2] for p in result])
	max_distance = max([p[2] for p in result])
	step = (max_distance - min_distance) / 1000
	for i in range(1000):
		thr = min_distance + i * step
		(p, r, ok) = calculate_values(result, thr, gt)
		if ok:
			pr_values.append((p, r))
			f1 = max(f1,2*p*r/(p+r))
			if(r < minRecall) :
				minRecall = r
				precious_minRecall = p
			if(r > maxRecall and p==1) :
				maxRecall = r

	EP = (precious_minRecall + maxRecall) / 2
	print('F1score:',f1)
	print('EP:',EP)
	return pr_values

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

def getName(file_path):
	(filepath,tempfilename) = os.path.split(file_path)
	(filename,extension) = os.path.splitext(tempfilename)
	return filename

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
