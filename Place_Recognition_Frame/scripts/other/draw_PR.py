import matplotlib.pyplot as plt
import sys
import os

def read_gt(gt_filename):
    # 读取ground truth文件，并将数据存储在字典中
    # 格式为 {frame_index: [loop_id1, loop_id2, ...]}
    gt = {}
    with open(gt_filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line == '':
                continue
            if line.find(' ') != -1:
                objs = line.split(' ')
                gt[int(objs[0])] = [int(i) for i in objs[1:] if int(i) < int(objs[0]) - 50]
    # 过滤掉没有loop_id的frame_index
    return {k: v for k, v in gt.items() if len(v) > 0}

def draw_points(points, names):
    # 绘制Precision-Recall曲线图
    plt.figure()
    colors = ['purple', 'pink', 'b', 'g', 'orange', 'r', 'yellow']
    plots = []
    for i in range(len(points)):
        d = sorted(points[i], key=lambda x: x[1])
        p, = plt.plot([p[1] for p in d], [p[0] for p in d], colors[i], ls="-", linewidth=1.8, markersize=1.0)
        plots.append(p)

    plt.title("Precision --- Recall", fontsize="x-large")
    plt.xlabel("Recall", fontsize=14)
    plt.ylabel("Precision", fontsize=14)
    # 设置x轴和y轴范围
    plt.xlim(0, 1.02)
    plt.ylim(0, 1.02)
    plt.legend(plots, names, loc="lower left", numpoints=1, fontsize=12)
    plt.show()

def is_true(a, b, gt):
    # 判断给定的frame_index a和loop_id b是否在ground truth数据中存在
    if a in gt and b in gt[a]:
        return True
    return False

def is_positive(a, gt):
    # 判断给定的frame_index a是否在ground truth数据中存在
    return a in gt


# 主要是阈值
def calculate_values(result, thr, gt):
    # 根据给定的结果数据、阈值和ground truth数据计算Precision和Recall
    TP = 0
    TN = 0
    FP = 0
    FN = 0
    

    for p in result:
        if p[2] <= thr and p[1] >= 0:  # 正例
            if is_true(p[0], p[1], gt):
                TP += 1
            else:
                FP += 1
        else:  # 负例
            if is_positive(p[0], gt):
                FN += 1
            else:
                TN += 1
    
    print(f"TP:{TP} TN:{TN} FP:{FP} FN:{FN}")
    if (TP + FP == 0) or (TP + FN == 0) or TP == 0:
        return (0.0, 1.0, False)
    precision = TP / (TP + FP)
    recall = TP / (TP + FN)
    return (precision, recall, True)

def calculate_PR(result, gt):
    # 计算Precision-Recall曲线上的一系列点的数值
    f1 = 0.0
    minRecall = 1.0
    maxRecall = 0.0
    precious_minRecall = 0.0
    EP = 0.0
    pr_values = []
    # 不同算法需要修改最大最小值计算方法，目前是sc
    min_distance = min([p[2] for p in result if p[2] > 0])
    max_distance = max([p[2] for p in result if p[2] < 100])
    step = (max_distance - min_distance) / 1000
    print(f"最小值：{min_distance};最大值：{max_distance};间距：{step}")

    for i in range(1000):
        thr = min_distance + i * step
        (p, r, ok) = calculate_values(result, thr, gt)
        print(f"阈值为：{thr},准确率：{p},召回率：{r}")
        if ok:
            pr_values.append((p, r))
            f1 = max(f1, 2 * p * r / (p + r))
            if r < minRecall:
                minRecall = r
                precious_minRecall = p
            if r > maxRecall and p == 1:
                maxRecall = r

    EP = (precious_minRecall + maxRecall) / 2
    print('F1score:', f1)
    print('EP:', EP)
    return pr_values

def read_result(path):
    # 读取结果文件
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
    # 获取文件名
    (filepath, tempfilename) = os.path.split(file_path)
    (filename, extension) = os.path.splitext(tempfilename)
    return filename

def main(args):

    # sequence = "05"
    # gt_path = '/home/lmay/test/data/loop_gt_' + f'{sequence}.txt'  # 回环真值文件路径
    # test_path = '/home/lmay/test/data/result_' + f'{sequence}.txt'  # 算法结果文件路径
    
    # pr曲线和对应的名字
    prs = []
    names = []


    for i in range(2, len(args), 1):
        result = read_result(args[i])
        pr_values = calculate_PR(result, read_gt(args[1]))
        name = getName(args[i])
        names.append(name)
        prs.append(pr_values)

    draw_points(prs, names)

if __name__ == '__main__':
    main(sys.argv)

