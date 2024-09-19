import sys
import os
import numpy as np
import matplotlib.pyplot as plt


# 构造不同的位置描述符字典
lcd_names = ['ESF', 'M2DP', 'SC', 'ISC', 'Iris', 'CSSC', 'NDD', 'NDTMC']
LCD_KITTI = { lcd_names[0]: [(0.508, 0.511), (0.221, 0.25), (0.434, 0.530), (0.195, 0.505)],
        lcd_names[1]: [(0.888, 0.859), (0.652, 0.511), (0.784, 0.814), (0.051, 0.033)],
        lcd_names[2]: [(0.931, 0.850), (0.823, 0.589), (0.878, 0.882), (0.591, 0.634)],
        lcd_names[3]: [(0.927, 0.879), (0.758, 0.595), (0.906, 0.910), (0.404, 0.547)],
        lcd_names[4]: [(0.945, 0.929), (0.867, 0.879), (0.947, 0.949), (0.627, 0.595)],
        lcd_names[5]: [(0.906, 0.884), (0.805, 0.623), (0.842, 0.862), (0.5, 0.628)],
        lcd_names[6]: [(0.968, 0.932), (0.858, 0.654), (0.915, 0.914), (0.840, 0.595)],
        lcd_names[7]: [(0.953, 0.939), (0.883, 0.856), (0.951, 0.945), (0.812, 0.544)]
        }
# labels_KITTI = ['KITTI 00', 'KITTI 02', 'KITTI 05', 'KITTI 08']

LCD_JORD = { lcd_names[0]: [(0.472, 0.501), (0.523, 0.333), (0.206, 0.045), (0.548, 0.529), (0.027, 0.001)],
        lcd_names[1]: [(0.383, 0.525), (0.338, 0.556), (0.105, 0.507), (0.513, 0.671), (0.117, 0.527)],
        lcd_names[2]: [(0.733, 0.579), (0.698, 0.542), (0.561, 0.622), (0.739, 0.755), (0.350, 0.6)],
        lcd_names[3]: [(0.817, 0.683), (0.844, 0.545), (0.809, 0.821), (0.818, 0.809), (0.508, 0.632)],
        lcd_names[4]: [(0.804, 0.781), (0.854, 0.542), (0.774, 0.809), (0.827, 0.739), (0.584, 0.698)],
        lcd_names[5]: [(0.784, 0.554), (0.761, 0.25), (0.687, 0.677), (0.748, 0.768), (0.482, 0.564)],
        lcd_names[6]: [(0.788, 0.75), (0.829, 0.527), (0.756, 0.784), (0.801, 0.811), (0.472, 0.655)],
        lcd_names[7]: [(0.798, 0.811), (0.867, 0.570), (0.756, 0.660), (0.761, 0.702), (0.502, 0.577)],
        }

colors = [(42, 47, 128), (153, 51, 250), (67, 120, 188), (111, 204, 222), (153, 203, 111), (246, 235, 20), (244, 164, 96), (246, 127, 33), (238, 32, 36), (125, 20, 21)]  # 颜色列表


def test1():
    #显示中文和负号
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    # 定义要在雷达图上绘制的属性标签
    results = [{"大学英语": 87, "高等数学": 79, "体育": 95, "计算机基础": 92, "程序设计": 85},
            {"大学英语": 80, "高等数学": 90, "体育": 91, "计算机基础": 85, "程序设计": 88}]
    data_length = len(results[0])
    # 将极坐标根据数据长度进行等分
    angles = np.linspace(0, 2*np.pi, data_length, endpoint=False)
    labels = [key for key in results[0].keys()]
    score = [[v for v in result.values()] for result in results]
    # 使雷达图数据封闭
    score_a = np.concatenate((score[0], [score[0][0]]))
    score_b = np.concatenate((score[1], [score[1][0]]))
    angles = np.concatenate((angles, [angles[0]]))
    labels = np.concatenate((labels, [labels[0]]))
    # 设置图形的大小
    fig = plt.figure(figsize=(8, 6), dpi=100)
    # 新建一个子图
    ax = plt.subplot(111, polar=True)
    # 绘制雷达图
    ax.plot(angles, score_a, color='g')
    ax.plot(angles, score_b, color='b')
    # 设置雷达图中每一项的标签显示
    ax.set_thetagrids(angles*180/np.pi, labels)
    # 设置雷达图的0度起始位置
    ax.set_theta_zero_location('N')
    # 设置雷达图的坐标刻度范围
    ax.set_rlim(0, 100)
    # 设置雷达图的坐标值显示角度，相对于起始角度的偏移量
    ax.set_rlabel_position(270)
    ax.set_title("计算机专业大一(上)")
    plt.legend(["弓长张", "口天吴"], loc='best')
    plt.show()


def draw_KITTI_F1():
    # 定义要在雷达图上绘制的属性标签
    labels_KITTI = ['KITTI00', 'KITTI02', 'KITTI05', 'KITTI08']
    # 计算标签数量
    num_vars = len(labels_KITTI)
    # 将圆分割为等份并保存角度以确定每个轴的位置
    angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
    # 为了形成闭环，将起始值添加到末尾
    angles += angles[:1]
    # 创建极坐标图
    fig, ax = plt.subplots(figsize=(5, 5), subplot_kw=dict(polar=True))
    # 辅助函数，用于绘制雷达图中的每个位置描述符
    def add_to_radar(lcd_name, color):
        # values = dft.loc[lcd_name].tolist()
        # values = LCD_KITTI[lcd_name]
        # 从LCD_KITTI中每个值对中提取第一个值来生成values
        values = [pair[0] for pair in LCD_KITTI[lcd_name]]
        values += values[:1]
        # ax.plot(angles, values, color=color, linewidth=1, label=lcd_name)
        # ax.fill(angles, values, color=color, alpha=0.05)  # 填充
        ax.plot(angles, values, color=[c / 255.0 for c in color], linewidth=1.5, label=lcd_name)
        # ax.fill(angles, values, color=[c / 255.0 for c in color], alpha=0.05)  # 填充

    # 将每个位置描述符添加到雷达图中
    # add_to_radar(lcd_names[0], '#1aaf6c')
    # add_to_radar(lcd_names[1], '#429bf4')
    # add_to_radar(lcd_names[2], '#d42cea')
    # add_to_radar(lcd_names[3], '#f9a9c7')
    for i in range(0, len(lcd_names)):
        add_to_radar(lcd_names[i], colors[i])
    
    # 调整极坐标轴的方向和起始位置
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)

    # 绘制雷达图的轴线和标签
    ax.set_thetagrids(np.degrees(angles), labels_KITTI, fontsize=12)

    # 根据角度调整轴标签的对齐方式
    for label, angle in zip(ax.get_xticklabels(), angles):
        if angle in (0, np.pi):
            label.set_horizontalalignment('center')
        elif 0 < angle < np.pi:
            label.set_horizontalalignment('left')
        else:
            label.set_horizontalalignment('right')

    # Ensure radar goes from 0 to 1.
    # 设置雷达图的值范围 [0, 1]
    ax.set_ylim(0, 1.0)
    # You can also set gridlines manually like this:
    # 设置雷达图的网格线
    ax.set_rgrids([0.2, 0.4, 0.6, 0.8, 1.0])

    # 设置y轴标签位置， 将y标签的位置（0-100）设置为位于前两个轴的中间。
    ax.set_rlabel_position(180 / num_vars)

    # 添加自定义样式
    # Change the color of the tick labels.
    ax.tick_params(colors='#222222')
    # Make the y-axis (0-100) labels smaller.
    ax.tick_params(axis='y', labelsize=10)
    # Change the color of the circular gridlines.
    ax.grid(color='#AAAAAA')
    # Change the color of the outermost gridline (the spine).
    ax.spines['polar'].set_color('#222222')
    # Change the background color inside the circle itself.
    ax.set_facecolor('#FAFAFA')

    # 添加标题
    ax.set_title('Max F1 Score', y=1.08)
    # 添加图例
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    # 显示雷达图
    plt.show()

    # 保存雷达图（已注释掉）
    # plt.savefig("star.png")

def draw_KITTI_EP():
    # 定义要在雷达图上绘制的属性标签
    labels_KITTI = ['KITTI00', 'KITTI02', 'KITTI05', 'KITTI08']
    # 计算标签数量
    num_vars = len(labels_KITTI)
    # 将圆分割为等份并保存角度以确定每个轴的位置
    angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
    # 为了形成闭环，将起始值添加到末尾
    angles += angles[:1]
    # 创建极坐标图
    fig, ax = plt.subplots(figsize=(5, 5), subplot_kw=dict(polar=True))
    # 辅助函数，用于绘制雷达图中的每个位置描述符
    def add_to_radar(lcd_name, color):
        # values = dft.loc[lcd_name].tolist()
        # values = LCD_KITTI[lcd_name]
        # 从LCD_KITTI中每个值对中提取第一个值来生成values
        values = [pair[1] for pair in LCD_KITTI[lcd_name]]
        values += values[:1]
        # ax.plot(angles, values, color=color, linewidth=1, label=lcd_name)
        # ax.fill(angles, values, color=color, alpha=0.05)  # 填充
        ax.plot(angles, values, color=[c / 255.0 for c in color], linewidth=1.5, label=lcd_name)
        # ax.fill(angles, values, color=[c / 255.0 for c in color], alpha=0.05)  # 填充

    # 将每个位置描述符添加到雷达图中
    # add_to_radar(lcd_names[0], '#1aaf6c')
    # add_to_radar(lcd_names[1], '#429bf4')
    # add_to_radar(lcd_names[2], '#d42cea')
    # add_to_radar(lcd_names[3], '#f9a9c7')
    for i in range(0, len(lcd_names)):
        add_to_radar(lcd_names[i], colors[i])
    
    # 调整极坐标轴的方向和起始位置
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)

    # 绘制雷达图的轴线和标签
    ax.set_thetagrids(np.degrees(angles), labels_KITTI, fontsize=12)

    # 根据角度调整轴标签的对齐方式
    for label, angle in zip(ax.get_xticklabels(), angles):
        if angle in (0, np.pi):
            label.set_horizontalalignment('center')
        elif 0 < angle < np.pi:
            label.set_horizontalalignment('left')
        else:
            label.set_horizontalalignment('right')

    # Ensure radar goes from 0 to 1.
    # 设置雷达图的值范围 [0, 1]
    ax.set_ylim(0, 1.0)
    # You can also set gridlines manually like this:
    # 设置雷达图的网格线
    ax.set_rgrids([0.2, 0.4, 0.6, 0.8, 1.0])

    # 设置y轴标签位置， 将y标签的位置（0-100）设置为位于前两个轴的中间。
    ax.set_rlabel_position(180 / num_vars)

    # 添加自定义样式
    # Change the color of the tick labels.
    ax.tick_params(colors='#222222')
    # Make the y-axis (0-100) labels smaller.
    ax.tick_params(axis='y', labelsize=10)
    # Change the color of the circular gridlines.
    ax.grid(color='#AAAAAA')
    # Change the color of the outermost gridline (the spine).
    ax.spines['polar'].set_color('#222222')
    # Change the background color inside the circle itself.
    ax.set_facecolor('#FAFAFA')

    # 添加标题
    ax.set_title('EP Score', y=1.08)
    # 添加图例
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    # 显示雷达图
    plt.show()

    # 保存雷达图（已注释掉）
    # plt.savefig("star.png")


def draw_JORD_F1():
    # 定义要在雷达图上绘制的属性标签
    labels_JORD = ['JORD051', 'JORD052', 'JORD061', 'JORD0621', 'JORD0622']
    # 计算标签数量
    num_vars = len(labels_JORD)
    # 将圆分割为等份并保存角度以确定每个轴的位置
    angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
    # 为了形成闭环，将起始值添加到末尾
    angles += angles[:1]
    # 创建极坐标图
    fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))
    # 辅助函数，用于绘制雷达图中的每个位置描述符
    def add_to_radar(lcd_name, color):
        # values = dft.loc[lcd_name].tolist()
        # values = LCD_JORD[lcd_name]
        # 从LCD_JORD中每个值对中提取第一个值来生成values
        values = [pair[0] for pair in LCD_JORD[lcd_name]]
        values += values[:1]
        # ax.plot(angles, values, color=color, linewidth=1, label=lcd_name)
        # ax.fill(angles, values, color=color, alpha=0.05)  # 填充
        ax.plot(angles, values, color=[c / 255.0 for c in color], linewidth=1.5, label=lcd_name)
        # ax.fill(angles, values, color=[c / 255.0 for c in color], alpha=0.05)  # 填充

    # 将每个位置描述符添加到雷达图中
    # add_to_radar(lcd_names[0], '#1aaf6c')
    # add_to_radar(lcd_names[1], '#429bf4')
    # add_to_radar(lcd_names[2], '#d42cea')
    # add_to_radar(lcd_names[3], '#f9a9c7')
    for i in range(0, len(lcd_names)):
        add_to_radar(lcd_names[i], colors[i])
    
    # 调整极坐标轴的方向和起始位置
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)

    # 绘制雷达图的轴线和标签
    ax.set_thetagrids(np.degrees(angles), labels_JORD, fontsize=12)

    # 根据角度调整轴标签的对齐方式
    for label, angle in zip(ax.get_xticklabels(), angles):
        if angle in (0, np.pi):
            label.set_horizontalalignment('center')
        elif 0 < angle < np.pi:
            label.set_horizontalalignment('left')
        else:
            label.set_horizontalalignment('right')

    # Ensure radar goes from 0 to 1.
    # 设置雷达图的值范围 [0, 1]
    ax.set_ylim(0, 1.0)
    # You can also set gridlines manually like this:
    # 设置雷达图的网格线
    ax.set_rgrids([0.2, 0.4, 0.6, 0.8, 1.0])

    # 设置y轴标签位置， 将y标签的位置（0-100）设置为位于前两个轴的中间。
    ax.set_rlabel_position(180 / num_vars)

    # 添加自定义样式
    # Change the color of the tick labels.
    ax.tick_params(colors='#222222')
    # Make the y-axis (0-100) labels smaller.
    ax.tick_params(axis='y', labelsize=10)
    # Change the color of the circular gridlines.
    ax.grid(color='#AAAAAA')
    # Change the color of the outermost gridline (the spine).
    ax.spines['polar'].set_color('#222222')
    # Change the background color inside the circle itself.
    ax.set_facecolor('#FAFAFA')

    # 添加标题
    ax.set_title('Max F1 Score', y=1.08)
    # 添加图例
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    # 显示雷达图
    plt.show()

    # 保存雷达图（已注释掉）
    # plt.savefig("star.png")

def draw_JORD_EP():
    # 定义要在雷达图上绘制的属性标签
    labels_JORD = ['JORD051', 'JORD052', 'JORD061', 'JORD0621', 'JORD0622']
    # 计算标签数量
    num_vars = len(labels_JORD)
    # 将圆分割为等份并保存角度以确定每个轴的位置
    angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
    # 为了形成闭环，将起始值添加到末尾
    angles += angles[:1]
    # 创建极坐标图
    fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))
    # 辅助函数，用于绘制雷达图中的每个位置描述符
    def add_to_radar(lcd_name, color):
        # values = dft.loc[lcd_name].tolist()
        # values = LCD_JORD[lcd_name]
        # 从LCD_JORD中每个值对中提取第一个值来生成values
        values = [pair[1] for pair in LCD_JORD[lcd_name]]
        values += values[:1]
        # ax.plot(angles, values, color=color, linewidth=1, label=lcd_name)
        # ax.fill(angles, values, color=color, alpha=0.05)  # 填充
        ax.plot(angles, values, color=[c / 255.0 for c in color], linewidth=1.5, label=lcd_name)
        # ax.fill(angles, values, color=[c / 255.0 for c in color], alpha=0.05)  # 填充

    # 将每个位置描述符添加到雷达图中
    # add_to_radar(lcd_names[0], '#1aaf6c')
    # add_to_radar(lcd_names[1], '#429bf4')
    # add_to_radar(lcd_names[2], '#d42cea')
    # add_to_radar(lcd_names[3], '#f9a9c7')
    for i in range(0, len(lcd_names)):
        add_to_radar(lcd_names[i], colors[i])
    
    # 调整极坐标轴的方向和起始位置
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)

    # 绘制雷达图的轴线和标签
    ax.set_thetagrids(np.degrees(angles), labels_JORD, fontsize=12)

    # 根据角度调整轴标签的对齐方式
    for label, angle in zip(ax.get_xticklabels(), angles):
        if angle in (0, np.pi):
            label.set_horizontalalignment('center')
        elif 0 < angle < np.pi:
            label.set_horizontalalignment('left')
        else:
            label.set_horizontalalignment('right')

    # Ensure radar goes from 0 to 1.
    # 设置雷达图的值范围 [0, 1]
    ax.set_ylim(0, 1.0)
    # You can also set gridlines manually like this:
    # 设置雷达图的网格线
    ax.set_rgrids([0.2, 0.4, 0.6, 0.8, 1.0])

    # 设置y轴标签位置， 将y标签的位置（0-100）设置为位于前两个轴的中间。
    ax.set_rlabel_position(180 / num_vars)

    # 添加自定义样式
    # Change the color of the tick labels.
    ax.tick_params(colors='#222222')
    # Make the y-axis (0-100) labels smaller.
    ax.tick_params(axis='y', labelsize=10)
    # Change the color of the circular gridlines.
    ax.grid(color='#AAAAAA')
    # Change the color of the outermost gridline (the spine).
    ax.spines['polar'].set_color('#222222')
    # Change the background color inside the circle itself.
    ax.set_facecolor('#FAFAFA')

    # 添加标题
    ax.set_title('EP Score', y=1.08)
    # 添加图例
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    # 显示雷达图
    plt.show()

    # 保存雷达图（已注释掉）
    # plt.savefig("star.png")


def main(args):
    dataset = args[1]
    score = args[2]
    if dataset == "KITTI":
        if score == "F1":
            draw_KITTI_F1()
        else:
            draw_KITTI_EP()
    else:
        if score == "F1":
            draw_JORD_F1()
        else:
            draw_JORD_EP()


if __name__ == '__main__':
    main(sys.argv)












def test_KITTI_for_4parts():
    # 定义要在雷达图上绘制的属性标签
    labels_KITTI = ['KITTI 00', 'KITTI 02', 'KITTI 05', 'KITTI 08']
    # 计算标签数量
    num_vars = len(labels_KITTI)

    # 将圆分割为等份并保存角度以确定每个轴的位置
    # angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()

    # 数据集的数量
    num_datasets = len(labels_KITTI)

    # 角度（根据数据集数量均匀分布）
    angles = np.linspace(0, 2 * np.pi, len(lcd_names), endpoint=False).tolist()

    # 为了使雷达图闭合，需要在列表末尾添加第一个值
    angles += angles[:1]

    # 绘图
    fig, axs = plt.subplots(2, 2, figsize=(10, 10), subplot_kw=dict(polar=True))
    fig.suptitle('Radar Chart for Different Datasets')

    for i in range(num_datasets):
        ax = axs[i // 2, i % 2]
        data = LCD_KITTI[lcd_names[i]]
        values = np.array(data).flatten().tolist()
        values += values[:1]  # 为了使雷达图闭合，需要在列表末尾添加第一个值
        ax.plot(angles, values, 'o-', label=lcd_names[i])
        ax.fill(angles, values, alpha=0.25)
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(lcd_names)
        ax.set_title(labels_KITTI[i])

    # 显示图例
    plt.legend(loc='upper right', bbox_to_anchor=(1.3, 1.3))

    plt.tight_layout()
    plt.show()



def test1():
    print(np.__version__)
    # 定义要在雷达图上绘制的属性标签
    labels_JORD = ['JORD 051', 'KITTI 052', 'KITTI 061', 'KITTI 0621', 'JORD 0622']
    results = [{"大学英语": 87, "高等数学": 79, "体育": 95, "计算机基础": 92, "程序设计": 85},
            {"大学英语": 80, "高等数学": 90, "体育": 91, "计算机基础": 85, "程序设计": 88}]
    data_length = len(results[0])
    # 将极坐标根据数据长度进行等分
    angles = np.linspace(0, 2*np.pi, data_length, endpoint=False)
    labels = [key for key in results[0].keys()]
    score = [[v for v in result.values()] for result in results]
    # 使雷达图数据封闭
    score_a = np.concatenate((score[0], [score[0][0]]))
    score_b = np.concatenate((score[1], [score[1][0]]))
    angles = np.concatenate((angles, [angles[0]]))
    labels = np.concatenate((labels, [labels[0]]))
    # 设置图形的大小
    fig = plt.figure(figsize=(8, 6), dpi=100)
    # 新建一个子图
    ax = plt.subplot(111, polar=True)
    # 绘制雷达图
    ax.plot(angles, score_a, color='g')
    ax.plot(angles, score_b, color='b')
    # 设置雷达图中每一项的标签显示
    ax.set_thetagrids(angles*180/np.pi, labels)
    # 设置雷达图的0度起始位置
    ax.set_theta_zero_location('N')
    # 设置雷达图的坐标刻度范围
    ax.set_rlim(0, 100)
    # 设置雷达图的坐标值显示角度，相对于起始角度的偏移量
    ax.set_rlabel_position(270)
    ax.set_title("计算机专业大一(上)")
    plt.legend(["弓长张", "口天吴"], loc='best')
    plt.show()



def draw_KITTI_merge():

    # 定义要在雷达图上绘制的属性标签
    labels_KITTI = ['KITTI 00', 'KITTI 02', 'KITTI 05', 'KITTI 08']
    # 计算标签数量
    num_vars = len(labels_KITTI)
    # 将圆分割为等份并保存角度以确定每个轴的位置
    angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
    # 为了形成闭环，将起始值添加到末尾
    angles += angles[:1]

    # 创建一个包含两个子图的图形
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5), subplot_kw=dict(polar=True))

    # 辅助函数，用于绘制雷达图中的每个位置描述符
    def add_to_radar(ax, car_model, color, values_idx):
        values = [pair[values_idx] for pair in LCD_KITTI[car_model]]
        values += values[:1]
        ax.plot(angles, values, color=color, linewidth=1, label=car_model)
        ax.fill(angles, values, color=color, alpha=0.25)

    # 将每个位置描述符添加到第一个子图
    add_to_radar(ax1, lcd_names[0], '#1aaf6c', 0)
    add_to_radar(ax1, lcd_names[1], '#429bf4', 0)
    add_to_radar(ax1, lcd_names[2], '#d42cea', 0)
    add_to_radar(ax1, lcd_names[3], '#f9a9c7', 0)
    ax1.set_title('F1 Score')  # 设置第一个子图标题

    # 将每个位置描述符添加到第二个子图
    add_to_radar(ax2, lcd_names[0], '#1aaf6c', 1)
    add_to_radar(ax2, lcd_names[1], '#429bf4', 1)
    add_to_radar(ax2, lcd_names[2], '#d42cea', 1)
    add_to_radar(ax2, lcd_names[3], '#f9a9c7', 1)
    ax2.set_title('EP Score')  # 设置第二个子图标题

    # 调整极坐标轴的方向和起始位置
    ax1.set_theta_offset(np.pi / 2)
    ax1.set_theta_direction(-1)
    ax2.set_theta_offset(np.pi / 2)
    ax2.set_theta_direction(-1)

    # 绘制雷达图的轴线和标签
    ax1.set_thetagrids(np.degrees(angles), labels_KITTI)
    ax2.set_thetagrids(np.degrees(angles), labels_KITTI)

    # 根据角度调整轴标签的对齐方式
    for label, angle in zip(ax1.get_xticklabels(), angles):
        if angle in (0, np.pi):
            label.set_horizontalalignment('center')
        elif 0 < angle < np.pi:
            label.set_horizontalalignment('left')
        else:
            label.set_horizontalalignment('right')

    for label, angle in zip(ax2.get_xticklabels(), angles):
        if angle in (0, np.pi):
            label.set_horizontalalignment('center')
        elif 0 < angle < np.pi:
            label.set_horizontalalignment('left')
        else:
            label.set_horizontalalignment('right')

    # 设置雷达图的值范围 [0, 1]
    ax1.set_ylim(0, 1.0)
    ax2.set_ylim(0, 1.0)

    # 设置雷达图的网格线
    ax1.set_rgrids([0.2, 0.4, 0.6, 0.8, 1.0])
    ax2.set_rgrids([0.2, 0.4, 0.6, 0.8, 1.0])

    # 设置y轴标签位置，将y标签的位置（0-100）设置为位于前两个轴的中间。
    ax1.set_rlabel_position(180 / num_vars)
    ax2.set_rlabel_position(180 / num_vars)

    # 添加自定义样式
    # Change the color of the tick labels.
    ax1.tick_params(colors='#222222')
    ax2.tick_params(colors='#222222')
    # Make the y-axis (0-100) labels smaller.
    ax1.tick_params(axis='y', labelsize=8)
    ax2.tick_params(axis='y', labelsize=8)
    # Change the color of the circular gridlines.
    ax1.grid(color='#AAAAAA')
    ax2.grid(color='#AAAAAA')
    # Change the color of the outermost gridline (the spine).
    ax1.spines['polar'].set_color('#222222')
    ax2.spines['polar'].set_color('#222222')
    # Change the background color inside the circle itself.
    ax1.set_facecolor('#FAFAFA')
    ax2.set_facecolor('#FAFAFA')

    # 添加图例
    ax1.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    ax2.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))

    # 添加主标题
    plt.suptitle('KITTI', fontsize=16, y=1.08)

    # 显示图形
    plt.show()








