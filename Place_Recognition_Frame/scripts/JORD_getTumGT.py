# !/usr/bin/python3
# -*- coding: utf-8 -*-
# @Author : jiamian22
# @Time : 2023/7/21 15:13

import argparse
import os
import math
from math import sin, cos, radians
import numpy as np  # NumPy用于数值计算
from scipy.interpolate import interp1d  # SciPy的插值函数interp1d用于线性插值
from scipy.spatial.transform import Rotation  # Rotation类用于旋转操作


def lat_lon_alt_to_xyz(lat_lon_alt_data):
    """经纬度、高程信息转换为XYZ坐标"""
    lat = float(lat_lon_alt_data[0])
    lon = float(lat_lon_alt_data[1])
    alt = float(lat_lon_alt_data[2])
    a = 6378137. #长轴
    f = 1 / 298.257223563 #扁率
    lat = radians(lat)
    lon = radians(lon)
    b = a * (1 - f)
    e = (a ** 2 - b ** 2) ** 0.5 / a
    N = a / (1 - e ** 2 * sin(lat) ** 2) ** 0.5

    # 计算XYZ坐标
    X = (N + alt) * cos(lat) * cos(lon)
    Y = (N + alt) * cos(lat) * sin(lon)
    Z = (N * (1 - e ** 2) + alt) * sin(lat)

    return X,Y,Z

# 计算XYZ信息，即tum中的平移向量
def calculate_xyz(gps_data):
    """根据GPS信息计算XYZ信息，默认第一帧为起始坐标系"""
    lat_lon_alt_data = []
    xyz_data = []

    # 首先提取 纬度、经度、高程信息
    for line in gps_data:
        lat_lon_alt_data.append(line[1:4])

    # 计算起始点的XYZ
    X0, Y0, Z0 = lat_lon_alt_to_xyz(lat_lon_alt_data[0])

    # 计算所有点对应的XYZ信息
    for line in lat_lon_alt_data:
        X, Y, Z = lat_lon_alt_to_xyz(line)
        # xyz_current = [X-X0,Y-Y0,Z-Z0]
        xyz_current = [X,Y,Z]
        xyz_data.append(xyz_current)

    return xyz_data


def euler_to_quaternion(roll_pitch_heading_data):
    """欧拉角转换为四元数"""

    roll_rad = float(roll_pitch_heading_data[0])
    pitch_rad = float(roll_pitch_heading_data[1])
    yaw_rad = float(roll_pitch_heading_data[2])

    # 计算每个旋转分量的旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll_rad), -math.sin(roll_rad)],
                    [0, math.sin(roll_rad), math.cos(roll_rad)]])

    R_y = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)],
                    [0, 1, 0],
                    [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])

    R_z = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0],
                    [math.sin(yaw_rad), math.cos(yaw_rad), 0],
                    [0, 0, 1]])

    # 计算总的旋转矩阵 R_total
    R_total = np.dot(np.dot(R_z, R_y), R_x)

    # 从旋转矩阵计算四元数
    w = math.sqrt(1 + R_total.trace()) / 2
    x = (R_total[2, 1] - R_total[1, 2]) / (4 * w)
    y = (R_total[0, 2] - R_total[2, 0]) / (4 * w)
    z = (R_total[1, 0] - R_total[0, 1]) / (4 * w)

    return x, y, z, w


def gps_to_tum(GPS_path, output_path):
    # 读取GPS文件数据
    gps_data = []
    with open(GPS_path, 'r') as file:
        for line in file:
            line_data = line.strip().split(',')
            if len(line_data) == 10:
                gps_data.append(line_data)
            else:
                print(f"错误的格式: {line.strip()}")

    # 时间戳信息
    timestamp = [line[0] for line in gps_data]

    # 计算XYZ信息，即tum中的平移向量
    # xyz = calculate_xyz(gps_data)

    # 直接获取文件中的XYZ值
    xyz = []
    for line in gps_data:
        xyz.append(line[1:4])

    # 根据GPS信息计算四元数，即tum中的旋转四元数
    pose_data = []
    roll_pitch_heading_data = []
    # 首先提取 横滚、俯仰、航向 信息
    for line in gps_data:
        roll_pitch_heading_data.append(line[4:7])
    # 计算对应的四元数
    for line in roll_pitch_heading_data:
        x, y, z, w = euler_to_quaternion(line)
        pose_data.append([x, y, z, w])

    # 保存为tum格式
    gps_tum_gt = []
    if len(timestamp) != len(xyz) or len(timestamp) != len(pose_data):
        raise ValueError("各参数对应的帧数不同，不能处理。")
        return gps_tum_gt
    if os.path.exists(output_path):
        pass
    else:
        os.makedirs(output_path)
    with open(output_path + "/gps_tum_gt.txt", 'w') as file:
        for i in range(len(timestamp)):
            # 注意这里数据间加上逗号，会导致numpy读文件时出现“ValueError: could not convert string to float”错误
            line = f"{timestamp[i]} {xyz[i][0]} {xyz[i][1]} {xyz[i][2]} {pose_data[i][0]} {pose_data[i][1]} {pose_data[i][2]} {pose_data[i][3]}\n"
            gps_tum_gt.append(np.array([timestamp[i], xyz[i][0], xyz[i][1], xyz[i][2], pose_data[i][0], pose_data[i][1], pose_data[i][2], pose_data[i][3]]))
            file.write(line)
    return gps_tum_gt


# slerp 函数实现了球面线性插值（Slerp）算法。它用于四元数空间中在两个四元数之间插值，并确保在插值过程中保持四元数的单位长度。
def slerp(q0, q1, t):
    # 将四元数归一化，以确保单位长度
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    # 计算两个四元数的内积
    dot_product = np.sum(q0 * q1)  # dot_product = np.dot(q0, q1)
    # 如果内积小于0，说明两个四元数的夹角大于90度，需要反转一个四元数，以确保插值在最短路径上进行
    if dot_product < 0.0:
        q1 = -q1
        dot_product = -dot_product

    # 如果内积接近1，表示两个四元数非常接近，直接进行线性插值，以避免除以零的情况
    if dot_product > 0.995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    # 否则，根据 Slerp 插值公式，计算插值四元数。
    # 定义插值时使用的角度
    theta_0 = np.arccos(dot_product)
    theta = theta_0 * t

    q2 = q1 - q0 * dot_product
    q2 = q2 / np.linalg.norm(q2)

    result = q0 * np.cos(theta) + q2 * np.sin(theta)
    return result


# 对真值轨迹文件中的位置和姿态数据进行插值，以使其与算法生成的轨迹文件在时间上对齐，并将插值后的结果保存到一个新文件中。
def estimate_pc_tum_gt(gps_tum_gt, point_cloud_timestamps_path, output_path):
    # 直接读入np.array(gps_tum_gt)会报数据格式错误
    gps_tum_gt_data = np.loadtxt(output_path + "/gps_tum_gt.txt")
    # print(type(gps_tum_gt_data))

    # 创建一个空列表来存储第二列的long类型数据
    point_cloud_timestamps = []  # 点云时间戳（待插值时间戳）

    # 读取文件，并将第二列的字符串转换为long类型
    with open(point_cloud_timestamps_path, "r") as file:
        for line in file:
            # 将每一行按逗号分割，获取第二列的字符串
            columns = line.strip().split(" ")
            second_column_string = columns[1]
            # 将第二列的字符串转换为long类型，并添加到列表中
            try:
                point_cloud_timestamps.append(float(second_column_string) / 1e9)
            except ValueError:
                print(f"Error: Unable to convert '{second_column_string}' to long type.")
                # 如果转换失败，则在此处处理异常情况
    print(f'文件:{point_cloud_timestamps_path}加载成功')

    gps_tum_gt_timestamps = gps_tum_gt_data[:, 0]  # 时间戳
    gps_tum_gt_positions = gps_tum_gt_data[:, 1:4]  # 位置数据
    gps_tum_gt_quaternions = gps_tum_gt_data[:, 4:]  # 姿态数据

    # 确保目标时间戳在真值文件时间戳范围之内
    # np.clip函数的作用是将数组中的元素限制在一个范围内。
    # 该函数会返回一个新的NumPy数组，其中的元素是第一个参数数组中的元素，但是如果它们小于下界，会被替换成下界的值；如果它们大于上界，会被替换成上界的值。
    target_timestamps = np.clip(point_cloud_timestamps, gps_tum_gt_timestamps[0], gps_tum_gt_timestamps[-1])

    # 对真值位置进行线性插值（interpolate_linear），使其时间戳与算法生成的轨迹文件对齐
    interpolator = interp1d(gps_tum_gt_timestamps, gps_tum_gt_positions, axis=0, kind='linear', bounds_error=False, fill_value="extrapolate")
    interpolated_positions = interpolator(target_timestamps)

    print("线性差值完成")

    # 对真值姿态进行球面线性插值（interpolate_rotation_matrix），将四元数转换为旋转矩阵进行插值
    # 首先四元数数据转换为旋转矩阵，并通过调用interpolate_slerp函数对姿态数据进行球面线性插值。
    # 然后，将插值后的四元数转换回旋转对象，并返回插值后的四元数数组。

    # 在下面代码中，将四元数转换成旋转矩阵再转换回四元数的操作看起来似乎多余，但实际上是为了确保在插值过程中得到正确的姿态插值结果。
    # 这个两步的转换看起来多余，但是在实际中，如果直接使用原始的四元数进行球面线性插值可能会导致不稳定的结果。四元数在球面插值中会遇到奇异点（例如两个四元数之间的夹角非常接近180度时），这可能导致插值结果出现异常。
    # 通过先将四元数转换为旋转矩阵进行插值，然后再将插值后的旋转矩阵转回四元数，可以帮助避免插值过程中的奇异点问题，从而得到更稳定和正确的插值结果。
    # 因此，尽管在代码中进行了两次转换，但这种做法是为了保证插值结果的稳定性和正确性，确保插值后的四元数数据在进行姿态插值时能够获得准确和平滑的过渡效果。

    # 将真值数据中的四元数表示的旋转转换为旋转矩阵
    ground_truth_rotations = Rotation.from_quat(gps_tum_gt_quaternions)
    # 将真值数据中的旋转矩阵转换为四元数表示
    ground_truth_quat_array = ground_truth_rotations.as_quat()

    # 首先找到与目标时间戳最接近的两个时间戳在 reference_timestamps 中的索引
    lower_index = np.searchsorted(gps_tum_gt_timestamps, target_timestamps) - 1
    upper_index = lower_index + 1

    # 获取与目标时间戳最接近的两个时间戳值
    lower_timestamp = gps_tum_gt_timestamps[lower_index]
    upper_timestamp = gps_tum_gt_timestamps[upper_index]

    # 计算插值权重
    t = (target_timestamps - lower_timestamp) / (upper_timestamp - lower_timestamp)

    # 进行球面线性插值
    interpolated_quat_array = np.array([slerp(q0, q1, t_i) for q0, q1, t_i in
                                            zip(ground_truth_quat_array[lower_index], ground_truth_quat_array[upper_index],
                                                t)])

    print("球面线性差值完成")

    # 将插值四元数转换回旋转矩阵
    interpolated_rotations = Rotation.from_quat(interpolated_quat_array)
    interpolated_quaternions = interpolated_rotations.as_quat()

    # 合并插值后的时间戳、位置和姿态数据
    interpolated_data = np.column_stack([target_timestamps, interpolated_positions, interpolated_quaternions])

    print(f'文件:{point_cloud_timestamps_path}已插值完成')
    # print(interpolated_data.shape)

    # 将插值后的结果保存到新文件
    output_file = output_path + "/pc_tum_gt.txt"
    np.savetxt(output_file, interpolated_data, fmt='%.6f')
    print("插值完成并保存到文件:", output_file)

    return interpolated_data


def interpolate_slerp(reference_timestamps, reference_rotations, target_timestamps):
    # Convert reference rotations to arrays for interpolation 将参考旋转转换为用于插值的数组 将四元数转换为数组进行插值
    reference_quat_array = reference_rotations.as_quat()

    # Perform spherical linear interpolation (slerp) on quaternion components 对四元数组执行球面线性插值（slerp）
    interpolated_quat_array = np.array([slerp(q0, q1, (t - reference_timestamps[0]) / (reference_timestamps[-1] - reference_timestamps[0]))
                                        for q0, q1, t in zip(reference_quat_array[:-1], reference_quat_array[1:], target_timestamps)])

    return interpolated_quat_array

def main():
    parser = argparse.ArgumentParser(description="Command-line argument example")
    parser.add_argument("-f1", "--function_1", action="store_true", help="Call function A")
    # parser.add_argument("-b", "--option_b", action="store_true", help="Call function B")
    parser.add_argument("-gd", "--gps_data", type=str, required=True, help="the path of GPS_data")
    parser.add_argument("-pt", "--point_cloud_timestamps", type=str, required=True, help="the path of point_cloud_timestamps")
    parser.add_argument("-o", "--output", type=str, default=None, help="the path of result")
    args = parser.parse_args()
    GPS_path = args.gps_data
    point_cloud_timestamps_path = args.point_cloud_timestamps
    output_path = args.output

    if output_path is None:
        current_folder_path = os.getcwd()  # 当前文件夹路径
        output_path = current_folder_path + "/output"
        print(f'保存结果的路径为:{output_path}')

    if args.function_1:
        try:
            gps_tum_gt = gps_to_tum(GPS_path, output_path)
        except ValueError as ve:
            print("各参数对应的帧数不同，不能处理")
        
        try:
            pc_tum_gt = estimate_pc_tum_gt(gps_tum_gt, point_cloud_timestamps_path, output_path)
        except Exception:
            print("更新真值文件失败！")
        
    #elif args.option_b:
    #    function_b()
    else:
        print("No valid option specified. Use -f1 or -other.")


if __name__ == "__main__":
    main()
