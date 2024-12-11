#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import yaml
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt


class PoleDetector:
    def __init__(self):
        # rospy.init_node('pole_detector', anonymous=True)

        # 创建机器人控制的发布者
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 订阅激光雷达数据
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        #订阅amcl数据
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        # 初始化激光雷达,amcl数据
        self.laser_data = None
        self.robot_pose = None

        # 机器人控制信息
        self.twist = Twist()

        # 距离阈值，用于检测杆的位置（单位：米）
        self.close_distance = 0.25    # 靠近杆的目标距离（单位：米）

        #机器人状态（是否到达杆）
        self.state = False

        # 运行主循环
        self.rate = rospy.Rate(10)
        # self.run()

    def laser_callback(self, msg):
        """激光雷达数据回调函数"""
        self.laser_data = np.array(msg.ranges)

    def amcl_callback(self,msg):
        # 获取机器人的当前位置（x, y坐标）
        self.robot_pose = msg.pose.pose

    def laser_to_world(self):
        """将激光雷达数据转换到世界坐标系"""
        if self.laser_data is None or self.robot_pose is None:
            return []

        laser_points = []
        laser_origin = []

        for i, distance in enumerate(self.laser_data):
            if distance < 0.05 or distance > 10.0:  # 排除无效数据
                continue

            # 激光雷达角度（与机器人的朝向）
            angle = i * (2 * np.pi / len(self.laser_data))
            if angle > np.pi:
                angle = angle - 2*np.pi

            # 转换为世界坐标系
            robot_angle = 2*np.arctan2(self.robot_pose.orientation.z, self.robot_pose.orientation.w)
            x = self.robot_pose.position.x + distance * np.cos(robot_angle + angle)
            y = self.robot_pose.position.y + distance * np.sin(robot_angle + angle)

            laser_points.append((x, y))
            laser_origin.append((distance,angle))
        print(f'length of laser points:{len(laser_points)}')
            # print(f'laser_points:{laser_points}')

        return laser_points, laser_origin

    def filter_wall_points(self, laser_points, laser_origin, wall_points, threshold=0.3):
        """筛选掉与地图边界重合的激光雷达数据点"""
        filtered_points = []
        filtered_distance = []
        filtered_angle = []
        
        for i,laser_point in enumerate(laser_points):
            x, y = laser_point
            distance,angle = laser_origin[i]
            is_wall = False
            
            # 遍历地图边界点
            
            for wall_point in wall_points:
                wx, wy = wall_point
                distance_to_wall = np.sqrt((x - wx) ** 2 + (y - wy) ** 2)
                # print(distance_to_wall)
                
                if distance_to_wall < threshold:
                    is_wall = True
                    break


            if not is_wall:
                filtered_points.append(laser_point)
                filtered_distance.append(distance)
                filtered_angle.append(angle)
        
        # print(f'filtered_distance:{filtered_distance}, filtered_angle:{filtered_angle}')
        print(f'length of filter points:{len(filtered_distance)}')
        
        return filtered_points,filtered_distance,filtered_angle

    # def detect_pole(self):
    #     """
    #     检测激光雷达数据中的杆位置。
    #     返回值：
    #     - angle (float): 杆的角度
    #     - distance (float): 杆的距离
    #     """
    #     if self.laser_data is None:
    #         return None, None

    #     # 排除无效数据（某些激光雷达会返回 inf 或 0）
    #     valid_indices = np.where((self.laser_data > 0.05) & (self.laser_data < 10.0))[0]
    #     valid_ranges = self.laser_data[valid_indices]

    #     # 检测距离突变点（通过梯度计算）
    #     gradients = np.abs(np.diff(valid_ranges))  # 计算相邻点之间的差值

    #     # 找到突变最大的点
    #     max_gradient_index = np.argmax(gradients)

    #     if gradients[max_gradient_index] <= self.distance_threshold:
    #         rospy.loginfo("No significant pole detected.")
    #         return None, None

    #     # 使用突变最大的点作为杆的位置
    #     pole_index = max_gradient_index

    #     # pole_indices = np.where(gradients > self.distance_threshold)[0]  # 找到突变点

    #     # if len(pole_indices) == 0:
    #     #     rospy.loginfo("No pole detected.")
    #     #     return None, None

    #     # # 找到第一根杆（或者选择最近的杆）
    #     # pole_index = pole_indices[0]

    #     # 角度和距离
    #     angle = valid_indices[pole_index] * (np.pi / len(self.laser_data)) - (np.pi / 2)  # 转换为弧度（相对于中心）
    #     distance = valid_ranges[pole_index]

    #     return angle, distance

    def approach_pole(self, angle, distance):
        """
        控制机器人靠近杆。
        参数：
        - angle (float): 杆的角度
        - distance (float): 杆的距离
        """
        rospy.loginfo(f"Approaching pole: angle={angle:.2f} rad, distance={distance:.2f} m")

        # 如果距离小于目标距离，停止
        if distance < self.close_distance:
            rospy.loginfo("Reached the pole.")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            return True

        # 根据角度调整方向
        if angle > 0.1:  # 向左转
            self.twist.angular.z = 0.5
        elif angle < -0.1:  # 向右转
            self.twist.angular.z = -0.5
        else:
            self.twist.angular.z = 0.0

        # 根据距离调整速度
        self.twist.linear.x = 0.1 if distance > self.close_distance else 0.0

        # 发布控制命令
        self.cmd_pub.publish(self.twist)
        return False

    def run(self):
        """主循环"""
        map_yaml_path = '/home/eliana/map.yaml'  # 替换为你的yaml文件路径
        map_image, resolution, origin, map_width, map_height = load_map(map_yaml_path)
        wall_points = extract_wall_points(map_image, resolution, origin)
        img = np.zeros((map_height,map_width,3),dtype=np.uint8)
        # draw(img,wall_points,map_width,map_height,resolution,(255,0,0))
        # 打印提取到的墙壁坐标
        rospy.loginfo(f"Extracted {len(wall_points)} wall points.")

        # for point in wall_points:
        #     rospy.loginfo(f"Wall point: {point}")

        while not rospy.is_shutdown():
            # 检测杆的位置
            laser_points, laser_origin = self.laser_to_world()
            # draw(img,laser_points,map_width,map_height,resolution,(0,255,0))
            filtered_points,filtered_distance,filtered_angle = self.filter_wall_points(laser_points, laser_origin, wall_points)
            
            if len(filtered_angle)>0 and len(filtered_distance)>0 :
                angle, distance = np.min(filtered_angle), np.min(filtered_distance)
                # 靠近杆
                reached = self.approach_pole(angle, distance)
                if reached:
                    rospy.loginfo("Pole approach complete. Waiting for next command...")
                    rospy.sleep(2)  # 停留 2 秒
                    self.state = True
                    # print(self.state)
                    return 
            else:
                # 如果没有检测到杆，原地旋转搜索
                rospy.loginfo("Searching for pole...")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3  # 原地旋转
                self.cmd_pub.publish(self.twist)

            # self.rate.sleep()

def load_map(map_yaml_path):
    with open(map_yaml_path, 'r') as f:
        map_metadata = yaml.load(f, Loader=yaml.FullLoader)
    
    map_image_path = map_metadata['image']
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']

    # 加载地图图像
    map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
    # cv2.imshow('show',map_image)
    # cv2.waitKey(0)
    # plt.figure(figsize=(10, 10))
    # plt.imshow(map_image, cmap='gray')

    # # 添加每个像素的值
    # rows, cols = map_image.shape
    # for i in range(0, rows, 10):  # 每隔 10 行显示一个像素值，避免过密
    #     for j in range(0, cols, 10):  # 每隔 10 列显示一个像素值
    #         plt.text(j, i, str(map_image[i, j]), color="red", fontsize=8)

    # plt.title("Pixel Values")
    # plt.axis("off")
    # plt.show()
    # filtered_image = np.where((map_image == 206) | (map_image == 254), 0, map_image)  # 替换为 0 表示忽略

    # # 打印过滤后的像素矩阵
    # print("Filtered Pixel values (excluding 206 and 254):")
    # print(filtered_image)

    # 获取地图的尺寸
    map_width = map_image.shape[1]
    map_height = map_image.shape[0]

    return map_image, resolution, origin, map_width, map_height
    
def grid_to_world(x_pixel, y_pixel, origin, resolution):
    """将栅格坐标转换为世界坐标"""
    origin_x, origin_y, _ = origin  # 提取原点坐标
    # print(origin)
    world_x = origin_x + x_pixel * resolution
    world_y = -origin_y - y_pixel * resolution-1
    return world_x, world_y

def extract_wall_points(map_image, resolution, origin):
    wall_points = []
    #print(map_image)
    # for pix in map_image[map_image<=100]:
    #     print(pix)

    # 遍历地图图像的每个像素
    for y in range(map_image.shape[0]):  # 遍历行
        for x in range(map_image.shape[1]):  # 遍历列
            if map_image[y, x] <= 100:  # 假设0表示墙壁(黑色)
                world_x, world_y = grid_to_world(x, y, origin, resolution)
                wall_points.append((world_x, world_y))
    

    # print(wall_points)

    return wall_points

def coordinate(x,y,map_width,map_height,resolution):
    print((x+10)/19.2,(y+10)/19.2)
    x_img = np.uint8((x+10)/19.2*map_width)
    y_img = np.uint8((y+10)/19.2*map_height)
    # print(x_img,y_img)
    return x_img, y_img


def draw(img,wallpoints,map_width,map_height,resolution,color):
    
    for point in wallpoints:
        x,y=point
        x_img,y_img=coordinate(x,y,map_width,map_height,resolution)
        cv2.circle(img,(x_img,map_height-y_img),1,color,-1)
    
    cv2.imshow('wall_points',img)
    cv2.waitKey(0)


if __name__ == '__main__':
    try:
        detector = PoleDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass