#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

# 全局变量用于存储当前位置
current_position = None

def amcl_callback(msg):
    global current_position
    # 获取机器人的当前位置（x, y坐标）
    current_position = msg.pose.pose.position

def distance_to_goal(x, y):
    if current_position:
        # 计算机器人当前位置和目标位置之间的欧几里得距离
        dx = current_position.x - x
        dy = current_position.y - y
        distance = math.sqrt(dx**2 + dy**2)
        print(distance)
        return distance
    return float('inf')  # 如果当前位置不可用，返回一个大的值

def move_to_goal(x, y, threshold=0.1):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("waiting for server")
    client.wait_for_server()

    # 创建目标位置
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # 默认朝向

    client.send_goal(goal)
    
     # 循环检查当前位置
    rate = rospy.Rate(10)  # 设置检查频率为10Hz
    while not rospy.is_shutdown():
        # 检查当前位置与目标的距离
        if distance_to_goal(x, y) <= threshold:
            rospy.loginfo("Already close to goal, stopping.")
            client.cancel_goal()  # 停止移动
            return None, "Already close"
        
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
            return client.get_result(), "Goal reached"

        rate.sleep()

    rospy.loginfo("Action server not available or interrupted.")
    return None, "Action server not available"

# if __name__ == '__main__':
#     try:
#         rospy.init_node('movebase_client_py')

#         # 订阅机器人定位信息
#         rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)

#         # 给定目标位置
#         # result, state = move_to_goal(2.196, 3.454)

#         if result:
#             rospy.loginfo("Goal execution done!")
#         else:
#             rospy.loginfo(f"Goal skipped: {state}")

#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation test finished.")
