#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def test_robot_velocity():
    # 初始化 ROS 节点
    rospy.init_node('test_robot_velocity', anonymous=True)

    # 创建一个发布者，发布到 '/cmd_vel' 话题（通常是机器人控制速度的话题）
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 创建 Twist 消息对象
    twist = Twist()

    # 设置线速度和角速度
    twist.linear.x = 0.5   # 向前移动，单位为米/秒
    twist.angular.z = 0.2  # 向右转动，单位为弧度/秒

    # 设定发布频率（10Hz）
    rate = rospy.Rate(10)

    # 发布速度命令
    rospy.loginfo("Publishing velocity commands...")
    for _ in range(50):  # 例如，发布 50 次速度命令
        cmd_pub.publish(twist)
        rospy.sleep(5)
        rate.sleep()  # 控制发布频率

    # 停止机器人
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_pub.publish(twist)
    rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        test_robot_velocity()
    except rospy.ROSInterruptException:
        pass
