import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def initial_pose(x, y, z, w):
    """
    发布初始位置到 /initialpose 话题，供 AMCL 初始化使用。

    参数:
    x (float): 初始位置的 x 坐标（单位：米）。
    y (float): 初始位置的 y 坐标（单位：米）。
    w (float): 朝向的四元数 w 分量（假设没有旋转的 x, y, z 分量）。
    """
    # 初始化 ROS 节点
    # rospy.init_node('initial_pose_publisher', anonymous=True)

    # 创建发布者
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    # 创建初始位置消息
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"  # 固定地图框架
    initial_pose.header.stamp = rospy.Time.now()  # 时间戳

    # 设置位置
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.position.z = 0.0  # 通常是 0

    # 设置朝向（只设置 w，默认无旋转）
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = z
    initial_pose.pose.pose.orientation.w = w

    # 设置协方差（可以根据实际需要调整）
    initial_pose.pose.covariance = [0.18, 0, 0, 0, 0, 0,  # x
                                    0, 0.17, 0, 0, 0, 0,  # y
                                    0, 0, 0.0, 0, 0, 0,  # z
                                    0, 0, 0, 0.0, 0, 0,  # roll
                                    0, 0, 0, 0, 0.0, 0,  # pitch
                                    0, 0, 0, 0, 0, 0.065]  # yaw

    rospy.loginfo("Publishing initial pose: x=%f, y=%f, w=%f", x, y, w)

    # 等待发布者就绪
    rospy.sleep(1)

    # 发布初始位置
    initial_pose_pub.publish(initial_pose)
    rospy.loginfo("Initial pose published!")

def main():
    try:
        # 示例初始位置参数（根据实际场景修改）
        x = 2.0
        y = 3.0
        z = 0
        w = 1.0  

        # 发布初始位置
        initial_pose(x, y, z, w)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    main()
