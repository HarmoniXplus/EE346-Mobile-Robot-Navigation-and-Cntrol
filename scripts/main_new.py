#!/usr/bin/env python

from statemachine import State, StateMachine
from navigation import navigator
from pole_detection import PoleDetector
import rospy

class RobotStateMachine(StateMachine):
    # 定义状态
    idle = State("Idle", initial=True)
    #navigating_p2 = State("Navigating to P2")
    #detecting_pole_p2 = State("Detecting Pole at P2")
    navigating_p3 = State("Navigating to P3")
    detecting_pole_p3 = State("Detecting Pole at P3")
    navigating_p4 = State("Navigating to P4")
    detecting_pole_p4 = State("Detecting Pole at P4")
    navigating_p1 = State("Navigating to P1")
    mission_completed = State("Mission Completed")
    mission_failed = State("Mission Failed")

    # 定义状态转换
    # start_navigation_p2 = idle.to(navigating_p2)
    # detect_pole_at_p2 = navigating_p2.to(detecting_pole_p2)
    navigate_to_p3 = idle.to(navigating_p3)
    detect_pole_at_p3 = navigating_p3.to(detecting_pole_p3)
    navigate_to_p4 = detecting_pole_p3.to(navigating_p4)
    detect_pole_at_p4 = navigating_p4.to(detecting_pole_p4)
    return_to_p1 = detecting_pole_p4.to(navigating_p1)
    complete_mission = navigating_p1.to(mission_completed)
    fail_mission = (
        # navigating_p2.to(mission_failed)
         navigating_p3.to(mission_failed)
        | navigating_p4.to(mission_failed)
        | navigating_p1.to(mission_failed)
    )

    def __init__(self, points):
        super().__init__()
        self.navigator = navigator()
        self.detector = PoleDetector()
        self.points = points

    def navigate_to(self, target):
        rospy.loginfo(f"Navigating to {target}...")
        client_result, _ = self.navigator.move_to_goal(*target)
        if client_result == "SUCCEEDED":
            rospy.loginfo(f"Successfully reached {target}.")
            return True
        else:
            rospy.loginfo(f"Failed to reach {target}.")
            return False

    def detect_pole_near_target(self):
        rospy.loginfo("Starting pole detection while approaching...")
    
        detected = False      
        self.detector.run()
        detected = self.detector.state
        print(detected)

        # 返回检测结果
        if detected:
            rospy.loginfo("Successfully approached the pole!")
            return True
        else:
            rospy.loginfo("Pole not detected!")
            return False

    def on_start_navigation_p2(self):
        if self.navigate_to(self.points["P2"]):
            self.detect_pole_at_p2()
        else:
            self.fail_mission()

    def on_detect_pole_at_p2(self):
        if self.detect_pole_near_target():
            self.navigate_to_p3()
        else:
            self.fail_mission()

    def on_navigate_to_p3(self):
        if self.navigate_to(self.points["P3"]):
            self.detect_pole_at_p3()
        else:
            self.fail_mission()

    def on_detect_pole_at_p3(self):
        if self.detect_pole_near_target():
            self.navigate_to_p4()
        else:
            self.fail_mission()

    def on_navigate_to_p4(self):
        if self.navigate_to(self.points["P4"]):
            self.detect_pole_at_p4()
        else:
            self.fail_mission()

    def on_detect_pole_at_p4(self):
        if self.detect_pole_near_target():
            self.return_to_p1()
        else:
            self.fail_mission()

    def on_return_to_p1(self):
        if self.navigate_to(self.points["P1"]):
            self.complete_mission()
        else:
            self.fail_mission()


def main():
    rospy.init_node("robot_navigation_statemachine")

    # 定义导航点
    points = {
        "P1": (5.35, 3.41),
        "P2": (1.984, 3.775),
        "P3": (0.929, 0.276),
        "P4": (4.83, -0.464),
    }

    # 创建状态机实例
    sm = RobotStateMachine(points)

    try:
        sm.start_navigation_p3()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        sm.fail_mission()

    if sm.is_mission_completed:
        rospy.loginfo("Mission completed successfully!")
    else:
        rospy.loginfo("Mission failed.")

if __name__ == "__main__":
    main()

