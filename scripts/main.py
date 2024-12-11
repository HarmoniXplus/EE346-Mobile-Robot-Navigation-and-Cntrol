#!usr/bin/env python

import rospy
import smach
from navigation import navigator
from pole_detection import PoleDetector
from pose_init import initial_pose

class NavigateState(smach.State):
    def __init__(self, x, y):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.target_x = x
        self.target_y = y

    def execute(self, userdata):
        nav = navigator()
        rospy.loginfo(f"Navigating to ({self.target_x}, {self.target_y})...")
        client_result, client_state = nav.move_to_goal(self.target_x,self.target_y)
        # Add navigation logic here
        # Example: Using move_base action client
        if client_result == "SUCCEEDED":
          rospy.loginfo("Navigation succeeded!")
          success = True
        else:
          success = False# Replace with actual navigation success check
        return 'succeeded' if success else 'aborted'


class PoleDetectionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'])
        self.detector = PoleDetector()  # 假设 PoleDetector 实现了实时检测杆子的逻辑

    def execute(self, userdata):
        rospy.loginfo("Starting pole detection while approaching...")
    
        detected = False      
        self.detector.run()
        detected = self.detector.state
        print(detected)

        # 返回检测结果
        if detected:
            rospy.loginfo("Successfully approached the pole!")
            return 'detected'
        else:
            rospy.loginfo("Pole not detected!")
            return 'not_detected'

     
def main():
    rospy.init_node('robot_navigation_smach')
    initial_pose(5.35,3.41,0.999,0.0434)

    # 创建顶层状态机
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
    

    # 定义目标点的坐标
    points = {
        'P1': (5.35, 3.41),
        'P2': (1.984, 3.775),
        'P3': (0.929, 0.276),
        'P4': (4.83, -0.464)
    }


    with sm:
        # 导航到 P1
        # smach.StateMachine.add('NAVIGATE_P1', NavigateState(*points['P1']), transitions={'succeeded': 'NAVIGATE_P2', 'aborted': 'mission_failed'})

        # 导航到 P2
        smach.StateMachine.add('NAVIGATE_P2', NavigateState(*points['P2']), transitions={'succeeded': 'DETECT_POLE_P2', 'aborted': 'mission_failed'})
        smach.StateMachine.add('DETECT_POLE_P2', PoleDetectionState(), transitions={'detected': 'NAVIGATE_P3', 'not_detected': 'DETECT_POLE_P2'})
        
        # 导航到 P3
        smach.StateMachine.add('NAVIGATE_P3', NavigateState(*points['P3']), transitions={'succeeded': 'DETECT_POLE_P3', 'aborted': 'mission_failed'})
        smach.StateMachine.add('DETECT_POLE_P3', PoleDetectionState(), transitions={'detected': 'NAVIGATE_P4', 'not_detected': 'DETECT_POLE_P3'})
        
        # 导航到 P4
        smach.StateMachine.add('NAVIGATE_P4', NavigateState(*points['P4']), transitions={'succeeded': 'DETECT_POLE_P4', 'aborted': 'mission_failed'})
        smach.StateMachine.add('DETECT_POLE_P4', PoleDetectionState(), transitions={'detected': 'RETURN_TO_P1', 'not_detected': 'DETECT_POLE_P4'})
        
        # 返回 P1
        smach.StateMachine.add('RETURN_TO_P1', NavigateState(*points['P1']), transitions={'succeeded': 'mission_completed', 'aborted': 'mission_failed'})

        # smach.StateMachine.add('NAVIGATE_P1', NavigateState(*points['P1']), transitions={'succeeded': 'NAVIGATE_P2', 'aborted': 'mission_failed'})

 
    # 执行状态机
    
    outcome = sm.execute()
    rospy.loginfo(f"Mission outcome: {outcome}")

if __name__ == '__main__':
    main()