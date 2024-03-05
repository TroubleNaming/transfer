#!/usr/bin/env python3

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Empty




if __name__ == "__main__":
    try:
        # Initialise moveit_commander and rosnode
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_place', anonymous=False)
        
                
        # Diagnostic publisher for waypoint poses
        pose_pub = rospy.Publisher('/checker',PoseStamped,latch=True,queue_size=5)
        
        # Initialise robot and move groups
        robot = moveit_commander.robot.RobotCommander()
        arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
        gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")

        rospy.sleep(2)
        
        # Start at home position
        update_octomap()
        home_state = arm_group.get_current_state().joint_state
        home_state.name = list(home_state.name)[:6]
        home_state.position = [arm_group.get_named_target_values('home')['shoulder_pan_joint'],
                                arm_group.get_named_target_values('home')['shoulder_lift_joint'],
                                arm_group.get_named_target_values('home')['elbow_joint'],
                                arm_group.get_named_target_values('home')['wrist_1_joint'],
                                arm_group.get_named_target_values('home')['wrist_2_joint'],
                                arm_group.get_named_target_values('home')['wrist_3_joint']]
        plan = arm_group.plan(home_state)
        success = arm_group.execute(plan[1], wait=True)
        arm_group.stop()
        while not success:  # FALLBACK FOR SAFETY
            arm_group.stop()
            plan = arm_group.plan(home_state)
            success = arm_group.execute(plan[1], wait=True)
            arm_group.stop()

        
        rospy.sleep(1)
        
        # Open gripper
        update_octomap()
        open_gripper = [gripper_group.get_named_target_values('open')['robotiq_85_left_knuckle_joint']]
        gripper_group.go(open_gripper, wait=True)
        gripper_group.stop()
        rospy.sleep(1)        
        
        
        # Close gripper
        update_octomap()
        close_gripper = [gripper_group.get_named_target_values('closed')['robotiq_85_left_knuckle_joint']]
        gripper_group.go(close_gripper, wait=True)
        gripper_group.stop()        

        rospy.sleep(1)

        
        
            
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
