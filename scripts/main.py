#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
from yumi_demos import yumi_moveit_utils as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
import argparse

def run(planning_frame="/world"):

    """Starts the node

    Runs to start the node and initialize everthing.

    :returns: Nothing
    :rtype: None
    """

    #Start by connecting to ROS and MoveIt!
    yumi.init_Moveit(planning_frame)


    # Print current joint angles
    rospy.loginfo("Printing right arm joint states...")
    yumi.print_current_joint_states(yumi.RIGHT)
    rospy.loginfo("Printing left arm joint states...")
    yumi.print_current_joint_states(yumi.LEFT)

    # Drive YuMi end effectors to a desired position (pose_ee_x), and perform a grasping task with a given effort (grip_effort)
    # Gripper effort: opening if negative, closing if positive, static if zero


    # Right arm Target pose
    pose_ee_r = [0.3, -0.3, 0.25, 0.0, 3.14, 0.0]

    yumi.open_grippers(yumi.LEFT)
    yumi.open_grippers(yumi.RIGHT)
    yumi.move_global_planning(yumi.RIGHT, pose_ee_r)
    rospy.sleep(2.0)
    pose_ee_r[2] = 0.2
    yumi.move_global_planning(yumi.RIGHT, pose_ee_r)
    rospy.sleep(2.0)
    pose_ee_r[2] = 0.25
    yumi.move_global_planning(yumi.RIGHT, pose_ee_r)
    rospy.sleep(2.0)

    pose_ee_l = [0.3, 0.3, 0.25, 0.0, 3.14, 0.0]

    yumi.move_global_planning(yumi.LEFT, pose_ee_l)
    rospy.sleep(2.0)
    pose_ee_l[2] = 0.2
    yumi.move_global_planning(yumi.LEFT, pose_ee_l)
    rospy.sleep(2.0)
    pose_ee_l[2] = 0.25
    yumi.move_global_planning(yumi.LEFT, pose_ee_l)
    rospy.sleep(2.0)

    # Reset YuMi joints to "home" position
    yumi.reset_pose()



if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--planning_frame",type=str, default="/world",
        help='Moveit planning frame')

    args = parser.parse_args(rospy.myargv()[1:])


    rospy.init_node('yumi_moveit_demo_test')

    try:
        run(args.planning_frame)

    	print "####################################     Program finished     ####################################"
    except rospy.ROSInterruptException:
        pass
