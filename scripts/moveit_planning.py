#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from yumi_demos import yumi_moveit_utils as yumi
import moveit_msgs.msg
from moveit_msgs.msg import RobotTrajectory
import geometry_msgs.msg
from std_srvs.srv import Empty
from yumi_demos.srv import *
import argparse

"""
This script is for generating trajectory plans for an action
using Moveit

"""

def plan_point(goal):
    """
    Function that generates the plans for the point action
    """

    plans = []

    hand_id = -1

    if goal.position.y > 0.0:
        hand_id = yumi.LEFT
        move_group = yumi.group_l
    else:
        hand_id = yumi.RIGHT
        move_group = yumi.group_r

    if goal.position.z < 0.3:
        goal.position.z = 0.3


    pose_ee_t = [goal.position.x, goal.position.y, goal.position.z, 0.0,3.14, 0.0]
    plan = RobotTrajectory()
    plan = yumi.plan_path_local([pose_ee_t],hand_id)

    if(plan and len(plan.joint_trajectory.points) != 0 ):
        plans.append(plan)
    else:
        return []


    return plans


def plan_pickplace(goal):
    """
    Function that generates the plans for the pick_place action
    """

    plans = []

    hand_id = -1

    if goal.position.y > 0.0:
        hand_id = yumi.LEFT
        move_group = yumi.group_l
    else:
        hand_id = yumi.RIGHT
        move_group = yumi.group_r

    if goal.position.z < 1.2:
        goal.position.z = 1.2

    """ Move the arm on top of the object """
    pose_ee_t = [goal.position.x, goal.position.y, goal.position.z, 0.0,3.14, 0.0]

    plan = yumi.plan_path_global(move_group,pose_ee_t)

    if(plan and len(plan.joint_trajectory.points) != 0 ):
        plans.append(plan)
    else:
        return []
    """"""

    """ Move the arm down """
    pose_ee_t[2] = 1.11
    pose_ee_t[5] = goal.orientation.z#-3.14159

    ## Get the robot state when the arm reaches to the previous waypoint
    current_robot_state = yumi.CurrentRobotState(plan.joint_trajectory.joint_names,plan.joint_trajectory.points[-1].positions)

    plan = yumi.plan_path_local([pose_ee_t],hand_id,current_robot_state)

    if(plan and len(plan.joint_trajectory.points) != 0 ):
        plans.append(plan)
    else:
        return []
    """"""

    ''' Go up again '''
    pose_ee_t[2] = 1.2

    current_robot_state.joint_positions = plan.joint_trajectory.points[-1].positions
    plan = yumi.plan_path_local([pose_ee_t],hand_id,current_robot_state)

    if(plan and len(plan.joint_trajectory.points) != 0 ):
        plans.append(plan)
    else:
        return []

    """"""
    """ Go to the drop location """
    if hand_id == yumi.LEFT:
        pose_ee = [0.3, 0.3, 1.2, 0.0, 3.14, 0.0]
        current_robot_state.joint_positions = plan.joint_trajectory.points[-1].positions
        plan = yumi.plan_path_local([pose_ee],hand_id,current_robot_state)

        if(plan and len(plan.joint_trajectory.points) != 0 ):
            plans.append(plan)
        else:
            return []

    else:
        pose_ee = [0.3, -0.3, 1.2, 0.0, 3.14, 0.0]
        current_robot_state.joint_positions = plan.joint_trajectory.points[-1].positions
        plan = yumi.plan_path_local([pose_ee],hand_id,current_robot_state)

        if(plan and len(plan.joint_trajectory.points) != 0 ):
            plans.append(plan)
        else:
            return []
    """"""
    return plans


def planforaction_callback(req):

    """
    Function for receiving service callback
    """

    if req.action == 0:
        return {'trajectories':plan_pickplace(req.location)}
    #elif req.action == 1:

    return {'trajectories':plan_point(req.location)}



if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--planning_frame",type=str, default="/yumi_pedestal",
        help='Moveit planning frame')

    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('moveit_yumi_planning')

    try:
        yumi.init_Moveit(args.planning_frame)
    except Exception as ex:
        rospy.logwarn("Moveit cannot be initialized!! %s",str(ex))
        rospy.signal_shutdown("Moveit cannot be initialized!!")

    s = rospy.Service('moveit_yumi_plan_action', PlanforAction, planforaction_callback)
    rospy.spin()
