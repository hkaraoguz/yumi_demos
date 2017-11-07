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

def plan_point(goal):

    plans = []

    hand_id = -1

    if goal.position.y > 0.0:
        hand_id = yumi.LEFT
        move_group = yumi.group_l
    else:
        hand_id = yumi.RIGHT
        move_group = yumi.group_r


    print hand_id

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

    plans = []

    hand_id = -1

    if goal.position.y > 0.0:
        hand_id = yumi.LEFT
        move_group = yumi.group_l
    else:
        hand_id = yumi.RIGHT
        move_group = yumi.group_r


    print hand_id

    if goal.position.z < 0.3:
        goal.position.z = 0.3


    pose_ee_t = [goal.position.x, goal.position.y, goal.position.z, 0.0,3.14, 0.0]

    plan = yumi.plan_path_global(move_group,pose_ee_t)

    if(plan and len(plan.joint_trajectory.points) != 0 ):
        plans.append(plan)
    else:
        return []

    pose_ee_t[2] = 0.2
    pose_ee_t[5] = goal.orientation.z#-3.14159

    plan = yumi.plan_path_local([pose_ee_t],hand_id)

    if(plan and len(plan.joint_trajectory.points) != 0 ):
        plans.append(plan)
    else:
        return []

    ''' Go up again '''
    pose_ee_t[2] = 0.4

    plan = yumi.plan_path_local([pose_ee_t],hand_id)

    if(plan and len(plan.joint_trajectory.points) != 0 ):
        plans.append(plan)
    else:
        return []


    if hand_id == yumi.LEFT:
        pose_ee = [0.1, 0.4, 0.3, 0.0, -2.0, 0.0]
        plan = yumi.plan_path_local([pose_ee],hand_id)

        if(plan and len(plan.joint_trajectory.points) != 0 ):
            plans.append(plan)
        else:
            return []

    else:
        pose_ee = [0.1, -0.4, 0.3, 0.0, -2.0, 0.0]
        plan = yumi.plan_path_local([pose_ee],hand_id)

        if(plan and len(plan.joint_trajectory.points) != 0 ):
            plans.append(plan)
        else:
            return []

    return plans


def planforaction_callback(req):

    if req.action == 0:
        return {'trajectories':plan_pickplace(req.location)}
    #elif req.action == 1:

    return {'trajectories':plan_point(req.location)}



if __name__ == '__main__':

    rospy.init_node('yumi_moveit_planning')
    yumi.init_Moveit()
    s = rospy.Service('yumi_plan_action', PlanforAction, planforaction_callback)
    rospy.spin()
