#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("kuka_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

roll = 0
pitch = 1.57  # 90 grados para que esté mirando hacia abajo
yaw = 0

waypoints = []

pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 1.0
pose_target.position.x = 0.497586
pose_target.position.y = 0.281507
pose_target.position.z = 0.3

quat = tf.quaternion_from_euler(roll, pitch, yaw)

pose_target.orientation.x = quat[0]
pose_target.orientation.y = quat[1]
pose_target.orientation.z = quat[2]
pose_target.orientation.w = quat[3]

waypoints.append(copy.deepcopy(pose_target))


# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
fraction = 0.0
maxtries = 400
attempts = 0
#(plan, fraction) = group.compute_cartesian_path(
#                                   waypoints,   # waypoints to follow
#                                   0.01,        # eef_step
#                                   0.0)         # jump_threshold
while fraction < 1.0 and attempts < maxtries:
    (plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    
    attempts+=1
    if attempts % 10 == 0:
        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

group.execute(plan, wait=True)
# Note: We are just planning, not asking move_group to actually move the robot yet:
rospy.sleep(1)


# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
rospy.sleep(5)

moveit_commander.roscpp_shutdown()
