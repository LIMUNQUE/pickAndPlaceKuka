#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray

class ArmController:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()    
        self.group = moveit_commander.MoveGroupCommander("kuka_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        
        self.coord_sub = rospy.Subscriber('/blue_object_coords', Float32MultiArray, self.move_to_object)
    
    def move_to_object(self, coord_msg):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = coord_msg.data[0]
        pose_target.position.y = coord_msg.data[1]
        pose_target.position.z = 0.1  # Asumimos que z es constante
        
        self.group.set_pose_target(pose_target)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

if __name__ == "__main__":
    arm_controller = ArmController()
    rospy.spin()

