#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
import tf.transformations as tf
from time import sleep

roll = 0
pitch = 1.57  # 90 grados para que esté mirando hacia abajo
yaw = 0

caution_length = 0.30
gripper_length = 0.15

main_joint_values = {
                'joint_a1': 0.000,
                'joint_a2': -0.500,
                'joint_a3': 0.500,
                'joint_a4': 0.000,
                'joint_a5': 0.000,
                'joint_a6': 0.000
            }

class ArmController:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()    
        self.arm_group = moveit_commander.MoveGroupCommander("kuka_arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self.arm_group.set_planner_id("RRTstar")
        self.arm_group.set_planning_time(10)
        
        self.coord_sub = rospy.Subscriber('/object_coords', Float32MultiArray, self.move_to_object)
        self.is_busy = False

    def move_arm(self, pose_target):
        self.arm_group.set_pose_target(pose_target)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        sleep(0.1)

    def move_gripper(self, angle):
        self.gripper_group.set_joint_value_target({'finger_joint': angle})
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        sleep(0.1)

    def move_arm_joint(self, joint_values):
        self.arm_group.set_joint_value_target(joint_values)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        sleep(0.1)

    def move_to_object(self, coord_msg):
        if self.is_busy:
            return

        self.is_busy = True
        
        # Posición inicial
        self.move_arm_joint(main_joint_values)
        self.move_gripper(0.0)
        
        x, y, color_id = coord_msg.data
        
        # Mover el brazo a la posición del objeto
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x - caution_length
        pose_target.position.y = y
        pose_target.position.z = 0.35  # Asumimos que z es constante
        quat = tf.quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        
        self.move_arm(pose_target)
        
        
        
        # Mover el brazo a la posición del objeto
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x -gripper_length
        pose_target.position.y = y
        pose_target.position.z = 0.25  # Asumimos que z es constante
        quat = tf.quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        
        self.move_arm(pose_target)
        # Cerrar el gripper
        self.move_gripper(0.155)
        
        # Cerrar el gripper
        self.move_gripper(0.155)
        
        # Mover el brazo a la posición de depósito según el color
        if color_id == 1:  # Azul
            joint_values = {
                'joint_a1': -2.000,
                'joint_a2': -0.162,
                'joint_a3': 1.151,
                'joint_a4': 0.083,
                'joint_a5': -0.991,
                'joint_a6': -0.048
            }
        else:  # Rojo
            joint_values = {
                'joint_a1': 2.000,
                'joint_a2': -0.162,
                'joint_a3': 1.151,
                'joint_a4': 0.083,
                'joint_a5': -0.991,
                'joint_a6': -0.048
            }
        
        self.move_arm_joint(joint_values)
        
        # Abrir el gripper para soltar el objeto
        self.move_gripper(0.0)
        
        # Volver a la posición inicial
        self.move_arm_joint(main_joint_values)
        
        self.is_busy = False

        # Cierra el nodo después de completar la tarea
        rospy.signal_shutdown("Task completed")

if __name__ == "__main__":
    arm_controller = ArmController()
    rospy.spin()

