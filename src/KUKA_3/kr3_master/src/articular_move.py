#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg

# Inicialización de ROS y MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('joint_motion_python_interface', anonymous=True)

# Configuración de MoveIt
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("kuka_arm")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20
)

# Primera trayectoria
joint_goal1 = group.get_current_joint_values()
joint_goal1[0] = 0
joint_goal1[1] = -0.5
joint_goal1[2] = 0.5
joint_goal1[3] = 0
joint_goal1[4] = 1.5
joint_goal1[5] = 0

# Planificar y ejecutar el primer movimiento
group.go(joint_goal1, wait=True)
group.stop()

# Imprimir la posición actual de las articulaciones
current_joints = group.get_current_joint_values()
rospy.loginfo("Posición después de la primera trayectoria: %s" % current_joints)

# Esperar 1 segundo
rospy.sleep(1)

# Segunda trayectoria
joint_goal2 = group.get_current_joint_values()
joint_goal2[0] = -0.5
joint_goal2[1] = -0.5
joint_goal2[2] = 0.5
joint_goal2[3] = 0
joint_goal2[4] = 1.5
joint_goal2[5] = 0

# Planificar y ejecutar el segundo movimiento
group.go(joint_goal2, wait=True)
group.stop()

# Imprimir la posición final de las articulaciones
current_joints = group.get_current_joint_values()
rospy.loginfo("Posición después de la segunda trayectoria: %s" % current_joints)

rospy.sleep(5)

moveit_commander.roscpp_shutdown()
