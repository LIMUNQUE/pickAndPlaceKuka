#!/usr/bin/env python3
#Este es el mismo script para mover el brazo pero uso scipy para graficar la posiciòn de los motores (lo pidiò el profe)
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
import tf.transformations as tf
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

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

        # Listas para registrar los ángulos de las articulaciones y los tiempos
        self.joint_a1_vals = []
        self.joint_a2_vals = []
        self.joint_a3_vals = []
        self.joint_a4_vals = []
        self.joint_a5_vals = []
        self.joint_a6_vals = []
        self.times = []

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
        
        # Registrar los valores actuales de las articulaciones y el tiempo
        current_joints = self.arm_group.get_current_joint_values()
        current_time = rospy.get_time()
        
        self.joint_a1_vals.append(current_joints[0])
        self.joint_a2_vals.append(current_joints[1])
        self.joint_a3_vals.append(current_joints[2])
        self.joint_a4_vals.append(current_joints[3])
        self.joint_a5_vals.append(current_joints[4])
        self.joint_a6_vals.append(current_joints[5])
        self.times.append(current_time)

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
        pose_target.position.z = 0.37  # Asumimos que z es constante
        quat = tf.quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        
        self.move_arm(pose_target)
        
        # Mover el brazo a la posición del objeto
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x - gripper_length
        pose_target.position.y = y
        pose_target.position.z = 0.25  # Asumimos que z es constante
        quat = tf.quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        
        self.move_arm(pose_target)
        
        # Cerrar el gripper
        self.move_gripper(0.190)
        
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

        # Graficar los ángulos de las articulaciones
        self.plot_joint_angles()

        # Cierra el nodo después de completar la tarea
        rospy.signal_shutdown("Task completed")

    def plot_joint_angles(self):
        plt.figure(figsize=(14, 8))
        
        # Aplicar interpolación para suavizar las curvas
        time_smooth = np.linspace(min(self.times), max(self.times), 300)
        
        joint_a1_smooth = make_interp_spline(self.times, self.joint_a1_vals)(time_smooth)
        joint_a2_smooth = make_interp_spline(self.times, self.joint_a2_vals)(time_smooth)
        joint_a3_smooth = make_interp_spline(self.times, self.joint_a3_vals)(time_smooth)
        joint_a4_smooth = make_interp_spline(self.times, self.joint_a4_vals)(time_smooth)
        joint_a5_smooth = make_interp_spline(self.times, self.joint_a5_vals)(time_smooth)
        joint_a6_smooth = make_interp_spline(self.times, self.joint_a6_vals)(time_smooth)

        # Graficar los ángulos de todas las articulaciones con líneas suaves
        plt.plot(time_smooth, joint_a1_smooth, label='Joint A1', color='red', linewidth=2)
        plt.plot(time_smooth, joint_a2_smooth, label='Joint A2', color='blue', linewidth=2)
        plt.plot(time_smooth, joint_a3_smooth, label='Joint A3', color='green', linewidth=2)
        plt.plot(time_smooth, joint_a4_smooth, label='Joint A4', color='purple', linewidth=2)
        plt.plot(time_smooth, joint_a5_smooth, label='Joint A5', color='orange', linewidth=2)
        plt.plot(time_smooth, joint_a6_smooth, label='Joint A6', color='cyan', linewidth=2)
        
        # Ajustes adicionales para mejorar la visualización
        plt.xlabel('Tiempo (s)', fontsize=14)
        plt.ylabel('Ángulo (rad)', fontsize=14)
        plt.title('Movimiento de las Articulaciones del Brazo', fontsize=16)
        plt.legend(fontsize=12)
        plt.grid(True, which='both', linestyle='--', linewidth=0.5)
        plt.tight_layout()

        # Mostrar la gráfica
        plt.show()

if __name__ == "__main__":
    arm_controller = ArmController()
    rospy.spin()

