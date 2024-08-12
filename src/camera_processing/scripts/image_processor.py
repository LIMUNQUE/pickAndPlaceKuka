#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/color/image_raw", Image, self.image_callback)
        
        # Definir los rangos de color azul en HSV
        self.azulBajo = np.array([100,100,20], np.uint8)
        self.azulAlto = np.array([125,255,255], np.uint8)
        
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def dibujar(self, frame, mask, color):
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 3000:
                M = cv2.moments(c)
                if M["m00"] == 0:
                    M["m00"] = 1
                x = int(M["m10"] / M["m00"])
                y = int(M['m01'] / M['m00'])
                nuevoContorno = cv2.convexHull(c)
                cv2.circle(frame, (x,y), 7, (0,255,0), -1)
                cv2.putText(frame, f'{x},{y}', (x+10,y), self.font, 0.75, (0,255,0), 1, cv2.LINE_AA)
                cv2.drawContours(frame, [nuevoContorno], 0, color, 3)
        return frame

    def image_callback(self, msg):
        # Convertir la imagen ROS a una imagen OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convertir la imagen a HSV
        frameHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Crear una máscara para el color azul
        maskAzul = cv2.inRange(frameHSV, self.azulBajo, self.azulAlto)
        
        # Dibujar contornos y centros de los objetos azules
        result = self.dibujar(cv_image, maskAzul, (210,162,26))
        
        # Mostrar la imagen resultante
        cv2.imshow("Blue Object Detection", result)
        cv2.waitKey(1)

def main():
    rospy.init_node('blue_object_detector')
    processor = ImageProcessor()
    rospy.spin()

if __name__ == '__main__':
    main()
