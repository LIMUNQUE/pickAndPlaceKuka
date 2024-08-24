#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

M_homography = np.array([
    [-4.32451535e-03,  7.24517618e-05,  1.36641253e+00],
    [4.12785446e-05, -4.30712150e-03,  1.54470356e+00],
    [7.78377698e-05,  1.56970603e-04,  1.00000000e+00]
])

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/color/image_raw", Image, self.image_callback)
        self.coords_pub = rospy.Publisher('/object_coords', Float32MultiArray, queue_size=10)
        
        self.azulBajo = np.array([100,100,20], np.uint8)
        self.azulAlto = np.array([125,255,255], np.uint8)
        # Añadimos rangos para el color rojo
        self.rojoBajo1 = np.array([0,100,20], np.uint8)
        self.rojoAlto1 = np.array([10,255,255], np.uint8)
        self.rojoBajo2 = np.array([170,100,20], np.uint8)
        self.rojoAlto2 = np.array([180,255,255], np.uint8)
        
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def dibujar(self, frame, mask, color, name):
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 200:
                M = cv2.moments(c)
                if M["m00"] == 0:
                    M["m00"] = 1
                x = int(M["m10"] / M["m00"])
                y = int(M['m01'] / M['m00'])
                nuevoContorno = cv2.convexHull(c)
                cv2.circle(frame, (x,y), 7, (0,255,0), -1)
                cv2.putText(frame, f'{x},{y}', (x+10,y), self.font, 0.75, (0,255,0), 1, cv2.LINE_AA)
                cv2.drawContours(frame, [nuevoContorno], 0, color, 3)
                
                new_point_pixel = np.array([[x, y]], dtype=np.float32)
                new_point_global = cv2.transform(np.array([new_point_pixel]), M_homography)
                
                scale = new_point_global[0][0][2]
                global_y = new_point_global[0][0][0] / scale
                global_x = (new_point_global[0][0][1] / scale)
                
                # Publicar las coordenadas globales y el nombre (1 para azul, 2 para rojo)
                coord_msg = Float32MultiArray()
                coord_msg.data = [global_x, global_y, 1 if name == "azul" else 2]
                self.coords_pub.publish(coord_msg)
        return frame

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = cv_image.shape[:2]
        
        text = "Workspace"
        text_size = cv2.getTextSize(text, self.font, 1, 2)[0]
        text_x = int(width*0.5 - text_size[0]/2)  # Centramos el texto horizontalmente
        text_y = int(height*0.1) - 10  # 10 píxeles arriba del rectángulo negro
        cv2.putText(cv_image, text, (text_x, text_y), self.font, 1, (0, 0, 0), 2, cv2.LINE_AA)
        # Dibujamos el rectángulo negro para la zona de detección
        detection_zone = [(int(width*0.2), int(height*0.3)), (int(width*0.80), int(height*0.70))]
        cv2.rectangle(cv_image, detection_zone[0], detection_zone[1], (0,0,0), 2)
        
        # Dibujamos los rectángulos azul y rojo para las zonas de depósito
        blue_deposit = [(int(width*0.1), int(height*0.75)), (int(width*0.4), int(height*0.95))]
        red_deposit = [(int(width*0.6), int(height*0.75)), (int(width*0.9), int(height*0.95))]
        cv2.rectangle(cv_image, blue_deposit[0], blue_deposit[1], (255,0,0), 2)
        cv2.rectangle(cv_image, red_deposit[0], red_deposit[1], (0,0,255), 2)
        
        # Creamos una máscara para la zona de detección
        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        cv2.rectangle(mask, detection_zone[0], detection_zone[1], 255, -1)
        
        frameHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        maskAzul = cv2.inRange(frameHSV, self.azulBajo, self.azulAlto)
        maskRojo1 = cv2.inRange(frameHSV, self.rojoBajo1, self.rojoAlto1)
        maskRojo2 = cv2.inRange(frameHSV, self.rojoBajo2, self.rojoAlto2)
        maskRojo = cv2.add(maskRojo1, maskRojo2)
        
        # Aplicamos la máscara de la zona de detección
        maskAzul = cv2.bitwise_and(maskAzul, mask)
        maskRojo = cv2.bitwise_and(maskRojo, mask)
        
        result = self.dibujar(cv_image, maskAzul, (210,162,26), "azul")
        result = self.dibujar(result, maskRojo, (39,76,241), "rojo")
        
        cv2.imshow("Object Detection", result)
        cv2.waitKey(1)

def main():
    rospy.init_node('object_detector')
    processor = ImageProcessor()
    rospy.spin()

if __name__ == '__main__':
    main()
