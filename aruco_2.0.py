#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(msg):
    try:
        cv_image = CvBridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Definir el diccionario de marcadores
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

        # Definir los parámetros del detector
        parameters = cv2.aruco.DetectorParameters()

        # Definimos los parámetros de la cámara
        camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
        distortion = np.array([0, 0, 0, 0])

        # Detectamos los códigos ArUco en el cuadro
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Leemos la imagen
        frame = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Convertimos el cuadro a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Si se encontraron códigos ArUco, calculamos las coordenadas en 3D
        if len(corners) > 0:
            # Calculamos las coordenadas de los códigos ArUco
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion)

            # Dibujamos los ejes de coordenadas para cada código ArUco
            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Imprimimos las coordenadas de cada código ArUco encontrado
            for i in range(len(ids)):
                print("Código ArUco ID: ", ids[i])
                print("Coordenadas (x, y, z): ", tvecs[i][0])
    except Exception as e:
        print(e)

rospy.init_node('image_listener')
image_topic = "/camera/color/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)
rospy.spin()