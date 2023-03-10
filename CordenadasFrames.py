#!/usr/bin/env python3
import cv2
import numpy as np

# Cargamos un diccionario ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

# Definimos los parámetros de detección de ArUco
parameters = cv2.aruco.DetectorParameters()

# Definimos los parámetros de la cámara
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
distortion = np.array([0, 0, 0, 0])

# Leemos la imagen
frame = cv2.imread('test_image.png')

# Convertimos el cuadro a escala de grises
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Detectamos los códigos ArUco en el cuadro
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

# Si se encontraron códigos ArUco, calculamos las coordenadas en 3D
if len(corners) > 0:
    # Calculamos las coordenadas de los códigos ArUco
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion)

    # Dibujamos los ejes de coordenadas para cada código ArUco
    for i in range(len(ids)):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    
    # Imprimimos las coordenadas de cada código ArUco encontrado
    for i in range(len(ids)):
        print(f"Código ArUco ID: {ids[i][0]}")
        if tvecs[i][0][0] > 0: 
            print(f"Coordenadas (x, y, z).\nx: {tvecs[i][0][0]} y: {tvecs[i][0][1]} z: {tvecs[i][0][2]}") # Si encuentra profundidad real
        else: print(f"Coordenadas (x, y, z):\nx: 0.0 y: 0.0 z: 0.0]") 
            # Se setea a 0 porque resulta leer una imagen literal, porque tratara de leer la profundidad con negativos