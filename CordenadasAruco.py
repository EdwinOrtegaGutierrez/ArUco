import cv2
import numpy as np

# Cargamos un diccionario ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)


# Definimos los parámetros de detección de ArUco
parameters = cv2.aruco.DetectorParameters()

# Definimos los parámetros de la cámara
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
distortion = np.array([0, 0, 0, 0])

# Iniciamos la captura de video
cap = cv2.VideoCapture(0)

while True:
    # Leemos el cuadro actual del video
    ret, frame = cap.read()

    if ret:
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
                print("Código ArUco ID: ", ids[i])
                print("Coordenadas (x, y, z): ", tvecs[i][0])

        # Mostramos el cuadro con los códigos ArUco detectados y las coordenadas calculadas
        cv2.imshow('frame', frame)

    # Esperamos a que se presione la tecla 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberamos la captura de video y cerramos todas las ventanas
cap.release()
cv2.destroyAllWindows()
