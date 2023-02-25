import cv2
import numpy as np

# Carga el diccionario de códigos ArUco predefinido
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)

# Configura los parámetros del detector de marcadores ArUco
parameters = cv2.aruco.DetectorParameters()

# Configura la cámara
cap = cv2.VideoCapture(0)

# Define las coordenadas 3D del objeto que se desea seguir
object_points = np.array([[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]], dtype=np.float32)

while True:
    # Captura una imagen de la cámara
    ret, frame = cap.read()

    # Detecta los marcadores ArUco en la imagen
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dictionary, parameters=parameters)

    # Si se detecta al menos un marcador, calcula la pose del objeto
    if ids is not None:
        # Valores genéricos para una cámara
        cameraMatrix = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1]])
        distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 1, cameraMatrix, distCoeffs)
        
        # Proyecta las coordenadas 3D del objeto en la imagen
        image_points, _ = cv2.projectPoints(object_points, rvec, tvec, cameraMatrix, distCoeffs)
        
        # Dibuja un objeto virtual en la imagen que sigue la posición y orientación del objeto
        cv2.drawContours(frame, [np.int32(image_points)], -1, (0, 255, 0), 2)
        print(f"ID: {ids[0][0]}")
    # Dibuja los marcadores detectados en la imagen
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    
    # Muestra la imagen resultante
    cv2.imshow('frame', frame)

    # Espera por una tecla para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera la cámara y cierra la ventana
cap.release()
cv2.destroyAllWindows()
