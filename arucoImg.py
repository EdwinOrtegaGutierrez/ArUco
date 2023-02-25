import cv2

# Cargar la imagen desde un archivo
aruco_image = cv2.imread('test_image.png')

# Definir el diccionario de marcadores
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

# Definir los parámetros del detector
parameters = cv2.aruco.DetectorParameters()

# Detectar los marcadores en la imagen
corners, ids, rejected_img_points = cv2.aruco.detectMarkers(aruco_image, aruco_dict, parameters=parameters)

# Dibujar los marcadores detectados
aruco_image_with_corners = cv2.aruco.drawDetectedMarkers(aruco_image, corners, ids)

# Mostrar la imagen con los marcadores detectados
cv2.imshow('Image with ArUco markers', aruco_image_with_corners)
cv2.waitKey(0)
cv2.destroyAllWindows()

