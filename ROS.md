# TO DO
1. Crear un paquete de ROS, utilizando el comando "catkin_create_pkg". 
    - incluir las dependencias "cv_bridge" y "rospy".

2. Crear un nodo de ROS, definir el tópico de la cámara, que estará utilizando la cámara ZED.

3. Modificar el código para que use la librería "cv_bridge" para convertir las imágenes de la cámara ZED a un formato de imagen que pueda ser utilizado por OpenCV. 

4. Usar los mensajes de ROS para publicar la imagen de los códigos ArUco en un tópico, en lugar de utilizar "cv2.imshow" para mostrar la imagen en una ventana.
# Pasos resumidos
1. Crear un paquete de ROS con las dependencias necesarias.
2. Crear un nodo de ROS que ejecute el código proporcionado.
3. Modificar el código para que utilice "cv_bridge" y mensajes de ROS.
4. Incluir el nodo en los archivos "CMakeLists.txt" y "package.xml" del paquete de ROS.