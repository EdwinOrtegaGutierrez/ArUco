import cv2 as cv
from cv2 import aruco
import numpy as np

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_makers = aruco.DetectorParameters_create()

cap = cv.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
		gray_frame, marker_dict, parameters=param_makers
	)
    
    if marker_IDs: 
        for ids, corners in zip(marker_IDs, marker_corners): 
            cv.polylines(frame, [corners.astype(np.int32)], True, (0,255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4,2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            cv.putText(frame, f"id: {ids[0]}", top_right, cv.FONT_HERSHEY_PLAIN, 1.3, (0,255,0), 2, cv.LINE_AA)
    
    cv.imshow("Read ArUco", frame)
    
    key = cv.waitKey(1)
    if key == ord("q"):
        break
    
cap.release()
cv.destroyAllWindows()