import cv2
import queue
import threading
import signal

ARUCO_DICT = {i: cv2.aruco.DICT_4X4_50 + i for i in range(21)}
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[1])
arucoParams = cv2.aruco.DetectorParameters()

detection_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()
threads = []

def detect_markers():
    cap = cv2.VideoCapture(0)
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            break
        corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        detection_queue.put((corners, ids, frame))
    cap.release()

def display_markers():
    while not stop_event.is_set():
        corners, ids, frame = detection_queue.get()
        detected_markers = aruco_display(corners, ids, frame)
        cv2.imshow("Image", detected_markers)
        if cv2.waitKey(1) == ord("q"):
            stop_event.set()
    cv2.destroyAllWindows()

def aruco_display(corners, ids, image):
    if ids is not None and len(ids) > 0:
        for markerCorner, markerID in zip(corners, ids):
            markerCorner = markerCorner.reshape((4, 2)).astype(int)
            cv2.polylines(image, [markerCorner], True, (0, 255, 0), 2)
            cv2.putText(image, str(markerID), tuple(markerCorner[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(f"[Inference] ArUco marker ID: {markerID}")
    return image

threads.append(threading.Thread(target=detect_markers))
threads.append(threading.Thread(target=display_markers))

for thread in threads:
    thread.start()

signal.signal(signal.SIGINT, lambda signum, frame: stop_event.set())

for thread in threads:
    thread.join()