#test tello ryze camera for aruco detection

from djitellopy import Tello
import cv2 as cv
import time

# Create an ArUco dictionary
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)

# Initialize the detector parameters
parameters = cv.aruco.DetectorParameters()

# Create the ArUco detector
detector = cv.aruco.ArucoDetector(dictionary, parameters)

# Detection parameters
detection_threshold = 10  # Minimum consecutive detections to confirm an object

# Dictionary to store consecutive detection counters
consecutive_detections = {}

tello = Tello()
tello.connect()
print(tello.get_battery())

tello.streamon()
tello.takeoff()

# Send command every 5 seconds
last_command_time = time.time()

while True:
    # Send command every 5 seconds
    current_time = time.time()
    if current_time - last_command_time > 5:
        tello.send_command_without_return("command")
        last_command_time = current_time

    # Read a frame from the camera
    frame = tello.get_frame_read().frame

    # Detect ArUco markers in the frame
    markerCorners, markerIds, _ = detector.detectMarkers(frame)

    # Process detected markers
    if markerIds is not None:
        for markerId in markerIds.flatten():
            markerId = int(markerId)
            consecutive_detections[markerId] = consecutive_detections.get(markerId, 0) + 1

            # Check if the detection threshold is reached
            if consecutive_detections[markerId] >= detection_threshold:
                print(f"Detected ID {markerId}")

    # Process undetected markers (missed detections)
    if markerIds is not None:
        detected_marker_ids = set(markerIds.flatten())
        for markerId in set(consecutive_detections.keys()) - detected_marker_ids:
            consecutive_detections[markerId] = 0

    # Draw the markers on the frame
    frame = cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

    # Display the frame
    cv.imshow('ArUco Marker Detection', frame)

    # Press 'q' to exit
    if cv.waitKey(1) & 0xFF == ord('q'):
        print('q command was pressed')
        break

# Turn off video stream
tello.streamoff()
tello.land()

cv.destroyAllWindows()