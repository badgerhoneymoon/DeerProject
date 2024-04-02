from djitellopy import Tello
import cv2
import numpy as np
import time

# Constants
MARKER_SIZE = 0.2  # ArUco marker size in meters
YAW_THRESHOLD = 5  # Threshold for yaw angle (in degrees)
LAND_KEY = ord('q')  # Key to land the drone (default: 'q')

CAMERA_MATRIX = np.array([[921.170702, 0.000000, 459.904354],
                          [0.000000, 919.018377, 351.238301],
                          [0.000000, 0.000000, 1.000000]], dtype=np.float32)

DIST_COEFFS = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000], dtype=np.float32)

def connect_drone():
    tello = Tello()
    tello.connect()
    print(tello.get_battery())
    tello.streamon()

    while True:

        cv2.namedWindow("Drone Feed", cv2.WINDOW_NORMAL)
        #cv2.resizeWindow("Drone Feed", 1280, 720)  # Set the window size to 640x480 pixels
        cv2.setWindowProperty("Drone Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        frame = tello.get_frame_read().frame
        cv2.putText(frame, f"Press T for take off and then G to start", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                    2)
        cv2.imshow("Drone Feed", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('t'):
            tello.takeoff()
            tello.move_up(50)
            while True:
                # Check for altitude control keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('g'):
                    return tello

    #return tello

def detect_marker(frame, detector):
    markerCorners, markerIds, _ = detector.detectMarkers(frame)
    frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
    return markerCorners, markerIds, frame

def calculate_angles(corners, marker_size, camera_matrix, dist_coeffs):
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
    if rvecs is not None and tvecs is not None:
        rvec = rvecs[0]
        tvec = tvecs[0]
        rotation_vector, _ = cv2.Rodrigues(rvec)
        yaw = np.degrees(rotation_vector[2][0])
        distance = np.linalg.norm(tvec)
        return yaw, distance
    else:
        return None, None


def approach(tello, detector):
    TARGET_DISTANCE = 1.5  # Target distance to land the drone (in meters)
    FORWARD_SPEED = 30  # Forward speed of the drone (adjust as needed)


    while True:
        frame = tello.get_frame_read().frame
        markerCorners, markerIds, frame = detect_marker(frame, detector)

        if check_landing_key():
            tello.land()
            return

        if markerIds is not None:
            _, distance = calculate_angles(markerCorners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)

            frame_center_x = frame.shape[1] // 2
            marker_center_x = (markerCorners[0][0][0][0] + markerCorners[0][0][1][0] + markerCorners[0][0][2][0] + markerCorners[0][0][3][0]) / 4

            yaw_error = marker_center_x - frame_center_x

            # Calculate the yaw speed based on the precise distance
            yaw_speed = yaw_error * 0.05

            print(f"yaw speed: {yaw_speed}")
            print(f"yaw error:  {yaw_error}")
            print(f"distance: {distance}")

            yaw = "unavailable"
            display_info_on_frame(frame, yaw, int(yaw_speed), distance, tello)

            # Rotate the drone based on the yaw error and adjusted yaw speed
            if yaw_speed > 4:
                tello.rotate_clockwise(int(yaw_speed))
            elif yaw_speed < -4:
                tello.rotate_counter_clockwise(abs(int(yaw_speed)))

            if distance > TARGET_DISTANCE:
                tello.move_forward(FORWARD_SPEED)
            else:
                tello.flip_back()
                tello.land()
                print("Landing...")
                return
        else:
            print("No ArUco marker detected. ...")
            return

        cv2.putText(frame, f"Battery life: {tello.get_battery()}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                    2)
        cv2.imshow("Drone Feed", frame)
        cv2.waitKey(100)

def rotate_drone_approach(tello, current_yaw, target_yaw):
    yaw_difference = target_yaw - current_yaw
    if yaw_difference > 0:
        tello.rotate_clockwise(int(yaw_difference))
    elif yaw_difference < 0:
        tello.rotate_counter_clockwise(int(abs(yaw_difference)))

#camera nomral is parallel marker normal
def rotate_drone(tello, yaw_degrees, target_yaw=0):
    print(f'precise rotation is active with yaw={yaw_degrees}')
    if yaw_degrees > target_yaw:
        tello.rotate_counter_clockwise(5)
    else:
        tello.rotate_clockwise(5)

def rotate_search_marker(tello, detector):
    rotation_angle = 25
    max_rotation_angle = 360
    clockwise = True

    while rotation_angle <= max_rotation_angle:
        if clockwise:
            tello.rotate_clockwise(rotation_angle)
        else:
            tello.rotate_counter_clockwise(rotation_angle)

        # Check if the marker is detected after each rotation
        frame = tello.get_frame_read().frame
        markerCorners, markerIds, frame = detect_marker(frame, detector)
        cv2.putText(frame, f"Searching for my friend Aruco :)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0),
                    2)
        cv2.putText(frame, f"Battery life: {tello.get_battery()}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                    2)
        cv2.imshow("Drone Feed", frame)
        cv2.waitKey(100)

        # Marker found, exit the function
        if markerIds is not None:
            return

        # Switch rotation direction and increase the angle
        clockwise = not clockwise
        rotation_angle += 55


def display_info_on_frame(frame, c_yaw, t_yaw, distance, tello):
    cv2.putText(frame, f"Yaw to Marker Normal: {c_yaw}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Yaw correction: {t_yaw}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Battery life: {tello.get_battery()}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Drone Feed", frame)
    cv2.waitKey(1)

def check_landing_key():
    key = cv2.waitKey(1) & 0xFF
    if key == LAND_KEY:
        print("Landing key pressed. Landing the drone...")
        return True
    return False

def align_with_marker(tello, detector):
    while True:
        frame = tello.get_frame_read().frame
        markerCorners, markerIds, frame = detect_marker(frame, detector)

        if check_landing_key():
            tello.end()
            return True

        if markerIds is not None:
            yaw, distance = calculate_angles(markerCorners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
            t_yaw = 'unavailable'
            display_info_on_frame(frame, yaw, t_yaw, distance, tello)

            # while abs(yaw) > YAW_THRESHOLD:
            #     rotate_drone(tello, yaw)
            #     frame = tello.get_frame_read().frame
            #     markerCorners, markerIds, frame = detect_marker(frame, detector)
            #
            #     if markerIds is None:
            #         continue
            #
            #     if check_landing_key():
            #         return True
            #
            #     yaw, distance = calculate_angles(markerCorners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
            #     if yaw is not None:
            #         t_yaw = 'unavailable'
            #         display_info_on_frame(frame, yaw, t_yaw, distance, tello)
            #     else:
            #         print("Invalid yaw, skipping this iteration.")
            #         continue
            #
            #     time.sleep(0.5)  # Adjust the delay as needed for smoother rotation

            print("Drone is facing the marker perpendicularly.")
            approach(tello, detector)
            #return False
        else:
            print("No ArUco marker detected. Rotating 360 degrees...")
            rotate_search_marker(tello, detector)

        cv2.putText(frame, f"Battery life: {tello.get_battery()}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Drone Feed", frame)
        cv2.waitKey(100)

def main():
    tello = connect_drone()
    #tello.takeoff()

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    quit_program = False

    while not quit_program:
        land_requested = align_with_marker(tello, detector)
        if land_requested:
            tello.land()
            tello.streamoff()
            tello.end()
            quit_program = True
        else:
            while not quit_program:
                frame = tello.get_frame_read().frame
                cv2.putText(frame, f"Adjustment complete!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 255, 0), 2)
                cv2.putText(frame, f"Battery life: {tello.get_battery()}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 255, 0), 2)
                cv2.imshow("Drone Feed", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    tello.land()
                    tello.streamoff()
                    tello.end()
                    quit_program = True

if __name__ == "__main__":
    main()