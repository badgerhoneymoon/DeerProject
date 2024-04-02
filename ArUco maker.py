import cv2
import os

# Create a dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Create a directory to store the marker images
output_dir = 'aruco_markers'
os.makedirs(output_dir, exist_ok=True)

# Marker size
marker_size = 1000

# Generate and save ArUco markers
for marker_id in range(1, 5):  # Generate markers with IDs 1-4
    marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
    cv2.imwrite(os.path.join(output_dir, f'marker_{marker_id}.png'), marker_image)

print(f"ArUco markers saved in the '{output_dir}' directory.")