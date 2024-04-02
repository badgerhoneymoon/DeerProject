#Motion detection for Lepton 3.1R through background subtraction algorithm
import cv2
import numpy as np
import logging
import time

#sweet spot: history=1000, varThreshold=210, kernel =3*3, score_threshold=0.4, nms_threshold=0.4, cv2.contourArea(cnt) >= 5)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Open the video file
cap = cv2.VideoCapture('media/videos/1.mov')

# Create a background subtractor object
background_subtractor = cv2.createBackgroundSubtractorMOG2(history=1000, varThreshold=210, detectShadows=False)

# Assuming a screen resolution of 120x160
screen_width = 120
screen_height = 160

# Create named windows with specific flags
cv2.namedWindow('With Morphological Operations', cv2.WINDOW_NORMAL)

frame_count = 0
total_boxes_with_morph = 0

fps = 30  # Desired frame rate for slow motion
delay = int(1000 / fps)  # Delay between frames in milliseconds

# Get the video properties
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

# # Create a VideoWriter object to save the output video
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output_video.avi', fourcc, fps, (width, height))
out.set(cv2.VIDEOWRITER_PROP_QUALITY, 1.0)

while True:
    start_time = time.time()
    # Read the next frame
    ret, frame = cap.read()
    if not ret:
        break
    frame_count += 1

    # Convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply background subtraction
    fg_mask = background_subtractor.apply(gray_frame)

    # Apply morphological operations to remove noise
    kernel = np.ones((3, 3), np.uint8)
    fg_mask_morph = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
    fg_mask_morph = cv2.morphologyEx(fg_mask_morph, cv2.MORPH_CLOSE, kernel)

    # Count bounding boxes with morphological operations
    contours_with_morph, _ = cv2.findContours(fg_mask_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes_with_morph = []
    confidences_with_morph = []
    for cnt in contours_with_morph:
        if cv2.contourArea(cnt) >= 5:
            x, y, w, h = cv2.boundingRect(cnt)
            boxes_with_morph.append([x, y, x + w, y + h])
            confidences_with_morph.append(cv2.contourArea(cnt))

    # Perform non-maximum suppression
    indices_with_morph = cv2.dnn.NMSBoxes(boxes_with_morph, confidences_with_morph, score_threshold=0.4,
                                          nms_threshold=0.4)

    # Draw bounding boxes on the original colorful frame after non-maximum suppression
    for i in indices_with_morph:
        x1, y1, x2, y2 = boxes_with_morph[i]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

    total_boxes_with_morph += len(indices_with_morph)

    # Put the frame number and total bounding boxes as text on the original colorful frame with smaller font size
    cv2.putText(frame, f"Frame: {frame_count}", (2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.putText(frame, f"Total Boxes: {total_boxes_with_morph}", (2, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (0, 255, 0), 1)

    # Write the frame with bounding boxes to the output video
    out.write(frame)

    # Display the original colorful frame with detection results
    cv2.imshow('With Morphological Operations', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Limit the frame rate for slow motion
    elapsed_time = time.time() - start_time
    if elapsed_time < delay / 1000:
        time.sleep(delay / 1000 - elapsed_time)

# Release the video capture and writer objects
cap.release()
out.release()
cv2.destroyAllWindows()

# Log the total number of bounding boxes
logger.info(f"Total bounding boxes with morphological operations: {total_boxes_with_morph}")