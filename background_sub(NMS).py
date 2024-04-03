import matplotlib.pyplot as plt
import cv2
import numpy as np
import logging
import time

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

def apply_morphology(fg_mask):
    kernel = np.ones((3, 3), np.uint8)
    fg_mask_morph = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
    fg_mask_morph = cv2.morphologyEx(fg_mask_morph, cv2.MORPH_CLOSE, kernel)
    return fg_mask_morph

def apply_nms(boxes, confidences, score_threshold=0.4, nms_threshold=0.4):
    indices = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold=score_threshold, nms_threshold=nms_threshold)
    return indices


def analyze_density(window_start_time, window_detections, video_start_time, window_data, window_size=5):
    current_time = time.time()
    if current_time - window_start_time >= window_size:
        window_end_time = current_time
        start_time_formatted = time.strftime("%H:%M:%S", time.localtime(window_start_time))
        end_time_formatted = time.strftime("%H:%M:%S", time.localtime(window_end_time))

        relative_start_time = window_start_time - video_start_time
        relative_end_time = window_end_time - video_start_time
        relative_start_time_formatted = time.strftime("%H:%M:%S", time.gmtime(relative_start_time))
        relative_end_time_formatted = time.strftime("%H:%M:%S", time.gmtime(relative_end_time))

        logger.info(f"Window: {start_time_formatted} - {end_time_formatted} | "
                    f"Relative: {relative_start_time_formatted} - {relative_end_time_formatted} | "
                    f"Detections: {window_detections}")

        window_data.append((relative_start_time, window_detections))

        window_start_time = window_end_time
        window_detections = 0
    return window_start_time, window_detections, window_data

def plot_density_graph(window_data):
    plt.figure(figsize=(10, 6))
    plt.plot([x[0] for x in window_data], [x[1] for x in window_data], marker='o')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Number of Detections')
    plt.title('Density Graph')
    plt.grid(True)
    plt.show()

def process_video(cap, background_subtractor, out, fps, window_size):
    frame_count = 0
    total_boxes_with_morph = 0
    window_start_time = time.time()
    window_detections = 0
    video_start_time = time.time()  # Get the video start time
    window_data = []  # Initialize an empty list to store window data

    delay = int(1000 / fps)  # Delay between frames in milliseconds

    while True:
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        fg_mask = background_subtractor.apply(gray_frame)

        fg_mask_morph = apply_morphology(fg_mask)

        contours_with_morph, _ = cv2.findContours(fg_mask_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes_with_morph = []
        confidences_with_morph = []
        for cnt in contours_with_morph:
            if cv2.contourArea(cnt) >= 5:
                x, y, w, h = cv2.boundingRect(cnt)
                boxes_with_morph.append([x, y, x + w, y + h])
                confidences_with_morph.append(cv2.contourArea(cnt))

        indices_with_morph = apply_nms(boxes_with_morph, confidences_with_morph)

        for i in indices_with_morph:
            x1, y1, x2, y2 = boxes_with_morph[i]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

        total_boxes_with_morph += len(indices_with_morph)
        window_detections += len(indices_with_morph)

        window_start_time, window_detections, window_data = analyze_density(window_start_time, window_detections,
                                                                            video_start_time, window_data, window_size)
        cv2.putText(frame, f"Frame: {frame_count}", (2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(frame, f"Total Boxes: {total_boxes_with_morph}", (2, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 0), 1)

        out.write(frame)

        cv2.imshow('With Morphological Operations', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        elapsed_time = time.time() - start_time
        if elapsed_time < delay / 1000:
            time.sleep(delay / 1000 - elapsed_time)
    plot_density_graph(window_data)  # Plot the density graph after video processing
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    logger.info(f"Total bounding boxes with morphological operations: {total_boxes_with_morph}")

# Open the video file
cap = cv2.VideoCapture('media/videos/1.mov')

# Create a background subtractor object
background_subtractor = cv2.createBackgroundSubtractorMOG2(history=1000, varThreshold=210, detectShadows=False)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#fps = 30
fps = int(cap.get(cv2.CAP_PROP_FPS))

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output_video.avi', fourcc, fps, (width, height))
out.set(cv2.VIDEOWRITER_PROP_QUALITY, 1.0)

# Process the video and perform real-time density analysis
process_video(cap, background_subtractor, out, fps, window_size=10)