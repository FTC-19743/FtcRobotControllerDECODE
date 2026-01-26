import cv2
import numpy as np
from collections import deque
import statistics

# --- Global Variables for State Maintenance ---
HISTORY_LENGTH = 5
intakeHistory = deque([(0, 0, 0)] * HISTORY_LENGTH, maxlen=HISTORY_LENGTH)
loadedHistory = deque([(0, 0, 0)] * HISTORY_LENGTH, maxlen=HISTORY_LENGTH)

# Mode Constants (as before)
MODE_INTAKE = 1  # OpMode sends 1 for INTAKE mode
MODE_LOADED = 2  # OpMode sends 2 for LOADED mode

NONE = 0
GREEN = 1
PURPLE = 2

left_result = NONE
middle_result = NONE
right_result = NONE

# LL Settings
# 640x480
# Exposure 2000
# Sensor Gain 10
# Red/Blue Balance 1300/1975  Defaults are 1200/1975

LOWER_GREEN = np.array([40, 100, 70])
UPPER_GREEN = np.array([100, 255, 255]) #was 80 before led strip

LOWER_PURPLE = np.array([118, 50, 50])
UPPER_PURPLE = np.array([135, 255, 255])

# ---  BLUR FACTOR ---
# Must be a positive, odd integer (e.g., 3, 5, 7, 9).
# Larger number = more blur. Start with 5 and tune as needed.
BLUR_KERNEL_SIZE = 11

# --- Clean up tape edges and other noise
# --- MORPHOLOGICAL KERNEL ---
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# Minimum Bounding Box Area (Adjust based on your target size/distance)
MIN_AREA_CONSTRAINT = 500.0 # TODO Might need to be different for intake and loaded? need small number for loaded sides

CROP_BOTTOM_PIXELS = 240
CROP_TOP_PIXELS = 100
CROP_RIGHT_PIXELS = 5

DOUBLE_BALL_MAX = 410
SINGLE_BALL_MAX = 210
LEFT_DOUBLE_THRESHOLD = 320
LEFT_THRESHOLD = 200
RIGHT_THRESHOLD = 400
LEFT_LOADED_THRESHOLD = 25
RIGHT_LOADED_THRESHOLD = 640-CROP_RIGHT_PIXELS - 30
MIDDLE_LOADED_CENTER = 300



def intakeDetect(boxes, color):
    global left_result, middle_result, right_result

    for x, y, w, h, area, cx, cy in boxes:
        if (w > DOUBLE_BALL_MAX):
            left_result = color
            middle_result = color
            right_result = color
        elif (w > SINGLE_BALL_MAX):
            if (cx < LEFT_DOUBLE_THRESHOLD):
                left_result = color
                middle_result = color
            else:
                middle_result = color
                right_result = color
        elif (cx< LEFT_THRESHOLD):
            left_result = color
        elif (cx > RIGHT_THRESHOLD):
            right_result = color
        else:
            middle_result = color
        #print(f"Box: Area {area} | w: {w} | X,Y: {cx}, {cy}")
        # print(f"MIN_AREA_CONSTRAINT {MIN_AREA_CONSTRAINT} | DOUBLE_BALL_MAX {DOUBLE_BALL_MAX} | SINGLE_BALL_MAX {SINGLE_BALL_MAX}")
    #print(f" Left: {left_result} | Middle : {middle_result} | Right: {right_result}")


def loadedDetect(boxes, color):
    global left_result, middle_result, right_result

    for x, y, w, h, area, cx, cy in boxes:
        if (x < LEFT_LOADED_THRESHOLD): # something on far left
            left_result = color
        if (x+w > RIGHT_LOADED_THRESHOLD): # something on far right
            right_result = color
        if (x < MIDDLE_LOADED_CENTER and x+w > MIDDLE_LOADED_CENTER ):
            middle_result = color
        #print(f"Box: Area {area} | L: {x} | R: {x+w} | Center: {cx}, {cy}")
    #print(f" Left: {left_result} | Middle : {middle_result} | Right: {right_result}")


def runPipeline(image, llrobot):
    global history
    global DOUBLE_BALL_MAX, SINGLE_BALL_MAX, LEFT_THRESHOLD, RIGHT_THRESHOLD, MIN_AREA_CONSTRAINT
    global left_result, middle_result, right_result

    # Initialize outputs
    largestContour = np.array([[]])

    # 1. Receive Input Mode from OpMode (llrobot[0])
    #current_mode = int(round(llrobot[0]))
    current_mode = MODE_INTAKE # hardcode to intake only for now

    # Black out the bottom region on the original image
    height, width = image.shape[:2] # Get image height and width
    crop_height_start = height - CROP_BOTTOM_PIXELS
    crop_width_start = width - CROP_RIGHT_PIXELS

    # Crop rear wall
    image[crop_height_start:height, 0:width] = 0

    kernel = (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE)
    image[0:CROP_TOP_PIXELS, 0:width] = 0 # crop top
    image[CROP_TOP_PIXELS:crop_height_start, crop_width_start:width] = 0 # crop far right

    # Apply Gaussian Blur to the HSV image
    # This reduces noise and helps smooth the edges of the blobs.
    img_blurred = cv2.GaussianBlur(image, kernel, 0)

    # Convert to HSV color space once
    img_hsv = cv2.cvtColor(img_blurred, cv2.COLOR_BGR2HSV)

    # Create masks using the blurred HSV image
    mask_green = cv2.inRange(img_hsv, LOWER_GREEN, UPPER_GREEN)
    mask_purple = cv2.inRange(img_hsv, LOWER_PURPLE, UPPER_PURPLE)

    # clean up purple mask with erosion and dilation
    mask_purple = cv2.erode(mask_purple, MORPH_KERNEL, iterations=1)
    mask_purple = cv2.dilate(mask_purple, MORPH_KERNEL, iterations=1)

    # Dictionaries to hold the bounding boxes: {'color': [(x, y, w, h), ...]}
    green_boxes = []
    purple_boxes = []

    # Process Green Blobs
    contours_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours_g:
        area = cv2.contourArea(c)
        if area >= MIN_AREA_CONSTRAINT:
            x, y, w, h = cv2.boundingRect(c)
            area = w*h
            cx = x + (w // 2)
            cy = y + (h // 2)
            green_boxes.append((x, y, w, h, area, cx, cy))
            cv2.rectangle(img_blurred, (x, y), (x + w, y + h), (0, 255, 0), 2) # Green box on image

    # Process Purple Blobs
    contours_p, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours_p:
        area = cv2.contourArea(c)
        if area >= MIN_AREA_CONSTRAINT:
            x, y, w, h = cv2.boundingRect(c)
            area = w*h
            cx = x + (w // 2)
            cy = y + (h // 2)
            purple_boxes.append((x, y, w, h, area, cx, cy))
            cv2.rectangle(img_blurred, (x, y), (x + w, y + h), (255, 0, 255), 2) # Purple box on image

    # Update the Limelight's largest contour for standard tx/ty/ta outputs
    all_contours = contours_g + contours_p
    if all_contours: # check that all_contours isn't empty
        largestContour = max(all_contours, key=cv2.contourArea)

    num_green = len(green_boxes)
    num_purple = len(purple_boxes)
    # Print the counts and the current mode for debugging
    #print(f"Mode: {current_mode} | Green Detections: {num_green} | Purple Detections: {num_purple}")

    left_result = NONE
    middle_result = NONE
    right_result = NONE
    # Run through detections
    intakeDetect (green_boxes, GREEN)
    intakeDetect (purple_boxes, PURPLE)
    intakeHistory.append((left_result, middle_result, right_result))

    left_result = NONE
    middle_result = NONE
    right_result = NONE
    # Run through detections
    loadedDetect (green_boxes, GREEN)
    loadedDetect (purple_boxes, PURPLE)
    loadedHistory.append((left_result, middle_result, right_result))

    # Calculate most common values in the temporal history
    left_values = [res[0] for res in intakeHistory]
    middle_values = [res[1] for res in intakeHistory]
    right_values = [res[2] for res in intakeHistory]
    try:
        most_common_left_intake = statistics.mode(left_values)
        most_common_middle_intake = statistics.mode(middle_values)
        most_common_right_intake = statistics.mode(right_values)
    except statistics.StatisticsError:
        # Fallback to the latest value if all are unique
        most_common_left_intake = left_values[-1]
        most_common_middle_intake = middle_values[-1]
        most_common_right_intake = right_values[-1]

    # Calculate most common values in the temporal history
    left_values = [res[0] for res in loadedHistory]
    middle_values = [res[1] for res in loadedHistory]
    right_values = [res[2] for res in loadedHistory]
    try:
        most_common_left_loaded = statistics.mode(left_values)
        most_common_middle_loaded = statistics.mode(middle_values)
        most_common_right_loaded = statistics.mode(right_values)
    except statistics.StatisticsError:
        # Fallback to the latest value if all are unique
        most_common_left_loaded = left_values[-1]
        most_common_middle_loaded = middle_values[-1]
        most_common_right_loaded = right_values[-1]

    print(f"INTAKE L: {most_common_left_intake} | M : {most_common_middle_intake} | R: {most_common_right_intake} LOADED L: {most_common_left_loaded} | M : {most_common_middle_loaded} | R: {most_common_right_loaded}")

    # Populate the llpython array with the smoothed results and status
    llpython = [0.0] * 8

    # llpython[0]: Signal valid data
    llpython[0] = 23

    # llpython[1]: Flag (1.0 if targets found, 0.0 otherwise)
    llpython[1] = float(1.0) if (green_boxes or purple_boxes) else float(0.0)

    # llpython[2], [3], [4]: Smoothed (most common) results
    llpython[2] = float(most_common_left_intake)
    llpython[3] = float(most_common_middle_intake)
    llpython[4] = float(most_common_right_intake)
    llpython[5] = float(most_common_left_loaded)
    llpython[6] = float(most_common_middle_loaded)
    llpython[7] = float(most_common_right_loaded)

    # Return the required tuple
    #return largestContour, mask_purple, llpython
    return largestContour, img_blurred, llpython