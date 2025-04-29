import cv2
import torch
import numpy as np
from ultralytics import YOLO


import time

# Import the RealSense setup function from cameras.py
from cameras import initialize_realsense
from steer_encoder import send_encoder_value
from pid_controller import PIDController
import logging

# # Configure logging
# logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


# ------------------- RealSense SETUP -------------------
# Use the RealSense setup function from cameras.py
pipeline, align = initialize_realsense()

print("Lane_steer")

print("Initializing PID controller, Kp=10000, Ki=20")
pid = PIDController(Kp=10000, Ki=20, Kd=15, max_output=5000, min_output=-5000)

# ------------------- MODEL & GLASBEY SETUP -------------------






# ------------------- MAIN LOOP -------------------
frame_count = 0
last_send_time = 0
last_sent_value = None
min_send_interval = 0.7  # Minimum 200 ms between sends
change_threshold = 200    # Minimum value change needed to trigger a send
    
def steer_lane(masked, start_time):
    centerline = []
    h, w = masked.shape
    center_x = w / 2
    for y in range(h):
        x_coords = np.where(masked[y, :] > 0)[0]
        if len(x_coords):
            centerline.append(np.mean(x_coords))

    if centerline:
        avg_centerline = np.mean(centerline)
        error = (avg_centerline - center_x) / (w / 2)
    else:
        error = 0

    delta_time = time.time() - start_time
    encoder_value = pid.calculate(error, delta_time)

    current_time = time.time()
    if (current_time - last_send_time >= min_send_interval and
        (last_sent_value is None or abs(encoder_value - last_sent_value) > change_threshold)):
        send_encoder_value(encoder_value)
        last_sent_value = encoder_value
        last_send_time = current_time
    else:
        print("[ENC] Skipping send: rate-limited or minor change")

    