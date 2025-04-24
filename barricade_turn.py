import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import time


# ------------------- DEPTH TRIGGER SETUP -------------------
TRIGGER_DEPTH =  8 #n meters
TRIGGERED = False  # To avoid repeated triggers

from cameras import initialize_realsense
from steer_encoder import send_encoder_value
from pid_controller import PIDController

pipeline, align = initialize_realsense()

print("Barricade_turn")

# ------------------- MAIN LOOP -------------------
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Red color mask (two ranges for hue)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) < 500:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w // 2, y + h // 2
            depth = depth_image[cy, cx]
            depth_m = depth / 1000.0
            print(f"Detected red object at {depth_m:.2f} meters")

            cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(color_image, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(color_image, f"{depth_m:.2f} m", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Trigger action based on depth threshold
            if not TRIGGERED and 0 < depth_m < TRIGGER_DEPTH:
                print("Target detected within depth threshold.")
                TRIGGERED = True
                print("anti clock wise")
                set_encoder_position(-5000)
                time.sleep(6)
                print("clock wise")
                set_encoder_position(5000)
                time.sleep(10)
                TRIGGERED = False

        cv2.imshow("Red Object Detection", color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()