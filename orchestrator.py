import cv2
from cameras import initialize_realsense, initialize_webcam, get_sensor_frames
from detection import detect_lane, detect_orange_markers, detect_red_barricade, detect_obstacle
from lane_steer import steer_lane
from cone_steer import cone_following
from barricade_turn import perform_barricade_maneuver
from throttle_braking import vehicle_go, vehicle_stop
from ultralytics import YOLO
import numpy as np
import time


TRIGGERED = False
frame_count = 0

model_lane = YOLO(r"best.pt")
model_obstacle = YOLO('yolov8n.pt')

pipeline, align = initialize_realsense()
webcam = initialize_webcam()

while True:

    frame, depth_image, hsv, webcam_frame, mask = get_sensor_frames(pipeline, align, webcam)

    # Waitying for the next frame
    if frame is None:
        continue

    frame_count += 1
    print(f"\n--- Frame {frame_count} ---")
    start_time = time.time()

    # Lane Detection
    lane_detected, lane_mask = detect_lane(frame, model_lane, mask, start_time)

    # Cone Detection
    cone_detected, cone_centers = detect_orange_markers(webcam_frame)

    # Obstacle Detection
    obstacle_in_lane, closest_distance = detect_obstacle(frame, lane_mask, depth_image, model_obstacle)

    # Barricade Detection and Maneuver
    TRIGGERED = detect_red_barricade(frame, depth_image, TRIGGERED)
    if TRIGGERED:
        print("[INFO] Barricade maneuver triggered.")
        vehicle_go()
        perform_barricade_maneuver()
        TRIGGERED = False
        continue

    elif cone_detected and cone_centers:
        print("[INFO] Cone detected → Executing cone steering.")
        cone_following(webcam_frame, cone_centers)
        vehicle_go()
        continue


    # Obstacle Handling
    elif obstacle_in_lane:
        print(f"[INFO] Obstacle in lane at {closest_distance:.2f}m → Stopping vehicle.")
        vehicle_stop()
        continue

    # Cone Steering

    # Lane Steering
    elif lane_detected and lane_mask is not None:
        print("[INFO] Lane detected → Executing lane steering.")
        steer_lane(lane_mask)
        vehicle_go
        continue

    # Default fallback
    print("[INFO] No valid path detected → stopping.")
    vehicle_stop()
