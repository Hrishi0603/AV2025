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
print("Loading AI models...")
print("Loading lane detection model from E:/Autonomous_vehicle_project/lane/best.pt")
model_lane = YOLO(r"C:\Users\hrish\Autonomous-2024\lane\best.pt")


print("Setting up region of interest mask...")
trap_vertices = np.array([[100, 480], [540, 480], [420, 300], [220, 300]], dtype=np.int32)
mask = np.zeros((480, 640), dtype=np.uint8)
cv2.fillPoly(mask, [trap_vertices], 255)
print(f"Mask created with shape {mask.shape}, vertices at {trap_vertices}")

# ------------------- MAIN LOOP -------------------
try:
    frame_count = 0
    last_send_time = 0
    last_sent_value = None
    min_send_interval = 0.7  # Minimum 200 ms between sends
    change_threshold = 200    # Minimum value change needed to trigger a send


    

    while True:
        frame_count += 1
        print(f"\n--- Frame {frame_count} ---")
        start_time = time.time()
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            print("WARNING: Missing color or depth frame - skipping")
            continue

        frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Process frame with YOLO model (assuming model_lane is defined elsewhere)
        results = model_lane(frame)
        lane_mask = None
        if results[0].masks is not None:
            lane_mask = results[0].masks.data[0].cpu().numpy()
            lane_mask = (lane_mask * 255).astype(np.uint8)

            if mask.dtype != np.uint8:
                mask = mask.astype(np.uint8)
            if mask.shape != lane_mask.shape:
                mask = cv2.resize(mask, (lane_mask.shape[1], lane_mask.shape[0]))

            masked = cv2.bitwise_and(lane_mask, lane_mask, mask=mask)

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

        else:
            print("No lane mask found in model results")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quit key pressed, exiting loop")
            break

        loop_time = time.time() - start_time
        print(f"Complete frame processing time: {loop_time:.4f}s ({1/loop_time:.2f} FPS)")

except Exception as e:
    print(f"ERROR in main loop: {e}")

finally:
    print("Shutting down...")
    print("Stopping RealSense pipeline...")
    pipeline.stop()
    print("Closing OpenCV windows...")
    cv2.destroyAllWindows()
    print("Closing serial connections...")
    print("System shutdown complete")


    def steer_lane(masked):
    centerline = []
    h, w = masked.shape
    center_x = w / 2

    for y in range(h):
        x_coords = np.where(masked[y, :] > 0)[0]
        if len(x_coords):
            centerline.append(np.mean(x_coords))

    if centerline:
        avg_centerline = np.mean(centerline)
        error = (avg_centerline - center_x) / (w / 2)  # Normalized error
        print(f"Lane center error: {error:.2f}")
        send_encoder_signal(error)
    else:
        print("No valid lane centerline found.")