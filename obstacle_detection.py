import cv2
import torch
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs
import serial
import time

# ------------------- SERIAL CONFIG -------------------
VEHICLE_ARDUINO_PORT = 'COM8'
BAUD_RATE = 9600

print("Starting autonomous vehicle control system...")
print(f"Connecting to Arduino on {VEHICLE_ARDUINO_PORT}...")
vehicle_arduino = serial.Serial(VEHICLE_ARDUINO_PORT, BAUD_RATE, timeout=3)
print("Serial connection established")

# ------------------- VEHICLE CONTROL -------------------
brake_active = 0
maxspeed = 190
currentspeed = 0.0
increment = 5.0
go_count = 0
max_go_count = 0
stop_distance = 4  # meters

def send_vehicle_command(command):
    print(f"[CMD] {command.strip()}")
    vehicle_arduino.write(command.encode())

def vehicle_stop():
    global go_count, brake_active, currentspeed
    go_count = 0
    if currentspeed == 0:
        return
    send_vehicle_command('ACCEL_OFF\n')
    currentspeed = 0
    send_vehicle_command('BRAKE_ON\n')
    brake_active = 1
    time.sleep(0.5)
    send_vehicle_command('BRAKE_OFF\n')
    brake_active = 0
    print("[ACTION] Vehicle stopped")

def vehicle_go():
    global go_count, brake_active, currentspeed
    go_count += 1
    if go_count > max_go_count:
        if brake_active:
            send_vehicle_command('BRAKE_OFF\n')
            brake_active = 0
        if currentspeed < maxspeed:
            currentspeed += increment
            currentspeed = min(currentspeed, maxspeed)
        send_vehicle_command(f'ACCEL_{int(currentspeed)}\n')
        print(f"[ACTION] Vehicle speed updated to {currentspeed}")

# ------------------- MODEL SETUP -------------------
print("Loading models...")
model_lane = YOLO(r"C:\\Users\\hrish\\Autonomous-2024\\lane\\best.pt")
model_obstacle = YOLO('yolov8n.pt')

if torch.cuda.is_available():
    model_lane.to('cuda')
    model_obstacle.to('cuda')
    print("Models moved to GPU")

        # OBSTACLE DETECTION
        obstacle_in_lane = False
        closest_distance = float('inf')

        objects = model_obstacle(frame)
        for result in objects:
            for box, conf, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
                x1, y1, x2, y2 = map(int, box)
                print(f"[INFO] Detected object {int(cls)} at box ({x1},{y1}) to ({x2},{y2})")
                if lane_mask is not None:
                    object_lane_region = lane_mask[y1:y2, x1:x2]
                    total_area = (x2 - x1) * (y2 - y1)
                    overlap_area = np.count_nonzero(object_lane_region)
                    overlap_ratio = overlap_area / total_area if total_area > 0 else 0

                    if overlap_ratio < 0.1:
                        print("[DEBUG] Object outside lane")
                        continue

                # Depth check
                depth_region = depth_image[max(0, y1):min(y2, depth_image.shape[0]),
                                           max(0, x1):min(x2, depth_image.shape[1])]
                if np.count_nonzero(depth_region) > 0:
                    distance = np.min(depth_region[depth_region != 0]) * 0.001
                    print(f"[DEBUG] Object in lane at {distance:.2f} m")
                    if distance < closest_distance:
                        closest_distance = distance
                    if distance < stop_distance:
                        obstacle_in_lane = True
                else:
                    print("[DEBUG] No valid depth data")

        # VEHICLE CONTROL
        if obstacle_in_lane:
            print(f"[DETECT] Obstacle in lane at {closest_distance:.2f} m. Triggering stop.")
            vehicle_stop()
        else:
            vehicle_go()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"[ERROR] {e}")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle_arduino.close()
    print("[INFO] System shutdown complete")
