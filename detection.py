import cv2
import numpy as np


def detect_orange_markers(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centers = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append((cx, cy))

    # Decision logic
    cone_detected = len(centers) > 0
    return cone_detected, centers




def detect_red_barricade(frame, depth_image, TRIGGERED, TRIGGER_DEPTH=9.0):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Detect red color in HSV space
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

        
        # Trigger maneuver if red barricade is within depth
        if not TRIGGERED and 0 < depth_m < TRIGGER_DEPTH:
            print("Target detected within depth threshold.")
            TRIGGERED = True
            break  # Only trigger on the first valid detection

    return TRIGGERED


def detect_lane(frame, model_lane, mask):
    results = model_lane(frame)
    
    if results[0].masks is None:
        return False, None  # No lane detected

    lane_mask = results[0].masks.data[0].cpu().numpy()
    lane_mask = (lane_mask * 255).astype(np.uint8)

    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)
    if mask.shape != lane_mask.shape:
        mask = cv2.resize(mask, (lane_mask.shape[1], lane_mask.shape[0]))

    masked = cv2.bitwise_and(lane_mask, lane_mask, mask=mask)
    
    return True, masked


stop_distance = 4.0  # meters

def detect_obstacle(frame, lane_mask, depth_image, model_obstacle):
    objects = model_obstacle(frame)
    obstacle_in_lane = False
    closest_distance = float('inf')

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

            depth_region = depth_image[max(0, y1):min(y2, depth_image.shape[0]),
                                       max(0, x1):min(x2, depth_image.shape[1])]
            if np.count_nonzero(depth_region) > 0:
                distance = np.min(depth_region[depth_region != 0]) * 0.001  # mm to meters
                print(f"[DEBUG] Object in lane at {distance:.2f} m")
                if distance < closest_distance:
                    closest_distance = distance
                if distance < stop_distance:
                    obstacle_in_lane = True
            else:
                print("[DEBUG] No valid depth data")
    return obstacle_in_lane, closest_distance

