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




def detect_barricade(color_image, depth_image):
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Red mask
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
        if depth_m < 2.0:  # You can set this threshold as per your environment
            return True  # Barricade detected
    return False


def detect_lane(frame, model_lane, mask):
    results = model_lane(frame)
    
    if results[0].masks is None:
        return False, None  # No lane detected

    lane_mask = results[0].masks.data[0].cpu().numpy()
    lane_mask = (lane_mask * 255).astype(np.uint8)

    if mask.shape != lane_mask.shape:
        mask = cv2.resize(mask, (lane_mask.shape[1], lane_mask.shape[0]))

    masked = cv2.bitwise_and(lane_mask, lane_mask, mask=mask)
    
    return True, masked


