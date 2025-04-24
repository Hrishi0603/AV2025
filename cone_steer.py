import cv2
import numpy as np
import serial
import time
from scipy.interpolate import splprep, splev

# ------------------- SERIAL CONFIG -------------------
from cameras import initialize_webcam
from steer_encoder import send_encoder_value
from pid_controller import PIDController

print("Cone_steer")

print("Initializing PID controller, Kp=10000, Ki=20")
pid = PIDController(Kp=15000, Ki=10, Kd=15, max_output=5000, min_output=-5000)


# ------------------- SPLINE AND CONE DETECTION -------------------
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
    return centers

def compute_spline_path(centers):
    if len(centers) >= 2:
        centers = sorted(centers, key=lambda x: x[1])
        x, y = zip(*centers)
        k = min(3, len(centers) - 1)
        try:
            tck, u = splprep([x, y], s=0, k=k)
            u_fine = np.linspace(0, 1, 100)
            x_fine, y_fine = splev(u_fine, tck)
            return list(zip(x_fine, y_fine))
        except:
            return []
    return []

# ------------------- CAMERA SETUP -------------------
cap = initialize_webcam()

# ------------------- MAIN LOOP -------------------
try:
    while True:
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            print("Frame capture failed.")
            break

        centers = detect_orange_markers(frame)
        for i, (cx, cy) in enumerate(centers):
            cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
            cv2.putText(frame, f"Cone {i+1}", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 2)

        vehicle_cx = frame.shape[1] // 2
        cv2.line(frame, (vehicle_cx, frame.shape[0]), (vehicle_cx, 0), (255, 255, 255), 1)

        if len(centers) >= 2:
            spline_path = compute_spline_path(centers)
            if spline_path:
                spline_path_points = np.array(spline_path, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [spline_path_points], isClosed=False, color=(0, 255, 0), thickness=2)

                # Follow point at lower end (closest to car)
                cx, cy = spline_path[-1]
                error = -(vehicle_cx - cx) / (frame.shape[1] / 2)
                print(f"[Spline Mode] Target: ({cx:.1f}, {cy:.1f}) | Error: {error:.3f}")
            else:
                error = 0  # Fallback
                print("[Spline Mode] Spline failed, error=0")
        elif len(centers) == 1:
            cx, cy = centers[0]
            error = -(vehicle_cx - cx) / (frame.shape[1] / 2)
            print(f"[Closest Cone Mode] Cone: ({cx}, {cy}) | Error: {error:.3f}")
        elif len(centers) == 0:
            print("[No Cone] Skipping control...")
            cv2.imshow("Camera View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        delta_time = time.time() - start_time
        encoder_value = pid.calculate(error, delta_time)
        send_encoder_value(encoder_value)

        cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"[ERROR] {e}")
finally:
    cap.release()
    cv2.destroyAllWindows()
    
    print("Shutdown complete.")
