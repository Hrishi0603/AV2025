import cv2
import numpy as np
from scipy.interpolate import splprep, splev
import time

# ------------------- SERIAL CONFIG -------------------
from cameras import initialize_webcam
from steer_encoder import send_encoder_value
from pid_controller import PIDController

print("Cone_steer")

print("Initializing PID controller, Kp=10000, Ki=20")
pid = PIDController(Kp=15000, Ki=10, Kd=15, max_output=5000, min_output=-5000)


# ------------------- SPLINE AND CONE DETECTION -------------------


def compute_spline_path(centers):
    if len(centers) >= 2:
        centers = sorted(centers, key=lambda x: x[1])  # Sort from bottom to top
        x, y = zip(*centers)
        k = min(3, len(centers) - 1)
        try:
            tck, u = splprep([x, y], s=0, k=k)
            u_fine = np.linspace(0, 1, 100)
            x_fine, y_fine = splev(u_fine, tck)
            return list(zip(x_fine, y_fine))
        except Exception as e:
            print(f"[Spline Error] {e}")
            return []
    return []

def cone_steer(frame, centers, pid, send_encoder_value):
    start_time = time.time()

    # Visualize cone centers
    for i, (cx, cy) in enumerate(centers):
        cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
        cv2.putText(frame, f"Cone {i+1}", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

    vehicle_cx = frame.shape[1] // 2
    cv2.line(frame, (vehicle_cx, frame.shape[0]), (vehicle_cx, 0), (255, 255, 255), 1)

    error = 0
    if len(centers) >= 2:
        spline_path = compute_spline_path(centers)
        if spline_path:
            spline_path_points = np.array(spline_path, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [spline_path_points], isClosed=False, color=(0, 255, 0), thickness=2)

            cx, cy = spline_path[-1]  # Closest point to vehicle
            error = -(vehicle_cx - cx) / (frame.shape[1] / 2)
            print(f"[Spline Mode] Target: ({cx:.1f}, {cy:.1f}) | Error: {error:.3f}")
        else:
            print("[Spline Mode] Spline failed, error=0")

    elif len(centers) == 1:
        cx, cy = centers[0]
        error = -(vehicle_cx - cx) / (frame.shape[1] / 2)
        print(f"[Closest Cone Mode] Cone: ({cx}, {cy}) | Error: {error:.3f}")

    elif len(centers) == 0:
        print("[No Cone] Skipping control...")
        return

    delta_time = time.time() - start_time
    encoder_value = pid.calculate(error, delta_time)
    send_encoder_value(encoder_value)
