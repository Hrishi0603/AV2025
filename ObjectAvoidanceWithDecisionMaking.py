import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

# ------------------- SERIAL CONFIG -------------------
LANE_ARDUINO_PORT = 'COM10'   # For steering control
BAUD_RATE = 9600
print("Starting lane detection with obstacle avoidance (webcam)...")
ser = serial.Serial(LANE_ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(0.2)

# ------------------- STEERING CONTROL -------------------
def send_encoder_value(value):
    """Send encoder target via serial in the format 'SET: <value>'"""
    original = value
    limited  = max(min(value, 5000), -5000)
    if original != limited:
        print(f"[ENC] Limited {original:.2f} to {limited:.2f}")
    cmd = f"SET: {int(limited)}\n"
    print(f"[ENC] Sending {cmd.strip()}")
    ser.write(cmd.encode('utf-8'))
    resp = ser.readline().decode('utf-8', errors='ignore').strip()
    if resp:
        print(f"[ENC] Arduino: {resp}")

# ------------------- PID CONTROLLER -------------------
class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output=None, min_output=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.max_output, self.min_output = max_output, min_output
        self.integral = 0.0
        self.prev_error = 0.0

    def calculate(self, error, delta_t):
        p = self.Kp * error
        self.integral += error * delta_t
        i = self.Ki * self.integral
        d = self.Kd * ((error - self.prev_error) / delta_t) if delta_t > 0 else 0.0
        self.prev_error = error
        out = p + i + d
        if self.max_output is not None:
            out = min(out, self.max_output)
        if self.min_output is not None:
            out = max(out, self.min_output)
        return out

# initialize PID
pid = PIDController(Kp=10000, Ki=20, Kd=15, max_output=5000, min_output=-5000)

# ------------------- MODEL SETUP -------------------
lane_model = YOLO(r"C:\Users\dosid\Data Science\Autonomous Vehicle\Object Avoidance Sem2\best.pt")
obj_model  = YOLO("yolov8n.pt")
allowed_classes = {'person','bird','cat','dog','horse','cow','chair','backpack'}

# ------------------- ROI MASK -------------------
roi_vertices = np.array([[100,480],[540,480],[420,300],[220,300]], dtype=np.int32)
roi_mask     = np.zeros((480,640), dtype=np.uint8)
cv2.fillPoly(roi_mask, [roi_vertices], 255)


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

# ------------------- HELPER FUNCTIONS -------------------
def remove_obstacle_regions(lane_mask, bboxes):
    m = lane_mask.copy()
    for x,y,w,h in bboxes:
        x1,y1 = max(0,int(x)), max(0,int(y))
        x2,y2 = min(m.shape[1],int(x+w)), min(m.shape[0],int(y+h))
        m[y1:y2, x1:x2] = 0
    return m


def get_free_region_center(lane_mask, obs_center):
    h, w = lane_mask.shape
    mid = int(obs_center)
    left, right = lane_mask[:, :mid], lane_mask[:, mid:]
    cl, cr = cv2.countNonZero(left), cv2.countNonZero(right)
    if cl >= cr and cl > 0:
        xs = np.where(left>0)[1]
        return int(np.mean(xs))
    elif cr > 0:
        xs = np.where(right>0)[1]
        return int(np.mean(xs)) + mid
    else:
        return w//2


def compute_lane_center(mask):
    pts = []
    h, w = mask.shape
    for y in range(h):
        xs = np.where(mask[y]>0)[0]
        if xs.size:
            pts.append((int(xs.mean()), y))
    avg_x = int(np.mean([p[0] for p in pts])) if pts else w//2
    return pts, avg_x

# ------------------- MAIN LOOP -------------------
frame_idx = 0
try:
    while True:
        frame_idx += 1
        start = time.time()

        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        frame = cv2.resize(frame, (640,480))

        # lane mask
        lr = lane_model(frame)
        if lr and lr[0].masks is not None:
            m = (lr[0].masks.data[0].cpu().numpy()*255).astype(np.uint8)
            m = cv2.resize(m, frame.shape[1::-1])
            lane_mask = cv2.bitwise_and(m, roi_mask)
        else:
            lane_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        # object detection
        obs_boxes = []
        orr = obj_model(frame)
        if orr and orr[0].boxes is not None:
            for b in orr[0].boxes:
                cls = int(b.cls[0]); name = obj_model.names[cls]
                if name in allowed_classes:
                    x1,y1,x2,y2 = b.xyxy.cpu().numpy()[0]
                    obs_boxes.append([x1,y1,x2-x1,y2-y1])

        # free region and modified mask
        if obs_boxes:
            x,y,w,h = obs_boxes[0]
            obs_c = x + w/2
            mod_mask = remove_obstacle_regions(lane_mask, [obs_boxes[0]])
            free_c   = get_free_region_center(mod_mask, obs_c)
        else:
            mod_mask, free_c = lane_mask.copy(), compute_lane_center(lane_mask)[1]

        # visuals
        out = frame.copy()
        pts, _ = compute_lane_center(mod_mask)
        for p in pts: cv2.circle(out, p, 1, (0,255,0), -1)
        cv2.line(out, (int(free_c),0), (int(free_c),480), (255,0,0), 2)

        # steering control
        err = (free_c - 320) / 320
        dt  = time.time() - start
        cmd = pid.calculate(err, dt)
        send_encoder_value(cmd)

        # overlays
        cv2.putText(out, f"Err: {err:.2f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        cv2.putText(out, f"Cmd: {cmd:.0f}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        for x,y,w,h in obs_boxes:
            cv2.rectangle(out, (int(x),int(y)), (int(x+w),int(y+h)), (0,255,255), 2)

        cv2.imshow("Lane Mask", lane_mask)
        cv2.imshow("Mod Lane Mask", mod_mask)
        cv2.imshow("Output", out)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        fps = 1.0 / (time.time() - start)
        print(f"Frame {frame_idx} @ {fps:.2f} FPS")

except Exception as e:
    print(f"Error: {e}")

finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
