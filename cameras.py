# camera_setup.py
import cv2
import pyrealsense2 as rs
import numpy as np


def initialize_realsense():
    """
    Initializes the RealSense camera and returns the pipeline, align object, and first frames.
    """
    print("Initializing RealSense camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    print("RealSense camera initialized successfully")
    return pipeline, align


# def initialize_webcam(index=2):
#     """
#     Initializes the webcam and returns the capture object.
#     """
#     print("Initializing webcam...")
#     cap = cv2.VideoCapture(index)
#     if not cap.isOpened():
#         raise RuntimeError("Could not open webcam")
#     print("Webcam initialized successfully")
#     return cap

# In cameras.py -> initialize_webcam
def initialize_webcam(index=6): # Use your confirmed index
    print(f"Initializing webcam at index {index} using V4L2 backend...")
    # Try explicitly using the V4L2 backend
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
         # If V4L2 fails, try default again as fallback
         print(f"WARN: V4L2 backend failed for index {index}. Trying default...")
         cap = cv2.VideoCapture(index) # Fallback to default
         if not cap.isOpened():
             print(f"ERROR: Could not open webcam at index {index} with default backend either.")
             # ... rest of error handling ...
             raise RuntimeError(f"Could not open webcam at index {index}")

    print(f"Webcam at index {index} initialized successfully")
    return cap


def get_sensor_frames(pipeline, align, webcam):
    """
    Captures and prepares frames from RealSense and webcam for detection modules.

    Returns:
        frame (np.ndarray): Color frame from RealSense.
        depth_image (np.ndarray): Depth frame from RealSense.
        hsv (np.ndarray): HSV converted frame (for color detection).
        webcam_frame (np.ndarray): Frame from the webcam.
        mask (np.ndarray): Binary mask for lane detection.
    """
    # --- Get RealSense frames ---
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame or not depth_frame:
        print("WARNING: Missing color or depth frame from RealSense.")
        return None, None, None, None, None

    frame = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Lane mask preparation (trapezoidal mask) ---
    trap_vertices = np.array(
        [[100, 480], [540, 480], [420, 300], [220, 300]], dtype=np.int32
    )
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    cv2.fillPoly(mask, [trap_vertices], 255)

    # --- Get webcam frame for cone detection ---
    ret, webcam_frame = webcam.read()
    if not ret:
        print("Webcam read failed.")
        return None, None, None, None, None

    return frame, depth_image, hsv, webcam_frame, mask
