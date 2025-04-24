from cameras import initialize_realsense, initialize_webcam
from obstacle_detection import detect_obstacles
from lane_steer import lane_following
from cone_steer import cone_following
from barricade_turn import barricade_handling

import numpy as np
import time

# Initialize cameras
pipeline, align = initialize_realsense()
webcam = initialize_webcam()

# State tracking
active_mode = 'lane'

try:
    while True:
        # --- Get RealSense frame ---
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        if not color_frame:
            continue
        rs_image = np.asanyarray(color_frame.get_data())

        # --- Get webcam frame for cone detection ---
        ret, webcam_frame = webcam.read()
        if not ret:
            print("Webcam read failed")
            continue

        # --- 1. Always run obstacle detection on RealSense ---
        detect_obstacles(rs_image)

        # --- 2. Run cone detection on webcam frame ---
        if cone_following(webcam_frame):
            if active_mode != 'cone':
                print("Switching to CONE steer (Webcam)")
                active_mode = 'cone'
            continue

        # --- 3. Run barricade detection on RealSense ---
        if active_mode != 'cone' and barricade_handling(rs_image):
            if active_mode != 'barricade':
                print("Switching to BARRICADE steer (RealSense)")
                active_mode = 'barricade'
            continue

        # --- 4. Default to lane following ---
        if active_mode != 'lane':
            print("Switching to LANE steer (RealSense)")
        active_mode = 'lane'
        lane_following(rs_image)

        time.sleep(0.01)

finally:
    pipeline.stop()
    webcam.release()
    print("Clean shutdown")




if detect_cone(webcam_frame):
    active_mode = 'cone'
    steer_cone(webcam_frame)
elif detect_barricade(rs_image, depth_image):
    active_mode = 'barricade'
    steer_barricade(rs_image)
elif detect_lane(rs_image, model_lane, mask):
    active_mode = 'lane'
    steer_lane(rs_image)



    lane_found, masked_lane = detect_lane(realsense_frame, model_lane, roi_mask)
if lane_found:
    steer_lane(masked_lane)

####

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


trap_vertices = np.array([[100, 480], [540, 480], [420, 300], [220, 300]], dtype=np.int32)
mask = np.zeros((480, 640), dtype=np.uint8)
cv2.fillPoly(mask, [trap_vertices], 255)




#####
while True:
    frame = get_webcam_frame()
    color_frame, depth_frame = get_realsense_frames()

    # 1. Cone detection from webcam
    cone_detected, cone_centers = detect_orange_markers(frame)

    if cone_detected:
        cone_steer(frame)
        continue  # Priority to cones, skip rest

    # 2. Barricade detection
    barricade_detected = detect_barricade(color_frame, depth_frame)

    if barricade_detected:
        barricade_turn()
        continue

    # 3. Lane detection
    lane_detected, lane_mask = detect_lane(frame)
    
    if lane_detected:
        lane_steer(lane_mask)
    else:
        # Default action (slow stop, alert, or continue forward with caution)
        handle_no_path_found()
