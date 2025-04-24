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

def initialize_webcam(index=0):
    """
    Initializes the webcam and returns the capture object.
    """
    print("Initializing webcam...")
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")
    print("Webcam initialized successfully")
    return cap
