import time
from steer_encoder import send_encoder_value

def perform_barricade_maneuver():
    print("[ACTION] anti clockwise turn")
    send_encoder_value(-5000)
    time.sleep(6)
    print("[ACTION] clockwise turn")
    send_encoder_value(5000)
    time.sleep(10)
