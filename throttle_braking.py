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


def handle_obstacle_decision(obstacle_in_lane, closest_distance, stop_distance=4.0):
    """
    Handles vehicle control based on obstacle detection results.

    Parameters:
        obstacle_in_lane (bool): True if an object is detected overlapping the lane.
        closest_distance (float): Distance (in meters) to the closest detected object.
        stop_distance (float): Threshold distance (in meters) to trigger stopping. Default is 4.0m.
    """
    if obstacle_in_lane and closest_distance < stop_distance:
        print(f"[DECISION] Obstacle detected within {closest_distance:.2f} m. Stopping vehicle.")
        vehicle_stop()
    else:
        print("[DECISION] Path is clear. Proceeding forward.")
        vehicle_go()
