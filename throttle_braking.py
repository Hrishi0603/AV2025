import time
import serial

VEHICLE_ARDUINO_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200

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

