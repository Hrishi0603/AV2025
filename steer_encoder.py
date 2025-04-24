import serial
import time

LANE_ARDUINO_PORT = 'COM8'      # For steering control
BAUD_RATE = 9600

# ------------------- SERIAL SETUP -------------------
print(f"Attempting to connect to lane Arduino on {LANE_ARDUINO_PORT}...")
lane_arduino = serial.Serial(LANE_ARDUINO_PORT, BAUD_RATE, timeout=1)
print('Serial connections established successfully!')



def send_encoder_value(value):
    # Limit encoder value to ±5000
    original_value = value
    limited_value = max(min(value, 5000), -5000)
    
    if original_value != limited_value:
        print(f"[ENC] Value limited from {original_value:.2f} to {limited_value:.2f}")
    
    try:
        command = f"SET: {int(limited_value)}\n"
        print(f"[ENC] Sending command: {command.strip()}")
        lane_arduino.write(command.encode('utf-8'))
        
        print("[ENC] Waiting for Arduino response...")
        response = lane_arduino.readline().decode('utf-8', errors='ignore').strip()
        if response:
            print(f"[ENC] Arduino Response: {response}")
        else:
            print("[ENC] No response received from Arduino")
    except Exception as e:
        print(f"[ENC] Serial error: {e}")


