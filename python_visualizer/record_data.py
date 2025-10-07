import serial
import time

# Configure Serial port (adjust COM port and baud rate)
PORT = 'COM3'  # Change to your ESP32's COM port (e.g., /dev/ttyUSB0 on Linux/Mac)
BAUD_RATE = 921600

# Open Serial connection
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {PORT} at {BAUD_RATE} baud")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

OUTPUT_FILE = "imu_data_2025-09-06.txt"

# Open file for writing
with open(OUTPUT_FILE, 'w') as f:
    f.write("Timestamp(ms),GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ\n")  # Header

    start_time = time.time()
    while time.time() - start_time < 60:  # Record for 60 seconds
        line = ser.readline()  # Read raw bytes
        try:
            decoded_line = line.decode('utf-8').strip()
            if decoded_line:
                if "Recording complete" in decoded_line:
                    print(decoded_line)
                    break
                elif "Read failed" in decoded_line:
                    print(decoded_line)
                else:
                    f.write(decoded_line + "\n")
                    print(decoded_line)  # Optional: print to console
        except UnicodeDecodeError as e:
            print(f"Decode error: {e}, skipping line: {line.hex()}")  # Log raw bytes in hex

print(f"Data saved to {OUTPUT_FILE}")

# Close Serial connection
ser.close()