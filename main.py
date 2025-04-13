import serial
import time
import struct
from threading import Thread, Lock
import signal
import sys

# LIDAR Serial Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Change to your LIDAR serial port
LIDAR_BAUDRATE = 460800

# Camera Serial Configuration
CAMERA_PORT = "/dev/ttyUSB1"  # Change to your ESP-EYE serial port
CAMERA_BAUDRATE = 115200

# Data storage
check_angles = [i for i in range(0, 360, 15)]  # 0°, 15°, ..., 345°
current_array = [0] * len(check_angles)
old_array = [0] * len(check_angles)
count = 0

# Thread control
lidar_lock = Lock()
camera_lock = Lock()
running = True  # Shared flag for clean shutdown

# LIDAR Commands
CMD_STOP = b'\xA5\x25'
CMD_SCAN = b'\xA5\x20'

def signal_handler(sig, frame):
    global running
    print("\n[INFO] Caught interrupt signal. Stopping program...")
    running = False

# Function to parse LIDAR packet
def parse_packet(packet):
    if len(packet) != 5:
        return None
    b0, b1, b2, b3, b4 = struct.unpack('<BBBBB', packet)

    # Validate sync bits
    start_flag = b0 & 0x01
    inverted_start_flag = (b0 >> 1) & 0x01
    if start_flag != (~inverted_start_flag & 0x01):
        return None

    # Extract quality, angle, distance
    quality = b0 >> 2
    angle_q6 = ((b2 << 7) | (b1 >> 1))
    angle = angle_q6 / 64.0
    dist_q2 = (b4 << 8) | b3
    distance = dist_q2 / 4.0

    return angle, distance, quality

# Function to get descriptor
def get_descriptor(ser):
    descriptor = ser.read(7)
    if descriptor[:2] != b'\xA5\x5A':
        raise Exception("Failed to receive scan descriptor.")

# LIDAR scan thread
def scan_lidar():
    global count, current_array, old_array, running

    with serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1) as ser:
        print("[INFO] Starting LIDAR scan...")

        ser.write(CMD_STOP)
        time.sleep(0.1)
        ser.reset_input_buffer()

        ser.write(CMD_SCAN)
        get_descriptor(ser)
        print("[INFO] LIDAR scan started.")

        try:
            while running:
                packet = ser.read(5)
                if len(packet) != 5:
                    continue

                result = parse_packet(packet)
                if result is None:
                    continue

                angle, distance, quality = result
                angle_rounded = round(angle)

                # Loosen angle match to ±2°
                target_angle = check_angles[count]
                if abs(angle_rounded - target_angle) <= 2:
                    with lidar_lock:
                        current_array[count] = round(distance)
                        count += 1

                        if count >= len(check_angles):
                            count = 0
                            old_array = current_array[:]
                            print("[INFO] Full LIDAR frame captured:", old_array)

        except Exception as e:
            print("[ERROR] LIDAR thread exception:", e)
        finally:
            print("[INFO] Stopping LIDAR...")
            ser.write(CMD_STOP)
            time.sleep(0.1)
            ser.close()

# Camera thread
def read_camera_data():
    global running
    with serial.Serial(CAMERA_PORT, CAMERA_BAUDRATE, timeout=1) as ser:
        print("[INFO] Reading Camera Data from ESP-EYE...")
        try:
            while running:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    with camera_lock:
                        print("[Camera Data] ", line)
        except Exception as e:
            print("[ERROR] Camera thread exception:", e)
        finally:
            print("[INFO] Stopping Camera...")
            ser.close()

# Main
def main():
    signal.signal(signal.SIGINT, signal_handler)

    lidar_thread = Thread(target=scan_lidar)
    camera_thread = Thread(target=read_camera_data)

    lidar_thread.start()
    camera_thread.start()

    lidar_thread.join()
    camera_thread.join()
    print("[INFO] Program finished cleanly.")

if __name__ == "__main__":
    print("Starting both LIDAR and Camera sensors.")
    main()
