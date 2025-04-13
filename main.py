import serial
import time
import struct
from threading import Thread, Lock

# LIDAR Serial Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Change to your LIDAR serial port
LIDAR_BAUDRATE = 460800

# Camera Serial Configuration
CAMERA_PORT = "/dev/ttyUSB1"  # Change to your ESP-EYE serial port
CAMERA_BAUDRATE = 115200

# Storage for processed LIDAR data
check_angles = [i for i in range(0, 360, 15)]  # 0°, 15°, ..., 345°
current_array = [0] * len(check_angles)
old_array = [0] * len(check_angles)
count = 0
lidar_lock = Lock()

# Lock to prevent race condition for camera data
camera_lock = Lock()

# Control turn variable
turn_lock = Lock()
turn = "lidar"  # The first sensor to output (LIDAR)

# LIDAR Commands
CMD_STOP = b'\xA5\x25'
CMD_SCAN = b'\xA5\x20'

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
    angle_q6 = ((b2 << 7) | (b1 >> 1))  # 15 bits
    angle = angle_q6 / 64.0
    dist_q2 = (b4 << 8) | b3
    distance = dist_q2 / 4.0

    return angle, distance, quality

# Function to get descriptor (start scan)
def get_descriptor(ser):
    descriptor = ser.read(7)
    if descriptor[:2] != b'\xA5\x5A':
        raise Exception("Failed to receive scan descriptor.")

# LIDAR scan loop
def scan_lidar():
    global count, current_array, old_array, turn  # Declare turn as global

    with serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1) as ser:
        print("[INFO] Starting LIDAR scan...")

        # Stop and flush any previous state
        ser.write(CMD_STOP)
        time.sleep(0.1)
        ser.reset_input_buffer()

        # Start scan
        ser.write(CMD_SCAN)
        get_descriptor(ser)
        print("[INFO] LIDAR scan started.")

        while True:
            packet = ser.read(5)
            if len(packet) != 5:
                ser.reset_input_buffer()
                continue

            result = parse_packet(packet)
            if result is None:
                continue

            angle, distance, quality = result
            angle_rounded = round(angle)

            # Only keep measurements at 15-degree intervals
            if angle_rounded in check_angles and angle_rounded == check_angles[count]:
                with lidar_lock:
                    current_array[count] = round(distance)
                    count += 1

                    if count >= len(check_angles):
                        count = 0
                        old_array = current_array[:]
                        print("[INFO] Full LIDAR frame captured:", old_array)
            
            # Control output alternating between lidar and camera
            with turn_lock:
                if turn == "lidar":
                    print("[LIDAR] Full frame captured:", old_array)
                    turn = "camera"  # Switch turn to camera

# Camera reading loop
def read_camera_data():
    global turn  # Declare turn as global

    with serial.Serial(CAMERA_PORT, CAMERA_BAUDRATE, timeout=1) as ser:
        print("Reading Camera Data from ESP-EYE...")
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    with camera_lock:
                        print("[Camera] ", line)

                # Control output alternating between lidar and camera
                with turn_lock:
                    if turn == "camera":
                        print("[Camera] ", line)
                        turn = "lidar"  # Switch turn to lidar
        except KeyboardInterrupt:
            print("\nStopping Camera reading.")
        finally:
            ser.close()

# Main function to start both LIDAR and Camera threads
def main():
    # Create LIDAR and Camera reading threads
    lidar_thread = Thread(target=scan_lidar)
    camera_thread = Thread(target=read_camera_data)

    # Start the threads
    lidar_thread.start()
    camera_thread.start()

    # Wait for both threads to finish
    lidar_thread.join()
    camera_thread.join()

if __name__ == "__main__":
    print("Starting both LIDAR and Camera sensors.")
    main()
