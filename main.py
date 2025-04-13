import serial
import time
import struct
from threading import Thread, Lock, Event

# LIDAR Serial Configuration
LIDAR_PORT = "/dev/ttyUSB0"  # Update if needed
LIDAR_BAUDRATE = 460800

# Camera Serial Configuration
CAMERA_PORT = "/dev/ttyUSB1"  # Update if needed
CAMERA_BAUDRATE = 115200

# Setup LIDAR angle intervals
check_angles = [i for i in range(0, 360, 15)]
current_array = [0] * len(check_angles)
old_array = [0] * len(check_angles)
count = 0
lidar_lock = Lock()
camera_lock = Lock()

# Threading events for alternation
lidar_event = Event()
camera_event = Event()

# Start with LIDAR
lidar_event.set()

# LIDAR Commands
CMD_STOP = b'\xA5\x25'
CMD_SCAN = b'\xA5\x20'

def parse_packet(packet):
    if len(packet) != 5:
        return None
    b0, b1, b2, b3, b4 = struct.unpack('<BBBBB', packet)

    start_flag = b0 & 0x01
    inverted_start_flag = (b0 >> 1) & 0x01
    if start_flag != (~inverted_start_flag & 0x01):
        return None

    quality = b0 >> 2
    angle_q6 = ((b2 << 7) | (b1 >> 1))
    angle = angle_q6 / 64.0
    dist_q2 = (b4 << 8) | b3
    distance = dist_q2 / 4.0

    return angle, distance, quality

def get_descriptor(ser):
    descriptor = ser.read(7)
    if descriptor[:2] != b'\xA5\x5A':
        raise Exception("Failed to receive scan descriptor.")

def scan_lidar():
    global count, current_array, old_array
    with serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1) as ser:
        print("[INFO] Starting LIDAR scan...")
        ser.write(CMD_STOP)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.write(CMD_SCAN)
        get_descriptor(ser)
        print("[INFO] LIDAR scan started.")

        while True:
            lidar_event.wait()  # Wait for signal
            lidar_event.clear()

            frame_collected = False
            while not frame_collected:
                packet = ser.read(5)
                if len(packet) != 5:
                    ser.reset_input_buffer()
                    continue

                result = parse_packet(packet)
                if result is None:
                    continue

                angle, distance, quality = result
                angle_rounded = round(angle)

                if angle_rounded in check_angles and angle_rounded == check_angles[count]:
                    with lidar_lock:
                        current_array[count] = round(distance)
                        count += 1

                        if count >= len(check_angles):
                            count = 0
                            old_array = current_array[:]
                            print("[LIDAR] Full frame:", old_array)
                            frame_collected = True

            camera_event.set()  # Now let camera run

def read_camera_data():
    with serial.Serial(CAMERA_PORT, CAMERA_BAUDRATE, timeout=1) as ser:
        print("[INFO] Reading Camera Data...")
        while True:
            camera_event.wait()  # Wait for signal
            camera_event.clear()

            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                with camera_lock:
                    print("[Camera] ", line)

            lidar_event.set()  # Now let lidar run

def main():
    lidar_thread = Thread(target=scan_lidar)
    camera_thread = Thread(target=read_camera_data)

    lidar_thread.start()
    camera_thread.start()

    lidar_thread.join()
    camera_thread.join()

if __name__ == "__main__":
    print("Starting LIDAR and Camera with Alternating Output.")
    main()
