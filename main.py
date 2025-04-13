import serial
import time
import struct
from threading import Thread, Lock, Event

# Configurations
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
CAMERA_PORT = "/dev/ttyUSB1"
CAMERA_BAUDRATE = 115200

check_angles = [i for i in range(0, 360, 15)]
angle_tolerance = 2.0  # Degrees
frame_timeout = 1.0    # Seconds

current_array = [0] * len(check_angles)
old_array = [0] * len(check_angles)

# Locks and Events
lidar_lock = Lock()
camera_lock = Lock()
lidar_event = Event()
camera_event = Event()
running = True  # Controls main loops

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
    global current_array, old_array, running
    ser = None

    while running:
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1)
                ser.write(CMD_STOP)
                time.sleep(0.1)
                ser.reset_input_buffer()
                ser.write(CMD_SCAN)
                get_descriptor(ser)
                print("[INFO] LIDAR scan started.")

            if not lidar_event.wait(timeout=5):
                print("[ERROR] LIDAR event timeout.")
                camera_event.set()
                continue

            lidar_event.clear()
            filled_flags = [False] * len(check_angles)
            frame_start = time.time()

            while not all(filled_flags) and running:
                if time.time() - frame_start > frame_timeout:
                    print("[WARNING] LIDAR frame timeout. Restarting frame.")
                    filled_flags = [False] * len(check_angles)
                    frame_start = time.time()

                packet = ser.read(5)
                if len(packet) != 5:
                    continue

                result = parse_packet(packet)
                if result is None:
                    continue

                angle, distance, quality = result

                for i, target_angle in enumerate(check_angles):
                    if not filled_flags[i] and abs(angle - target_angle) <= angle_tolerance:
                        with lidar_lock:
                            current_array[i] = round(distance)
                            filled_flags[i] = True
                        break

            with lidar_lock:
                old_array = current_array[:]
                print("[LIDAR] Full frame:", old_array)

            camera_event.set()

        except serial.SerialException as e:
            print(f"[ERROR] LIDAR serial error: {e}. Reconnecting...")
            if ser:
                try:
                    ser.write(CMD_STOP)
                except:
                    pass
                ser.close()
                ser = None
            time.sleep(1)
        except Exception as e:
            print(f"[EXCEPTION] LIDAR thread error: {e}")
            time.sleep(1)

    if ser and ser.is_open:
        try:
            ser.write(CMD_STOP)
            ser.close()
        except:
            pass
    print("[INFO] LIDAR thread exited.")

def read_camera_data():
    global running
    ser = None

    while running:
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial(CAMERA_PORT, CAMERA_BAUDRATE, timeout=1)
                print("[INFO] Camera connected.")

            if not camera_event.wait(timeout=5):
                print("[ERROR] Camera event timeout.")
                lidar_event.set()
                continue

            camera_event.clear()

            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                with camera_lock:
                    print("[Camera] ", line)

            lidar_event.set()

        except serial.SerialException as e:
            print(f"[ERROR] Camera serial error: {e}. Reconnecting...")
            if ser:
                ser.close()
                ser = None
            time.sleep(1)
        except Exception as e:
            print(f"[EXCEPTION] Camera thread error: {e}")
            time.sleep(1)

    if ser and ser.is_open:
        ser.close()
    print("[INFO] Camera thread exited.")

def main():
    global running

    try:
        lidar_thread = Thread(target=scan_lidar)
        camera_thread = Thread(target=read_camera_data)

        lidar_thread.start()
        camera_thread.start()

        while True:
            time.sleep(0.5)  # Keep main thread alive
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected. Shutting down...")
        running = False
        lidar_event.set()
        camera_event.set()
    finally:
        print("[INFO] Waiting for threads to finish...")
        lidar_thread.join()
        camera_thread.join()
        print("[INFO] Shutdown complete.")

if __name__ == "__main__":
    print("[START] LIDAR + Camera synchronized reader")
    main()
