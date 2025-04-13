from multiprocessing import Process, Queue
import time
import lidar
import camera

def lidar_task(queue):
    """Run the lidar task"""
    lidar.scan_loop()  # Run the lidar scanning loop

def camera_task(queue):
    """Run the camera task"""
    camera.camera_task(queue)  # Run the camera frame capture and processing

def main():
    queue = Queue()  # Queue for inter-process communication

    # Create processes for lidar and camera
    lidar_process = Process(target=lidar_task, args=(queue,))
    camera_process = Process(target=camera_task, args=(queue,))

    lidar_process.start()
    camera_process.start()

    # Main loop to collect results from both tasks
    try:
        while True:
            if not queue.empty():
                data = queue.get()  # Retrieve data from the queue (either lidar or camera)
                print(f"Received data: {data}")  # Process or print the data
    except KeyboardInterrupt:
        print("Terminating processes...")
        lidar_process.terminate()
        camera_process.terminate()
        lidar_process.join()
        camera_process.join()

if __name__ == "__main__":
    main()
