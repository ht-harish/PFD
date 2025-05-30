## realsense_tcp_stream.py (Modified)
import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import struct
import time

# TCP Configuration
TCP_IP = "172.16.13.97"  # Host Computer- Inference device IP
TCP_PORT = 5005

# RealSense Setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# TCP Socket Setup with connection retry
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
connected = False

print("Waiting for receiver connection...")
while not connected:
    try:
        sock.connect((TCP_IP, TCP_PORT))
        connected = True
        print("Successfully connected to receiver!")
    except socket.error:
        print("Receiver not available, retrying in 1 second...")
        time.sleep(1)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting.")
        pipeline.stop()
        sock.close()
        exit()

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Process color frame
        color_img = cv2.resize(np.asanyarray(color_frame.get_data()), (320, 240))
        _, color_buf = cv2.imencode('.jpg', color_img)
        
        # Process depth frame (16-bit unsigned)
        depth_img = cv2.resize(np.asanyarray(depth_frame.get_data()), (320, 240))
        depth_buf = depth_img.astype(np.uint16).tobytes()

        # Send color data with frame header
        sock.sendall(
            struct.pack('!I', len(color_buf)) +  # 4-byte length prefix
            b'COLOR' +                           # 5-byte identifier
            color_buf.tobytes()                  # JPEG data
        )
        
        # Send depth data with frame header
        sock.sendall(
            struct.pack('!I', len(depth_buf)) +  # 4-byte length prefix  
            b'DEPTH' +                           # 5-byte identifier
            depth_buf                            # Raw depth data
        )

except KeyboardInterrupt:
    print("\nUser requested shutdown.")
finally:
    pipeline.stop()
    sock.close()
    print("Pipeline and socket closed properly.")