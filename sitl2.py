import cv2
import numpy as np
import socket
import struct
import time
from pymavlink import mavutil
from ultralytics import YOLO

# TCP Configuration
TCP_IP = "0.0.0.0"
TCP_PORT = 5005

# MAVLink Connection
connection = mavutil.mavlink_connection('udp:127.0.0.1:5760')
model = YOLO("yolov8n.pt")

# Depth configuration
DEPTH_SCALE = 0.001  # Convert mm to meters
MIN_DEPTH = 0.5      # Minimum valid depth in meters
MAX_DEPTH = 10.0     # Maximum valid depth in meters

def send_command(command, *args):
    """Send MAVLink command with proper argument padding"""
    params = list(args) + [0]*(7-len(args))  # Pad with zeros
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        command,
        0,  # Confirmation flag
        *params
    )

def send_movement_command(forward, right, down=10):
    """Send movement commands with altitude hold"""
    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, -down,  # Position (not used)
        forward, right, 0,  # Velocity in m/s
        0, 0, 0, 0, 0)     # Acceleration (not used)

# Initialize MAVLink connection
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received. Connected to SITL.")

# Arm and takeoff sequence
send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
time.sleep(2)
send_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 4)  # GUIDED mode = 4 for ArduCopter
time.sleep(5)
send_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10)
time.sleep(10)

# TCP Socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_IP, TCP_PORT))
sock.listen(1)
sock.settimeout(5)  # Set socket timeout

print("Waiting for input stream...")
conn, addr = sock.accept()
print(f"Connected to stream from {addr}")

color_frame = None
depth_frame = None
last_display_time = time.time()

try:
    while True:
        try:
            # Read message header
            raw_len = conn.recv(4)
            if not raw_len:
                print("Waiting for input stream...")
                time.sleep(1)
                continue

            msg_len = struct.unpack('!I', raw_len)[0]
            msg_type = conn.recv(5)

            # Read message body
            data = b''
            while len(data) < msg_len:
                packet = conn.recv(msg_len - len(data))
                if not packet: break
                data += packet

            if msg_type == b'COLOR':
                color_frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

            elif msg_type == b'DEPTH':
                depth_frame = np.frombuffer(data, np.uint16).reshape(240, 320).astype(np.float32) * DEPTH_SCALE
                depth_frame = np.clip(depth_frame, MIN_DEPTH, MAX_DEPTH)

            # Process when both frames are available
            if color_frame is not None and depth_frame is not None:
                results = model(color_frame)
                detections = results[0].boxes.data.cpu().numpy()
                
                person_found = False
                for bbox in detections:
                    x_min, y_min, x_max, y_max, conf, cls = bbox
                    if int(cls) == 0:  # Person class
                        # Convert coordinates to depth resolution
                        h, w = color_frame.shape[:2]
                        scale_x = 320 / w
                        scale_y = 240 / h

                        # Calculate depth ROI
                        dx_min = int(x_min * scale_x)
                        dy_min = int(y_min * scale_y)
                        dx_max = int(x_max * scale_x)
                        dy_max = int(y_max * scale_y)

                        # Get median depth
                        depth_roi = depth_frame[dy_min:dy_max, dx_min:dx_max]
                        valid_depths = depth_roi[(depth_roi > MIN_DEPTH) & (depth_roi < MAX_DEPTH)]
                        distance = np.median(valid_depths) if valid_depths.size > 0 else MAX_DEPTH

                        # Control logic
                        x_center = (x_min + x_max) / 2
                        frame_center = w / 2
                        position_error = (x_center - frame_center) / frame_center
                        
                        # Speed calculation
                        forward_speed = 0.0
                        if distance > 3.0:
                            forward_speed = 1.5
                        elif distance > 2.0:
                            forward_speed = 1.0
                            
                        lateral_speed = np.clip(position_error * 1.2, -1.0, 1.0)

                        send_movement_command(forward_speed, lateral_speed)

                        # Draw visualization
                        cv2.rectangle(color_frame, (int(x_min), int(y_min)), 
                                    (int(x_max), int(y_max)), (0, 255, 0), 2)
                        cv2.putText(color_frame, f"{distance:.2f}m", 
                                  (int(x_min), int(y_min)-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                        person_found = True
                        break

                if not person_found:
                    send_movement_command(0, 0)

                # Display frame
                cv2.imshow('Tracking', color_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except socket.timeout:
            print("No stream input detected. Waiting...")
            time.sleep(1)
            
        except KeyboardInterrupt:
            break

finally:
    # Landing sequence
    send_command(mavutil.mavlink.MAV_CMD_NAV_LAND)
    time.sleep(7)
    send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
    conn.close()
    cv2.destroyAllWindows()
    print("Landed and disarmed successfully")

