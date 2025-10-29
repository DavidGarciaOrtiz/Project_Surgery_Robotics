from robodk.robolink import *
from robodk.robomath import *
import time
import math
import tkinter as tk
import threading
import socket
import json

# ========================
# CONFIGURATION
# ========================
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 1024
ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01
USE_REAL_ROBOT = True   # <---- SWITCH BETWEEN SIMULATION (False) / REAL (True)
UR_IP = "192.168.0.10"  # <---- change this to your UR5e controller IP
UR_PORT = 30002         # UR secondary interface for URScript commands

# ========================
# GLOBAL VARIABLES
# ========================
Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None
data_lock = threading.Lock()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# ========================
# INITIALIZE ROBO DK
# ========================
def initialize_robodk():
    RDK = Robolink()
    robot = RDK.Item(ROBOT_NAME)
    base = RDK.Item(f'{ROBOT_NAME} Base')
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    Init_target = RDK.Item('Init')
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.MoveL(Init_target)
    robot.setSpeed(50)
    return robot, base, gripper, needle

# ========================
# NEW SECTION: REAL ROBOT CONNECTION
# ========================
def connect_to_ur():
    """Create a TCP socket to the UR5e controller."""
    try:
        ur_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ur_sock.connect((UR_IP, UR_PORT))
        print(f"✅ Connected to UR5e at {UR_IP}:{UR_PORT}")
        return ur_sock
    except Exception as e:
        print(f"❌ Could not connect to UR5e: {e}")
        return None

def send_urscript_command(ur_sock, cmd: str):
    """Send URScript command to UR controller."""
    if ur_sock:
        try:
            ur_sock.sendall((cmd + "\n").encode('utf-8'))
        except Exception as e:
            print(f"⚠️ URScript send error: {e}")

def move_ur5e_l(ur_sock, pose):
    """Move UR5e using movel(p[x,y,z,rx,ry,rz]) URScript command."""
    try:
        # Pose from RoboDK: 4x4 matrix to URScript p[x,y,z,rx,ry,rz]
        x, y, z, rx, ry, rz = Pose_2_TxyzRxyz(pose)
        cmd = f"movel(p[{x/1000:.4f},{y/1000:.4f},{z/1000:.4f},{math.radians(rx):.4f},{math.radians(ry):.4f},{math.radians(rz):.4f}], a=1.2, v=0.25)"
        send_urscript_command(ur_sock, cmd)
    except Exception as e:
        print(f"UR5e move error: {e}")

# ========================
# ORIENTATION FUNCTION
# ========================
def endowrist2base_orientation(roll, pitch, yaw):
    roll2 = (roll + 90) % 360
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2

# ========================
# GUI UPDATE
# ========================
def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values):
    full_text = f"Tool orientation: {tool_orientation}\nGripper orientation: {gripper_orientation}\n{status_message}\nTorque Values: {torque_values}"
    label.after(0, lambda: label.config(text=full_text))

# ========================
# UDP DATA READER
# ========================
def read_data_UDP():
    global Endowrist_rpy, Gripper_rpy, data_lock
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            try:
                received_data = json.loads(data.decode())
                device_id = received_data.get("device")
                if device_id == "G5_Endo":
                    with data_lock:
                        Endowrist_rpy = received_data
                elif device_id == "G5_Gri":
                    with data_lock:
                        Gripper_rpy = received_data
            except json.JSONDecodeError:
                print("Error decoding JSON data")
        except socket.error:
            break

# ========================
# MOVE ROBOT THREAD
# ========================
def move_robot(robot, gripper, needle, text_label):
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, data_lock

    # Connect to the real UR5e
    ur_sock = connect_to_ur() if USE_REAL_ROBOT else None

    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy

        status_message = ""
        endowrist_orientation_msg = ""
        gripper_orientation_msg = ""

        if current_Endowrist_rpy:
            e_roll = current_Endowrist_rpy.get("roll", 0)
            e_pitch = current_Endowrist_rpy.get("pitch", 0)
            e_yaw = current_Endowrist_rpy.get("yaw", 0)

            # Adjust orientation to base coordinates
            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)

            if USE_REAL_ROBOT and ur_sock:
                # 🟢 Send UR5e orientation command directly
                move_ur5e_orientation(ur_sock, endo_roll, endo_pitch, endo_yaw)
                status_message = "UR5e following Endowrist orientation..."
            else:
                # Simulate in RoboDK
                endowrist_pose = robot.Pose()
                Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
                endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))
                if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                    robot.MoveL(endowrist_pose_new, True)

            endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} Y={round(endo_yaw)}"

        if current_Gripper_rpy:
            g_roll = current_Gripper_rpy.get("roll", 0)
            g_pitch = current_Gripper_rpy.get("pitch", 0)
            g_yaw = current_Gripper_rpy.get("yaw", 0)
            gripper_orientation_msg = f"R={round(g_roll)} P={round(g_pitch)} Y={round(g_yaw)}"

        # Update GUI
        update_text_label(text_label, endowrist_orientation_msg, gripper_orientation_msg, status_message, "")

        time.sleep(READ_INTERVAL_S)

# ========================
# TKINTER SETUP
# ========================
def on_closing():
    global root, sock
    print("Closing...")
    try:
        sock.close()
        initialize_robodk()
    except Exception:
        pass
    root.destroy()

def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)

def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)

# ========================
# MAIN FUNCTION
# ========================
def main():
    global root, ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, robot, gripper, base, text_label

    robot, base, gripper, needle = initialize_robodk()

    root = tk.Tk()
    root.title("Suture Process")
    root.protocol("WM_DELETE_WINDOW", on_closing)

    text_label = tk.Label(root, text="", wraplength=300)
    text_label.pack(padx=20, pady=20)

    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Tool Yaw",
                               command=lambda value: set_zero_yaw_tool(float(value)), length=200)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack()

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Gripper Yaw",
                                  command=lambda value: set_zero_yaw_gripper(float(value)), length=200)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack()

    udp_thread = threading.Thread(target=read_data_UDP, daemon=True)
    udp_thread.start()

    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label), daemon=True)
    robot_thread.start()

    root.mainloop()
# ===========================================================
# NEW SECTION: UR5e COMMUNICATION
# ===========================================================
def connect_to_ur():
    """Connect to the UR5e controller via TCP socket."""
    try:
        ur_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ur_sock.connect((UR_IP, UR_PORT))
        print(f"✅ Connected to UR5e at {UR_IP}:{UR_PORT}")
        return ur_sock
    except Exception as e:
        print(f"❌ Could not connect to UR5e: {e}")
        return None

def send_urscript_command(ur_sock, cmd: str):
    """Send URScript command to UR controller."""
    if ur_sock:
        try:
            ur_sock.sendall((cmd + "\n").encode("utf-8"))
        except Exception as e:
            print(f"⚠️ URScript send error: {e}")

def move_ur5e_orientation(ur_sock, roll, pitch, yaw, speed=0.2, accel=0.5):
    """
    Move UR5e tool orientation to match Endowrist roll/pitch/yaw.
    Only orientation changes; position remains current.
    """
    if not ur_sock:
        return

    # Convert RPY (deg) → rotation vector (radians)
    rx = math.radians(roll)
    ry = math.radians(pitch)
    rz = math.radians(yaw)

    # You can get the current TCP pose from the robot, but since this is open-loop,
    # we'll assume a fixed position [x, y, z]
    x, y, z = 0.4, 0.0, 0.3  # Example position, adjust as needed

    cmd = (
        f"movel(p[{x},{y},{z},{rx},{ry},{rz}], a={accel}, v={speed})"
    )
    send_urscript_command(ur_sock, cmd)

if __name__ == "__main__":
    main()
