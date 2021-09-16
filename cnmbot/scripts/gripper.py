import robotiq_gripper
import time
import socket


HOST = "192.168.1.15" # The UR IP address
PORT = 30002 # UR secondary client

def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3}  "
          f"Open: {gripper.is_open(): <2}  "
          f"Closed: {gripper.is_closed(): <2}  ")

def close_vacuum():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    f = open ("Close_vacuum.script", "rb")   #Robotiq Gripper

    l = f.read(1024)
    while (l):
        s.send(l)
        l = f.read(1024)
    #s.shutdown(1)
    s.close()

def open_vacuum():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    f = open ("Open_vacuum.script", "rb")   #Robotiq Gripper

    l = f.read(1024)
    while (l):
        s.send(l)
        l = f.read(1024)
    #s.shutdown(1)
    s.close()

def move_2F_gripper(pos):
    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(HOST, 63352)
    print("Activating gripper...")
    gripper.activate()

    print("Testing gripper...")
    gripper.move_and_wait_for_pos(pos, 255, 255)
    log_info(gripper)
    gripper.move_and_wait_for_pos(0, 255, 255)
    log_info(gripper)