#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import time
import sys
import threading
from std_msgs.msg import String

def print_info(msg):
    print(f"\033[94m[INFO] {msg}\033[0m")

def print_recv(msg):
    print(f"\033[92m[RECV] {msg}\033[0m")

def print_cmd(msg):
    print(f"\033[93m[SEND] {msg}\033[0m")

def print_err(msg):
    print(f"\033[91m[ERROR] {msg}\033[0m")

last_angles = {}
last_status = {}

def feedback_callback(msg):
    global last_angles, last_status
    try:
        data = json.loads(msg.data)
        seq = data.get("seq")
        address = data.get("address")
        funcode = data.get("funcode")
        cmd_data = data.get("data", {})
        
        if address == 2 and funcode == 1:
            # Subscribed 10Hz angle feedback (store to prevent screen flooding, print on change)
            last_angles = cmd_data
        elif address == 2 and funcode == 3:
            # Subscribed 10Hz status feedback
            last_status = cmd_data
        elif address == 3 and funcode == 1:
            print_recv(f"RECEIPT FEEDBACK (seq={seq}) -> recv_status: {cmd_data.get('recv_status')} (1=Success, 0=Error)")
        elif address == 3 and funcode == 2:
            print_recv(f"EXECUTION FEEDBACK (seq={seq}) -> exec_status: {cmd_data.get('exec_status')} (1=Success, 0=Error)")
        else:
            print_recv(f"UNKNOWN FEEDBACK: {data}")
    except Exception as e:
        print_err(f"Parsing feedback failed: {e}")

def send_command(pub, seq, address, funcode, data):
    msg = {
        "seq": seq,
        "address": address,
        "funcode": funcode,
        "data": data
    }
    json_str = json.dumps(msg)
    print_cmd(json_str)
    pub.publish(json_str)

def print_menu():
    print("\n" + "="*50)
    print(" D1 Robotic Arm Command Line Test Interface")
    print("="*50)
    print(" 1. Power ON  (address=1, funcode=6, power=1)")
    print(" 2. Power OFF (address=1, funcode=6, power=0)")
    print(" 3. Enable All Joints (address=1, funcode=5, mode=1)")
    print(" 4. Unload All Joints (address=1, funcode=5, mode=0)")
    print(" 5. Control Single Joint (address=1, funcode=1)")
    print(" 6. Control All Joints (address=1, funcode=2, mode=0/1)")
    print(" 7. Move Gripper (Joint 6) (address=1, funcode=1, id=6)")
    print(" 8. Return to Zero (address=1, funcode=7)")
    print(" 9. Show Current Joint Angles & Status")
    print(" 0. Exit")
    print("="*50)

def main():
    rospy.init_node('d1_test_client', anonymous=True)
    
    pub = rospy.Publisher('rt/arm_Command', String, queue_size=10)
    rospy.Subscriber('rt/arm_Feedback', String, feedback_callback)
    
    # Wait for publisher connections
    time.sleep(0.5)
    
    seq_counter = 100
    
    # Run loop
    while not rospy.is_shutdown():
        print_menu()
        try:
            choice = input("Enter choice [0-9]: ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting...")
            break
            
        if choice == '0':
            print("Exiting...")
            break
            
        seq_counter += 1
        
        if choice == '1':
            send_command(pub, seq_counter, 1, 6, {"power": 1})
        elif choice == '2':
            send_command(pub, seq_counter, 1, 6, {"power": 0})
        elif choice == '3':
            send_command(pub, seq_counter, 1, 5, {"mode": 1})
        elif choice == '4':
            send_command(pub, seq_counter, 1, 5, {"mode": 0})
        elif choice == '5':
            try:
                jid = int(input("Enter Joint ID [0-6]: ").strip())
                angle = float(input("Enter Target Angle (deg / % for gripper): ").strip())
                delay = int(input("Enter Delay (ms, 0 for default speed): ").strip())
                send_command(pub, seq_counter, 1, 1, {"id": jid, "angle": angle, "delay_ms": delay})
            except ValueError:
                print_err("Invalid inputs. Please enter numbers.")
        elif choice == '6':
            try:
                mode = int(input("Enter Smooth Mode [0: 10Hz smooth, 1: traj smooth]: ").strip())
                angles = {}
                for i in range(7):
                    val_str = input(f"Enter angle for Joint {i} (default 0.0): ").strip()
                    angles[f"angle{i}"] = float(val_str) if val_str else 0.0
                angles["mode"] = mode
                send_command(pub, seq_counter, 1, 2, angles)
            except ValueError:
                print_err("Invalid inputs.")
        elif choice == '7':
            try:
                percent = float(input("Enter Gripper Position % [0: fully open, 100: fully closed]: ").strip())
                delay = int(input("Enter Delay (ms, 0 for default speed): ").strip())
                send_command(pub, seq_counter, 1, 1, {"id": 6, "angle": percent, "delay_ms": delay})
            except ValueError:
                print_err("Invalid input.")
        elif choice == '8':
            send_command(pub, seq_counter, 1, 7, {})
        elif choice == '9':
            print("\n--- Current Joint Angles ---")
            for i in range(7):
                print(f"  Joint {i}: {last_angles.get(f'angle{i}', 'N/A')} deg")
            print("--- Arm Status ---")
            print(f"  power_status : {last_status.get('power_status', 'N/A')}")
            print(f"  enable_status: {last_status.get('enable_status', 'N/A')}")
            print(f"  error_status : {last_status.get('error_status', 'N/A')}")
        else:
            print_err("Invalid choice. Try again.")
            
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
