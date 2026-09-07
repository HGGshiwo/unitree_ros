#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import math
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

# Joint Limits mapping (in degrees, based on URDF limits converted from radians)
# Joint 0 to 5 are revolute (Joint1 to Joint6 in URDF).
# Joint 6 is the gripper (Joint7_1 and Joint7_2 in URDF, mapped as 0 to 100%).
JOINT_LIMITS = [
    (-135.0, 135.0), # Joint 0 -> Joint1 (URDF: -2.35 to 2.35 rad)
    (-90.0, 90.0),   # Joint 1 -> Joint2 (URDF: -1.57 to 1.57 rad)
    (-90.0, 90.0),   # Joint 2 -> Joint3 (URDF: -1.57 to 1.57 rad)
    (-135.0, 135.0), # Joint 3 -> Joint4 (URDF: -2.35 to 2.35 rad)
    (-90.0, 90.0),   # Joint 4 -> Joint5 (URDF: -1.57 to 1.57 rad)
    (-135.0, 135.0), # Joint 5 -> Joint6 (URDF: -2.35 to 2.35 rad)
    (0.0, 100.0)     # Joint 6 -> Gripper (0: fully open, 100: fully closed)
]

# Global controller state
current_angles = [0.0] * 7     # Real angles read from Gazebo
commanded_angles = [0.0] * 7   # Smoothly interpolated target angles sent to Gazebo
target_angles = [0.0] * 7      # Target angles set by commands
start_angles = [0.0] * 7       # Start angles when interpolation begins
transition_times = [0.01] * 7
elapsed_times = [0.01] * 7

initialized = False
joint_enables = [False] * 7
power_status = 0     # 0: Powered OFF, 1: Powered ON
enable_status = 0    # 0: Torque OFF, 1: Enabled
error_status = 1     # 1: Normal, 0: Error

# Mutex to guard state access across ROS callbacks and update loops
state_lock = threading.Lock()
pub_feedback = None

def set_target_angle(jid, angle, delay_ms):
    """
    Sets target angle for a single joint and calculates interpolation trajectory timing.
    """
    global commanded_angles, target_angles, start_angles, transition_times, elapsed_times
    
    low, high = JOINT_LIMITS[jid]
    # Clip targets within URDF/logical joint limits
    clipped_angle = max(low, min(high, float(angle)))
    if clipped_angle != angle:
        rospy.logwarn("Joint %d target %f clipped to limits [%f, %f]", jid, angle, low, high)
        
    target_angles[jid] = clipped_angle
    start_angles[jid] = commanded_angles[jid]
    elapsed_times[jid] = 0.0
    
    if delay_ms > 0:
        transition_times[jid] = float(delay_ms) / 1000.0
    else:
        # Default velocity limits for delay_ms == 0 (deg/s or %/s)
        max_vel = 100.0 if jid == 6 else 60.0
        dist = abs(clipped_angle - commanded_angles[jid])
        transition_times[jid] = dist / max_vel if dist > 0 else 0.01

def send_feedback_message(seq, address, funcode, data):
    """
    Utility to serialize and publish feedback JSON strings.
    """
    global pub_feedback
    if pub_feedback is not None:
        msg = {
            "seq": seq,
            "address": address,
            "funcode": funcode,
            "data": data
        }
        pub_feedback.publish(json.dumps(msg))

def validate_command(data):
    """
    Performs validation on standard JSON fields.
    Returns (seq, address, funcode, cmd_data, is_valid)
    """
    if not isinstance(data, dict):
        return 0, 0, 0, {}, False
        
    seq = data.get("seq")
    address = data.get("address")
    funcode = data.get("funcode", data.get("code")) # Support both funcode and code
    cmd_data = data.get("data", {})
    
    if seq is None or address is None or funcode is None:
        return 0, 0, 0, {}, False
        
    if address not in (1, 2, 3):
        return seq, address, funcode, cmd_data, False
        
    return seq, address, funcode, cmd_data, True

def handle_command(address, funcode, cmd_data):
    """
    Executes command based on address and function code.
    Returns 1 for success, 0 for failure.
    """
    global power_status, enable_status, error_status, joint_enables
    
    if address != 1:
        return 0
        
    # 1. Power control (address: 1, funcode: 6)
    if funcode == 6:
        power = cmd_data.get("power")
        if power in (0, 1):
            power_status = power
            rospy.loginfo("Power state changed to: %d", power_status)
            if power == 0:
                # Powering off automatically disables torque on all joints
                enable_status = 0
                for i in range(7):
                    joint_enables[i] = False
            return 1
        return 0
        
    # Check if power is ON for all other command types
    if not power_status:
        rospy.logwarn("Command rejected: Robot is powered off.")
        return 0
        
    # 2. Single joint enable/unload control (address: 1, funcode: 4)
    if funcode == 4:
        jid = cmd_data.get("id")
        mode = cmd_data.get("mode")
        if jid in range(7) and mode in (0, 1):
            joint_enables[jid] = (mode == 1)
            enable_status = 1 if any(joint_enables) else 0
            rospy.loginfo("Joint %d enable state: %r. Overall enable status: %d", jid, joint_enables[jid], enable_status)
            return 1
        return 0
        
    # 3. All joints enable/unload control (address: 1, funcode: 5)
    if funcode == 5:
        mode = cmd_data.get("mode")
        if mode in (0, 1):
            enable_status = mode
            for i in range(7):
                joint_enables[i] = (mode == 1)
            rospy.loginfo("All joints enable state changed to: %d", enable_status)
            return 1
        return 0
        
    # Check if enabled for movement commands
    if not enable_status:
        rospy.logwarn("Command rejected: Joints are not enabled (Torque OFF).")
        return 0
        
    # 4. Pose zeroing (address: 1, funcode: 7)
    if funcode == 7:
        rospy.loginfo("Pose zeroing command received.")
        for i in range(7):
            if joint_enables[i]:
                set_target_angle(i, 0.0, 1000) # Zero joints smoothly in 1 sec
        return 1
        
    # 5. Single joint angle control (address: 1, funcode: 1)
    if funcode == 1:
        jid = cmd_data.get("id")
        angle = cmd_data.get("angle")
        delay_ms = cmd_data.get("delay_ms", 0)
        
        if jid not in range(7) or angle is None:
            return 0
            
        if not joint_enables[jid]:
            rospy.logwarn("Command rejected: Joint %d is not enabled.", jid)
            return 0
            
        set_target_angle(jid, angle, delay_ms)
        return 1
        
    # 6. All joints angle control (address: 1, funcode: 2)
    if funcode == 2:
        mode = cmd_data.get("mode")
        if mode not in (0, 1):
            return 0
            
        duration_ms = 100 if mode == 0 else 1000
        
        for i in range(7):
            key = f"angle{i}"
            if key in cmd_data and joint_enables[i]:
                set_target_angle(i, cmd_data[key], duration_ms)
        return 1
        
    return 0

def command_callback(msg):
    """
    Subscribes to rt/arm_Command, parses JSON, validates, runs command handler,
    and returns receipt/execution feedback.
    """
    try:
        data = json.loads(msg.data)
    except Exception as e:
        rospy.logwarn("Malformed JSON received on rt/arm_Command: %s", str(e))
        send_feedback_message(0, 3, 1, {"recv_status": 0})
        return
        
    seq, address, funcode, cmd_data, is_valid = validate_command(data)
    
    if not is_valid:
        rospy.logwarn("Invalid Command: seq=%s, address=%s, funcode=%s", seq, address, funcode)
        send_feedback_message(seq, 3, 1, {"recv_status": 0})
        return
        
    # 1. Immediate receipt status response (recv_status = 1)
    send_feedback_message(seq, 3, 1, {"recv_status": 1})
    
    # 2. Execute command
    with state_lock:
        exec_status = handle_command(address, funcode, cmd_data)
        
    # 3. Execution status response
    send_feedback_message(seq, 3, 2, {"exec_status": exec_status})

def joint_state_callback(msg):
    """
    Reads actual joint positions from Gazebo, updates current_angles,
    and initializes commanded targets on first callback.
    """
    global current_angles, commanded_angles, target_angles, start_angles, initialized
    
    with state_lock:
        temp_angles = list(current_angles)
        for name, pos in zip(msg.name, msg.position):
            if name == 'Joint1':
                temp_angles[0] = pos * 180.0 / math.pi
            elif name == 'Joint2':
                temp_angles[1] = pos * 180.0 / math.pi
            elif name == 'Joint3':
                temp_angles[2] = pos * 180.0 / math.pi
            elif name == 'Joint4':
                temp_angles[3] = pos * 180.0 / math.pi
            elif name == 'Joint5':
                temp_angles[4] = pos * 180.0 / math.pi
            elif name == 'Joint6':
                temp_angles[5] = pos * 180.0 / math.pi
            elif name == 'Joint7_1':
                # Map prismatic meters (0 to 0.03) to 0-100%
                temp_angles[6] = (pos / 0.03) * 100.0

        current_angles = list(temp_angles)
        
        # Initialize commanded targets on first received Gazebo state
        if not initialized:
            commanded_angles = list(temp_angles)
            target_angles = list(temp_angles)
            start_angles = list(temp_angles)
            initialized = True

def run_feedback_loop():
    """
    Publishes joint angles and statuses at 10Hz to rt/arm_Feedback.
    """
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not initialized:
            rate.sleep()
            continue
            
        with state_lock:
            angles_data = {
                f"angle{i}": round(current_angles[i], 2) for i in range(7)
            }
            status_data = {
                "enable_status": enable_status,
                "power_status": power_status,
                "error_status": error_status
            }
            
        # Send angles feedback (seq: 10, address: 2, funcode: 1)
        send_feedback_message(10, 2, 1, angles_data)
        
        # Send status feedback (seq: 10, address: 2, funcode: 3)
        send_feedback_message(10, 2, 3, status_data)
        
        rate.sleep()

def main():
    rospy.init_node('d1_sim_control_node', anonymous=False)
    
    global pub_feedback
    
    # ROS Pub/Sub configuration
    pub_feedback = rospy.Publisher('rt/arm_Feedback', String, queue_size=10)
    rospy.Subscriber('rt/arm_Command', String, command_callback, queue_size=10)
    
    # Gazebo joint states subscriber and command publisher
    rospy.Subscriber('/d1/joint_states', JointState, joint_state_callback, queue_size=10)
    cmd_pub = rospy.Publisher('/d1/arm_controller/command', Float64MultiArray, queue_size=10)
    
    # Spawn background thread for 10Hz feedback sending
    feedback_thread = threading.Thread(target=run_feedback_loop)
    feedback_thread.daemon = True
    feedback_thread.start()
    
    sim_rate = rospy.Rate(50) # 50 Hz control loops
    dt = 0.02
    
    rospy.loginfo("D1 Gazebo Control Bridge Node successfully started.")
    
    while not rospy.is_shutdown():
        if not initialized:
            try:
                sim_rate.sleep()
            except rospy.ROSInterruptException:
                break
            continue
            
        with state_lock:
            # Interpolate commanded_angles towards target_angles
            for i in range(7):
                if elapsed_times[i] < transition_times[i]:
                    elapsed_times[i] += dt
                    if elapsed_times[i] >= transition_times[i]:
                        commanded_angles[i] = target_angles[i]
                    else:
                        t = elapsed_times[i] / transition_times[i]
                        commanded_angles[i] = start_angles[i] + t * (target_angles[i] - start_angles[i])
            
            # Map commanded_angles to ROS float list (radians / meters)
            cmd_data = [0.0] * 8
            for i in range(6):
                cmd_data[i] = commanded_angles[i] * math.pi / 180.0
                
            # Joint7_1 (prismatic): 0 to 0.03m mapped from 0 to 100%
            gripper_percent = commanded_angles[6]
            cmd_data[6] = (gripper_percent / 100.0) * 0.03
            
            # Joint7_2 (prismatic): -0.03 to 0m mapped from 0 to 100%
            cmd_data[7] = -(gripper_percent / 100.0) * 0.03
            
        cmd_msg = Float64MultiArray()
        cmd_msg.data = cmd_data
        cmd_pub.publish(cmd_msg)
        
        try:
            sim_rate.sleep()
        except rospy.ROSInterruptException:
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
