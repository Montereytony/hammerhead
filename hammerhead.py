#!/usr/bin/env python3
# Fixed hammerhead maneuver simulation for Python 3.12
# Using direct pymavlink instead of dronekit

import time
from datetime import datetime
import math
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import subprocess
import threading
import os
import signal

# Global variable to track if the simulation is running
simulation_running = True

def start_sitl():
    """Start the SITL simulator using subprocess"""
    print("Starting SITL simulation...")
    # Start ArduCopter SITL
    sitl_process = subprocess.Popen(
        ['ardupilot_sitl', 'copter', '--model', 'quad', '--home=37.7749,-122.4194,0,180'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT
    )
    return sitl_process

def connect_to_vehicle():
    """Connect to the vehicle using pymavlink"""
    print("Connecting to vehicle...")
    # Connect to the Vehicle through UDP
    connection_string = 'udp:127.0.0.1:14550'
    vehicle = mavutil.mavlink_connection(connection_string)
    
    # Wait for the first heartbeat to confirm connection
    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat()
    print("Connected to vehicle!")
    
    return vehicle

# Data collection for visualization
trajectory = {
    'time': [],
    'altitude': [],
    'pitch': [],
    'roll': [],
    'yaw': [],
    'north': [],
    'east': [],
    'throttle': []
}

def record_data(vehicle):
    """Record current vehicle state for later analysis"""
    # Request current data
    vehicle.mav.request_data_stream_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    
    # Get position and attitude
    pos_msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    att_msg = vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    rc_msg = vehicle.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
    
    now = time.time()
    if pos_msg and att_msg and rc_msg:
        # Position data
        alt = pos_msg.relative_alt / 1000.0  # Convert mm to m
        
        # Attitude data
        pitch = att_msg.pitch
        roll = att_msg.roll
        yaw = att_msg.yaw
        
        # Location data
        # We'll use a local coordinate system with the starting point as origin
        if len(trajectory['north']) == 0:
            trajectory['north'].append(0)
            trajectory['east'].append(0)
        else:
            # We'd normally calculate this from lat/lon, 
            # but for simplicity in simulation, we'll use a simple model
            dt = now - trajectory['time'][-1]
            forward_speed = 5  # Estimated speed in m/s
            
            # Rough approximation of movement based on attitude
            north_change = math.cos(yaw) * forward_speed * dt * math.cos(pitch)
            east_change = math.sin(yaw) * forward_speed * dt * math.cos(pitch)
            
            trajectory['north'].append(trajectory['north'][-1] + north_change)
            trajectory['east'].append(trajectory['east'][-1] + east_change)
        
        # RC data
        throttle = (rc_msg.chan3_raw - 1000) / 1000.0  # Scale to 0-1
        
        trajectory['time'].append(now)
        trajectory['altitude'].append(alt)
        trajectory['pitch'].append(pitch)
        trajectory['roll'].append(roll)
        trajectory['yaw'].append(yaw)
        trajectory['throttle'].append(throttle)

def arm_and_takeoff(vehicle, target_altitude):
    """Arm the vehicle and fly to target_altitude"""
    print("Basic pre-arm checks")
    
    # Set mode to GUIDED
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
    
    # Arm the vehicle
    print("Arming motors")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    
    # Wait for arming
    armed = False
    while not armed:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                print("Vehicle armed!")
        time.sleep(0.1)
    
    # Command takeoff
    print("Taking off!")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, target_altitude)
    
    # Wait until the vehicle reaches a safe height
    reached_altitude = False
    while not reached_altitude and simulation_running:
        record_data(vehicle)
        if len(trajectory['altitude']) > 0:
            current_altitude = trajectory['altitude'][-1]
            print(f"Altitude: {current_altitude}")
            if current_altitude >= target_altitude * 0.95:
                print("Reached target altitude")
                reached_altitude = True
        time.sleep(1)

def send_attitude_target(vehicle, roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    """
    Control vehicle attitude.
    """
    # Use current yaw if not specified
    if yaw_angle is None:
        if len(trajectory['yaw']) > 0:
            yaw_angle = trajectory['yaw'][-1]
        else:
            yaw_angle = 0
    
    # Convert Euler angles to quaternion
    q = to_quaternion(roll_angle, pitch_angle, yaw_angle)
    
    # Thrust is from 0 to 1
    vehicle.mav.set_attitude_target_send(
        0,                                  # Timestamp (ms since boot, not used)
        vehicle.target_system,              # Target system
        vehicle.target_component,           # Target component
        0b00000000 if use_yaw_rate else 0b00000100,  # Use yaw rate flag
        q,                                  # Quaternion
        0, 0, yaw_rate,                     # Roll, pitch, yaw rates (rad/s)
        thrust                              # Thrust (0-1)
    )
    record_data(vehicle)

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert Euler angles to quaternion.
    """
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]

def execute_hammerhead_turn(vehicle):
    """Execute a hammerhead 180° turn maneuver"""
    print("Starting hammerhead 180° turn...")
    
    # Set initial attitude reference
    if len(trajectory['yaw']) > 0:
        initial_yaw = trajectory['yaw'][-1]
    else:
        initial_yaw = 0
    
    # Step 2: Initial vertical climb with high pitch and full throttle
    print("Initiating vertical climb...")
    for i in range(20) and simulation_running:  # Run for 2 seconds (20 * 0.1)
        # Set attitude for vertical climb (pitch up 75 degrees)
        send_attitude_target(
            vehicle,
            roll_angle=0,
            pitch_angle=math.radians(-75),  # Negative is pitch up in NED frame
            yaw_angle=initial_yaw,
            thrust=0.9  # High throttle for climb
        )
        time.sleep(0.1)
    
    # Keep climbing for a few seconds
    print("Continuing vertical climb...")
    for i in range(40) and simulation_running:  # Run for 4 seconds
        send_attitude_target(
            vehicle,
            roll_angle=0,
            pitch_angle=math.radians(-80),  # More vertical
            yaw_angle=initial_yaw,
            thrust=0.8  # Slightly reduced throttle as we gain height
        )
        time.sleep(0.1)
    
    # Step 3: Cut throttle to initiate stall at top of maneuver
    print("Cutting throttle for stall...")
    for i in range(20) and simulation_running:  # Run for 2 seconds
        send_attitude_target(
            vehicle,
            roll_angle=0,
            pitch_angle=math.radians(-85),  # Almost vertical
            yaw_angle=initial_yaw,
            thrust=0.2  # Low throttle to initiate stall
        )
        time.sleep(0.1)
    
    # Step 4: Apply yaw rotation for 180° turn at apex of maneuver
    print("Rotating 180 degrees...")
    target_yaw = initial_yaw + math.pi  # 180 degrees in radians
    if target_yaw > math.pi:
        target_yaw -= 2 * math.pi  # Keep in range -pi to pi
        
    for i in range(30) and simulation_running:  # Run for 3 seconds
        # Calculate intermediate yaw for smooth rotation
        progress = i / 30.0
        current_yaw = initial_yaw + progress * math.pi
        if current_yaw > math.pi:
            current_yaw -= 2 * math.pi
            
        send_attitude_target(
            vehicle,
            roll_angle=0,
            pitch_angle=math.radians(-60),  # Start transitioning nose down
            yaw_angle=current_yaw,  # Apply rotation
            thrust=0.2  # Maintain low throttle
        )
        time.sleep(0.1)
    
    # Step 5: Transition to nose down for descent
    print("Transitioning to descent...")
    for i in range(20) and simulation_running:  # Run for 2 seconds
        progress = i / 20.0
        # Transition from -60 to 60 degrees (nose down)
        current_pitch = math.radians(-60 + progress * 120)
        
        send_attitude_target(
            vehicle,
            roll_angle=0,
            pitch_angle=current_pitch,  # Transition to nose down
            yaw_angle=target_yaw,  # Maintain new heading
            thrust=0.3  # Slight throttle for control
        )
        time.sleep(0.1)
    
    # Step 6: Controlled descent in opposite direction
    print("Controlled descent...")
    for i in range(30) and simulation_running:  # Run for 3 seconds
        send_attitude_target(
            vehicle,
            roll_angle=0,
            pitch_angle=math.radians(45),  # Nose down for descent
            yaw_angle=target_yaw,
            thrust=0.4  # Medium throttle for controlled descent
        )
        time.sleep(0.1)
    
    # Step 7: Level out
    print("Leveling out...")
    for i in range(20) and simulation_running:  # Run for 2 seconds
        progress = i / 20.0
        # Transition from 45 to 0 degrees (level)
        current_pitch = math.radians(45 - progress * 45)
        
        send_attitude_target(
            vehicle,
            roll_angle=0,
            pitch_angle=current_pitch,
            yaw_angle=target_yaw,
            thrust=0.5  # Balanced throttle
        )
        time.sleep(0.1)
    
    # Return to standard GUIDED mode control
    print("Returning to position hold...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
    
    print("Hammerhead 180° turn completed!")

def return_to_launch(vehicle):
    """Command the vehicle to return to launch"""
    print("Returning to launch...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
        0, 0, 0, 0, 0, 0, 0)

def visualize_flight_data():
    """Create visualization of flight data"""
    if len(trajectory['time']) < 2:
        print("Not enough flight data collected for visualization")
        return
        
    # Convert time to relative seconds
    start_time = trajectory['time'][0]
    rel_time = [t - start_time for t in trajectory['time']]
    
    # Create figure with multiple subplots
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(trajectory['east'], trajectory['north'], trajectory['altitude'])
    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    ax1.set_zlabel('Altitude (m)')
    ax1.set_title('3D Flight Path')
    
    # Altitude vs time
    ax2 = fig.add_subplot(222)
    ax2.plot(rel_time, trajectory['altitude'])
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude (m)')
    ax2.set_title('Altitude vs Time')
    
    # Attitude angles vs time
    ax3 = fig.add_subplot(223)
    ax3.plot(rel_time, [math.degrees(p) for p in trajectory['pitch']], label='Pitch')
    ax3.plot(rel_time, [math.degrees(r) for r in trajectory['roll']], label='Roll')
    ax3.plot(rel_time, [math.degrees(y) for y in trajectory['yaw']], label='Yaw')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (degrees)')
    ax3.set_title('Attitude vs Time')
    ax3.legend()
    
    # Throttle vs time
    ax4 = fig.add_subplot(224)
    ax4.plot(rel_time, trajectory['throttle'])
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Throttle')
    ax4.set_title('Throttle vs Time')
    
    plt.tight_layout()
    
    # Save the figure
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = f"hammerhead_flight_data_{timestamp}.png"
    plt.savefig(filename)
    print(f"Flight data visualization saved as {filename}")
    
    # Show the plot
    plt.show()

def keyboard_interrupt_handler():
    """Listen for keyboard interrupt to stop the simulation gracefully"""
    global simulation_running
    try:
        # Wait for keyboard interrupt
        input("Press Enter to stop the simulation...\n")
    except KeyboardInterrupt:
        pass
    
    simulation_running = False
    print("\nStopping simulation...")

# MAIN EXECUTION
if __name__ == "__main__":
    sitl_process = None
    
    try:
        # Start the keyboard interrupt handler in a separate thread
        interrupt_thread = threading.Thread(target=keyboard_interrupt_handler)
        interrupt_thread.daemon = True
        interrupt_thread.start()
        
        # Start SITL and connect
        sitl_process = start_sitl()
        time.sleep(5)  # Give SITL time to initialize
        vehicle = connect_to_vehicle()
        
        # Take off to sufficient altitude for maneuver
        arm_and_takeoff(vehicle, 40)  # 40 meters for safety
        
        # Wait for a moment to stabilize
        print("Stabilizing at altitude...")
        for i in range(10):
            if not simulation_running:
                break
            record_data(vehicle)
            time.sleep(1)
        
        # Execute the hammerhead 180° turn
        if simulation_running:
            execute_hammerhead_turn(vehicle)
        
        # Hover for a moment to collect more data
        print("Collecting final data...")
        for i in range(10):
            if not simulation_running:
                break
            record_data(vehicle)
            time.sleep(1)
        
        # Return to launch
        if simulation_running:
            return_to_launch(vehicle)
        
        # Wait for landing
        print("Waiting for landing...")
        for i in range(60):  # Wait up to 60 seconds
            if not simulation_running:
                break
            record_data(vehicle)
            if len(trajectory['altitude']) > 0:
                current_altitude = trajectory['altitude'][-1]
                print(f"Altitude: {current_altitude}")
                if current_altitude < 0.5:
                    print("Landed!")
                    break
            time.sleep(1)
        
        # Create visualization
        visualize_flight_data()
        
        print("Simulation complete!")

    except Exception as e:
        print(f"Error occurred: {e}")
        
    finally:
        # Clean up
        global simulation_running
        simulation_running = False
        print("Cleaning up...")
        
        if sitl_process:
            print("Terminating SITL process...")
            sitl_process.terminate()
            try:
                sitl_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sitl_process.kill()
                
        print("Done!")
