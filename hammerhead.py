#!/usr/bin/env python3
# Testing the hammerhead maneuver in a SITL simulation
# This script uses DroneKit with SITL for simulation testing

import time
from datetime import datetime
import math
import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil

# Start SITL simulation
print("Starting SITL simulation...")
sitl = dronekit_sitl.start_default(lat=37.7749, lon=-122.4194)  # San Francisco coordinates
connection_string = sitl.connection_string()

# Connect to the Vehicle
print(f"Connecting to vehicle on: {connection_string}")
vehicle = connect(connection_string, wait_ready=True)

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

def record_data():
    """Record current vehicle state for later analysis"""
    now = time.time()
    trajectory['time'].append(now)
    trajectory['altitude'].append(vehicle.location.global_relative_frame.alt)
    trajectory['pitch'].append(vehicle.attitude.pitch)
    trajectory['roll'].append(vehicle.attitude.roll)
    trajectory['yaw'].append(vehicle.attitude.yaw)
    trajectory['north'].append(vehicle.location.local_frame.north)
    trajectory['east'].append(vehicle.location.local_frame.east)
    
    # Get current throttle from RC channel 3
    trajectory['throttle'].append(vehicle.channels['3'])

def arm_and_takeoff(target_altitude):
    """Arm the vehicle and fly to target_altitude"""
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
        
    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)
    
    # Wait until the vehicle reaches a safe height
    while True:
        record_data()
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude}")
        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    """
    Control vehicle attitude.
    """
    if yaw_angle is None:
        # Use current yaw if not specified
        yaw_angle = vehicle.attitude.yaw
    
    # Thrust is from 0 to 1
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,                                  # Timestamp (ms since boot, not used)
        0, 0,                               # Target system, target component
        0b00000000 if use_yaw_rate else 0b00000100,  # Use yaw rate flag
        # Quaternion representation of attitude
        # We'll convert Euler angles to quaternions here
        to_quaternion(roll_angle, pitch_angle, yaw_angle),
        0, 0, yaw_rate,                     # Roll, pitch, yaw rates (rad/s)
        thrust                              # Thrust (0-1)
    )
    vehicle.send_mavlink(msg)
    record_data()

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

def execute_hammerhead_turn():
    """Execute a hammerhead 180° turn maneuver"""
    print("Starting hammerhead 180° turn...")
    
    # Save the starting position
    start_location = vehicle.location.global_relative_frame
    
    # Step 1: Set mode to GUIDED_NOGPS for direct attitude control
    # Note: In SITL, we'll simulate this using regular GUIDED with attitude control
    print("Setting up for direct attitude control...")
    
    # Set initial attitude reference
    initial_yaw = vehicle.attitude.yaw
    
    # Step 2: Initial vertical climb with high pitch and full throttle
    print("Initiating vertical climb...")
    for i in range(20):  # Run for 2 seconds (20 * 0.1)
        # Set attitude for vertical climb (pitch up 75 degrees)
        send_attitude_target(
            roll_angle=0,
            pitch_angle=math.radians(-75),  # Negative is pitch up in NED frame
            yaw_angle=initial_yaw,
            thrust=0.9  # High throttle for climb
        )
        time.sleep(0.1)
    
    # Keep climbing for a few seconds
    print("Continuing vertical climb...")
    for i in range(40):  # Run for 4 seconds
        send_attitude_target(
            roll_angle=0,
            pitch_angle=math.radians(-80),  # More vertical
            yaw_angle=initial_yaw,
            thrust=0.8  # Slightly reduced throttle as we gain height
        )
        time.sleep(0.1)
    
    # Step 3: Cut throttle to initiate stall at top of maneuver
    print("Cutting throttle for stall...")
    for i in range(20):  # Run for 2 seconds
        send_attitude_target(
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
        
    for i in range(30):  # Run for 3 seconds
        # Calculate intermediate yaw for smooth rotation
        progress = i / 30.0
        current_yaw = initial_yaw + progress * math.pi
        if current_yaw > math.pi:
            current_yaw -= 2 * math.pi
            
        send_attitude_target(
            roll_angle=0,
            pitch_angle=math.radians(-60),  # Start transitioning nose down
            yaw_angle=current_yaw,  # Apply rotation
            thrust=0.2  # Maintain low throttle
        )
        time.sleep(0.1)
    
    # Step 5: Transition to nose down for descent
    print("Transitioning to descent...")
    for i in range(20):  # Run for 2 seconds
        progress = i / 20.0
        # Transition from -60 to 60 degrees (nose down)
        current_pitch = math.radians(-60 + progress * 120)
        
        send_attitude_target(
            roll_angle=0,
            pitch_angle=current_pitch,  # Transition to nose down
            yaw_angle=target_yaw,  # Maintain new heading
            thrust=0.3  # Slight throttle for control
        )
        time.sleep(0.1)
    
    # Step 6: Controlled descent in opposite direction
    print("Controlled descent...")
    for i in range(30):  # Run for 3 seconds
        send_attitude_target(
            roll_angle=0,
            pitch_angle=math.radians(45),  # Nose down for descent
            yaw_angle=target_yaw,
            thrust=0.4  # Medium throttle for controlled descent
        )
        time.sleep(0.1)
    
    # Step 7: Level out
    print("Leveling out...")
    for i in range(20):  # Run for 2 seconds
        progress = i / 20.0
        # Transition from 45 to 0 degrees (level)
        current_pitch = math.radians(45 - progress * 45)
        
        send_attitude_target(
            roll_angle=0,
            pitch_angle=current_pitch,
            yaw_angle=target_yaw,
            thrust=0.5  # Balanced throttle
        )
        time.sleep(0.1)
    
    # Return to standard GUIDED mode control
    print("Returning to position hold...")
    vehicle.mode = VehicleMode("GUIDED")
    
    # Go to a point 50m ahead in our new direction
    current_location = vehicle.location.global_relative_frame
    heading = math.degrees(target_yaw)
    if heading < 0:
        heading += 360
        
    # Calculate new position 50m ahead
    north = math.cos(math.radians(heading)) * 50
    east = math.sin(math.radians(heading)) * 50
    
    # Create a LocationGlobal object for the destination
    destination = get_location_metres(current_location, north, east)
    
    # Command the vehicle to the destination
    vehicle.simple_goto(destination)
    
    # Wait until we reach the destination
    print("Flying to new position...")
    for i in range(30):  # Wait up to 30 seconds
        record_data()
        time.sleep(1)
    
    print("Hammerhead 180° turn completed!")

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude
    coordinates of a position that is 'dNorth' and 'dEast' metres from the 
    specified 'original_location'.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    
    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    
    return LocationGlobal(newlat, newlon, original_location.alt)

def visualize_flight_data():
    """Create visualization of flight data"""
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
    
    plt.close(fig)

# MAIN EXECUTION
try:
    # Take off to sufficient altitude for maneuver
    arm_and_takeoff(40)  # 40 meters for safety
    
    # Wait for a moment to stabilize
    print("Stabilizing at altitude...")
    for i in range(10):
        record_data()
        time.sleep(1)
    
    # Execute the hammerhead 180° turn
    execute_hammerhead_turn()
    
    # Hover for a moment to collect more data
    print("Collecting final data...")
    for i in range(10):
        record_data()
        time.sleep(1)
    
    # Land
    print("Returning to launch...")
    vehicle.mode = VehicleMode("RTL")
    
    # Wait for landing
    print("Waiting for landing...")
    for i in range(60):  # Wait up to 60 seconds
        record_data()
        current_altitude = vehicle.location.global_relative_frame.alt
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
    # Close vehicle and SITL simulation
    print("Closing vehicle connection and simulation...")
    vehicle.close()
    sitl.stop()
