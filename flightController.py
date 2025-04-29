#!/usr/bin/env python
from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

class FlightController(object):
    def __init__(self, vehicle=None, connection_string='tcp:127.0.0.1:5763'):
            if vehicle is None:
                print("Connecting to vehicle on: {}".format(connection_string))
                self.vehicle = connect(connection_string, wait_ready=True)
            else:
                self.vehicle = vehicle

            # Set precision landing parameters
            self.vehicle.parameters['PLND_ENABLED'] = 0
            self.vehicle.parameters['PLND_TYPE'] = 0
            self.vehicle.parameters['PLND_EST_TYPE'] = 0
            self.vehicle.parameters['LAND_SPEED'] = 30

            self.camera_matrix = None



    def arm_and_takeoff(self, target_altitude):
        """Arms vehicle and fly to target altitude."""
        print("Waiting for vehicle to become armable...")
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to be armable...")
            time.sleep(1)

        print("Setting mode to GUIDED")
        self.vehicle.mode = VehicleMode("GUIDED")
        
        # Wait until the mode is actually set
        while self.vehicle.mode.name != "GUIDED":
            print("Waiting for GUIDED mode...")
            time.sleep(1)
        
        print("Arming motors")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)

        # Wait until the vehicle reaches a safe altitude
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print("Altitude: {:.2f} m".format(alt))
            if alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)



    def send_local_ned_velocity(self, vx, vy, vz):
        """
        Move vehicle in direction based on specified velocity vectors.
        vx: forward/backward (m/s)
        vy: right/left (m/s)
        vz: up/down (m/s)  (positive downward)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,  # time_boot_ms, target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # velocity components in m/s
            0, 0, 0,  # acceleration (not supported)
            0, 0)     # yaw, yaw_rate (not supported)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def land(self):
        """Initiate landing sequence."""
        print("Initiating landing...")
        self.vehicle.mode = VehicleMode("LAND")
        # Wait until the vehicle has landed.
        while self.vehicle.armed:
            print("Waiting for landing to complete...")
            time.sleep(1)
        print("Landed.")

    def close(self):
        """Close vehicle connection."""
        self.vehicle.close()
        print("Vehicle connection closed.")
        
    def send_yaw_rate(self, yaw_rate):
        """
        Rotate the drone to align with the marker.
        yaw_rate: Positive values rotate clockwise, negative counterclockwise.
        """
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            0, 0,  # target system, target component
            0b00000111,  # Ignore roll, pitch, but control yaw rate
            [1, 0, 0, 0],  # No roll/pitch control (neutral quaternion)
            0, 0, yaw_rate,  # Roll, Pitch, Yaw rate
            0.5  # ✅ Add thrust to maintain hover
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        
    
    def send_body_velocity(vehicle, yaw_rate=0):
        """
        Send a yaw rate command to the vehicle.
        This is used to rotate the drone for marker alignment.
        """
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, 0, 0,
            0b00000111,  # Ignore roll and pitch, control yaw rate only
            [1, 0, 0, 0],  # Neutral quaternion (no roll/pitch adjustment)
            0, 0, yaw_rate,
            0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

    def get_camera_frame(self):
        """Grab latest camera image from internal buffer or ROS topic"""
        if hasattr(self, 'latest_image'):
            return self.latest_image
        return None
