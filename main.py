#!/usr/bin/env python3
import time
import numpy as np
from math import pi
from dronekit import connect
import rclpy
from markerDetection import MarkerDetection
from navigation import goto_local, goto_gps
from balconyLanding import BalconyLanding
import flightController as fc
import os

# Global vehicle connection (created once)
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)

# Global camera configuration

# For simulation purposes only
# CAMERA_MATRIX = np.array([
#     [1061.65, 0.0, 640.5],
#     [0.0, 1061.65, 360.5],
#     [0.0, 0.0, 1.0]
# ], dtype=np.float32)

# For real-world application, load from calibration files
calib_path = "/home/drone/video2calibration/calibrationFiles/"
CAMERA_MATRIX = np.loadtxt(os.path.join(calib_path, 'cameraMatrix.txt'), delimiter=',')
DIST_COEFF = np.loadtxt(os.path.join(calib_path, 'cameraDistortion.txt'), delimiter=',')


DIST_COEFF = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
HORIZONTAL_RES = 600
VERTICAL_RES = 480
HORIZONTAL_FOV = 102 * (pi / 180)
VERTICAL_FOV = 67 * (pi / 180)

# Global flight parameters in meters
TAKEOFF_HEIGHT = 4 # meters

# Navigation coordinates
latitude = 11   # y
longitude = 17  # x
altitude = TAKEOFF_HEIGHT  # z

# Create a FlightController instance using the global vehicle
controller = fc.FlightController(vehicle=vehicle)
controller.camera_matrix = CAMERA_MATRIX

if __name__ == '__main__':
    try:
        print("Starting mission...")
        # Step 1: Arm and takeoff using the FlightController
        controller.arm_and_takeoff(TAKEOFF_HEIGHT)
        time.sleep(2)

        # Step 2: Navigate to a local coordinate (for testing)
        #goto_local(vehicle, longitude, latitude, altitude, yaw=0.0)

        # For GPS navigation, uncomment the following line 
        #goto_gps(vehicle, latitude, longitude, altitude) #ADJUST GPS coordinates!!!

        # Step 3: Start ROS 2 node for marker detection
        rclpy.init()
        marker_detection_node = MarkerDetection()
        marker_detection_node.vehicle = vehicle
        marker_detection_node.controller = controller
        marker_detection_node.CAMERA_MATRIX = CAMERA_MATRIX
        marker_detection_node.DIST_COEFF = DIST_COEFF
        marker_detection_node.HORIZONTAL_RES = HORIZONTAL_RES
        marker_detection_node.VERTICAL_RES = VERTICAL_RES
        marker_detection_node.HORIZONTAL_FOV = HORIZONTAL_FOV
        marker_detection_node.VERTICAL_FOV = VERTICAL_FOV

        #rclpy.spin(marker_detection_node)
        while rclpy.ok():
            rclpy.spin_once(marker_detection_node)
            if marker_detection_node.alignment_complete:
                print("Marker alignment complete. Proceeding to balcony landing...")
                break

        #marker_detection_node.destroy_node()
        #rclpy.shutdown()

        balcony_lander = BalconyLanding(controller, vehicle)
        landing_successful = balcony_lander.execute(controller.latest_image)


        if not landing_successful:
            print("❌ Landing was aborted.")
        else:
            print("✅ Landing executed successfully.")

    except Exception as e:
        print("An error occurred:", e)
    finally:
        # Cleanup
        marker_detection_node.destroy_node()
        rclpy.shutdown()
        vehicle.close()