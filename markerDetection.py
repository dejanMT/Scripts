#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import time
import math
import numpy as np
from cv_bridge import CvBridge
from dronekit import VehicleMode


class MarkerDetection(Node):
    def __init__(self):
        super().__init__('marker_detection')

        self.bridge = CvBridge()

        # Drone control variables
        self.vehicle = None
        self.controller = None
        self.CAMERA_MATRIX = None
        self.DIST_COEFF = None
        self.HORIZONTAL_RES = None
        self.VERTICAL_RES = None
        self.HORIZONTAL_FOV = None
        self.VERTICAL_FOV = None

        # Marker parameters
        self.id_to_find = 18
        self.marker_size = 20  # cm
        self.distance_from_marker = 2.0  # Desired hover distance in meters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        #self.parameters = aruco.DetectorParameters_create()
        self.parameters = aruco.DetectorParameters()


        # Control gains for fast & smooth response
        self.Kp_yaw = 1.0    # Increase yaw correction speed was 1.0
        self.Kp_x = 0.10      # Faster left/right movement
        self.Kp_y = 0.10      # Faster altitude correction
        self.Kp_z = 0.10      # Faster forward/backward movement

        self.deadband_px = 20 #8  # Reduce pixel tolerance for precise alignment
        self.yaw_alignment_counter = 0

        # State flags
        self.yaw_alignment_complete = False
        self.alignment_complete = False

        # ROS2 subscriber for camera feed
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.msg_receiver, 10)

    def msg_receiver(self, message):
        yaw_rate = 0.0

        # For simulation purposes only
        #np_data = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        #self.controller.latest_image = np_data 

        # For real-world-drone
        np_data = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        undistorted = cv2.undistort(np_data, self.CAMERA_MATRIX, self.DIST_COEFF)
        self.controller.latest_image = undistorted


        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        (corners, ids, _) = aruco.detectMarkers(
            gray_img, dictionary=self.aruco_dict, parameters=self.parameters)

        if ids is not None and self.id_to_find in ids:
            index = list(ids).index(self.id_to_find)
            ret = aruco.estimatePoseSingleMarkers(
                corners[index], self.marker_size, cameraMatrix=self.CAMERA_MATRIX, distCoeffs=self.DIST_COEFF)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

            x_avg = np.mean(corners[index][0][:, 0])
            center_x = self.HORIZONTAL_RES / 2.0
            x_error = x_avg - center_x
            #yaw_aligned = abs(x_error) <= self.deadband_px


            # if not self.yaw_alignment_complete:
            #     if not yaw_aligned:
            #         yaw_rate = -self.Kp_yaw * (x_error / center_x)
            #         yaw_rate = max(min(yaw_rate, 0.5), -0.5)
            #         print(f"Sending yaw command: x_error={x_error:.2f}, yaw_rate={yaw_rate:.2f}")
            #         self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)
            #         self.controller.send_yaw_rate(yaw_rate)
            #         return
            #     else:
            #         self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)  # stop everything
            #         self.controller.send_yaw_rate(0.0)
            #         self.yaw_alignment_complete = True



            # if not self.yaw_alignment_complete:
            #     yaw_rate = -self.Kp_yaw * (x_error / center_x)
            #     yaw_rate = max(min(yaw_rate, 0.5), -0.5)
            #     print(f"Sending yaw command: x_error={x_error:.2f}, yaw_rate={yaw_rate:.2f}")
            #     self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)
            #     self.controller.send_yaw_rate(yaw_rate)
            #     return

            if not self.yaw_alignment_complete:
                x_avg = np.mean(corners[index][0][:, 0])
                center_x = self.HORIZONTAL_RES / 2.0
                x_error = x_avg - center_x

                # Only consider it aligned if stable for multiple frames
                yaw_aligned = abs(x_error) <= self.deadband_px

                # Require stable alignment for 5 consecutive frames
                if self.yaw_alignment_counter >= 60:
                    self.yaw_alignment_counter += 0
                    print("✅ Yaw alignment complete.")
                    self.controller.send_yaw_rate(0.0)
                    self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)
                    self.yaw_alignment_complete = True
                    return
                else:
                    self.yaw_alignment_counter += 1
                    yaw_rate = -self.Kp_yaw * (x_error / center_x)
                    yaw_rate = max(min(yaw_rate, 0.4), -0.4)
                    print(f"🧭 Sending yaw command: x_error={x_error:.2f}, yaw_rate={yaw_rate:.2f}")
                    self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)
                    self.controller.send_yaw_rate(yaw_rate)
                    return









            # --- Position Alignment ---

            # Estimate marker distance from camera to marker (Z)
            marker_distance = tvec[2] / 100.0
            camera_to_center_offset = 0.20
            corrected_distance = marker_distance + camera_to_center_offset
            self.controller.marker_distance = corrected_distance  # Save distance to railing
            error_forward = corrected_distance - self.distance_from_marker

            # Lateral error in image pixels → meters (using fx = camera_matrix[0,0])
            fx = self.CAMERA_MATRIX[0][0]
            fy = self.CAMERA_MATRIX[1][1]

            x_px_offset = x_avg - center_x
            #y_avg = np.mean(corners[index][0][:, 1])
            target_y_position = self.VERTICAL_RES * 0.9 # closer to bottom    =============was * 0.9
            y_avg = np.mean(corners[index][0][:, 1])
            y_px_offset = y_avg - target_y_position


            # Convert pixel offset to meters at current depth
            lateral_error = (x_px_offset / fx) * marker_distance
            vertical_error = (y_px_offset / fy) * marker_distance

            # Alignment thresholds
            forward_aligned = abs(error_forward) <= 0.50 ####### was 0.10
            lateral_aligned = abs(lateral_error) <= 0.45 ####### was 0.10
            vertical_aligned = abs(vertical_error) <= 0.15 #was 0.2

            # Velocity corrections
            vx = self.Kp_z * error_forward if not forward_aligned else 0.0
            vy = self.Kp_x * lateral_error if not lateral_aligned else 0.0
            vz = self.Kp_y * vertical_error if not vertical_aligned else 0.0
            #vz = self.Kp_y * vertical_error if not vertical_aligned and vertical_error > 0 else 0.0



            # Clamp for stability
            vx = max(min(vx, 0.2), -0.2)
            vy = max(min(vy, 0.2), -0.2)
            vz = max(min(vz, 0.2), -0.2)

            # if self.alignment_complete:
            #     vz = 0.0

            # if yaw_aligned and forward_aligned and lateral_aligned and vertical_aligned:
            #     print("Full Alignment Complete. Hovering.")
            #     self.controller.send_local_ned_velocity(0.0, 0.0, vz)
            #     self.alignment_complete = True
            #     return

            #print(f"[DEBUG] Alignment Flags → yaw: {yaw_aligned}, forward: {forward_aligned}, " f"lateral: {lateral_aligned}, vertical: {vertical_aligned}")

            if lateral_aligned and vertical_aligned:
                print("✅ Full Alignment Complete. Rising to clear balcony.")
                self.controller.send_local_ned_velocity(0.0, 0.0, -0.2)  # ascend
                time.sleep(0.3)
                self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)

            # if marker_distance <= 4.2 and abs(lateral_error) <= 0.5 and abs(vertical_error) <= 0.1:
            #     print("✅ Full Alignment Complete. Proceeding to balcony.")
            #     self.alignment_complete = True


                # === Estimate Balcony Dimensions (from marker perspective) ===
                marker_corners = corners[index]
                c = marker_corners[0]  # shape: (4,2)

                # Marker width and height in pixels
                marker_px_width = np.linalg.norm(c[0] - c[1])  # top-left to top-right
                marker_px_height = np.linalg.norm(c[0] - c[3])  # top-left to bottom-left

                # Focal lengths
                fx = self.CAMERA_MATRIX[0][0]
                fy = self.CAMERA_MATRIX[1][1]

                # Use real-world marker size to compute scale
                marker_real_size = self.marker_size / 100.0  # convert cm to meters
                meters_per_pixel_x = marker_real_size / marker_px_width
                meters_per_pixel_y = marker_real_size / marker_px_height

                # Estimate full image dimensions in meters
                estimated_balcony_width = self.HORIZONTAL_RES * meters_per_pixel_x
                estimated_balcony_height = self.VERTICAL_RES * meters_per_pixel_y

                print(f"📏 Estimated Balcony Width: {estimated_balcony_width:.2f} m")
                print(f"📏 Estimated Balcony Height: {estimated_balcony_height:.2f} m")

                # === Drone Clearance Check ===
                drone_width = 0.6  # meters
                drone_height = 0.2  # meters

                if estimated_balcony_width < drone_width + 0.2 or estimated_balcony_height < drone_height + 0.2:
                    print("❌ Balcony too small. Aborting entry.")
                    self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)
                    return

                # ✅ All checks passed
                self.alignment_complete = True
                return



            self.controller.send_local_ned_velocity(vx, vy, vz)

            print(f"Aligning: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, "
                f"dist={corrected_distance:.2f}m, lat_err={lateral_error:.2f}, vert_err={vertical_error:.2f}")

        # else:
        #     if self.alignment_complete:
        #         print("Marker lost. Hovering.")
        #         self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)
        #         self.alignment_complete = False
        #     else:
        #         print("No marker detected.")
        #         self.controller.send_local_ned_velocity(0.0, 0.0, 0.0)


        time.sleep(0.1)


    def run(self):
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    node = MarkerDetection()
    node.run()
