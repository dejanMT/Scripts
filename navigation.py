from dronekit import LocationGlobalRelative
import time
from pymavlink import mavutil

def goto_gps(vehicle, latitude, longitude, altitude):
    """Fly to a specified GPS coordinate."""
    print("Navigating to GPS: ({}, {}, {})".format(latitude, longitude, altitude))
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)
    # A proper implementation would check for arrival.
    time.sleep(10)

def goto_local(vehicle, x, y, z, yaw=None):
    """
    Move the drone to a specific local coordinate.
    This function uses MAVLink's SET_POSITION_TARGET_LOCAL_NED to move
    the drone in the local frame.
    """
    print("Navigating to Local Position: X={}, Y={}, Z={}".format(x, y, z))

    type_mask = 0b0000111111111000  # position only

    if yaw is not None:
        type_mask = 0b0000111111110000  # include yaw

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, -z,
        0, 0, 0,
        0, 0, 0,
        yaw if yaw is not None else 0,
        0
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()


    # Wait until the drone is near the target position (example logic)
    while True:
        current_position = vehicle.location.local_frame
        dist_x = abs(current_position.north - x)
        dist_y = abs(current_position.east - y)
        dist_z = abs(current_position.down + z)  # In NED, z is negative when up

        print("Current Position: X={:.2f}, Y={:.2f}, Z={:.2f}".format(
            current_position.north, current_position.east, -current_position.down))

        if dist_x < 0.2 and dist_y < 0.2 and dist_z < 0.2:
            print("Arrived at target location.")
            break

        time.sleep(1)
