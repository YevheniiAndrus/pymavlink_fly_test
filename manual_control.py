from numpy import block
from pymavlink import mavutil
import time
from enum import Enum
import math

class FlightModes(int):
    MANUAL = 0
    FBWA = 1
    AUTO = 9
    TAKEOFF = 13
    GUIDED = 15

def print_plane_attitude(plane_attitude: dict):
    roll, pitch, yaw = euler_from_quaternion(plane_attitude['q1'], plane_attitude['q2'],
        plane_attitude['q3'], plane_attitude['q4'])
    print("Roll - {}, Pitch - {}, Yaw - {}".format(roll, pitch, yaw))

# Convert degrees to quaternion
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def euler_from_quaternion(q1, q2, q3, q4):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (q4 * q1 + q2 * q3)
    t1 = +1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (q4 * q2 - q3 * q1)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (q4 * q3 + q1 * q2)
    t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw_z = math.atan2(t3, t4)
     
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z) # in degrees


def get_vehicle_alt(the_connection):
    global_pos = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
    altitude = global_pos['alt'] * 0.0001
    print("Altitude is {}".format(altitude))
    return altitude


def send_attitude_target(the_connection, roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = 0.0, thrust = 0.5):
    print("Setting up attitude target...")

    attitude_quaternion = to_quaternion(roll_angle, pitch_angle, yaw_angle)
    msg = the_connection.mav.set_attitude_target_encode(
        0,
        the_connection.target_system,
        the_connection.target_component,
        0b10111000,
        attitude_quaternion,
        0, 0, 0,
        thrust
    )
    the_connection.mav.send(msg)


def set_attitude(the_connection, roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(the_connection, roll_angle, pitch_angle,
                         yaw_angle, thrust)
    start = time.time()
    while time.time() - start < duration:
        the_connection.mav.command_long_send(the_connection.target_system,
                                                   the_connection.target_component,
                                                   mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                                   0, 31, 0, 0, 0, 0, 0, 0)
        msg = the_connection.recv_match(type='ATTITUDE_QUATERNION')
        if(msg):
            print_plane_attitude(msg.to_dict())

        send_attitude_target(the_connection, roll_angle, pitch_angle,
                             yaw_angle, thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(the_connection, 0, 0, 0, thrust)


# create connection and arm vehicle
def init_plane():
    print("Plane initialization...")
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    the_connection.mav.command_long_send(the_connection.target_system, 
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    return the_connection


def start_plane(the_connection):
    # set takeoff mode
    print("Takeoff...")
    the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, 1, FlightModes.TAKEOFF, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)


def plane_manual_control(the_connection):
    print("Setting up guided mode")
    the_connection.mav.command_long_send(the_connection.target_system,
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                         0, 1, FlightModes.GUIDED, 0, 0, 0, 0, 0)

    roll_angle = 15.0
    pitch_angle = 0.0
    yaw_angle = 0.0
    set_attitude(the_connection, roll_angle, pitch_angle, yaw_angle, 0.9, 300)


def main():
    the_connection = init_plane()
    start_plane(the_connection)

    time.sleep(10)
    # turn on manual mode
    plane_manual_control(the_connection)


if __name__ == "__main__":
    main()