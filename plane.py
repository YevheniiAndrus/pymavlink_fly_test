from numpy import roll
from pymavlink import mavutil
import time
import math

from PIController import PIController


class FlightModes(int):
    MANUAL = 0
    FBWA = 1
    AUTO = 9
    TAKEOFF = 13
    GUIDED = 15


class Request(int):
    ATTITUDE_QUATERNION = 31
    LOCAL_POSIITON_NED = 32
    GLOBAL_POSITION_INT = 33


class Plane:
    def __init__(self, the_connection) -> None:
        """Arm plane before send commands"""
        the_connection.mav.command_long_send(the_connection.target_system, 
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1, 0, 0, 0, 0, 0, 0)

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)


    def takeoff(self, the_connection):
        """Setting up takeoff mode"""
        print("Takeoff...")
        the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, 1, FlightModes.TAKEOFF, 0, 0, 0, 0, 0)

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)


    def __get_current_heading(self, the_connection):
        the_connection.mav.command_long_send(the_connection.target_system,
                                                   the_connection.target_component,
                                                   mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                                   0, Request.GLOBAL_POSITION_INT, 0, 0, 0, 0, 0, 0)
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT')

        current_heading = None
        if (msg):
            current_heading = msg.to_dict()['hdg']

        if current_heading is not None:
            return current_heading * 0.01
        
        return None
    
    def establish_heading(self, the_connection, target_heading):
        print("Establishing heading of {}".format(target_heading))
        current_heading = None
        pi_controller = PIController(0.8, 2.0)

        while True:
            current_heading = self.__get_current_heading(the_connection)
            if current_heading is None:
                continue

            roll_angle = pi_controller.update(target_heading, current_heading)
            print("Roll angle: {}".format(roll_angle))
            if pi_controller.complete():
                return
                
            self._send_attitude_target(the_connection, roll_angle, 0.0, 0.0, 0.5)
            #time.sleep(0.05)


    def set_guided_mode(self, the_connection):
        print("Setting up guided mode")
        the_connection.mav.command_long_send(the_connection.target_system,
                                             the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                             0, 1, FlightModes.GUIDED, 0, 0, 0, 0, 0)


    def set_attitude(self, the_connection, roll_angle = 0.0,
                     pitch_angle = 0.0, yaw_angle = 0.0,
                     thrust = 0.5, duration = 0):
        """Manually set attitude for needed amount of time"""

        self._send_attitude_target(the_connection, roll_angle, pitch_angle,
                             yaw_angle, thrust)
        start = time.time()
        while time.time() - start < duration:
            the_connection.mav.command_long_send(the_connection.target_system,
                                                   the_connection.target_component,
                                                   mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                                   0, Request.GLOBAL_POSITION_INT, 0, 0, 0, 0, 0, 0)
            msg = the_connection.recv_match(type='GLOBAL_POSITION_INT')
            if(msg):
                print("Heading - {}".format(msg.to_dict()['hdg']))

            self._send_attitude_target(the_connection, roll_angle, pitch_angle,
                                 yaw_angle, thrust)
            time.sleep(0.05)
         # Reset attitude, or it will persist for 1s more due to the timeout
        self._send_attitude_target(the_connection, 0, 0, 0, thrust)


    def _send_attitude_target(self, the_connection, roll_angle = 0.0, pitch_angle = 0.0,
                                yaw_angle = 0.0, thrust = 0.5):
        """Setting up attitude target for a moment"""

        print("Setting up attitude target...")

        attitude_quaternion = self.__to_quaternion(roll_angle, pitch_angle, yaw_angle)
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


    def __to_quaternion(self, roll, pitch, yaw):
        """Convert roll, pitch, yaw in degrees to quaternion"""
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