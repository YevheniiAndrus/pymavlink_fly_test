from pymavlink import mavutil
import time
from dataclasses import dataclass 
import math

# Lenth of Earth ecuator in km
R_earth = 40000

# distance between two geographical points (in km)
# TODO: check calculations for correctness
def distance_to_wp(current_lat, current_lon, wp_lat, wp_lon):
    current_lat_rad = current_lat / 10**7  * math.pi / 180
    current_lon_rad = current_lon / 10**7  * math.pi / 180

    goal_lat_rad = wp_lat / 10**7 * math.pi / 180
    goal_lon_rad = wp_lon / 10**7 * math.pi / 180

    d = R_earth * math.acos((math.sin(current_lat_rad) * math.sin(goal_lat_rad) + 
            math.cos(current_lat_rad) * math.cos(goal_lat_rad) * math.cos(current_lon_rad - goal_lon_rad)))

    return d


@dataclass
class Waypoint:
    lat: float
    lon: float


# create connection and arm vehicle
def init_plane():
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
    the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, 1, 13, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)


def set_guidance_mode(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,
                                             the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                             0, 1, 15, 0, 0, 0, 0, 0)


def move_to_waypoint(the_connection, wp: Waypoint):
    the_connection.mav.mission_item_int_send(the_connection.target_system,
                                             the_connection.target_component,
                                             0, 3,
                                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2,
                                             0, 0, 0, 0, 0,
                                             int(wp.lat), int(wp.lon), 300)


def main():
    print("Mavlink test")
    the_connection = init_plane()
    start_plane(the_connection)
    
    # wait for 5 seconds before moving to waypoints
    time.sleep(10)

    # set guidance mode after takeoff
    set_guidance_mode(the_connection)

    # setup waypoints
    wp_list = [Waypoint(-35.3603495 * 10**7, 149.1600986 * 10**7),
               Waypoint(-35.3595795 * 10**7, 149.1691322 * 10**7),
               Waypoint(-35.3674714 * 10**7, 149.1713209 * 10**7),
               Waypoint(-35.3689762 * 10**7, 149.1608067 * 10**7)]
    current_wp = 0
    wp_reached = False


    print("Moving to waypoints")
    move_to_waypoint(the_connection, wp_list[current_wp])

    while True:
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT')
        if msg:
            if wp_reached:
                move_to_waypoint(the_connection, wp_list[current_wp])
                wp_reached = False

            msg_dict = msg.to_dict()
            current_lat = msg_dict['lat']
            current_lon = msg_dict['lon']

            d = distance_to_wp(current_lat, current_lon, wp_list[current_wp].lat, wp_list[current_wp].lon)
            print("Distance to next waypoint: {}".format(d))
            if d < 1.0:
                current_wp = current_wp + 1
                if current_wp == len(wp_list):
                    break
                wp_reached = True
                print("Waypoint has been reached")
            else:
                wp_reached = False

if __name__ == "__main__":
    main()
