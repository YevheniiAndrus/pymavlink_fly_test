from numpy import block, roll
from pymavlink import mavutil
import time

from plane import Plane


def main():
    # init mavlink connection
    print("Plane initialization...")
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
    
    # Plane instance
    plane = Plane(the_connection)
    plane.takeoff(the_connection)

    # wait until takeoff
    time.sleep(10)
    # set guided mode
    plane.set_guided_mode(the_connection)

    plane.establish_heading(the_connection, 70.0)
    plane.establish_heading(the_connection, 120)
    plane.establish_heading(the_connection, 230)


if __name__ == "__main__":
    main()