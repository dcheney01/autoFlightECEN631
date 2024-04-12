
import numpy as np

from message_types.msg_runway import MsgRunway
from message_types.msg_waypoints import MsgWaypoints

def generate_landing_trajectory(runway:MsgRunway, curr_position:np.ndarray):
    """
        # This function needs to return a list of waypoints where each of the following stages will happen
        - Descent
        - Flare-up
        - Touchdown
        - Taxi (slow-down to a stop)
    """
    touchdown_altitude = 0

    # find point on line that is 10m away from the end of the runway to stop the taxi
    taxi_waypoint = np.array([runway.start[0,0] + (runway.length-40)*np.cos(runway.orientation), runway.start[1,0] - (runway.length-40)*np.sin(runway.orientation), touchdown_altitude]).reshape((3,1))

    # touchdown 10 m after runway starts
    touchdown_waypoint = np.array([runway.start[0,0] + 10*np.cos(runway.orientation), runway.start[1,0] - 10*np.sin(runway.orientation), touchdown_altitude]).reshape((3,1))
    
    flare_altitude = -10
    flare_time = 30
    flare_length = flare_time * 25

    flare_waypoint = np.array([touchdown_waypoint[0,0] - flare_length*np.cos(runway.orientation), touchdown_waypoint[1,0] + flare_length*np.sin(runway.orientation), flare_altitude]).reshape((3,1))

    descent_multiplier = 10
    descent_time = (flare_waypoint[2,0] - curr_position[2,0]) *descent_multiplier
    descent_start_north = flare_waypoint[0,0] - descent_time*np.cos(runway.orientation)
    descent_start_east = flare_waypoint[1,0] + descent_time*np.sin(runway.orientation)
    descent_waypoint = np.array([descent_start_north, descent_start_east, curr_position[2,0]]).reshape((3,1))

    # create approach waypoint 100 m from the descent waypoint
    approach_waypoint = np.array([descent_waypoint[0,0] - 100*np.cos(runway.orientation), descent_waypoint[1,0] + 100*np.sin(runway.orientation), curr_position[2,0]]).reshape((3,1))

    waypoints = MsgWaypoints()
    waypoints.type = 'landing'
    waypoints.add(descent_waypoint, 25, 0, 0, 0, 0)
    waypoints.add(flare_waypoint, 15, 0, 0, 0, 0)
    waypoints.add(touchdown_waypoint, 10, 0, 0, 0, 0)
    waypoints.add(taxi_waypoint, 0, 0, 0, 0, 0)

    return waypoints, approach_waypoint