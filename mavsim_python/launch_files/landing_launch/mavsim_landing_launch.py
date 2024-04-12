import os, sys
import numpy as np
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import time

import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from controllers.autopilot import Autopilot
from planners.path_follower import PathFollower
from planners.path_manager import PathManager
from planners.path_planner import PathPlanner
from viewers.view_manager import ViewManager
from message_types.msg_world_map import MsgWorldMap
from message_types.msg_waypoints import MsgWaypoints
from message_types.msg_runway import MsgRunway
from message_types.msg_delta import MsgDelta
from planners.landing_planner import *

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
wind = WindSimulation(SIM.ts_simulation, gust_flag=True, steady_state=np.array([3, 3, 0]).reshape((3,1)))
autopilot = Autopilot(SIM.ts_simulation)
path_follower = PathFollower()
path_manager = PathManager()
viewers = ViewManager(animation=True, data=True, planning=True, map=False, runway=True, waypoint=True)
world_map = MsgWorldMap()
world_map.set_to_empty_map()
runway = MsgRunway()
planner_type = 'rrt_straightline'  # plan path to get to the descent waypoint
initial_path_planner = PathPlanner(type=planner_type)

# Sim Parameters
sim_time = SIM.start_time
end_time = 200
in_landing_mode = False


# DEMO SETTING STUFF ###################################################################
length, orientation = 900, np.deg2rad(0)
runway_start = np.array((3000, 0, -100)).reshape((3,1))
runway.update(runway_start, length, orientation)
mav_start = np.array([mav.true_state.north, mav.true_state.east, -mav.true_state.altitude]).reshape((3,1))

# descent_waypoint_for_initial_manager, specific_instructions_for_landing(descent, flare-up, touchdown) = landing_planner.plan()
landing_waypoints, approach_point = generate_landing_trajectory(runway, mav_start)

# -------path planner for navigating to the descent waypoint------
waypoints = MsgWaypoints()
waypoints.type = 'fillet'
waypoints.add(np.array([[mav.true_state.north, mav.true_state.east, -mav.true_state.altitude]]).T, 25, mav.true_state.chi, 0, 0, 0)
waypoints.add(approach_point, 25, orientation, 0, 0, 0)
waypoints.add(landing_waypoints.ned[:,0].reshape((3,1)), 25, landing_waypoints.course[0], 0, 0, 0)
########################################################################################

coutner = 0
path_tmp = None
landing_mode = "Approach" # "Descent", "Flare", "Touchdown", "Taxi"
# Main Sim Loop
print("Starting Main Sim Loop...")
while sim_time < SIM.end_time:
    # print(f"north: {mav.true_state.north}, east: {mav.true_state.east}, altitude: {mav.true_state.altitude}")
    if landing_mode == "Taxi":
        print("Taxiing to a stop...")
        delta.throttle = 0.0
        time.sleep(5)
        # commanded_state = mav.true_state
    else:
        # -------path manager-------------
        path = path_manager.update(waypoints, PLAN.R_min, mav.true_state)
        if path is not None:
            path_tmp = path

        if path is None:
            if landing_mode == "Approach":
                print("Switching to Descent...")
                path_manager = PathManager()
                waypoints = landing_waypoints
                landing_mode = "Descent"
                path = path_manager.update(waypoints, PLAN.R_min, mav.true_state)
                path.line_end = waypoints.ned[:, 1].reshape((3,1))
                continue
        if path_manager._ptr_current == 2 and landing_mode == "Descent":
            print("Switching to Flare...")
            landing_mode = "Flare"
            path.line_end = None #waypoints.ned[:, 2].reshape((3,1))
        elif path_manager._ptr_current == 3 and landing_mode == "Flare":
            print("Switching to Touchdown...")
            landing_mode = "Touchdown"
        elif path_manager._ptr_current == 4 and landing_mode == "Touchdown":
            print("Switching to Taxi...")
            landing_mode = "Taxi"
            continue

        # -------path follower-------------
        autopilot_commands = path_follower.update(path, mav.true_state)

        # -------autopilot-------------
        delta, commanded_state = autopilot.update(autopilot_commands, mav.true_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    if landing_mode == "Touchdown" or landing_mode == "Taxi":
        coutner += 1
        mav.true_state.Va -= 0.01*coutner
        mav.true_state.Vg -= 0.01*coutner
        if mav.true_state.Va < 0:
            mav.true_state.Va = 0
        if mav.true_state.Vg < 0:
            mav.true_state.Vg = 0

    if mav.true_state.altitude <= 0:
        mav.true_state.altitude = 0

    # -------update viewer-------------
    viewers.update(
        sim_time,
        true_state=mav.true_state,  # true states
        estimated_state=mav.true_state,  # estimated states
        commanded_state=commanded_state,  # commanded states
        delta=delta,  # inputs to aircraft
        path=path_tmp, # path
        waypoints=waypoints, # waypoints
        runway=runway,
        map=world_map,  # map of world
    )

    # -------increment time-------------
    sim_time += SIM.ts_simulation

# close viewers
viewers.close(dataplot_name="ch12_data_plot")