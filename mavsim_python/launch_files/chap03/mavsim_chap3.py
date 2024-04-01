"""
mavsimPy
    - Chapter 2 launch file for Beard & McLain, PUP, 2012
    - Update history:  
        12/27/2018 - RWB
        1/17/2019 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import numpy as np
import parameters.simulation_parameters as SIM
from message_types.msg_delta import MsgDelta
from models.mav_dynamics import MavDynamics
from viewers.manage_viewers import Viewers

#quitter = QuitListener()

#Running Parameters
VIDEO = True
PLOTS = False
ANIMATION = True
SAVE_PLOT_IMAGE = False

#initialize the viewers
if ANIMATION or PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    mav_view = MavViewer(app=app)  # initialize the mav viewer
if PLOTS:
    # initialize view of data plots
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="videos/chp3/test.avi",
                        bounding_box=(0, 0, 600, 600),
                        output_rate=SIM.ts_video)
    
# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()
viewers = Viewers(animation=True, data=True)

# initialize the simulation time
sim_time = SIM.start_time
end_time = 10

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:
    # ------- vary forces and moments to check dynamics -------------
    fx = 0  # 10
    fy = 0  # 10
    fz = 0  # 10
    Mx = 0.0  # 0.1
    My = 0.  # 0.1
    Mz = 1.0  # 0.1
    forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T

    # ------- physical system -------------
    mav.update(forces_moments)  # propagate the MAV dynamics

    # ------- update viewers -------------
    viewers.update(
        sim_time,
        mav.true_state,  # true states
        None,  # estimated states
        None,  # commanded states
        None,  # inputs to aircraft
        None,  # measurements
    )

    # ------- increment time -------------
    sim_time += SIM.ts_simulation

    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

# Save an Image of the Plot
viewers.close(dataplot_name="ch3_data_plot")