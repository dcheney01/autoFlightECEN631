"""
mavsimPy
    - Chapter 4 assignment for Beard & McLain, PUP, 2012
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
#from python_tools.quit_listener import QuitListener
import parameters.simulation_parameters as SIM
from models.mav_dynamics_control import MavDynamics
from models.wind_simulation import WindSimulation
from message_types.msg_delta import MsgDelta
from viewers.manage_viewers import Viewers

#quitter = QuitListener()

VIDEO = True
PLOTS = False
ANIMATION = True
SAVE_PLOT_IMAGE = False

if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="videos/chp4/positive_rudder.avi",
                        bounding_box=(0, 0, 600, 600),
                        output_rate=SIM.ts_video)

#initialize the visualization
if ANIMATION or PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    mav_view = MavViewer(app=app)  # initialize the mav viewer
if PLOTS:
    # initialize view of data plots
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation, gust_flag=False)
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()
viewers = viewers = Viewers(animation=True, data=True)

# initialize the simulation time
sim_time = SIM.start_time
plot_time = sim_time
end_time = 5

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:
    # ------- set control surfaces -------------
    delta.elevator = -0.1248
    delta.aileron = 0.001836*0.0
    delta.rudder = 0.03026#*0.0
    delta.throttle = 0.6768

    # ------- physical system -------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    viewers.update(
        sim_time,
        mav.true_state,  # true states
        None,  # estimated states
        None,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )
        
    # # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

viewers.close(dataplot_name="ch4_data_plot")
