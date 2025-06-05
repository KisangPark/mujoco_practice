"""
Mujoco Control
    - model: a0912 robotic arm
    - actuator: position controller
"""

import mujoco 
import mujoco.viewer

import numpy as np
import time

# two pre defined position / forward & backward

model = mujoco.MjModel.from_xml_path("robotic_arm/scene.xml")
data = mujoco.MjData(model)

print(data.ctrl) # list of length 6 -> 6 joint position

pos_1 = 0.0
pos_2 = 140.0


# 1. launch passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:

    # reset data
    mujoco.mj_resetData(model, data)
    
    # flag for position select
    control_signal = pos_1
    flag = 0
    count = 0

    while viewer.is_running():
        count += 1
        # print(f"count: {count}")

        # make position according to flag
        if flag == 1 and count>1000:
            count=0
            control_signal = pos_2
            flag = 0
        elif flag == 0 and count>1000:
            count=0
            control_signal = pos_1
            flag = 1
        
        # set input control
        data.ctrl = control_signal
        # print(data.ctrl)
        # step
        mujoco.mj_step(model, data)
        # sync viewer
        viewer.sync()
