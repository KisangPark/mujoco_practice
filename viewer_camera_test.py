import mujoco
import mujoco_viewer

import numpy as np
import time

model = mujoco.MjModel.from_xml_path('models/camera_scene.xml')
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# reference position
pos_1 = [0, -0.8, 2.5, 0, -0.4, 0, 0]
pos_2 = [0.8, 0.8, 0.4, 0, -0.3, 0, 200]

mujoco.mj_resetData(model, data)

control_signal = pos_1
flag = 0
count = 0

# simulate and render
while True:
    if viewer.is_alive:

        # count
        count += 1
        if flag == 1 and count>2000:
            count=0
            control_signal = pos_2
            flag = 0
        elif flag == 0 and count>2000:
            count=0
            control_signal = pos_1
            flag = 1

        # set input control
        data.ctrl = control_signal
        
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()