import mujoco
import mujoco_viewer

import numpy as np
import time

model = mujoco.MjModel.from_xml_path('mjcf/indy7/scene.xml')
data = mujoco.MjData(model)

# use mujoco_viewer
viewer = mujoco_viewer.MujocoViewer(model, data)

mujoco.mj_resetData(model, data)

# step when viewer alive
while True:
    if viewer.is_alive:

        # # count
        # count += 1
        # if flag == 1 and count>2000:
        #     count=0
        #     control_signal = pos_2
        #     flag = 0
        # elif flag == 0 and count>2000:
        #     count=0
        #     control_signal = pos_1
        #     flag = 1

        # # set input control
        # data.ctrl = control_signal
        
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()