import mujoco
import mujoco.viewer

import numpy as np
import time

model = mujoco.MjModel.from_xml_path('mjcf/indy7/scene.xml')
data = mujoco.MjData(model)


"""VIEW & SAVE XML WITH NATIVE VIEWER"""

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # step
        mujoco.mj_step(model, data)

        # sync viewer
        viewer.sync()


""" SAVE USING SAVELASTXML"""

# mjcf_temp_string = mujoco.mj_saveLastXML("mjcf/indy7/indy7.xml", model)
