"""
Mujoco Control
    - model: a0912 robotic arm
    - actuator: position controller
"""

import mujoco 
import mujoco.viewer

import numpy as np
import time

from PIL import Image
import cv2

# two pre defined position / forward & backward

model = mujoco.MjModel.from_xml_path("robotic_arm/arm_scene.xml")
data = mujoco.MjData(model)

print(data.ctrl) # list of length 6 -> 6 joint position

pos_1 = [0, -0.8, 2.5, 0, -0.3, 0, 0]
pos_2 = [0.8, 1.3, 0.2, 0, -0.3, 0, 200]


# 1. use launch passive viewer

with mujoco.viewer.launch_passive(model, data) as viewer:

    # reset data
    mujoco.mj_resetData(model, data)
    
    # flag for position select
    control_signal = pos_1
    flag = 0
    count = 0

    while viewer.is_running():
        # print(count)
        count += 1
        # print(f"count: {count}")

        # make position according to flag
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
        # print(data.ctrl)
        # step
        mujoco.mj_step(model, data)
        # sync viewer
        viewer.sync()


"""
# 2. use mujoco renderer

renderer = mujoco.Renderer(model)
# media.show_image(renderer.render())

# reset data
mujoco.mj_resetData(model, data)
    
# flag for position select
control_signal = pos_1
flag = 0
count = 0

while True:
    # print(count)
    count += 1
    # print(f"count: {count}")

    # make position according to flag
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

    # step
    mujoco.mj_step(model, data)

    # use renderer update_scene
    renderer.update_scene(data)

    rgb_image = renderer.render()

    # image = Image.fromarray(renderer.render())
    # image.show()
    # time.sleep(0.1)

    image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    cv2.imshow("mujoco rendering", image)
    cv2.waitKey(10)
    
cv2.destroyAllWindows()
"""