"""
Mujoco Arm
Keyboard teleop using glfw callback

1. GLFW multiple camera view
    - Two camera: overview & ego-centric
    - overview camera: free moving (using mouse)
    - ego-centric camera: fixed to gripper base joint
    
    GLFW Viewing code setup
        1) get mujoco model data & initialize glfw
        2) create glfw window
        3) setup scene & camera, append to list
        4) step & render through while loop


2. GLFW callback
    - keyboard callback for teleop moving
    - A,D: horizontal
    - NM, KL, OP: vertical
    - UI: gripper
    - R: reset
"""

import mujoco

import numpy as np
import time

import glfw
import sys


""" SETUP """

# 1-1. get model & data
model = mujoco.MjModel.from_xml_path('models/camera_scene.xml')
data = mujoco.MjData(model)

# return camera ID
# mujoco viewer camera -> not included in camera ID (id -1)
# print(model.camera("egocentric").id)



# 1-2. lists for multiple cameras & windows
windows = []
contexts = []
scenes = []
cameras = []
options = []
viewport=mujoco.MjrRect(0, 0, 0, 0)

if not glfw.init():
    sys.exit("couldn't initialize glfw")

# make mujoco context -> not neccessary...
# Attribute error: no attribute named mjr_makeContext // context generated before mjr_makeContext
# mujoco.mjr_makeContext(model, mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150))



# 1-3. create window and context

# 1-3-a. free camera

# create window
window = glfw.create_window(1280, 720, f"Camera: interactive", None, None)
# validate
if not window:
    glfw.terminate()
    raise RuntimeError("GLFW window creation failed")

# make glfw context
glfw.make_context_current(window)

# scene, camera, options
scene = mujoco.MjvScene(model, maxgeom=1000)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
opt = mujoco.MjvOption()

# append to list
scenes.append(scene)
cameras.append(cam)
options.append(opt)
windows.append(window)
contexts.append(mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150))


# 1-3-b. fixed camera
# create window
window = glfw.create_window(640, 480, f"Camera: egocentric fixed", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("GLFW window creation failed")

# make glfw context
glfw.make_context_current(window)

# scene, camera, options
scene = mujoco.MjvScene(model, maxgeom=1000)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
cam.fixedcamid = 0
opt = mujoco.MjvOption()

# append to list
scenes.append(scene)
cameras.append(cam)
options.append(opt)
windows.append(window)
contexts.append(mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150))


# 1-4. set rendering interval
render_interval = 0.1
last_render_time = time.time()



""" GLFW CALLBACK: CAMERA MOVEMENT """

# 2-1. initialize global variable
last_x, last_y = 0, 0
mouse_button = None
control_signal = [0, -0.8, 2.5, 0, -0.3, 0, 0]

# 2-2. define glfw callbacks

def mouse_button_callback(window, button, action, mods):
    global mouse_button
    if action == glfw.PRESS:
        mouse_button = button
        # print("mouse pressed")
    elif action == glfw.RELEASE:
        mouse_button = None

def cursor_pos_callback(window, xpos, ypos):
    global last_x, last_y, mouse_button
    dx = (xpos - last_x)/1000
    dy = (ypos - last_y)/1000
    last_x, last_y = xpos, ypos
    # print(f"mouse position: {xpos}, {ypos}")

    if mouse_button is not None:
        action = {
            glfw.MOUSE_BUTTON_LEFT: mujoco.mjtMouse.mjMOUSE_ROTATE_H,
            glfw.MOUSE_BUTTON_RIGHT: mujoco.mjtMouse.mjMOUSE_MOVE_H,
            glfw.MOUSE_BUTTON_MIDDLE: mujoco.mjtMouse.mjMOUSE_ZOOM
        }.get(mouse_button, None)

        if action is not None:
            mujoco.mjv_moveCamera(model, action, dx, dy, scenes[0], cameras[0])
            # print("cam moved")

def scroll_callback(window, xoffset, yoffset):
    # Zoom camera with scroll wheel
    mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -yoffset/100, scenes[0], cameras[0])
    # print(f"offsets x:{xoffset}, y:{yoffset}")




# keyboard teleop callback
def key_callback(window, key, scancode, action, mods):
    # control_signal list to control (0, 1, 2, 4, 6)
    # key: integer, key code -> glfw.KEY_R
    # scancode: integer, os-specific code for key, not used
    # action: integer, action -> glfw.PRESS, glfw.RELEASE
    # mods: integer, modifier key s.a. SHIFT, CTRL, ALT, etc.
    global control_signal

    if action == glfw.PRESS: # no need to 
        # 1. horizontal (index 0)
        if key == glfw.KEY_A: # left move
            control_signal[0] += 0.1
        elif key == glfw.KEY_D:
            control_signal[0] -= 0.1
        
        # 2. bottom vertical (index 1)
        if key == glfw.KEY_N: # left move
            control_signal[1] += 0.1
        elif key == glfw.KEY_M:
            control_signal[1] -= 0.1

        # 3. middle vertical (index 2)
        if key == glfw.KEY_K: # left move
            control_signal[2] += 0.1
        elif key == glfw.KEY_L:
            control_signal[2] -= 0.1

        # 4. bottom vertical (index 4)
        if key == glfw.KEY_O: # left move
            control_signal[4] += 0.1
        elif key == glfw.KEY_P:
            control_signal[4] -= 0.1

        # 5. gripper (index 6)
        if key == glfw.KEY_U: # left move
            control_signal[6] += 5
        elif key == glfw.KEY_I:
            control_signal[6] -= 5

        if key == glfw.KEY_R:
            control_signal = np.zeros(7).tolist()
            mujoco.mj_resetData(model, data)
            control_signal = [0, -0.8, 2.5, 0, -0.3, 0, 0]




# 2-3. set glfw callbacks
glfw.set_mouse_button_callback(windows[0], mouse_button_callback)
glfw.set_cursor_pos_callback(windows[0], cursor_pos_callback)
glfw.set_scroll_callback(windows[0], scroll_callback)

# 2-4. keyboard teleop callback
glfw.set_key_callback(windows[0], key_callback)



""" SIMULATION LOOP """

# while loop
while not any([glfw.window_should_close(win) for win in windows]):
    
    # refine control signal
    # if data too big or small, cut (1.707 radian)

    # set input control
    data.ctrl = control_signal

    # step
    mujoco.mj_step(model, data)
    # print("steped")

    # determine frame rate
    current_time = time.time()

    if current_time - last_render_time >= render_interval:
        last_render_time = current_time

        for i, window in enumerate(windows):
            
            # print(f"making context for window {i}")
            # make context, set width & height
            glfw.make_context_current(window)
            width, height = glfw.get_framebuffer_size(window)
            viewport.width = width
            viewport.height = height

            # update scene & render
            mujoco.mjv_updateScene(model, data, options[i], None, cameras[i],
                                mujoco.mjtCatBit.mjCAT_ALL, scenes[i])
            mujoco.mjr_render(viewport, scenes[i], contexts[i])

            glfw.swap_buffers(window)

        glfw.poll_events()
        time.sleep(0.01)

# Clean up
glfw.terminate()