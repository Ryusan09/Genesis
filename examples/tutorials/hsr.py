import numpy as np
import genesis as gs
from pynput import keyboard

########################## 初期化 ##########################
gs.init(backend=gs.gpu)

########################## シーンの作成 ##########################
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(0, -3.5, 2.0),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=30,
        res=(960, 640),
        max_FPS=60,
    ),
    sim_options=gs.options.SimOptions(dt=0.01),
    show_viewer=True,
)

########################## エンティティ ##########################
plane = scene.add_entity(gs.morphs.Plane())
hsr = scene.add_entity(
    gs.morphs.URDF(file="../../../hsrb_description/hsr.urdf"),
)

########################## ビルド ##########################
scene.build()

########################## 関節の設定 ##########################
# 推定される車輪関節名
jnt_names = ['base_r_drive_wheel_joint','base_l_drive_wheel_joint']

dofs_idx = [hsr.get_joint(name).dof_idx_local for name in jnt_names]

hsr.set_dofs_kp(kp=np.array([4500, 4500]), dofs_idx_local=dofs_idx)
hsr.set_dofs_kv(kv=np.array([450, 450]), dofs_idx_local=dofs_idx)
hsr.set_dofs_force_range(
    lower=np.array([-50, -50]),
    upper=np.array([50, 50]),
    dofs_idx_local=dofs_idx
)

########################## キー入力 ##########################
key_state = {'up': False, 'down': False, 'left': False, 'right': False}

def on_press(key):
    try:
        if key == keyboard.Key.up:
            key_state['up'] = True
        elif key == keyboard.Key.down:
            key_state['down'] = True
        elif key == keyboard.Key.left:
            key_state['left'] = True
        elif key == keyboard.Key.right:
            key_state['right'] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if key == keyboard.Key.up:
            key_state['up'] = False
        elif key == keyboard.Key.down:
            key_state['down'] = False
        elif key == keyboard.Key.left:
            key_state['left'] = False
        elif key == keyboard.Key.right:
            key_state['right'] = False
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

########################## メインループ ##########################
velocity_cmd = np.zeros(len(dofs_idx))
trk = 30.0  # 目標速度

while True:
    velocity_cmd[:] = 0
    if key_state['up']:
        velocity_cmd[:] = trk  # 前進（両輪前進）
    if key_state['down']:
        velocity_cmd[:] = -trk  # 後退
    if key_state['left']:
        velocity_cmd = np.array([-trk, trk])  # 左回転
    if key_state['right']:
        velocity_cmd = np.array([trk, -trk])  # 右回転

    hsr.control_dofs_velocity(velocity_cmd, dofs_idx)
    scene.step()
