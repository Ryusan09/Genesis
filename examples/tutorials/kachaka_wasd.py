import numpy as np
import genesis as gs
from pynput import keyboard

########################## 初期化 ##########################
gs.init(backend=gs.gpu)

########################## シーンの作成 ##########################
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(0, -3.5, 2.5),
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
franka = scene.add_entity(
    gs.morphs.URDF(file='../../../kachaka-api/ros2/kachaka.urdf'),
)

########################## ビルド ##########################
scene.build()

jnt_names = [
    'base_r_drive_wheel_joint',
    'base_l_drive_wheel_joint',
]
dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

############ オプション：制御ゲインの設定 ############
franka.set_dofs_kp(kp=np.array([4500, 4500]), dofs_idx_local=dofs_idx)
franka.set_dofs_kv(kv=np.array([450, 450]), dofs_idx_local=dofs_idx)
franka.set_dofs_force_range(
    lower=np.array([-87, -87]),
    upper=np.array([87, 87]),
    dofs_idx_local=dofs_idx,
)

########################## キー入力設定 ##########################
key_state = {'left': False, 'right': False, 'up': False, 'down': False}

def on_press(key):
    try:
        if key == keyboard.Key.left:
            key_state['left'] = True
        elif key == keyboard.Key.right:
            key_state['right'] = True
        elif key == keyboard.Key.up:
            key_state['up'] = True
        elif key == keyboard.Key.down:
            key_state['down'] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if key == keyboard.Key.left:
            key_state['left'] = False
        elif key == keyboard.Key.right:
            key_state['right'] = False
        elif key == keyboard.Key.up:
            key_state['up'] = False
        elif key == keyboard.Key.down:
            key_state['down'] = False
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

########################## メインループ ##########################
velocity_cmd = np.zeros(len(dofs_idx))

while True:
    velocity_cmd[:] = 0  # 初期化
    if key_state['up']:
        velocity_cmd[0] += 1.0  # base_r_drive_wheel_joint
    if key_state['down']:
        velocity_cmd[0] -= 1.0
    if key_state['left']:
        velocity_cmd[1] += 1.0  # base_l_drive_wheel_joint
    if key_state['right']:
        velocity_cmd[1] -= 1.0

    franka.control_dofs_velocity(velocity_cmd, dofs_idx)
    scene.step()
