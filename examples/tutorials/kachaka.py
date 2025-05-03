import numpy as np

import genesis as gs

########################## 初期化 ##########################
gs.init(backend=gs.gpu)

########################## シーンの作成 ##########################
scene = gs.Scene(
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (0, -3.5, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 30,
        res           = (960, 640),
        max_FPS       = 60,
    ),
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    show_viewer = True,
)

########################## エンティティ ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
     gs.morphs.URDF(file='../../../kachaka-api/ros2/kachaka.urdf'),
)
########################## ビルド ##########################
scene.build()

jnt_names = [
    # 'base_footprint_joint',
    'base_r_drive_wheel_joint',
    'base_l_drive_wheel_joint',
    # 'lidar_joint',  
    # 'camera_front_joint',
    # 'camera_back_joint',
    # 'tof_joint',
    # 'docking_joint',
    # 'imu_joint',
]
dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

############ オプション：制御ゲインの設定 ############
# 位置ゲインの設定
franka.set_dofs_kp(
    kp             = np.array([4500, 4500]),
    dofs_idx_local = dofs_idx,
)
# 速度ゲインの設定
franka.set_dofs_kv(
    kv             = np.array([450, 450]),
    dofs_idx_local = dofs_idx,
)
# 安全のための力の範囲設定
franka.set_dofs_force_range(
    lower          = np.array([-87, -87]),
    upper          = np.array([ 87,  87]),
    dofs_idx_local = dofs_idx,
)
# ハードリセット
for i in range(150):
    if i < 50:
        franka.set_dofs_position(np.array([1, 1]), dofs_idx)
    elif i < 100:
        franka.set_dofs_position(np.array([-1, 0.8]), dofs_idx)
    else:
        franka.set_dofs_position(np.array([0, 0]), dofs_idx)

    scene.step()

# PD制御
for i in range(1250):
    if i == 0:
        # 両輪を同じ速度で回転させて直進（50cm/s = 0.5m/s）
        # 車輪の半径を考慮して適切な角速度を設定
        wheel_velocity = 0.5  # 0.5 m/s = 50 cm/s
        franka.control_dofs_velocity(
            np.array([wheel_velocity, wheel_velocity]),
            dofs_idx,
        )
    
    # デバッグ用の出力（オプション）
    if i % 100 == 0:
        print('control force:', franka.get_dofs_control_force(dofs_idx))
        print('internal force:', franka.get_dofs_force(dofs_idx))

    scene.step()