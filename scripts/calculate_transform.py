import numpy as np


T_camera_lidar = np.array(
        [ 0,  0,  1,
         -1, 0, 0,
         0, -1, 0]
)
T_camera_lidar = T_camera_lidar.reshape(3, 3)

T_lidar_imu = np.array(
        [ 0, -1,  0,
         1,  0,  0,
         0,   0,  1]
)
T_lidar_imu = T_lidar_imu.reshape(3, 3)

T_camera_imu = T_camera_lidar @ T_lidar_imu
print(T_camera_imu)