# prior_map_localization

IMU 장착 회전: vectornav는 LiDAR 기준 z축으로 -90도 돌아가 있다(lidar_x = imu_y, lidar_y = -imu_x). 그런데 URDF와 `config/fast_lio.param.yaml`의 `extrinsic_R`은 아직 회전 0이다. bag 측정에서는 `extrinsic_R: [0,-1,0, 1,0,0, 0,0,1]`, `extrinsic_T: [0, 0.0807, 0.1909]`로 바꿔 돌렸다.
