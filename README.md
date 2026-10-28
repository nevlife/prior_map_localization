# prior_map_localization

IMU 축 회전: vectornav는 똑바로 장착돼 있다. 그러나 드라이버(`use_enu: true`)가 x 오른쪽, y 앞, z 위 축으로 내보낸다. 그래서 IMU 데이터의 축은 LiDAR 기준 z축으로 -90도 돌아가 있다(lidar_x = imu_y, lidar_y = -imu_x). `config/fast_lio.param.yaml`은 이 회전을 넣었다(`extrinsic_R: [0,-1,0, 1,0,0, 0,0,1]`, `extrinsic_T: [0, 0.08075, 0.19089]`). URDF(`hunter2_description`의 `vectornav_joint`)에도 같은 회전을 넣었다.
