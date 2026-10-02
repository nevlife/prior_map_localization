# prior_map_matcher 설계

사전 지도(PCD)에 라이다 스캔을 정합해 map 좌표의 절대 포즈를 낸다. `prior_map_localization` 패키지의 C++ 노드이고, lidar_localization_ros2를 대체한다. lidar_localization_ros2의 약 4,800줄 가운데 우리 설정에서 실제로 도는 경로만 옮겼다.

## 구성

| 위치 | 내용 |
| --- | --- |
| `thirdparty/ndt_omp` | 서브모듈(nevlife/ndt_omp `jazzy`, koide3 원본 + line search 스위치). 이 패키지 CMake가 정적 라이브러리로 빌드한다 |
| `small_gicp` (작업공간의 nevlife 포크) | 두 번째 정합 엔진 |
| `include/prior_map_localization/`, `src/` | 노드. 직접 작성 코드 규칙을 따른다 |
| `config/prior_map_matcher.param.yaml` | 노드 파라미터 전부. `map_pcd`, `transport`만 launch가 넘긴다 |
| `launch/prior_map_matcher.launch.py` | 노드 단독 실행. `localization.launch.py`가 include한다 |

ndt_omp를 colcon 패키지로 두지 않는다. 원본이 export하는 타깃이 QHULL, VTK 등 PCL 전체를 끌고 오고, `ndt_omp_ros2`와 같은 `include/pclomp`를 설치한다.

`ndt_omp_ros2`(rsasaki0109)는 쓰지 않는다. 2019년 원본에서 갈라져 이후 수정이 빠졌고, 스레드별 부분합 순서가 바뀌어 결과가 실행마다 다르다. 원본은 점마다 값을 저장한 뒤 정해진 순서로 합산한다.

포크에 `setUseLineSearch`를 더했다. 원본은 More-Thuente line search가 돌지만 이동 폭이 `[transformation_epsilon / 2, step_size]`로 묶여 있어, 잘라낸 bag 89개 스캔과 시드 오프셋 6종에서 수렴률, 반복 수, fitness가 끈 경우와 같았고 시간만 2~3.5배 들었다(평소 추적 18.3 ms 대 5.2 ms). 그래서 `registration.ndt_omp.line_search: false`로 둔다. 라이브러리 기본값은 upstream대로 켜짐이다.

## 데이터 흐름

```text
/velodyne_points -> scan_start_relay ─(dds|flux, lio_transport)─┬─> fast_lio
                                                                 └─> prior_map_matcher
/odometry/local (ekf_odom) ──────────────────────────────────────────> prior_map_matcher
/initialpose (graph_initialpose) ────────────────────────────────────> prior_map_matcher

prior_map_matcher ─> /pcl_pose ─> ekf_map
              ─> /reinitialization_requested ─> graph_initialpose
              ─> /alignment_status
```

| 노드 이름 | 토픽 | 타입 |
| --- | --- | --- |
| `~/input/points` | `/lio/points` | `sensor_msgs/PointCloud2` (DDS 또는 flux) |
| `~/input/odom` | `/odometry/local` | `nav_msgs/Odometry` |
| `~/input/initialpose` | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `~/output/pose` | `/pcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `~/output/reinitialization_requested` | `/reinitialization_requested` | `std_msgs/Bool` |
| `~/debug/alignment_status` | `/alignment_status` | `diagnostic_msgs/DiagnosticArray` |

- 스캔은 FAST-LIO와 같은 relay 출력을 받는다. DDS 역직렬화는 relay에서 한 번이고, flux에서는 두 노드가 같은 공유메모리 프레임을 복사 없이 읽는다.
- 트위스트는 `ekf_odom`의 `Odometry`를 바로 받는다. lidar_localization용 `odom_twist_relay`가 필요 없다.
- TF는 내지 않는다. map->odom은 `ekf_map`이 맡는다.

## 노드, executor, 스레드

- 노드는 하나다. 전처리 결과를 쓰는 곳이 정합 하나뿐이라 나누면 점군 전달만 늘어난다. 지도는 시작할 때 파일에서 읽는다.
- lifecycle, 컴포지션을 쓰지 않는다. 컨테이너는 자기 executor를 써서 flux 구독이 돌지 않고, 프로세스 간 복사 제거는 flux가 맡는다.
- 단일 스레드 executor다. `dds`는 `rclcpp::executors::SingleThreadedExecutor`, `flux`는 `flux::ros::Executor`(ROS 구독도 함께 돈다)를 쓴다. 상태를 한 스레드만 만지므로 mutex가 없다.
- 병렬화는 정합 엔진 안의 OpenMP뿐이다(`registration.num_threads`).
- `main`은 노드를 만든 뒤 `cyra::bind`로 메인 스레드를 stage에 걸고, 그다음 `load_map()`을 부른다. 타깃을 만드는 첫 정합이 OpenMP 팀을 만들기 때문에, 순서를 바꾸면 팀 스레드가 `children: inherit`를 물려받지 못한다.

## 스캔 하나의 처리

```text
on_points
  1. 초기 포즈 전, 또는 초기 포즈보다 이른 스캔은 버린다
  2. base_link <- 센서 TF (처음 한 번 조회해 고정)
  3. prepare_scan: 한 번 훑으며 TF 적용 -> 수평 거리 자르기 -> deskew, 그다음 VoxelGrid
  4. 시드 = 예측 포즈 + 트위스트 x dt (dt는 0..max_prediction_dt_sec)
  5. 정합 -> fitness (최종 변환 후 지도 KD-tree까지 평균 제곱 최근접 거리)
  6. converged이고 fitness <= fitness_threshold면 accept, 아니면 reject
  7. /reinitialization_requested, /alignment_status 발행
```

### deskew

- 점 시각 `t_i`와 스캔 길이 `d = max(t) - min(t)`로, 모든 점을 마지막 점의 시각으로 옮긴다. 포즈도 이 시각(`header + max(t)`)으로 찍는다.
- 움직임은 직전 두 accept 포즈의 상대 변환 `M = T_prev^-1 * T_new`와 그 사이 시간 `D`다.
- `T(f) = [slerp(I, q_M, f) | f * t_M]`로 두고, 점마다 `p' = T(d/D)^-1 * T((t_i - t_min)/D) * p`를 적용한다.
- 건너뛰는 경우:

  | 조건 | 상태 |
  | --- | --- |
  | `d > filter.max_scan_duration_sec` | `duration_too_large` |
  | 연속 두 번 accept된 적이 없음 | `no_motion` |
  | `d/D`가 (0, 1.5] 밖 | `interval_mismatch` |

### 예측과 상태

| 사건 | 예측 포즈 | 움직임 `M` |
| --- | --- | --- |
| 초기 포즈 | 받은 포즈 | 지운다 |
| accept | 정합 결과 | 직전 스캔도 accept였을 때만 갱신 |
| reject | 그 스캔의 시드 | 유지 |

- 트위스트는 스캔 끝 시각 이전의 가장 최근 값이다. body 좌표로 보고 `t += R v dt`, `R = R * Rx(wx dt) Ry(wy dt) Rz(wz dt)`로 적용한다.
- 공분산은 accept 때만 채운다. 표준편차는 `clamp(floor + per_fitness * fitness, floor, max)`이고 `[xy, z, roll/pitch, yaw]` 순서의 세 배열로 준다. 비대각은 0이다.
- 재초기화 요청은 마지막 accept 이후 `reinitialization_gap_sec`가 지나면 참이다. 스캔마다 발행하고, accept 하나나 초기 포즈로 거짓이 된다.

## lidar_localization_ros2와 다른 점

| 항목 | lidar_localization_ros2 | prior_map_matcher | 이유 |
| --- | --- | --- | --- |
| NDT 라이브러리 | `ndt_omp_ros2` (line search 꺼짐, 비결정적 합산) | koide3 `ndt_omp` + line search 스위치(끔) | 원본 수정 반영, 결정적 합산 |
| 입력 | `/velodyne_points` (DDS) | relay 출력 (DDS/flux) | FAST-LIO와 공유 |
| 점 읽기 | `fromROSMsg` 후 변환 | 버퍼에서 바로 읽음 | 변환 단계 제거 |
| deskew 움직임 | reject 뒤에도 accept 간 시간으로 갱신 | 연속 accept일 때만 갱신 | reject 연속 뒤 과소 보정 방지 |
| 재초기화 판정 | 점수식과 latch | 마지막 accept 이후 시간 | 실제로 걸리는 조건(30 s, 200회)과 같음 |
| executor | 3 스레드 + mutex | 단일 스레드 | 스캔 주기 대비 일이 짧다 |
| 파라미터 | 약 200개 | 20개 | |

## 정합 엔진

`pcl::Registration` 하나로 둘을 다룬다. `registration.method`로 고르고, 그 엔진의 파라미터만 선언한다.

| method | 엔진 | 파라미터 |
| --- | --- | --- |
| `ndt_omp` | `pclomp::NormalDistributionsTransform` | `resolution`, `step_size`, `transformation_epsilon`, `line_search` |
| `small_gicp` | `small_gicp::RegistrationPCL` (`type`: GICP, VGICP) | `num_neighbors`, `max_correspondence_distance`, `voxel_resolution`, `translation_epsilon`, `rotation_epsilon` |

fitness는 두 엔진 모두 PCL `getFitnessScore()`라 같은 기준으로 판정한다.

## 실행

```bash
ros2 launch prior_map_localization prior_map_matcher.launch.py map_config:=<map_pcd를 담은 param 파일> [transport:=flux]
ros2 launch prior_map_localization localization.launch.py localizer:=prior_map_matcher [lio:=true]
```

지도 경로는 `config/map.param.yaml` 한 곳에 둔다. prior_map_matcher와 graph_initialpose가 같은 파일을 파라미터로 받는다.

## 검증
