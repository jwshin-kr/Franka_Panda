# RoPhiTutorial — Franka Research 3 + Robotiq 2F-85 (DROID, 튜토리얼 01–06)

[RoPhiTutorial](https://github.com/jwshin-kr/RoPhiTutorial)의 env / controller / visualizer 구조와 코드·수식을 그대로 두고,
로봇만 Flexiv Rizon4 + Robotis RH-5 → **Franka Research 3(FR3) + Robotiq 2F-85**(DROID 셋업)로 바꾼 버전입니다.
현재 범위는 튜토리얼 06 (IK QP)까지입니다. 바닥·벽·enclosure(셀)·조명 등 로봇 외 환경은 원본과 동일합니다.

## 실행

터미널 3개에서 저장소 루트 기준으로 env → controller → visualizer 순서로 실행합니다. **모든 모드의 실행 명령(sim + 실제 로봇)은 [CMDS.md](CMDS.md)에 정리되어 있습니다.**

```bash
conda activate rophi

# 01–03 gravity compensation
python run_env.py --config configs/franka_fr3/env/default.yaml
python run_controller.py --config configs/franka_fr3/controllers/grav_comp.yaml
python run_visualizer.py --config configs/visualizer/franka_fr3/default.yaml

# 04 publish collision geometry (env 단독 실행 가능)
python run_env.py --config configs/franka_fr3/env/collision.yaml

# 05 collision-aware gravity compensation
python run_env.py --config configs/franka_fr3/env/collision.yaml
python run_controller.py --config configs/franka_fr3/controllers/col_aware_grav_comp.yaml
python run_visualizer.py --config configs/visualizer/franka_fr3/collision.yaml

# 06 inverse kinematics QP (controller 터미널에서 w/s, a/d, r/f, [/], h, o/c 그리퍼 열기/닫기)
python run_env.py --config configs/franka_fr3/env/collision.yaml
python run_controller.py --config configs/franka_fr3/controllers/ik_qp.yaml
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp.yaml

# 08 impedance control (controller 터미널에서 j = joint, t = task SE(3); RoPhiTutorial tutorial 08 v2)
python run_env.py --config configs/franka_fr3/env/default.yaml
python run_controller_v2.py --config configs/franka_fr3/controllers/impedance.yaml --name impedance_v2
python run_visualizer.py --config configs/visualizer/franka_fr3/impedance.yaml

# 14 differentiable base–camera calibration (MuJoCo 뷰어에서 팔을 끌어 자세 → Calib RGB 창에서 SPACE → 로봇 클릭 → ENTER)
python run_env.py --config configs/calibration_tutorial/env/franka_arm_hand_cam.yaml
python run_controller.py --config configs/calibration_tutorial/controllers/grav_comp.yaml
python run_camera_calibration.py --config configs/calibration_tutorial/calib_diff_render.yaml
```

08은 v2(`ImpedanceControl_v2.py`)만 쓰세요. 원본 controller는 측정값 backlog 때문에 sim에서도 발산했습니다(관절 속도 49 rad/s).
v2 controller는 RoPhiTutorial의 `ImpedanceControl_v2.py`와 같습니다(오래된 LCM 데이터 수정만; FR3용 변경은 `ImpedanceControl.py`의 task frame `tcp`와 Robotiq set-point). 실제 로봇:

```bash
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml
python run_controller_v2.py --config configs/franka_fr3/controllers/impedance_hw.yaml
python run_visualizer.py --config configs/visualizer/franka_fr3/impedance_hw.yaml
```

실제 로봇은 visualizer도 `*_hw.yaml`(hw_ 채널)을 써야 합니다. sim 설정(sw_ 채널)으로 띄우면 로봇이 보이지 않고 터미널에 경고가 나옵니다.
`impedance_hw.yaml`은 `impedance.yaml`과 게인이 같습니다(RoPhiTutorial 방식: kq = dq² / (4 M), task 200 / 5 / 30 / 0.8). 너무 뻣뻣하면 Viser의 `kq_scale` / `dq_scale` 슬라이더로 낮추세요.

### 14 differentiable base–camera calibration (sim)

RoPhiTutorial tutorial 14와 같은 코드입니다(SAM2 마스크 vs nvdiffrast로 그린 URDF 실루엣, Adam으로 `T_cam2base` 추정).
nvdiffrast / SAM2 / pytorch3d는 RoPhiTutorial tutorial 13 방식으로 `rophi` env에 설치되어 있어야 합니다(SAM2는 RoPhiTutorial의 `third_party/sam2`).
카메라(`calib_camera`, 위치·화각)는 원본 씬과 같고, 바뀐 것은 로봇뿐입니다.

| 원본 (Flexiv) | FR3 | 바뀐 것 |
|---|---|---|
| `assets/scene/flexiv_arm/hand_cam_calib.xml` | `assets/scene/franka_fr3/fr3_cam_calib.xml` | `fr3_default.xml` + 원본과 같은 `<visual>`·`calib_camera` |
| `env/calibration_tutorial/FlexivArmHandCamEnv.py` | `env/calibration_tutorial/FrankaArmHandCamEnv.py` | 클래스 이름 |
| `.../flexiv_arm_5F_hand/FlexivArmHandCamPubManager.py` | `.../franka_fr3/FrankaArmHandCamPubManager.py` | 클래스 이름, 기본 채널 |
| `configs/calibration_tutorial/env/flexiv_arm_hand_cam.yaml` | `configs/calibration_tutorial/env/franka_arm_hand_cam.yaml` | 씬, platform `franka_fr3`, 기본 자세, 채널, 관절 수 8 |
| `configs/calibration_tutorial/controllers/grav_comp.yaml` | 같은 경로 | `configs/franka_fr3/controllers/grav_comp.yaml`과 같음(원본도 기본 grav_comp와 같음) |
| `configs/calibration_tutorial/calib_diff_render.yaml` | 같은 경로 | 채널, 관절 수 8 = arm 7 + gripper 1, `fr3_robotiq_viser.urdf` |
| `utils/calibration/robot_mesh_fk.py` | 같은 경로 | 링크 선택: `link0..7`, `flange`, `robotiq_base_mount`(원본 `hand_mount`); `arm_only: false`면 `robotiq_base`, `robotiq_right_*`/`robotiq_left_*`(원본 palm/finger) |
| `utils/calibration/diff_base_to_cam.py` | 같은 경로 | 기본값(관절 수, URDF 경로)만 |

나머지(`run_camera_calibration.py`, `utils/calibration/diff_silhouette.py`, `base_to_eye.py`, `utils/mujoco/camera.py`,
`utils/perception/camera.py`, `utils/lie/se3.py`, `utils/communication/lcm/image_conversion.py`, `CameraInfoData`/`RGBDData`,
RGBD·CameraInfo publisher/subscriber, `lcm_type/vision/`)는 원본 그대로입니다. 시뮬레이션 전용이라 실제 로봇 설정은 없습니다.

### 클릭한 점 따라가기 (calibration + point tracking + IK QP, sim)

카메라 이미지에서 한 점을 클릭 → TAPIR가 픽셀을 추적 → depth로 카메라 좌표 3D 점 → 14의 calibration(`T_cam2base`)으로
로봇 base 좌표 → `run_perception.py`가 실시간 publish → IK QP v2가 그 점의 10 cm 위를 target으로 tcp를 보냅니다.
씬은 14의 카메라 씬에 MuJoCo 뷰어에서 끌 수 있는 큐브(mocap, 시각 전용) 하나를 더한 것입니다.

```bash
# 처음 한 번: TAPIR (rophi env) + 체크포인트(208 MB, git 제외)
pip install "git+https://github.com/google-deepmind/tapnet.git"
mkdir -p perception/tapnet/checkpoints && curl -L -o perception/tapnet/checkpoints/causal_bootstapir_checkpoint.pt \
    https://storage.googleapis.com/dm-tapnet/causal_bootstapir_checkpoint.pt

# 1) calibration (14와 같은 과정) + 결과 저장 → configs/calibration_tutorial/results/calib_camera.yaml
python run_env.py --config configs/calibration_tutorial/env/franka_arm_hand_cam.yaml
python run_controller.py --config configs/calibration_tutorial/controllers/grav_comp.yaml
python run_camera_calibration_v2.py --config configs/calibration_tutorial/calib_diff_render.yaml

# 2) 점 따라가기 (Click on Image 창에서 큐브 클릭, 뷰어에서 큐브 더블클릭 → Ctrl + 오른쪽 드래그로 이동)
python run_env.py --config configs/point_tracking/env/franka_arm_hand_col_cam.yaml
python run_controller_v2.py --config configs/point_tracking/controllers/ik_qp_point_follow.yaml
python run_perception.py --config configs/point_tracking/perception/tapnet_click_and_track.yaml
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp.yaml   # 선택: target / tip 표시
```

controller 키: `t` 따라가기 on/off(off면 06의 키보드 이동), `h` home, `o`/`c` 그리퍼. 클릭을 다시 하면 새 점을 추적합니다.

| 단계 | 파일 | 출처 |
|---|---|---|
| 클릭 + TAPIR 추적 + depth → 3D + publish | `run_perception.py`, `perception/tapnet/TapnetClickAndTrack.py`, `perception/BasePerception.py`, `SingleRGBDSubManager`, `Point3DPubManager`, `Point3DData`, `Point3D{Publisher,Subscriber}`, `lcm_type/geometry/` | RoPhiTutorial 그대로 (factory 3개는 쓰는 이름만 남김) |
| calibration 결과로 base 좌표 | `perception/tapnet/TapnetClickAndTrackCalib.py` (원본 노드 상속, `cam2world`만 `T_cam2base`로 교체; `cam2base_path: null`이면 원본 동작) | 새로 작성 |
| calibration 결과 저장 | `run_camera_calibration_v2.py` (원본 calibrator 그대로 실행 후 yaml 저장) | 새로 작성 |
| 3D 점을 IK QP target으로 | `controller/IKQPPointFollowControl.py` (`IKQPControllerV2` 상속: target = 점 + offset, 작업공간·도달거리 clamp, 저역통과 필터, 0.5 s 끊기면 유지), `ArmHandColInfoPointSubManager` | 새로 작성 (원본에 점을 받는 controller 없음) |
| 카메라 + 충돌 정보 env | `env/perception_tutorial/FrankaArmHandColCamEnv.py` (14 카메라 env + 06 충돌 env 다중상속), `FrankaArmHandColCamPubManager`, `assets/scene/franka_fr3/fr3_cam_track.xml` | 새로 작성 |

- depth 형식: 원본 sim 카메라는 float32 m(`depth_factor: 1.0`)인데 원본 `convert_pixel_to_world`는 값 ≤ 1을 무효로 버립니다
  (RealSense uint16 mm 기준). 그대로면 1 m보다 가까운 점이 모두 버려지므로, 이 env는 `depth_factor: 1000`일 때 RealSense와 같은
  uint16 mm로 보냅니다. perception 코드는 원본 그대로이고, 같은 perception 설정을 실제 RealSense에도 쓸 수 있습니다.
- sim 확인(카메라 30 Hz, 큐브를 30 s 동안 원 + 들어올리기로 이동, 4–7 cm/s): tcp가 목표(추적점 + 10 cm)를 평균 19 mm(최대 31 mm)
  뒤처져 따라가고, 멈추면 target에 1 mm 이내로 멈춥니다. 추적점은 원본 노드 방식대로 클릭 주변 10개 점의 평균이라 클릭한 픽셀에서
  ~1 cm 떨어진 큐브 표면 점이며, 30 s 이동 후에도 같은 점으로 돌아왔습니다(드리프트 없음). calibration 오차 1.6 mm / 0.11°.
- 로봇 자신을 클릭하면 로봇이 자기 몸을 쫓으므로 큐브(또는 로봇이 아닌 물체)를 클릭하세요. 방향은 제어하지 않습니다(06과 같이 tcp 위치만).

### 실제 로봇: RealSense + calibration + 점 따라가기

실행 명령과 순서는 [CMDS.md](CMDS.md)의 "Camera (RealSense)", "Differentiable base–camera calibration (14)",
"Click-and-track pointing", "Mock mode"(실제 로봇 절)에 있습니다. 카메라는 RoPhiTutorial과 같이 `run_sensors.py`(RealSense)가
`d455_1` / `d455_1_info` 채널로 보내고, calibration·perception·controller는 sim과 같은 코드에 `*_hw.yaml` 설정만 다릅니다.

| 파일 | 출처 |
|---|---|
| `run_sensors.py`, `sensor/`(BaseSensor·Manager·Info, `RealSenseManager`, `RealSense`, `CameraInfo`), `RealSensePubManager`, sensor sub/pub factory | RoPhiTutorial 그대로 (`sensor/__init__.py`에 mock 분기 4줄만 추가) |
| `configs/sensors/realsense/one_realsense.yaml` | 원본과 같은 키. serial·K는 원본 d455_1 값(**CHECK**), `tf_link_to_cam`은 자리표시(sim 카메라 자세) |
| `sensor/sensor/realsense/MockRealSense.py`, `sensor/sensor_manager/MockRealSenseManager.py` | 새로 작성: `mock_mode: true`면 카메라 대신 MuJoCo 렌더(측정 관절, 설정의 K, uint16 mm depth) |
| `utils/calibration/eyeball_camera_pose.py` | 새로 작성: 카메라 위치·바라보는 점 → `tf_link_to_cam` (calibration 시작값) |
| `configs/calibration_tutorial/calib_diff_render_hw.yaml` | sim 설정 + `d455_1`, `hw_` 채널, 시작 노이즈 0(시작값이 이미 추정값), 4장 |
| `configs/point_tracking/controllers/ik_qp_point_follow_hw.yaml` | `ik_qp_hw.yaml` + point_follow(15 cm 위, target z ≥ 0.10 m, 15 cm 넘는 점프는 0.5 s 유지될 때만) |
| `configs/point_tracking/perception/tapnet_click_and_track_hw.yaml` | sim 설정 + `d455_1`, `results/d455_1.yaml` |

`run_camera_calibration_v2.py`는 결과 yaml에 `tf_link_to_cam`(RealSense 설정 형식)도 저장·출력합니다. 실제 카메라에서
calibrator가 출력하는 "Error vs GT"는 정답 대비 오차가 아니라 설정의 시작값 대비 변화량입니다.

mock 점검(이 PC, 로봇·카메라 없이; libfranka를 이 PC에서 빌드할 수 없어 C++ bridge 대신 그 mock 모드와 같은 동작의 테스트용
Python 대역 사용): 일부러 4 cm / 3.7° 틀린 `tf_link_to_cam`에서 calibration(4장)이 mock 카메라의 실제 자세 대비 6.9 mm / 0.33°로
수렴했고, 점 따라가기에서 tcp가 target(추적점 + 15 cm)과 일치하며 움직이는 큐브를 따라갔습니다. 추적점은 클릭한 큐브 윗면
중심에서 ~2.8 cm 떨어진 점이었습니다(원본 노드의 ±15 px 샘플 평균 + calibration 오차).

### v2 (05·06 문제 수정판)

원본 파일과 factory는 그대로 두고, v2 runner가 v2 클래스를 직접 만듭니다(RophiTutorial의 v2와 같은 구성).

```bash
# 05 v2: 세 가지 원인 수정 + 디버그 오버레이(고스트 로봇, witness 점/궤적, v* 화살표, 충돌 구, 플롯)
python run_env.py --config configs/franka_fr3/env/collision.yaml
python run_controller_v2.py --config configs/franka_fr3/controllers/col_aware_grav_comp_v2.yaml
python run_visualizer_v2.py --config configs/visualizer/franka_fr3/collision_v2.yaml

# 원본 동작 재현 (수정 하나씩 꺼볼 수도 있음)
python run_controller_v2.py --config configs/franka_fr3/controllers/col_aware_grav_comp_v2.yaml \
    --v2.fix_fresh_state false --v2.fix_multi_point_witness false --v2.fix_smooth_command false

# 06 v2: 원인 1(오래된 LCM 데이터)만 수정, 원본 ik_qp.yaml 그대로 사용
python run_env.py --config configs/franka_fr3/env/collision.yaml
python run_controller_v2.py --config configs/franka_fr3/controllers/ik_qp.yaml --name ik_qp_v2
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp.yaml
```

| v2 파일 | 역할 |
|---|---|
| `controller/CollisionAwareGravCompControl_v2.py` | 05 원본 상속 + 세 가지 수정 (`v2.fix_*`로 개별 on/off) |
| `controller/IKQPControl_v2.py` | 06 원본 상속 + 원인 1 수정 |
| `utils/collision/sphere_swept_v2.py` | 링크 프리미티브 → 구 체인, 해석적 signed distance (원인 2) |
| `communication/.../JointCtrlColDebugPubManager_v2.py` | joint_ctrl + 디버그 채널 `sw_col_aware_debug_v2` |
| `visualizer/data_visualizer/ColAwareDebugVisualizer_v2.py` | 세 원인을 보여주는 Viser 오버레이 |
| `visualizer/vis_manager/franka_fr3/FrankaFR3VisManager_v2.py` | 원래 visualizer + 위 오버레이 |
| `run_controller_v2.py`, `run_visualizer_v2.py` | v2 runner |
| `configs/franka_fr3/controllers/col_aware_grav_comp_v2.yaml`, `configs/visualizer/franka_fr3/collision_v2.yaml` | v2 설정 |

### 실제 로봇 (FR3 + Robotiq 2F-85, DROID 셀)

팔은 C++ bridge(libfranka ≥ 0.15, FR3 system 5.8.1에는 0.17.0; `communication/franka/README.md`에서 빌드),
그리퍼는 Python(Modbus RTU, `Robotiq2F85Hardware.py`)이 구동합니다. 로봇 IP `192.168.0.113`, 그리퍼는 DROID NUC
(`socket://192.168.0.127:5020`)에 연결되어 있다고 가정합니다. 이 PC는 PREEMPT_RT 커널이 아니므로 `realtime_config: ignore`입니다. `configs/franka_fr3/hardware/bridge.yaml`의 `robot.mock_mode: true`(기본값)이면
로봇 연결 없이 bridge가 명령을 그대로 측정값으로 돌려주고, 그리퍼도 같은 Modbus 프로토콜 뒤의 가상 그리퍼로 바뀝니다
(`hand.mock_mode: null` = bridge를 따름). 실제 로봇에서는 `mock_mode: false`로 바꾸고 `hand.port`를 맞춥니다.

```bash
# 01–03 gravity compensation, 실제 로봇 (mock_mode에서도 같은 명령). 처음에는 이것부터.
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml
python run_controller_v2.py --config configs/franka_fr3/controllers/grav_comp_hw.yaml
python run_visualizer.py --config configs/visualizer/franka_fr3/default_hw.yaml

# 06 IK QP, 실제 로봇 (mock_mode에서도 같은 명령)
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml
python run_controller_v2.py --config configs/franka_fr3/controllers/ik_qp_hw.yaml
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp_hw.yaml
```

- 실제 로봇에서는 v2 controller(`grav_comp_hw.yaml`의 `grav_comp_v2`, `ik_qp_hw.yaml`의 `ik_qp_v2`)를 씁니다. 원본은 측정값을 루프당 1개씩 꺼내서
  점점 더 오래된 상태로 제어하고, bridge는 `command_timeout_s`(50 ms)보다 오래된 명령을 버립니다.
- 05·06 controller가 기다리는 충돌 정보(로봇 프리미티브 + `floor`/`wall_*`)는 실제 로봇에서는 `HardwareFrankaFR3`가
  `col_info.scene_xml_path`의 씬에서 만들어 보냅니다. **이 정적 장애물은 실제 셀과 맞아야 합니다.**
- Desk의 end effector를 Robotiq 2F-85로 설정합니다: 0.925 kg, 질량중심 flange z축으로 0.052 m.
  (bridge는 로봇이 더하는 중력을 다시 빼고 Pinocchio g(q)를 쓰므로 토크는 Desk 값과 무관하지만, 로봇의 외력 추정·반사는 이 값을 씁니다.)
- 그리퍼 USB 어댑터 포트 권한: `sudo usermod -aG dialout $USER`. 다른 PC(예: DROID NUC)에 꽂혀 있으면 그 PC에서
  `python3 -m serial.tools.tcp_serial_redirect -P 5020 /dev/ttyUSB0 115200`, 여기서는 `hand.port: socket://<ip>:5020`.

- Viser: http://localhost:8080
- `p` / `q`: 일시정지 / 종료 (각 터미널), `r`: env 리셋, MuJoCo 뷰어에서 `Space`: 일시정지
- MuJoCo 충돌 메시는 geom group 4라 기본으로 숨겨져 있습니다. MuJoCo 뷰어에서 geom group 4를 켜면 보입니다.
- Viser의 `Show collision geometry` / `Show collision distances`는 05·06 설정에서 env가 보내는 충돌 형상을 그립니다.
- 원본처럼 커맨드라인으로 설정을 덮어쓸 수 있습니다. 예: `--vis_mode vis_off`, `--arm_damping 0.5`

## LCM 채널

| Channel | Type | 방향 | 내용 |
|---|---|---|---|
| `sw_franka_arm_joint_meas` | `joint_meas_t` | env → controller, visualizer | 팔 q, q̇, τ (7) |
| `sw_franka_gripper_joint_meas` | `joint_meas_t` | env → controller, visualizer | 그리퍼 driver 각도 q, q̇, τ (1, rad: 0 열림 .. 0.8 닫힘) |
| `sw_franka_arm_hand_joint_ctrl` | `joint_ctrl_t` | controller → env | `[arm(7), gripper(1)] = 8` |
| `sim_clock` | `clock_t` | env → controller | 시뮬레이션 시간 |
| `sw_franka_arm_hand_robot_col_info` | `col_info_t` | env → controller, visualizer | 로봇 링크 프리미티브 (link-local) |
| `sw_franka_arm_hand_static_col_info` | `col_info_t` | env → controller, visualizer | floor / wall_back / wall_left (world) |
| `sw_ik_target_pose` | `pose3d_t` | controller → visualizer | IK 타깃, 현재 tip, tip 속도 (06) |
| `calib_camera` | `rgbd_t` | env → calibration | 고정 카메라 RGB-D 640×480 (14) |
| `calib_camera_info` | `camera_info_t` | env → calibration | K + GT OpenCV `T_world2cam` (평가용, 14) |
| `click_and_point` | `point_set_t` | perception → controller | 추적 중인 점의 base 좌표 (점 따라가기) |

실제 로봇은 같은 채널의 `hw_` 버전을 씁니다 (팔 측정값은 bridge, 그리퍼 측정값·충돌 정보는 `HardwareFrankaFR3`가 보냄).

## 구성

로봇에 해당하는 파일 (원본 Flexiv 파일과 1:1 대응):

```
assets/scene/franka_fr3/
  fr3_robot.xml        FR3 + Robotiq 2F-85 MJCF (MuJoCo Menagerie franka_fr3 + robotiq_2f85 기반, 팔 액추에이터 없음)
  fr3_default.xml      씬: floor / wall_back / wall_left / enclosure / 조명 (원본과 동일) + 팔 토크 모터 7개 + 그리퍼 위치 서보
  urdf/fr3_robotiq.urdf        Pinocchio(동역학, controller)용 URDF: 8 joint, 그리퍼 링크는 열린 자세로 고정
  urdf/fr3_robotiq_viser.urdf  Viser(시각화)용 URDF: 그리퍼 링크가 <mimic>으로 driver 각도를 따름
  urdf/make_fr3_robotiq_urdf.py  위 두 URDF를 fr3_robot.xml(로봇 전체) + urdf/_world_cell.urdf.in(world, 셀)에서 생성
  fr3/                   FR3 메시 (visual: OBJ, collision: STL), LICENSE (Apache-2.0, Menagerie franka_fr3)
  robotiq/               Robotiq 2F-85 메시, LICENSE (BSD-2-Clause, Menagerie robotiq_2f85)
  cell/                  enclosure 메시 (원본 Flexiv 셀과 같은 파일)
utils/gripper/robotiq_2f85.py                  driver 각도 ↔ 패드 간격 변환, 그리퍼 상수
utils/collision/fr3_robotiq_geoms.py         로봇 충돌 프리미티브 (sim / hardware 공용)
robot_platform/hardware/franka_fr3/Robotiq2F85Hardware.py   Robotiq Modbus RTU 드라이버 (+ mock transport)
robot_platform/hardware/franka_fr3/HardwareFrankaFR3.py   실제 로봇: bridge(팔) + Robotiq + 충돌 정보
configs/franka_fr3/hardware/, configs/franka_fr3/controllers/ik_qp_hw.yaml, configs/visualizer/franka_fr3/ik_qp_hw.yaml
robot_platform/sim/franka_fr3/SimFrankaFR3.py                                  ← SimFlexivArm5FHand.py
env/franka_fr3/FrankaArmHandColEnv.py                                            ← FlexivArmHandColEnv.py
communication/lcm/publisher/pub_manager/env/franka_fr3/FrankaArmHandPubManager.py        ← FlexivArmHandPubManager.py
communication/lcm/publisher/pub_manager/env/franka_fr3/FrankaArmHandColInfoPubManager.py ← FlexivArmHandColInfoPubManager.py
visualizer/vis_manager/franka_fr3/FrankaFR3VisManager.py                       ← FlexivArmHandVisManager.py
visualizer/data_visualizer/FrankaFR3Visualizer.py                                ← FlexivArmHandVisualizer.py
configs/franka_fr3/{env,controllers}/, configs/visualizer/franka_fr3/          ← configs/flexiv_arm_5F_hand/, configs/visualizer/flexiv_arm_5F_hand/
```

나머지 파일(`controller/*`, `env/BaseEnv.py`, `env/MujocoBaseEnv.py`, `communication/`, `data_type/`, `lcm_type/`, `utils/`,
`run_*.py`)은 GitHub 원본과 같은 경로·같은 내용입니다. 기본 채널 이름/관절 수 같은 로봇 기본값만 Franka로 바꿨고,
factory(`get_env`, `get_controller`, ...)에는 튜토리얼 06까지 쓰는 이름만 등록했습니다. Visualizer는 원본의 큰 클래스에서
06까지 쓰는 부분(URDF, 충돌 오버레이, IK 타깃 오버레이)만 같은 코드로 옮겼습니다.

## 모델 메모

- **기본 자세**: 팔 `[0, 0, 0, -90°, 0, 90°, 0°]` (~/jwjh/franka와 같음; joint 1, 2, 3, 5, 7이 FR3의 0점 표시(화살표)에 맞춰짐, Robotiq은 flange에 0°로 붙어 손가락이 셀 축과 나란함), 그리퍼 열림 (driver 각도 0).
- **FR3 팔** (`fr3_robot.xml`, Menagerie franka_fr3): 링크/조인트 이름은 `link0..link7`, `joint1..joint7`(`fr3_` 접두어 없음).
  FR3 관절 범위(예: joint6 0.5445..4.5169 rad), 관성, armature(0.195 / 0.074), damping 0.21, frictionloss(1.137 .. 0.248 Nm)를
  Menagerie 값 그대로 씁니다. FR3는 Panda와 기구학(DH)은 같지만 관성·관절 범위·형상이 다릅니다.
- **MJCF ↔ URDF**: 두 URDF 모두 `make_fr3_robotiq_urdf.py`가 MJCF에서 생성합니다. 그리퍼가 열린 상태에서 Pinocchio와
  MuJoCo의 팔 g(q)가 무작위 자세 300개에서 1.3e-7 Nm, tcp FK가 1e-8 이내로 일치하는 것을 확인했습니다. MJCF를 고치면
  스크립트를 다시 실행합니다.
- **Robotiq 2F-85** (`fr3_robot.xml`, Menagerie robotiq_2f85): flange에 DROID 방식으로 장착(z축 회전 없음, flange y축으로 닫힘;
  실제 장착 각도가 다르면 `gripper` body의 quat을 바꾸고 URDF를 다시 생성). 닫힌 4절 링크(equality)와 driver 관절 두 개에
  걸린 위치 서보를 그대로 씁니다. 자유도는 하나(`robotiq_right_driver_joint`, 0 열림 .. 0.8 rad 닫힘)이고, 실제 그리퍼처럼
  위치 제어라 명령의 그리퍼 칸은 q_des만 쓰고 qd / τ / kp / kd는 무시합니다. 질량 0.925 kg(커플링 포함), tcp는 flange에서 0.15 m.
  Pinocchio 3.4는 `<mimic>`을 읽지 못하므로 controller용 URDF는 링크를 열린 자세로 고정하고 오른쪽 driver만 움직입니다.
- **그리퍼 set-point**: controller는 그리퍼 q_des에 측정값 대신 `gripper_q_des`(설정, 06에서는 `o`/`c` 키)를 보냅니다.
  위치 제어 그리퍼에 측정값을 다시 목표로 주면 스프링·접촉으로 생긴 작은 움직임이 계속 새 목표가 되어 그리퍼가 서서히 닫힙니다.
- **Armature**: URDF에는 armature를 표현할 수 없으므로, Pinocchio의 M(q)를 MuJoCo와 맞추려면 `model.armature`에
  FR3 값(0.195 ×4, 0.074 ×3)을 넣어야 합니다(원본 코드에는 없음, 03–06에는 M(q) 미사용).
- **충돌 프리미티브** (`SimFrankaFR3.get_col_info_data`): 원본처럼 링크 이름마다 프리미티브 1개(link-local offset).
  - 팔 link1–link7: FR3 공식 자기충돌 캡슐(franka_description fr3, safety_distance 0)을 감싸는 실린더(길이 = 캡슐 길이 + 2r).
    링크당 1개라 두 번째 캡슐이 있는 link5·link7은 반경을 키웠습니다(link5 0.095, link7 0.091로 z −0.13..0.11).
    전체로 FR3 충돌 메시의 모든 정점을 무작위 자세 300개에서 감싸는 것을 확인했습니다. 원본의 "관절 사이를 잇는 z축 실린더"
    방식은 Franka의 DH 프레임(link1·link5는 다음 관절과 원점이 같음, link2·3·4·6은 x/y로 뻗음)에서 형상이 맞지 않습니다.
  - gripper: Robotiq 2F-85의 모든 충돌 geom을 전체 stroke에서 감싸는 박스 하나 (flange 프레임, 75 × 149 × 174 mm).
- **06 config의 로봇 의존 값**: `task_link: tcp`(원본 `index_tip`), `mean_mass_matrix_diag`는 원본과 같은 방법
  (env 기본 자세에서 MuJoCo `mj_fullM` 대각, Robotiq 장착 상태)으로 다시 계산, `Fjc`는 원본 주석대로 MJCF frictionloss에 맞춤
  (FR3: 1.137 ×4, 0.763, 0.44, 0.248 Nm). v2 controller는 이 마찰 보상의 변화율을 `friction_rate_max`(100 Nm/s)로 제한합니다
  (qd_des가 0이 되거나 부호가 바뀔 때 1 ms 안에 ~1 Nm가 튀어 libfranka의 토크 변화율 한계 1000 Nm/s를 넘기 때문).
  게인·임계값·주파수 등 튜닝 값은 원본과 같고, 그리퍼 칸은 위치 제어라 쓰이지 않습니다.
- **`<statistic meansize="0.1"/>`** (fr3_default.xml): MuJoCo는 뷰어 좌표축·접촉점·힘 화살표를 평균 바디 크기로
  키웁니다. 큰 enclosure가 평균을 끌어올려 좌표축이 크게 보이므로 Flexiv 씬과 같은 값으로 고정했습니다.

## 알려진 문제와 v2

원본 runner(`run_controller.py`)로 05·06을 실행하면 원본 코드와 같은 문제가 그대로 있습니다. 05·06은 위의 v2 runner로
실행하면 해결됩니다(RophiTutorial의 v2와 같은 세 가지 원인/수정, 원본 파일은 수정하지 않음).

1. **오래된 데이터 (`fix_fresh_state`, 05·06)**: 측정값 큐에서 한 번에 1개씩 꺼내고, `ctrl_pub_que`도 publisher가 한 번에
   1개씩 보내서 컨트롤러는 오래된 상태를 쓰고 env는 오래된 명령을 적용합니다. → 측정값은 최신 것만, 명령은 아직 안 보낸 것을
   버리고 최신 것만 큐에 남깁니다.
2. **최근접점 점프 (`fix_multi_point_witness`, 05)**: 링크당 FCL 최근접점이 하나라 벽과 거의 평행한 링크에서 끝에서 끝으로
   튑니다. → 링크 프리미티브를 구 체인으로 덮고 구마다 행을 만듭니다.
3. **과격하고 불연속적인 명령 (`fix_smooth_command`, 05)**: v*가 8 m/s까지 커지고, 게인이 0↔최대로 바뀌고, `collision`
   상태에서 qd_des = 0이라 벽에 들어가면 빠져나오지 못합니다. → 탈출 속도 상한, 연속 activation, QP smoothness 항,
   `collision`에서도 계속 밀어내기, 게인 램프, qd_des 변화율 제한.

Panda + Franka Hand 때 확인한 결과: 06 v2는 LCM 실행에서 타깃을 0.9 mm까지 따라갑니다(원본 runner는 0.6–0.7 m 이탈).
05 v2는 손을 벽/바닥으로 밀었다 놓으면 다시 떨어져 안정되고, 수정을 끈 원본 동작은 `collision` 상태에 갇힙니다.
FR3 + Robotiq 2F-85로 바꾼 뒤: 01–03 gravity comp은 sim에서 팔을 밀면 움직이고 놓은 자리에 그대로 머뭅니다. 06 v2는 sim에서
키 입력으로 옮긴 타깃을 `pos_tol`(5 mm) 안으로 따라갑니다(팔 토크 최대 43 Nm, 토크 변화율 최대 818 Nm/s). 실제 로봇 스택
(`run_hardware.py`, libfranka 0.17 bridge + 그리퍼 mock_mode)에서 gravity comp·06 모두 명령 1 kHz, 명령 간격 최대 ~4 ms(bridge 타임아웃
50 ms), bridge가 실행 중 매초 ~1000개의 fresh 명령을 받고, 그리퍼 열기/닫기·controller 재시작·종료가 정상입니다. mock_mode는 명령을
그대로 측정값으로 돌려주므로 실제 팔의 움직임(마찰, 센서 노이즈, 반사)은 확인하지 못했습니다.
