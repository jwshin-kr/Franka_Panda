# RoPhiTutorial — Franka Emika Panda (튜토리얼 01–06)

[RoPhiTutorial](https://github.com/jwshin-kr/RoPhiTutorial)의 env / controller / visualizer 구조와 코드·수식을 그대로 두고,
로봇만 Flexiv Rizon4 + Robotis RH-5 → **Franka Emika Panda + Franka Hand**로 바꾼 버전입니다.
현재 범위는 튜토리얼 06 (IK QP)까지입니다. 바닥·벽·enclosure(셀)·조명 등 로봇 외 환경은 원본과 동일합니다.

## 실행

터미널 3개에서 저장소 루트(`Franka_Panda/`) 기준으로 env → controller → visualizer 순서로 실행합니다.

```bash
conda activate rophi

# 01–03 gravity compensation
python run_env.py --config configs/franka_panda/env/default.yaml
python run_controller.py --config configs/franka_panda/controllers/grav_comp.yaml
python run_visualizer.py --config configs/visualizer/franka_panda/default.yaml

# 04 publish collision geometry (env 단독 실행 가능)
python run_env.py --config configs/franka_panda/env/collision.yaml

# 05 collision-aware gravity compensation
python run_env.py --config configs/franka_panda/env/collision.yaml
python run_controller.py --config configs/franka_panda/controllers/col_aware_grav_comp.yaml
python run_visualizer.py --config configs/visualizer/franka_panda/collision.yaml

# 06 inverse kinematics QP (controller 터미널에서 w/s, a/d, r/f, [/], h)
python run_env.py --config configs/franka_panda/env/collision.yaml
python run_controller.py --config configs/franka_panda/controllers/ik_qp.yaml
python run_visualizer.py --config configs/visualizer/franka_panda/ik_qp.yaml
```

### v2 (05·06 문제 수정판)

원본 파일과 factory는 그대로 두고, v2 runner가 v2 클래스를 직접 만듭니다(RophiTutorial의 v2와 같은 구성).

```bash
# 05 v2: 세 가지 원인 수정 + 디버그 오버레이(고스트 로봇, witness 점/궤적, v* 화살표, 충돌 구, 플롯)
python run_env.py --config configs/franka_panda/env/collision.yaml
python run_controller_v2.py --config configs/franka_panda/controllers/col_aware_grav_comp_v2.yaml
python run_visualizer_v2.py --config configs/visualizer/franka_panda/collision_v2.yaml

# 원본 동작 재현 (수정 하나씩 꺼볼 수도 있음)
python run_controller_v2.py --config configs/franka_panda/controllers/col_aware_grav_comp_v2.yaml \
    --v2.fix_fresh_state false --v2.fix_multi_point_witness false --v2.fix_smooth_command false

# 06 v2: 원인 1(오래된 LCM 데이터)만 수정, 원본 ik_qp.yaml 그대로 사용
python run_env.py --config configs/franka_panda/env/collision.yaml
python run_controller_v2.py --config configs/franka_panda/controllers/ik_qp.yaml --name ik_qp_v2
python run_visualizer.py --config configs/visualizer/franka_panda/ik_qp.yaml
```

| v2 파일 | 역할 |
|---|---|
| `controller/CollisionAwareGravCompControl_v2.py` | 05 원본 상속 + 세 가지 수정 (`v2.fix_*`로 개별 on/off) |
| `controller/IKQPControl_v2.py` | 06 원본 상속 + 원인 1 수정 |
| `utils/collision/sphere_swept_v2.py` | 링크 프리미티브 → 구 체인, 해석적 signed distance (원인 2) |
| `communication/.../JointCtrlColDebugPubManager_v2.py` | joint_ctrl + 디버그 채널 `sw_col_aware_debug_v2` |
| `visualizer/data_visualizer/ColAwareDebugVisualizer_v2.py` | 세 원인을 보여주는 Viser 오버레이 |
| `visualizer/vis_manager/franka_panda/FrankaPandaVisManager_v2.py` | 원래 visualizer + 위 오버레이 |
| `run_controller_v2.py`, `run_visualizer_v2.py` | v2 runner |
| `configs/franka_panda/controllers/col_aware_grav_comp_v2.yaml`, `configs/visualizer/franka_panda/collision_v2.yaml` | v2 설정 |

- Viser: http://localhost:8080
- `p` / `q`: 일시정지 / 종료 (각 터미널), `r`: env 리셋, MuJoCo 뷰어에서 `Space`: 일시정지
- MuJoCo 충돌 메시는 geom group 4라 기본으로 숨겨져 있습니다. MuJoCo 뷰어에서 geom group 4를 켜면 보입니다.
- Viser의 `Show collision geometry` / `Show collision distances`는 05·06 설정에서 env가 보내는 충돌 형상을 그립니다.
- 원본처럼 커맨드라인으로 설정을 덮어쓸 수 있습니다. 예: `--vis_mode vis_off`, `--arm_damping 0.5`

## LCM 채널

| Channel | Type | 방향 | 내용 |
|---|---|---|---|
| `sw_franka_arm_joint_meas` | `joint_meas_t` | env → controller, visualizer | 팔 q, q̇, τ (7) |
| `sw_franka_hand_joint_meas` | `joint_meas_t` | env → controller, visualizer | 손가락 q, q̇, τ (2, 단위 m) |
| `sw_franka_arm_hand_joint_ctrl` | `joint_ctrl_t` | controller → env | `[arm(7), hand(2)] = 9` |
| `sim_clock` | `clock_t` | env → controller | 시뮬레이션 시간 |
| `sw_franka_arm_hand_robot_col_info` | `col_info_t` | env → controller, visualizer | 로봇 링크 프리미티브 (link-local) |
| `sw_franka_arm_hand_static_col_info` | `col_info_t` | env → controller, visualizer | floor / wall_back / wall_left (world) |
| `sw_ik_target_pose` | `pose3d_t` | controller → visualizer | IK 타깃, 현재 tip, tip 속도 (06) |

## 구성

로봇에 해당하는 파일 (원본 Flexiv 파일과 1:1 대응):

```
assets/scene/franka_panda/
  panda_robot.xml        Panda + Hand MJCF (MuJoCo Menagerie 기반, 액추에이터 없음)
  panda_default.xml      씬: floor / wall_back / wall_left / enclosure / 조명 (원본과 동일) + 토크 모터 9개
  urdf/panda_viser.urdf  Pinocchio(동역학) + Viser(시각화)용 URDF
  panda/                 메시 (visual: OBJ, collision: STL/OBJ), LICENSE (Apache-2.0)
  cell/                  enclosure 메시 (원본 Flexiv 셀과 같은 파일)
robot_platform/sim/franka_panda/SimFrankaPanda.py                                  ← SimFlexivArm5FHand.py
env/franka_panda/FrankaArmHandColEnv.py                                            ← FlexivArmHandColEnv.py
communication/lcm/publisher/pub_manager/env/franka_panda/FrankaArmHandPubManager.py        ← FlexivArmHandPubManager.py
communication/lcm/publisher/pub_manager/env/franka_panda/FrankaArmHandColInfoPubManager.py ← FlexivArmHandColInfoPubManager.py
visualizer/vis_manager/franka_panda/FrankaPandaVisManager.py                       ← FlexivArmHandVisManager.py
visualizer/data_visualizer/FrankaPandaVisualizer.py                                ← FlexivArmHandVisualizer.py
configs/franka_panda/{env,controllers}/, configs/visualizer/franka_panda/          ← configs/flexiv_arm_5F_hand/, configs/visualizer/flexiv_arm_5F_hand/
```

나머지 파일(`controller/*`, `env/BaseEnv.py`, `env/MujocoBaseEnv.py`, `communication/`, `data_type/`, `lcm_type/`, `utils/`,
`run_*.py`)은 GitHub 원본과 같은 경로·같은 내용입니다. 기본 채널 이름/관절 수 같은 로봇 기본값만 Franka로 바꿨고,
factory(`get_env`, `get_controller`, ...)에는 튜토리얼 06까지 쓰는 이름만 등록했습니다. Visualizer는 원본의 큰 클래스에서
06까지 쓰는 부분(URDF, 충돌 오버레이, IK 타깃 오버레이)만 같은 코드로 옮겼습니다.

## 모델 메모

- **기본 자세**: 팔 `[0, 0, 0, -90°, 0, 90°, 45°]`, 손가락 0.04 m (열림).
  Franka의 표준 ready 자세 `[0, -45°, 0, -135°, 0, 90°, 45°]`는 이 셀에서 link3(팔꿈치)가 `wall_back`에 닿습니다.
- **MJCF ↔ URDF**: 링크/조인트 이름, 기구학, 관성이 동일합니다. Pinocchio와 MuJoCo의 g(q), M(q), FK가
  무작위 자세 500개에서 1e-8 이내로 일치하는 것을 확인했습니다. 한쪽을 고치면 다른 쪽도 같이 고쳐야 합니다.
- **Armature**: Menagerie 값(모든 조인트 0.1, damping 1)을 그대로 씁니다. URDF에는 armature를 표현할 수 없으므로,
  Pinocchio의 M(q)를 MuJoCo와 맞추려면 `model.armature[:] = 0.1`이 필요합니다(원본 코드에는 없음, 03–06에는 M(q) 미사용).
- **손가락**: Menagerie의 tendon/equality 대신 손가락마다 독립 모터(±70 N)를 둡니다. 원본의 손과 같은 방식입니다.
- **충돌 프리미티브** (`SimFrankaPanda.get_col_info_data`): 원본처럼 링크 이름마다 프리미티브 1개(link-local offset).
  - 팔 link1–link7: Franka 공식 자기충돌 캡슐(franka_description `franka_arm.xacro`, safety_distance 0)을 감싸는
    실린더(길이 = 캡슐 길이 + 2r). 원본의 "관절 사이를 잇는 z축 실린더" 방식은 Panda의 DH 프레임(link1·link5는 다음
    관절과 원점이 같음, link2·3·4·6은 x/y로 뻗음)에서 형상이 맞지 않아 이 데이터로 대체했습니다.
  - hand: 공식 hand 캡슐을 감싸는 y축 실린더, left/right_finger: 손가락 충돌 메시의 경계 박스.
- **06 config의 로봇 의존 값**: `task_link: tcp`(원본 `index_tip`), `mean_mass_matrix_diag`는 원본과 같은 방법
  (env 기본 자세에서 MuJoCo `mj_fullM` 대각)으로 다시 계산, `Fjc`는 원본 주석대로 MJCF frictionloss에 맞춰 0
  (Panda MJCF에는 frictionloss가 없음). 게인·임계값·주파수 등 튜닝 값은 원본과 같고, hand 부분은 원본 hand 값을 2개로 줄였습니다.
- **`<statistic meansize="0.1"/>`** (panda_default.xml): MuJoCo는 뷰어 좌표축·접촉점·힘 화살표를 평균 바디 크기로
  키웁니다. Panda는 바디가 14개뿐이라 큰 enclosure가 평균을 0.25로 끌어올려 좌표축이 2.5배 커 보였기 때문에,
  Flexiv 씬과 같은 값으로 고정했습니다.

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

Franka에서 확인한 결과: 06 v2는 LCM 실행에서 타깃을 0.9 mm까지 따라갑니다(원본 runner는 0.6–0.7 m 이탈).
05 v2는 손을 벽/바닥으로 밀었다 놓으면 다시 떨어져 안정되고, 수정을 끈 원본 동작은 `collision` 상태에 갇힙니다.
