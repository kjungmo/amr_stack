<!-- SPDX-License-Identifier: Apache-2.0 -->
# 코어 통합 안내서 — `amr_api`로 AMR 스택 구동하기

이 안내서는 AMR 스택 위에 별도의 상위 제어 모듈(이하 "코어")을 이식하는 방법을
설명합니다. 끝까지 읽으시면 직접 만드신 코어 저장소에서 표준 `amr_api`
인터페이스를 통해 SLAM, 위치 추정, 지도 작성, 경로 계획, 주행을 구동하고,
원하시는 모듈 하나를 직접 만든 구현으로 교체하실 수 있습니다.

함께 보면 좋은 문서: [`CONTRACT.md`](../CONTRACT.md) (고정된 설계 기준이며, §11에
데이터와 토픽 대응표가 있습니다), [`src/amr_api/README.md`](../src/amr_api/README.md),
그리고 `docs/superpowers/specs/` 아래의 설계 명세서입니다.

---

## 1. 개요

`amr_api`는 **노드가 없는 인터페이스 계층**입니다. 순수 데이터를 입력과 출력으로
주고받는 추상 C++ 인터페이스 여섯 개를 정의합니다. 기존 모듈마다 얇은 **위임
어댑터**가 들어 있어, 이미 검증된 알고리즘 클래스에 호출을 그대로 넘기는 방식으로
인터페이스 하나를 구현합니다. 알고리즘 코드 자체는 건드리지 않습니다.

코어는 **두뇌이자 버스**입니다. 즉 공유 상태판(최신 지도, 자세, 속도, 경로,
입자 군집)과 시계, 기록(로깅)을 직접 소유하며, 인터페이스를 통해 매 주기 모듈을
구동합니다.

의존 방향은 위에서 아래로 한쪽으로만 흐릅니다.

```
amr_core  →  amr_api  →  (각 모듈 + 어댑터)  →  amr_reference_core / 직접 만든 코어
```

`amr_api`는 공유 자료형이 들어 있는 `amr_core`에만 의존합니다. 모든 모듈
패키지는 `amr_api`에 의존하며 자신의 어댑터를 제공합니다. 코어는 `amr_api`와,
재사용하시려는 모듈 패키지에만 의존하면 됩니다.

---

## 2. 기본 개념

**알고리즘은 라이브러리, ROS 배선은 얇은 노드, 통합은 순수 데이터.**

- 각 모듈의 수치 핵심부는 노드가 없는 `*_lib` 대상에 들어 있고, 기존 시험
  43건으로 검증됩니다. 어댑터는 그대로 전달하는 계층이며, 동등성 검증 시험이
  어댑터의 출력이 원본 클래스의 출력과 같음을 증명합니다.
- 코어는 모듈과 ROS 메시지로 대화하지 않습니다. `amr_core` 구조체(`Pose2D`,
  `Twist2D`, `LaserScan`, `OccupancyGrid`)와 몇 가지 `amr_api` 묶음 자료형
  (`Path`, `ParticleCloud`)만 주고받습니다. 코어를 노드로 실행할 때 이 데이터가
  ROS 토픽에 어떻게 대응되는지는 `CONTRACT.md` §11에 한 번만 정리되어 있습니다.

구조도:

```
                         ┌───────────────────────────────┐
                         │  코어 (직접 만든 저장소)       │
                         │  두뇌 · 버스 · 시계 · 기록     │
                         └───────────────┬───────────────┘
                                         │ 다음 계층을 대상으로 프로그래밍
                                         ▼
                         ┌───────────────────────────────┐
                         │  amr_api   (노드 없음)         │
                         │  인터페이스 · 입출력 구조체 ·  │
                         │  CostmapView · Logger/Clock ·  │
                         │  version.hpp   [의존: amr_core]│
                         └───────────────┬───────────────┘
            (변경되지 않은) 라이브러리에 위임하는 어댑터들이 구현 ↓
   ┌──────────┬──────────────┬──────────┬───────────────┬──────────────┐
   │ amr_slam │amr_localiza- │amr_map-  │ amr_planning  │amr_navigation│
   │  ISlam   │tion ILocalizer│ping      │ IGlobalPlanner│  IBehavior   │
   │          │              │ IMapper  │ ILocalPlanner │              │
   │          │              │          │ CostmapView   │              │
   └──────────┴──────────────┴──────────┴───────────────┴──────────────┘
```

---

## 3. 여섯 개의 인터페이스

모든 입력과 출력은 순수 구조체입니다. 모든 메서드는 매 주기 호출하는 방식입니다.
요청 구조체 안의 포인터는 소유하지 않는 참조이며, 호출이 끝날 때까지 유효해야
합니다.

### 3.1 `ISlam` (`amr_api/slam.hpp`) — 기준 어댑터 `amr_slam::ScanMatchingSlamAdapter`

```cpp
struct SlamInput {
  Pose2D odom_delta;              // robot-frame increment since last tick
  std::optional<LaserScan> scan;  // present only on scan ticks
};
class ISlam {
 public:
  virtual ~ISlam() = default;
  virtual Pose2D update(const SlamInput& in) = 0;  // returns map-frame pose
  virtual Pose2D pose() const = 0;
  virtual OccupancyGrid map() const = 0;
};
```

주기 규약: 주행계 증분(그리고 스캔이 있을 때는 스캔)을 넣으면, 보정된 지도 좌표계
자세를 돌려받습니다. `map()`은 원하시는 주기로 읽으시면 됩니다(기준 SLAM은 "지도
변경" 신호를 내보내지 않으므로 코어가 직접 조회합니다).

### 3.2 `ILocalizer` (`amr_api/localizer.hpp`) — 기준 어댑터 `amr_localization::MclAdapter`

```cpp
struct LocalizerInput {
  Pose2D odom_delta;
  std::optional<LaserScan> scan;  // correction runs only when present
};
struct LocalizerOutput {
  Pose2D pose;
  ParticleCloud cloud;
};
class ILocalizer {
 public:
  virtual ~ILocalizer() = default;
  virtual LocalizerOutput update(const LocalizerInput& in) = 0;
  virtual Pose2D estimate() const = 0;
  virtual void set_pose(const Pose2D& pose) = 0;
};
```

주기 규약: `update()`는 주행계 증분으로 예측하고, 스캔이 있으면 보정한 뒤, 가중
평균 자세와 (시각화용) 입자 군집을 돌려줍니다. 어댑터가 내부의 예측·보정·추정
필터를 이 한 번의 호출로 이어 줍니다.

### 3.3 `IMapper` (`amr_api/mapper.hpp`) — 기준 어댑터 `amr_mapping::OccupancyGridMapperAdapter`

```cpp
struct MapperInput {
  Pose2D pose;
  LaserScan scan;
};
class IMapper {
 public:
  virtual ~IMapper() = default;
  virtual void integrate(const MapperInput& in) = 0;
  virtual OccupancyGrid map() const = 0;
};
```

주기 규약: 알려진 자세에서 얻은 스캔을 로그 확률비 지도에 누적하고, 그 결과를
`OccupancyGrid`로 읽어 옵니다.

### 3.4 `IGlobalPlanner` (`amr_api/global_planner.hpp`) — 기준 어댑터 `amr_planning::AstarGlobalPlanner`

```cpp
struct GlobalPlanRequest {
  std::array<double, 2> start_xy;
  std::array<double, 2> goal_xy;
  const CostmapView* costmap;  // non-owning; must outlive the call
};
class IGlobalPlanner {
 public:
  virtual ~IGlobalPlanner() = default;
  virtual std::optional<Path> plan(const GlobalPlanRequest& req) = 0;
};
```

주기 규약: 비용 지도 위에서 출발점부터 목표점까지 경로를 계획합니다. 도달할 수
없으면 `nullopt`입니다.

### 3.5 `ILocalPlanner` (`amr_api/local_planner.hpp`) — 기준 어댑터 `amr_planning::DwaLocalPlanner`

```cpp
struct LocalPlanRequest {
  Pose2D pose;
  Twist2D vel;
  const Path* path;            // non-owning; the global plan to track
  const CostmapView* costmap;  // non-owning
};
struct LocalPlanResult {
  Twist2D cmd;
  bool blocked;  // true if every sampled rollout collides
};
class ILocalPlanner {
 public:
  virtual ~ILocalPlanner() = default;
  virtual LocalPlanResult compute(const LocalPlanRequest& req) = 0;
};
```

주기 규약: 현재 자세와 속도, 전역 경로를 주면 속도 명령 하나(와 막힘 여부
`blocked`)를 돌려줍니다.

### 3.6 `IBehavior` (`amr_api/behavior.hpp`) — 기준 어댑터 `amr_navigation::NavigatorBehavior`

```cpp
enum class BehaviorState { IDLE, PLANNING, FOLLOWING, RECOVERY, SUCCEEDED, FAILED };
const char* to_string(BehaviorState s);

struct BehaviorInput {
  Pose2D pose;
  Twist2D vel;
  const CostmapView* costmap;  // non-owning
  double now;                  // seconds, from core's Clock
};
struct BehaviorOutput {
  Twist2D cmd;
  BehaviorState state;
  std::optional<Path> plan;    // current global plan, for viz
};
class IBehavior {
 public:
  virtual ~IBehavior() = default;
  virtual void set_goal(const Pose2D& goal) = 0;
  virtual void cancel() = 0;
  virtual BehaviorOutput update(const BehaviorInput& in) = 0;
  virtual BehaviorState state() const = 0;
};
```

주기 규약: 목표를 한 번 설정한 뒤, 매 주기 `update()`를 불러 속도 명령, 상태
기계의 상태, 그리고 (시각화용) 현재 전역 경로를 받습니다. `IBehavior`는 "경로
계획과 주행"을 합친 단위입니다. 기본 구현은 목표 지향 상태 기계 전체(전역 A* +
DWA + 복구 동작)를 실행합니다. `BehaviorState`는 주행기 내부의 `NavState`와 일대일로
대응합니다.

---

## 4. CostmapView 짝 맞춤 규약

`amr_api`는 (구체적인 `Costmap`과 비용 팽창 계산이 들어 있는) `amr_planning`에
의존하지 않으므로, 경로 계획기와 주행 동작은 읽기 전용 `amr_api::CostmapView`를
받습니다. 기준 구현은 이것을 `amr_planning`에서 만듭니다.

```cpp
#include "amr_planning/costmap_adapter.hpp"

std::unique_ptr<amr_api::CostmapView> costmap =
    amr_planning::make_costmap(grid, amr_core::CostmapConfig{}, robot.radius);
```

내부적으로 기준 경로 계획기는 `amr_planning::as_costmap(view)`로 구체 비용 지도를
다시 얻습니다. **경로 계획기와 그것이 소비하는 비용 지도는 한 쌍입니다.** 코어가
비용 지도 표현을 교체한다면, 그것을 읽는 경로 계획기도 함께 교체해야 합니다(또는
자신의 경로 계획기가 이해할 수 있는 `CostmapView`를 제공해야 합니다).

---

## 5. 진단 연결부 (`amr_api/diagnostics.hpp`)

기록과 시계는 코어가 소유합니다. `amr_api`는 연결부(`Logger`, `Clock`)와 아무 일도
하지 않는 `NullLogger`만 정의하므로, 어댑터는 코어 없이도 시험 안에서 단독으로
동작합니다. 기준 코어는 다음과 같이 구현합니다(`amr_reference_core/core_bus.hpp`).

```cpp
class StderrLogger : public amr_api::Logger {
 public:
  void log(Level level, const std::string& msg) override {
    const char* tag = "INFO";
    switch (level) {
      case Level::Debug: tag = "DEBUG"; break;
      case Level::Info:  tag = "INFO";  break;
      case Level::Warn:  tag = "WARN";  break;
      case Level::Error: tag = "ERROR"; break;
    }
    std::fprintf(stderr, "[core][%s] %s\n", tag, msg.c_str());
  }
};

class SimClock : public amr_api::Clock {
 public:
  double now() const override { return t_; }
  void advance(double dt) { t_ += dt; }
 private:
  double t_{0.0};
};
```

실제 시각 대신 단조 증가하는 모의 시계를 쓰면 실행이 결정적이고 재현 가능해집니다.
`clock.now()`를 `BehaviorInput::now`에 넣어 주십시오.

---

## 6. 코어 배선하기 — 단계별 설명

완전하고 실행 가능한 예제는
`src/amr_reference_core/test/test_reference_core.cpp`의 `NavMissionReachesGoal`
입니다. 공유 상태판은 `CoreBus`입니다.

```cpp
struct CoreBus {
  amr_api::OccupancyGrid map;
  amr_api::Pose2D pose;
  amr_api::Twist2D vel;
  amr_api::Path plan;
  amr_api::ParticleCloud particles;
  StderrLogger logger;
  SimClock clock;
};
```

주석을 단 주행 반복문:

```cpp
amr_reference_core::CoreBus bus;
bus.map  = open_arena();                       // 살아 있는 지도 (SLAM 또는 파일에서)
bus.pose = amr_core::Pose2D{0.5, 0.5, 0.0};
bus.vel  = amr_core::Twist2D{0.0, 0.0};

amr_core::RobotConfig robot;
amr_core::CostmapConfig costmap_cfg;

// (1) 코어가 비용 지도를 소유하며, 경로 계획 구현으로 만듭니다.
auto costmap = amr_planning::make_costmap(bus.map, costmap_cfg, robot.radius);

// (2) "경로 계획과 주행" 모듈을 인터페이스 뒤에 둡니다.
std::unique_ptr<amr_api::IBehavior> behavior =
    std::make_unique<amr_navigation::NavigatorBehavior>(
        amr_core::NavConfig{}, amr_core::AstarConfig{}, amr_core::DwaConfig{},
        robot);

// (3) 목표를 한 번 설정합니다.
const amr_core::Pose2D goal{3.0, 3.0, 0.0};
behavior->set_goal(goal);

// (4) 상태 기계를 매 주기 진행합니다. 코어는 순수 데이터를 넣고 명령을 받아 전달합니다.
const double dt = 0.1;
for (int ticks = 0; ticks < 600; ++ticks) {
  amr_api::BehaviorInput in;
  in.pose    = bus.pose;
  in.vel     = bus.vel;
  in.costmap = costmap.get();
  in.now     = bus.clock.now();

  const amr_api::BehaviorOutput out = behavior->update(in);
  bus.vel = out.cmd;
  if (out.plan.has_value()) bus.plan = *out.plan;

  // (5) 명령을 로봇/모의기에 전달합니다. 여기서는 단륜 모델로 적분합니다.
  bus.pose.x += out.cmd.v * std::cos(bus.pose.theta) * dt;
  bus.pose.y += out.cmd.v * std::sin(bus.pose.theta) * dt;
  bus.pose.theta += out.cmd.omega * dt;
  bus.clock.advance(dt);

  if (out.state == amr_api::BehaviorState::SUCCEEDED ||
      out.state == amr_api::BehaviorState::FAILED) break;
}
```

SLAM 이후 주행으로 이어지는 임무도 형태가 같습니다. SLAM 단계에서는
`SlamInput{odom_delta, scan}`을 `ISlam::update`에 넣고 `ISlam::map()`을
`bus.map`에 복사합니다. 이후 주행 단계로 전환하여 자세는 `ILocalizer`로, 명령은
`IBehavior`로 얻습니다.

---

## 7. 모듈 교체하기

예를 들어 SLAM을 직접 만든 구현으로 교체하려면 다음과 같이 합니다.

1. 코어 저장소에서 `amr_api::ISlam`을 구현하는 클래스를 작성합니다.
2. `amr_slam::ScanMatchingSlamAdapter` 대신 그 클래스를 생성합니다.
3. 나머지 배선은 그대로 둡니다. 스택의 다른 부분은 `ISlam`만 봅니다.

같은 방식이 모든 인터페이스에 적용됩니다. 교체 단위는 다음과 같습니다.

- `IBehavior`를 구현하면 "경로 계획과 주행" 단위 전체를 교체합니다.
- 또는 기본 주행 상태 기계는 그대로 두고, `IGlobalPlanner` / `ILocalPlanner`를
  구현하여 전역 또는 지역 경로 계획기만 교체한 뒤, 직접 만든 제어 반복문에서
  구동합니다.

인터페이스가 순수 데이터이므로, 구현 전체를 비공개 저장소 안에 두고 `amr_api`
(그리고 자료형을 위한 `amr_core`)만 연결하면 됩니다.

---

## 8. 외부 저장소에서 amr_api에 맞춰 코어 빌드하기

패키지의 `CMakeLists.txt`에서:

```cmake
find_package(amr_core REQUIRED)
find_package(amr_api REQUIRED)
find_package(amr_planning REQUIRED)      # 재사용하는 모듈만
find_package(amr_navigation REQUIRED)

ament_target_dependencies(<your_target>
  amr_core amr_api amr_planning amr_navigation)
```

`package.xml`에서:

```xml
<depend>amr_core</depend>
<depend>amr_api</depend>
<depend>amr_planning</depend>
<depend>amr_navigation</depend>
```

`find_package`가 해석되도록, 빌드 전에 이 작업 공간의 설치 환경을 불러옵니다.

```bash
micromamba run -n ros2_humble bash -c '
  cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  source install/setup.bash && \
  colcon build --packages-select <your_package>'
```

씨앗값을 받는 `MclAdapter`를 재사용한다면 `std::unique_ptr`로 보관하십시오. 내부
필터가 참조하는 난수 생성기를 소유하기 때문에 복사와 이동이 금지되어 있습니다.

---

## 9. 결정성과 충실성

- 무작위성의 연결부(난수 씨앗값)는 인터페이스가 아니라 어댑터에 있습니다. 예를
  들어 `MclAdapter(grid, cfg, seed, initial_pose)`처럼요. 같은 씨앗값이면 실행
  결과가 비트 단위로 같습니다.
- 모든 수치 상수는 `amr_core::*Config` 기본값에서 가져오며, 하드코딩하지
  않습니다(`CONTRACT.md` §9).
- 어댑터마다 동등성 검증 시험이 있어, 같은 입력에 대해 어댑터의 출력이 원본
  클래스의 출력과 같음을 단언합니다. 어댑터가 그대로 전달하는 계층임이
  증명됩니다.
- `CONTRACT.md` §2의 모든 규약(좌표계, 단위, 점유 의미)이 그대로 유지됩니다.
  어댑터는 `amr_core` 자료형을 그대로 나르므로 변환 의미가 추가되지 않습니다.

---

## 10. 검증

작업 공간 전체를 빌드하고 시험합니다(`CONTRACT.md` §8 기준).

```bash
micromamba run -n ros2_humble bash -c '
  cd /home/cona/kangj/amr_stack && unset PYTHONPATH && \
  colcon build && colcon test && colcon test-result --all'
```

예상 결과: 모든 패키지가 빌드되고, 기존 시험 43건이 여전히 통과하며, 새 어댑터
동등성 시험과 `amr_reference_core` 임무 시험이 통과하고, `amr_bringup`의
실행 시험 예제가 통과합니다. 이번 통합은 시험을 더할 뿐, 검증된 알고리즘 코드는
바꾸지 않습니다.

---

## 11. 참고 자료

- [`CONTRACT.md`](../CONTRACT.md) — §1 의존 관계도, §11 데이터와 토픽 대응표,
  `CONTRACT_VERSION` 1.0.
- [`src/amr_api/README.md`](../src/amr_api/README.md) — 인터페이스 목록과 비용 지도
  짝 맞춤 규약.
- `docs/superpowers/specs/2026-06-16-amr-core-integration-design.md` — 설계 근거.
- `src/amr_reference_core/` — 실행 가능한 기준 코어와 그 시험.
