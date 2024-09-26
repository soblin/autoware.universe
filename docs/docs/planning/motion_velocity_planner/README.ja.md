# motion_velocity_planner

他のbehavior\_path\_plannerなどに比べると構造はかなりシンプルで，入力のtrajectoryを受け取ると

```cpp title="autoware_motion_velocity_planner_node/src/node.cpp:66:69@MotionVelocityPlannerNode"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_planner_node/src/node.cpp:66:69
--8<--
```

により

```cpp title="autoware_motion_velocity_planner_node/src/node.cpp:66:69@on_trajectory"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_planner_node/src/node.cpp:278:281
--8<--
```

で**generate_trajectory**

```cpp title="autoware_motion_velocity_planner_node/src/node.cpp:66:69@generate_trajectory"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_planner_node/src/node.cpp:377:401
--8<--
```

と**PlannerManager::plan_velocities**

```cpp title="autoware_motion_velocity_planner_node/src/planner_manager.cpp:74:81"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_planner_node/src/node.cpp:74:81
--8<--
```

経由してプラグインの関数を呼び出している．

```cpp title="autoware_motion_velocity_planner_common/velocity_planning_result.hpp"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_planner_common/include/autoware/motion_velocity_planner_common/velocity_planning_result.hpp:74:81
--8<--
```
