# behavior_velcoity_crosswalk_module

## 起動条件

laneletの**_Crosswalk_**とregulatory elementの**_Crosswalk_**の2種類があり，それぞれについて立ち上げる必要があるかを求めている．前者についてはpedestrianのrouting graphのうち自車経路と幾何的に重なっているものを求めている．

```cpp title="autoware_behavior_crosswalk_module/src/util.cpp:56:90"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/util.cpp:56:90
--8<--
```

```cpp title="autoware_behavior_crosswalk_module/src/manager.cpp:198:207@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/manager.cpp:198:207
--8<--
```
