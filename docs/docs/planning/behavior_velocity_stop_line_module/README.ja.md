# behavior_velcoity_stop_line_module

経路上に道路標識の一時停止線があればそこで`$stopline_duration_sec`だけ一時停止を行う．

## 起動条件

*behavior_path_planner*からの出力の経路上に**_TrafficSign_**が紐付けられたレーンが存在したらそのレーンの数だけtraffic_lightモジュールが立ち上がるようになっている．

```cpp title="autoware_behavior_velocity_stop_line_module/src/manager.cpp:79:88@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/manager.cpp:79:88
--8<--
```

（firstがregulatory element，secondがlaneletのID）

## 停止するまで

初期状態は`APPROACH`である．

```cpp title="autoware_behavior_velocity_stop_line_module/src/scene.cpp:29:36@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/scene.cpp:29:36
--8<--
```

pathと停止線の交差する位置を求める．

```cpp title="autoware_behavior_velocity_stop_line_module/src/scene.cpp:56:69@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/scene.cpp:56:69
--8<--
```

`APPROACH`状態で停止線に経路が被った段階で停止線を挿入するので

```cpp title="autoware_behavior_velocity_stop_line_module/src/scene.cpp:90:90@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/scene.cpp:90:90
--8<--
```

そのうち車両は停止状態になることが期待される．一時停止線まで`$hold_stop_margin_distasnce`以内の所まで近づいて停止したら`STOPPED`に遷移する．またその時刻を`stopped_time_`に記録する．

```cpp title="autoware_behavior_velocity_stop_line_module/src/scene.cpp:108:114@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/scene.cpp:108:114
--8<--
```

`STOPPED`の場合は停止線を入れた上で`STOPPED`に遷移した時刻から現在までの経過時間を求める．もし`$stop_duration_sec`以上経過していたら`START`に状態遷移する．

```cpp title="autoware_behavior_velocity_stop_line_module/src/scene.cpp:155:157@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/scene.cpp:155:157
--8<--
```

`START`に遷移する場合必ず`STOPPED`を経由しているはずで，そのためには`signed_arc_dist_to_stop_point < planner_param_.hold_stop_margin_distance`であることが必要条件であるので，以下の部分はデッドコードである可能性が高い（制御やバック走行などの外乱がある場合はその限りではない）．

```cpp title="autoware_behavior_velocity_stop_line_module/src/scene.cpp:163:170@launchNewModules"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/scene.cpp:163:170
--8<--
```
