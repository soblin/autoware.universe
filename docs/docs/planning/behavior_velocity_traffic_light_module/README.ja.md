# behavior_velocity_traffic_light_module

## 起動条件

*behavior_path_planner*からの出力の経路上に**_TrafficLight_**が紐付けられたレーンが存在したらそのレーンの数だけtraffic_lightモジュールが立ち上がるようになっている．

```cpp title="behavior_velocity_traffic_light_module/src/manager.cpp:115:118@launchNewModules"
--8<--
planning/behavior_velocity_traffic_light_module/src/manager.cpp:115:118
--8<--
```

laneletに**_TrafficLight_**のRegulatoryElementが紐付けられているものがあればそれらを取得して，そのlaneletのIDをキーとして（`const auto lane_id = module_id = traffic_light_reg_elem.second.id()`）信号RegulatoryElementのインスタンスと当該laneletのインスタンスを渡してtraffic_lightモジュールを立ち上げる．

```cpp title="behavior_velocity_traffic_light_module/src/manager.cpp:128:137@launchNewModules"
--8<--
planning/behavior_velocity_traffic_light_module/src/manager.cpp:128:137
--8<--
```

/// tip | 注意点
`planning_utils::getRegElemMapOnPath<TrafficLight>`の返り値は`{TrafficLight: Lanelet}`の形式の辞書である
///

毎サイクルの更新においてモジュールを立ち上げる必要があるかどうかは以下の関数で確認される．

```cpp title="behavior_velocity_traffic_light_module/src/manager.cpp:175:191@isModuleRegisteredFromExistingAssociatedModule"
--8<--
planning/behavior_velocity_traffic_light_module/src/manager.cpp:175:191
--8<--
```

Path上のlanelet IDsと現在登録されているモジュールIDを比較し，動作させる必要のないモジュールを破棄している．

## 信号色の扱い

`isStopSignal()`で信号色の更新と進行してよいかどうかがハンドリングされている．

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:280:282"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:280:282
--8<--
```

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:305:318"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:305:318
--8<--
```

`updateTrafficSignal()`は対応する信号の情報が現在得られているかを確認し(**findValidTrafficSignal()**)，メンバ変数の`traffic_signal_stamp_`に値をセットする．値が得られていない場合は`traffic_signal_stamp_`と`looking_tl_state_`は**そのまま更新されない**．

対応する信号の情報がそもそも得られていない場合はsim/realで挙動が異なっており，Psimではユーザーはいちいち信号の色を設定せずともegoに走ってほしいのでその場合はGOし，実車ではfail-safeにするためにSTOPするようになっている．

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:289:293@isStopSignal"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:289:293
--8<--
```

もし情報が古くなったらタイムアウト処理により停止する．

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:297:299@isStopSignal"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:297:299
--8<--
```

`traffic_light_utils::isTrafficSignalStop()`はバルブの色が黄/赤かつ交差点において対応する矢印信号が点灯していなければ`true`すなわちSTOPを返す．

```cpp title="traffic_light_utils/src/traffic_light_utils.cpp:80:108"
--8<--
common/traffic_light_utils/src/traffic_light_utils.cpp:80:108
--8<--
```

## 停止する条件と状態遷移とヒステリシス

### 状態遷移

状態は`APPROACH`と`GO_OUT`の2種類ある．初期状態は`APPROACH`である．

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:173:173"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:173:173
--8<--
```

停止線までの距離`d`によって

- `d > 1.0`
  - `GO` -> `APPROACH`
- `d < -2.0`
  - `APPROACH` -> `GO`

と状態遷移し，初期状態は`GO`なので

- もしegoが信号停止線から距離1m以内の位置か，信号停止線を過ぎた位置からengageした場合は一切停止しない
- それ以外の場合は必ず`APPROACH`に遷移するので停止する可能性がある

### 停止する条件

RTCを使っているのでモジュール側の停止判断と実際の停止挙動は論理的に分離している．

1. `setSafe(false)`を送る条件
2. `not isActivated()`の場合に停止する追加条件

に分けて説明する．

#### setSafe(false)を送る条件

状態が`APPROACH`であること，かつ信号の色や矢印の情報的に`isStopSignal() == true`であることが必要条件である．それに加え，

- `isStopSignal() == true`になってから`$stop_time_histeresis`以上の時間が経った．これにより`is_prev_state_stop_ == true`に移行する OR
- すでに`is_prev_state_stop_ == true`である

であれば`setSafe(false)`を送るようになっている．

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:244:255@modifyPathVelocity"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:244:255
--8<--
```

#### not isActivated()の場合に停止する追加条件

追加で`isPassthrough() == false`であれば停止する(`is_prev_state_stop_ == true`になる)．

`stoppable`は制動距離的に停止可能(または十分速度が低い)かどうかを表す変数である．

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:331:341@isPassthrough"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:331:341
--8<--
```

まず

- `$enable_pass_judge == false`
- `stoppable == true`
- `is_prev_state_stop_ == true`

のいずれかであればこの関数は常に`false`を返すので停止する．

上記のいずれかが成り立たない場合，停止線までの距離が以下の`reachable_distance`以上であれば停止する．

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:326:327@isPassthrough"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:326:327
--8<--
```

```cpp title="behavior_velocity_traffic_light_module/src/scene.cpp:347:352@isPassthrough"
--8<--
planning/behavior_velocity_traffic_light_module/src/scene.cpp:347:352
--8<--
```
