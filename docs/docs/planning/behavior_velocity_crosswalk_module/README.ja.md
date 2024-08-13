# behavior_velocity_crosswalk_module

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

## 処理

経路のセグメントをイテレートし，横断歩道のポリゴンと一番初めと一番最後に交差する線分における交点をそれぞれ求めている．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:230:236@modifyPathVelocity"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:230:236
--8<--
```

```cpp title="autoware_behavior_crosswalk_module/src/util.cpp:124:146@getPathEndPointsOnCrosswalk"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/util.cpp:124:146
--8<--
```

### 停止する位置

regulatory_elementに停止線が紐付けられていればそれを`default_stop_pose`として，ない場合は衝突しうる位置からある程度マージンを空けた位置を停止位置として利用する．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:245:246@modifyPathVelocity"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:245:246
--8<--
```

**checkStopForCrosswalkUsers**と**checkStopForStuckVehicles**は停止位置の情報だけを求めている．この2つの関数はターゲットに対して停止する必要がある場合に有効値を返すようになっている．そのためこの2つが返した値はRTCのsafeの値と関係している．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:253:269@modifyPathVelocity"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:253:269
--8<--
```

RTCのdistanceの値は地図停止線があればそこを，なければ対象物に対して停止するであろう位置への距離を求める(停止するべき対象がなければ設定しない)．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:273:273@modifyPathVelocity"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:273:273
--8<--
```

RTCの承認に応じて停止・進行の判断を行う．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:275:280@modifyPathVelocity"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:275:280
--8<--
```

**planGO**の場合でも徐行は行われる．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1262:1273@planGo"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1262:1273
--8<--
```

**planSTOP**の場合は(関数はinsertDecelPointという名前だが)速度0を埋め込んでいる．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1275:1296@planStop"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1275:1296
--8<--
```

### 歩行者に対する停止判断(checkStopForCrosswalkUsers)

#### 概要

歩行者などに対して

```cpp title="autoware_behavior_crosswalk_module/include/autoware/behavior_velocity_crosswalk_module/util.hpp:48:48"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/include/autoware/behavior_velocity_crosswalk_module/util.hpp:48:48
--8<--
```

のうち`YIELD`と判断したものに対して衝突検知を行う．

#### 処理

**getAttentionRange**が求める値は**clampAttentionRangeByNeighborCrosswalks**と同じである．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:330:334"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:330:334
--8<--
```

**getAttentionArea**は(計算量を下げるための)`sparse_resample_path`上に自車footprintを置いていきそれらの幾何的な集合和を求める関数である．

`sparse_resample_path`上で自車位置から測って**getAttentionRange**で求めた`crosswalk_attention_range`より後ろ，前にある経路点は除外して，それらの上にfootprintを重ねた図形の集合和を`attention_area`に累積していく．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:908:932@getAttentionArea"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:908:932
--8<--
```

**updateObjectState**は各歩行者と自転車に対して衝突しうるかどうか判断に用いるデータを計算している．`ObjectInfoManager::objects`でオブジェクトの情報は保持されるので，前回サイクルから存在している物体についてはヒステリシスがある．

今回のサイクルで存在しているuuidは`ObjectInfoManager::current_uuids_`に保存され，

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1069:1069"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1069:1069
--8<--
```

そこに含まれていないuuidのオブジェクトは**ObjectInfoManager::finalize**でクリアされる．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.hpp:288:302"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.hpp:288:302
--8<--
```

歩行者信号が赤信号でなく横断歩道を渡るタイプのオブジェクトについて

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1065:1065"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1065:1065
--8<--
```

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1076:1081"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1076:1081
--8<--
```

与えられたオブジェクトの予測パス沿いのfootprintと`attention_area`が初めて交差する場所のうち，egoに最も近くかつ`crosswalk_attention_range`内に入っている点の位置を`collision_point`として求める．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1083:1083"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1083:1083
--8<--
```

**getCollisionPoint**は以下のようにして条件を満たすようなegoとobjectの衝突位置を`nearest_collision_point`として求める．

まず予測経路沿いにオブジェクトが一番始めに`attention_area`と交差する位置を求めてその予測経路点の添字を`start_idx`として求める(`is_start_idx_initialized`がそれが見つかったフラグ)．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:676:699"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:676:699
--8<--
```

次に`start_idx`から予測経路の現在のイテレートしている位置までの間沿いのfootprintポリゴンを生成しそれが`attention_area`と交差している位置を求める．その重心位置を`intersection_center_point`とする．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:702:721"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:702:721
--8<--
```

自車経路沿いでの`intersection_center_point`の位置を求めてそれが`crosswalk_attention_range`の範囲内に入っていれば`nearest_collision_point`として更新する．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:731:745"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:731:745
--8<--
```

**createCollisionPoint**では衝突に関する位置や予想衝突時間などの詳細情報を求めている．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:758:780"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:758:780
--8<--
```

- `time_to_collision`
  - `ego`が最低でも`$ego_min_assumed_speed`で進んだ場合の`nearest_collision_point`に到達するまでの時間
- `time_to_vehicle`
  - オブジェクトが`nearest_collision_point`に到達するまでの時間
- `object_passage_direction`
  - **findObjectPassageDirectionAlongVehicleLane**で求めた，オブジェクトの予測経路が横断歩道の両端（入り口と出口ではない方の左右の端のこと）を跨ぐ場合の方向．つまり横断歩道の上を道路沿いに横切る場合の方向のこと

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:627:658"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:627:658
--8<--
```

オブジェクトが`CollisionState`のうちどれに該当するか更新する．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1086:1090"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1086:1090
--8<--
```

**ObjectInfoManager::update**では信号のある横断歩道で横断歩道から0.5m以上離れているオブジェクトについては`IGNORE`で，それ以外については`YIELD`を初期状態として登録される．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.hpp:254:278"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:254:278
--8<--
```

次に初期状態から遷移させる．

- `IGNORE`でかつ速度が`$stop_object_velocity`以下
  - `IGNORE`のまま

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.hpp:187:211"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src//scene_crosswalk.hpp:187:211
--8<--
```

- 速度が`$stop_object_velocity`以下かつ横断歩道から0.5m以上離れており，一定時間以上停止し続けていた場合
  - `IGNORE`とする

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.hpp:198:208"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src//scene_crosswalk.hpp:198:208
--8<--
```

もし`collsion_point`が有効値を持つのであれば

- egoが衝突地点に到達する時刻 + `ego_pass_first_margin` < オブジェクトが衝突地点に到達する時刻
  - `EGO_PASS_FIRST`とする(egoが十分速い or オブジェクトが十分のろいので停止する必要がない)
- オブジェクトが衝突地点に到達する時刻 + `ego_pass_later_margin` < egoが衝突地点に到達する時刻
  - `EGO_PASS_LATER`とする(ego十分遅い，遠い or オブジェクトが渡り終わり，十分速いので停止する必要がない)
- それ以外なら停止する必要があるので`YIELD`とする

以上の判定をそれぞれのオブジェクトについて行った後古いオブジェクトをクリアする．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:1115:1115"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:1115:1115
--8<--
```

**updateObjectState**は行いつつ横断歩道手前で十分停止できなさそうであれば停止判断は行わない．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:342:351"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:342:351
--8<--
```

`object_info_manager_`の`YIELD`のもののうち一番近いものを求めて停止判断対象とする．ただしegoとほぼ同じ方向で(しきい値`$vehicle_object_cross_angle_threshold`以内)横断歩道を通過するオブジェクトについては無視する．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:360:389"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:360:389
--8<--
```

詳細はよく分からないが以下のようにして停止位置を算出する．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:392:414"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:392:414
--8<--
```

### 横断歩道出口付近のスタック車両に対する停止判断(checkStopForStuckVehicles)

todo

## 横断歩道の形状に関わる幾何計算を行っている関数

### getPathEndPointsOnCrosswalk

```cpp title="autoware_behavior_crosswalk_module/src/util.cpp:119:122"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/util.cpp:119:122
--8<--
```

横断歩道はほぼ長方形なので，その前提に基づいて自車経路が横断歩道に載っている区間の両端の点を求める．経路のセグメントごとにポリゴンと交差するかどうかを求めていき，2つ目の交点まで求める．当然その点は自車経路の上にあるので，egoの位置とのsigned arc lengthを求めて近い方と遠い方の点を求めて返す．

### sortCrosswalkByDistance

**getPathEndPointsOnCrosswalk**を使って自車経路との一番初めの交点までの距離をベースに横断歩道をsortする．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:112:133"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:112:133
--8<--
```

### clampAttentionRangeByNeighborCrosswalks

正直な所ソースコードレベルでは意図を幾何的に把握するのは難しいが，横断歩道の_SceneModule_が対象としているメンバ変数の`crosswalk_`とは別に自車経路と重なっている横断歩道がある場合に

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:506:525"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:506:525
--8<--
```

それらを自車経路との交点までの距離を基準にソートして`crosswalk_`の前後にある2つの横断歩道を求めている．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:527:543"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:527:543
--8<--
```

それぞれを`prev_crosswalk`と`next_crosswalk`としており，(スクランブル交差点などを考えて？)

- `prev_crosswalk`と自車経路の2つ目の交点(`path_end_points_on_prev_crosswalk`)
- `crosswalk_`と自車経路の初めのの交点

のうちegoから遠い方までの距離を`clamped_near_attention_range`，

- `next_crosswalk`と自車経路の1つ目の交点
- `crosswalk_`と自車経路の初めのの交点

のうちegoから近い方までの距離を`clamped_far_attention_range`としている．

```cpp title="autoware_behavior_crosswalk_module/src/scene_crosswalk.cpp:545:576"
--8<--
planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/src/scene_crosswalk.cpp:545:576
--8<--
```
