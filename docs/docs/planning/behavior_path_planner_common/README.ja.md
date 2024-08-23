# behavior_path_planner_common

**SceneModuleManagerInterface**はidle状態の**SceneModuleInterface**を生成したり，candidate/approvedに昇格してRUNNING状態のモジュールインスタンスをweak_ptrとして間接的に管理する役割を持つ．

- **PlannerManager**が`manager_ptrs_`として持っているのが，**SceneModuleManagerInterface**のshared_ptr
- **PlannerManager**が`approved_module_ptrs_`などとして持っているのが，**SceneModuleInterface**のshared_ptr

## 各モジュールへのCooperateCommandステータスの反映，Commandに応じたplanningとCooperateStatusの設定，状態遷移が行われる箇所

privateな方の**PlannerManager::run**で全て行われるのでこの関数が最も重要である．

```cpp title="autoware_behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:288:312"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/planner_manager.hpp:288:312
--8<--
```

## モジュールのRTCステータスが送られるサイクル

RTCを利用する以上

- 各モジュールへのRTC-CooperateCommandがサービスでセットされる
- 各モジュールがRTC-CooperateCommandに応じて動作（もしIDLEでなければ）
- 各モジュールがRTC-CooperateStatusをセット
- 各モジュールのRTC-CooperateStatusをpublishする

の各ステップが行われている．

### 各モジュールへのRTC-CooperateCommandがサービスでセットされる

各SceneModuleIntefaceは`rtc_interface_ptr_map_`として`RTCInterface`を所持しており(モジュールによっては`lane_change_left`と`lane_change_right`のようにパラメーターは共有しつつ左右で異なるサブモジュールがあったりするため`map`になっている)，`RTCInterface`はタイマーにより非同期に**RTCInterface::onCooperateCommandService**のコールバックでユーザーからの承認状態を`stored_commands_`にセットされる．

```cpp title="autoware_rtc_interface/src/rtc_interface.cpp:154:167"
--8<--
planning/autoware_rtc_interface/src/rtc_interface.cpp:154:167
--8<--
```

privateな方の**PlannerManager::run**で

```cpp title="autoware_behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:298:298"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/planner_manager.hpp:298:298
--8<--
```

すると

```cpp title="behavior_path_planner_common/scene_module_interface_interface.cpp:223:230"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:223:230
--8<--
```

により**RTCInterface::lockCommandUpdate**が呼ばれて

```cpp title="autoware_rtc_interface/src/rtc_interface.cpp:413:416"
--8<--
planning/autoware_rtc_interface/src/rtc_interface.cpp:413:416
--8<--
```

となり**RTCInterface::onCooperateCommandService**のコールバックでは`stored_commands_`を蓄積するだけになる．

```cpp title="autoware_rtc_interface/src/rtc_interface.cpp:154:165"
--8<--
planning/autoware_rtc_interface/src/rtc_interface.cpp:154:165
--8<--
```

`module_ptr->lockRTCCommand();`してから`module_ptr_->run();`しているのは**RTCInterface::isActivated()**などを計算する際に`registered_status_`への競合を防ぐためである(とはいえ非同期に`stored_commands_`を更新している間に`lockCommandUpdate()`を呼んでlockしたつもりにできてしまうので結局競合は起きているはず)．`module_ptr_->run();`してから`module_ptr->unlockRTCCommand();`すると

```cpp title="behavior_path_planner_common/scene_module_interface_interface.cpp:232:239"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:232:239
--8<--
```

`is_locked_`していた間に蓄積された`stored_commands_`が反映される．

```cpp title="autoware_rtc_interface/src/rtc_interface.cpp:418:422"
--8<--
planning/autoware_rtc_interface/src/rtc_interface.cpp:418:422
--8<--
```

**RTCInterface::updateCooperateCommandStatus**で`registered_status_`を更新する．

```cpp title="autoware_rtc_interface/src/rtc_interface.cpp:206:221"
--8<--
planning/autoware_rtc_interface/src/rtc_interface.cpp:206:221
--8<--
```

### 各モジュールがRTC-CooperateCommandに応じて動作

**RTCInterface::isActivated**の計算に`registered_status_`の値が利用される．

**SceneModuleInterface::run**ではsafetyの値に応じて**planWaitingApproval**または**plan**が呼ばれる．

```cpp title="behavior_path_planner_common/scene_module_interface.hpp:146:156"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:146:156
--8<--
```

/// tip | 注意点
実際はモジュールがAUTOモードで動いていて自分で計算したsafetyの値に応じて危険判断をしている場合も`WAITING_APPROVAL`として表現されて手動で動いているようなニュアンスを相手に与えるので少し注意が必要
///

### 各モジュールがRTC-CooperateStatusをセット

以下のモジュール

- avoidance_by_lane_change
- goal_planner(**GoalPlannerModule::postProcess**において)
- lane_change(**LaneChangeInterface::plan**など色々な箇所で)
- start_planer

では**SceneModuleInterface::updateRTCStatus**が使われている．

```cpp title="behavior_path_planner_common/scene_module_interface.hpp:507:517"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:507:517
--8<--
```

ただし以下のモジュール

- avoidance_by_lane_change
- static_avoidance

は`rtc_interface_ptr_map_`で**updateCooperateStatus**を用いて直接値を操作している．

### 各モジュールのRTC-CooperateStatusをpublishする

publicな方の**PlannerManager::run**の一番最後

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:205:207"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:205:207
--8<--
```

で**SceneModuleInterface::publishRTCStatus**を呼んでいる．

```cpp title="behavior_path_planner_common/scene_module_manager_interface.hpp:103:110"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:103:110
--8<--
```

```cpp title="autoware_rtc_interface/src/rtc_interface.cpp:147:152"
--8<--
planning/autoware_rtc_interface/src/rtc_interface.cpp:147:152
--8<--
```

## モジュールのライフサイクル

純粋仮想関数の場合は**GoalPlanner**を例に用いる．

### IDLE状態

まず**BehaviorPathPlannerNode**で`$launch_modules`パラメーターで宣言されたmanagerがloadされる．

```xml title="tier4_planning_launch/launch/~/behavior_planning.launch.xml:40:45"
--8<--
launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml:40:45
--8<--
```

```xml title="tier4_planning_launch/launch/~/behavior_planning.launch.xml:91:96"
--8<--
launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml:91:96
--8<--
```

```xml title="tier4_planning_launch/launch/~/behavior_planning.launch.xml:193:193""
--8<--
launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml:193:193
--8<--
```

```cpp title="autoware_behavior_path_planner/src/behavior_path_planner_node.cpp:76:82@BehaviorPathPlannerNode()"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp:76:82
--8<--
```

その際にモジュールの**SceneModuleManagerInterface::init()**が呼ばれる．

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:45:64"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:45:64
--8<--
```

```cpp title="behavior_path_planner_common/scene_module_manager_interface.hpp:58:58"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:58:58
--8<--
```

**SceneModuleManagerInterface::initInterface()**も各実装において先頭で呼ばれている．その他にはパラメーターの初期化が行われている．

```cpp title="autoware_behavior_path_goal_planner_module/src/manager.cpp:28:38"
--8<--
planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/manager.cpp:28:38
--8<--
```

**SceneModuleManagerInterface::initInterface()**はRTC・モジュールの優先度・共通のデバッグマーカーの設定が行われている．

```cpp title="autoware_behavior_path_planner_common/scene_module_manager_interface.hpp:282:330"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:282:330
--8<--
```

`rtc_type`はほとんどのモジュールでは空だが，lane_changeのみ`{rtc_type: "left", snake_case_name: "lane_change_left"}`という具合で利用されている．

ここで`SceneModuleManagerInterface`の`rtc_interface_ptr_map_`が初期化され，`SceneModuleInterface`が構築される際はこれがコピーされて渡される．

```cpp title="autoware_behavior_path_planner_common/include/scene_module_interface.hpp:89:107"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:89:107
--8<--
```

**_BehaviorPathPlanner_**では各モジュールは毎フレームで自分自身が立ち上がる必要があるかどうかを判断している．

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:350:364@getRequestModule"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:350:364
--8<--
```

各モジュールはhot-startできるよう， _isExecutionRequested_ でなくても毎回**PlannerManager::getRequestModule()**で`idle_module_ptr_`を**SceneModuleManagerInterface::updateIdleModuleInstance()**で更新している．そして _isExecutionRequested_ になったら**SceneModuleManagerInterface::getIdleModule()**で`idle_module_ptr_`の所有権を移している．

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:355:361@getRequestModule"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:355:361
--8<--
```

/// tip | 注意点
**SceneModuleManagerInterface::getIdleModule()**は自身のメンバーの`idle_module_ptr_`をmoveしているので，名前に反して非constなメンバ関数である
///

```cpp title="autoware_behavior_path_planner_common/include/scene_module_manager_interface.hpp:277:277"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:277:277
--8<--
```

**SceneModuleManagerInterface::updateIdleModuleInstance()**では`idle_module_ptr_`のインスタンス化（初回，または前サイクルでモジュールが _isExecutionRequested_ に昇格した場合）または既存の`idle_module_ptr_`の更新を行う．

```cpp title="autoware_behavior_path_planner_common/include/scene_module_manager_interface.hpp:60:68"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:60:68
--8<--
```

```cpp title="autoware_behavior_path_planner_common/include/scene_module_interface.hpp:176:183"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:176:183
--8<--
```

**SceneModuleInterface::processOnEntry()**はいくつかのモジュールではoverrideされている．

```cpp title="autoware_behavior_path_start_planner_module/src/start_planner_module.cpp:168:171"
--8<--
planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/start_planner_module.cpp:168:171
--8<--
```

/// info | 重要
全てのSceneModuleは**PlannerManager::updateIdleModuleInstance**を経由して，**SceneModuleInterface::onEntry**を経由して**SceneModuleInterface::processOnEntry**を呼ばれる
///

**SceneModuleInterface::createNewSceneModuleInstance()**は純粋仮想関数で，各サブモジュールが共変値として自身をインスタンス化して返す．

```cpp title="behavior_path_goal_planner_module/include/manager.hpp:37:42"
--8<--
planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/include/autoware/behavior_path_goal_planner_module/manager.hpp:37:42
--8<--
```

RUNNINGできる**SceneModule**の個数には各モジュールで制限があり，それらを**SceneModuleManagerInterface::observers\_**として管理している．

```cpp title="autoware_behavior_path_planner_common/interface/scene_module_manager_interface.hpp:218:218"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:218:218
--8<--
```

### IDLEからの昇格以降のモジュール数とobserver

_observer_ は生成した`idle_module_ptr_`で外部に保有されている個数を管理しており，**PlannerManager::runRequestModules()**（**PlannerManager::getRequestModules()**で求めたものから次のcandidateを求める関数）でexecutableになった`idle_module_ptr_`が**SceneModuleManagerInterface::registerNewModule()**でobserverとして登録される．

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:598:607@runRequestModules"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:598:607
--8<--
```

```cpp title="autoware_behavior_path_planner_common/interface/scene_module_manager_interface.hpp:79:91"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:79:91
--8<--
```

またそのうちcandidate/approvedとしての計算ですでにFAILURE/SUCCESSであるモジュールはexpired moduleとしてobserverからも削除する．

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:613:632@runRequestModules"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:613:632
--8<--
```

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:791:810@runApprovedModules"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:791:810
--8<--
```

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:855:876@runApprovedModules"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:855:876
--8<--
```

### モジュールの状態遷移とRTCによる承認

`IDLE`が初期ノードであり，一度でもPlannerManagerにおいてcandidteかapprovedに昇格したら`IDLE`ではなくなる．

各SceneModuleInterfaceは**SceneModuleInterface::updateRTCStatus**のsafetyの値としては**isExecutionReady**を送っている．

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:597:517"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:507:517
--8<--
```

AUTOモードの場合はモジュール自身が上で送ったsafetyの値が，MANUALモードの場合はユーザーが送ったsafetyの値が利用されることで，`RUNNING`の時にRTC非承認状態だと**canTransitWaitingApprovalState()**により`WAITING_APPROVAL`に，`WAITING_APPROVAL`の時に**canTransitWaitingApprovalToRunningState()**により`RUNNING`に遷移する．

```cpp title="behavior_path_planner_common/scene_module_interface.hpp:149:149"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:149:149
--8<--
```

#### SUCCESSから

`SUCCESS`は終端ノード

#### FAILUREから

`FAILURE`は終端ノード

#### IDLEからRUNNING

**getRequestModules**で生成された`module_ptr`はIDLE状態であり，**runRequestModules**でprivateの方の**PlannerManager::run**内で**updateCurrentStatus**を呼ばれることで`RUNNING`状態に移る．

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:449:449"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:449:449
--8<--
```

```cpp title="autoware_behavior_path_planner/src/planner_manager.cpp:606:606@runRequestModules"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp:606:606
--8<--
```

**SceneModuleInterface::updateCurrentStatus**，**SceneModuleInterface::updateState**
，**SceneModuleInterface::setInitState**を追うと分かるように，`IDLE`状態でこの関数が呼ばれると必ず`RUNNING`に遷移するようになっている．

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:161:170"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:161:170
--8<--
```

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:381:390"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:381:390
--8<--
```

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:473:473"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:473:473
--8<--
```

なので**SceneModuleInterface::isExecutionRequested** = trueを返したモジュールは必ずcandidateかapprovedとして一度は`RUNNING`を経由する．

#### RUNNING/WAITING_APPROVALから

どちらとも

- **canTransitSuccessState**なら`SUCCESS`
- **canTransitFailureState**なら`FAILURE`

を優先するのは共通している．そこから先は`RUNNING`からは

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:393:411"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:393:411
--8<--
```

- **canTransitWaitingApprovalState**なら`WAITING_APPROVAL`
- いずれでもなければ`RUNNING`のまま

`WAITING_APPROVAL`からは

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:413:430"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:413:430
--8<--
```

- **canTransitWaitingApprovalToRunningState**なら`RUNNING`
- いずれでもなければ`WAITING_APPROVAL`のまま

である．

### canTransit\*\*\*系の実装

**canTransitSuccessState**と**canTransitFailrureState**は純粋仮想関数になっている．これは各モジュールで終了条件が異なるためである．

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:461:468"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:461:468
--8<--
```

`WaitingApproval`からは仮想関数ではなく，**RTCの状態のみから遷移が決定する**．

```cpp title="autoware_behavior_path_planner_common/scene_module_interface.hpp:327:374"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/interface/scene_module_interface.hpp:327:374
--8<--
```

## 各モジュールの状態遷移の例

todo: isExecutionReadyの処理内容自体は意外とどれも短く書けているのでもう少し詳細な内容を解読する

### start_planner

**StartPlannerModule::canTransitFailureState**は常にfalseを返すようになっているため， _StartPlanner_ は必ず`SUCCESS`を返さないと終了しない．

freespaceが**StartPlannerModule::hasReachedFreespaceEnd**したり，バック走行が終わったりする必要のない状態でかつ幅寄せ経路が見つかり，**StartPlannerModule::hasReachedPulloutEnd**であれば**StartPlannerModule::canTransitSuccessState**trueを返す．

```cpp title="autoware_behavior_path_start_planner/src/start_planner_module.cpp:586:617"
--8<--
planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/start_planner_module.cpp:586:617
--8<--
```

#### isExecutionReadyの値の決まり方

_StartPlanner_ モジュール自身によるシーンの安全判断結果は以下のように計算している．

```cpp title="autoware_behavior_path_start_planner/src/start_planner_module.cpp:547:563"
--8<--
planning/behavior_path_planner/autoware_behavior_path_start_planner_module/src/start_planner_module.cpp:547:563
--8<--
```

### goal_planner

**GoalPlannerModule::canTransitSuccessState**/**GoalPlannerModule::canTransitFailureState**は常にfalseを返すようになっているため _GoalPlanner_ は終了しない．

```cpp title="autoware_behavior_path_goal_planner/include/goal_planner_module.hpp:434:458"
--8<--
planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/include/autoware/behavior_path_goal_planner_module/goal_planner_module.hpp:434:458
--8<--
```

そのため一旦は`RUNNING`となり，経路が確定するまでは`RUNNING`として動作するが**not hasDecidedPath()**であるため，**GoalPlannerModule::postProcess()**で**updateRTCStatus**を呼んでRTCの情報を送ったりはしないため承認の候補とはならない．

```cpp title="autoware_behavior_path_goal_planner/src/goal_planner_module.cpp:810:814"
--8<--
planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp:810:814
--8<--
```

```cpp title="autoware_behavior_path_goal_planner/src/goal_planner_module.cpp:1358:1364"
--8<--
planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp:1358:1363
--8<--
```

```cpp title="autoware_behavior_path_goal_planner/src/goal_planner_module.cpp:1476:1492"
--8<--
planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp:1476:1492
--8<--
```

ようになっており経路が確定してから安全確認ができるようになるとRTCの情報を送るようになるため，危険だったりすると`WAITING_APPROVAL`に遷移したりするようになる．

#### isExecutionReadyの値の決まり方

_GoalPlanner_ モジュール自身によるシーンの安全判断結果は以下のように計算している．

```cpp title="autoware_behavior_path_goal_planner/src/goal_planner_module.cpp:618:630"
--8<--
planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp:618:630
--8<--
```
