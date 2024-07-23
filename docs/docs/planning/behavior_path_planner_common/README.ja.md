# behavior_path_planner_common

**SceneModuleManagerInterface**はidle状態の**SceneModuleInterface**を生成したり，_candidate_/_approved_ に昇格してRUNNING状態のモジュールインスタンスをweak_ptrとして間接的に管理する役割を持つ．

- **PlannerManager**が`manager_ptrs_`として持っているのが，**SceneModuleManagerInterface**のshared_ptr
- **PlannerManager**が`approved_module_ptrs_`などとして持っているのが，**SceneModuleInterface**のshared_ptr

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

**SceneModuleManagerInterface::updateIdleModuleInstance()**では`idle_module_ptr_`のインスタンス化（初回，または前サイクルでモジュールが _isExecutionReady_ に昇格した場合）または既存の`idle_module_ptr_`の更新を行う．

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

_observer_ は生成した`idle_module_ptr_`で外部に保有されている個数を管理しており，**PlannerManager::runRequestModules()**（**PlannerManager::getRequestModules()**で求めたものから次の _candidate_ を求める関数）でexecutableになった`idle_module_ptr_`が**SceneModuleManagerInterface::registerNewModule()**で _observer_ として登録される．

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

またそのうち _candidate_/_approved_ としての計算ですでに _FAILURE_ / _SUCCESS_ であるモジュールは _expired module_ として _observer_ からも削除する．

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

### モジュールの状態遷移とRTCによる承認・isWaitingApprovalについて

`IDLE`が初期ノード

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

```cpp title="autoware_behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:288:312"
--8<--
planning/behavior_path_planner/autoware_behavior_path_planner/include/autoware/behavior_path_planner/planner_manager.hpp:288:312
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

#### RUNNINGから

todo

#### WAITING_APPROVALから

todo

#### SUCCESSから

`SUCCESS`は終端ノード

#### FAILUREから

`FAILURE`は終端ノード
