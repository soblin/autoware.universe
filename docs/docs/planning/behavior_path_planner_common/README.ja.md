# behavior_path_planner_common

**SceneModuleManagerInterface**はidle状態の**SceneModuleInterface**を生成したり，_candidate_/_approved_ に昇格してRUNNING状態のモジュールインスタンスをweak_ptrとして間接的に管理する役割を持つ．

- **PlannerManager**が`manager_ptrs_`として持っているのが，**SceneModuleManagerInterface**のshared_ptr
- **PlannerManager**が`approved_module_ptrs_`などとして持っているのが，**SceneModuleInterface**のshared_ptr

## モジュールのライフサイクル

純粋仮想関数の場合は**GoalPlanner**を例に用いる．

### IDLE状態

まず**BehaviorPathPlannerNode**で`$launch_modules`パラメーターで宣言されたmanagerがloadされる．

```xml title="tier4_planning_launch/launch/~/behavior_planning.launch.xml:42:46"
--8<--
launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml:42:46
--8<--
```

```xml title="tier4_planning_launch/launch/~/behavior_planning.launch.xml:94:98"
--8<--
launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml:94:98
--8<--
```

```xml title="tier4_planning_launch/launch/~/behavior_planning.launch.xml:205:205""
--8<--
launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml:205:205
--8<--
```

```cpp title="behavior_path_planner/src/behavior_path_planner_node.cpp:138:144@BehaviorPathPlannerNode()"
--8<--
planning/behavior_path_planner/src/behavior_path_planner_node.cpp:138:144
--8<--
```

その際にモジュールの**SceneModuleManagerInterface::init()**が呼ばれる．

```cpp title="behavior_path_planner/src/planner_manager.cpp:44:65"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:44:65
--8<--
```

```cpp title="behavior_path_planner_common/scene_module_manager_interface.hpp:58:58"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:58:58
--8<--
```

**SceneModuleManagerInterface::initInterface()**も各実装において先頭で呼ばれている．その他にはパラメーターの初期化が行われている．

```cpp title="behavior_path_goal_planner_module/src/manager.cpp:28:38"
--8<--
planning/behavior_path_goal_planner_module/src/manager.cpp:28:38
--8<--
```

**SceneModuleManagerInterface::initInterface()**はRTC・モジュールの優先度・共通のデバッグマーカーの設定が行われている．

```cpp title="behavior_path_planner_common/scene_module_manager_interface.hpp:264:305"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:264:305
--8<--
```

`rtc_type`はほとんどのモジュールでは空だが，lane_changeのみ`{rtc_type: "left", snake_case_name: "lane_change_left"}`という具合で利用されている．

**_BehaviorPathPlanner_**では各モジュールは毎フレームで自分自身が立ち上がる必要があるかどうかを判断している．

```cpp title="behavior_path_planner/src/planner_manager.cpp:324:339@getRequestModule"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:324:339
--8<--
```

各モジュールはhot-startできるよう， _isExecutionRequested_ でなくても毎回**PlannerManager::getRequestModule()**で`idle_module_ptr_`を**SceneModuleManagerInterface::updateIdleModuleInstance()**で更新している．そして _isExecutionRequested_ になったら**SceneModuleManagerInterface::getIdleModule()**で`idle_module_ptr_`の所有権を移している．

```cpp title="behavior_path_planner/src/planner_manager.cpp:329:336@getRequestModule"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:329:336
--8<--
```

/// tip | 注意点
**SceneModuleManagerInterface::getIdleModule()**は自身のメンバーの`idle_module_ptr_`をmoveしているので，名前に反して非constなメンバ関数である
///

```cpp title="behavior_path_planner_common/include/scene_module_manager_interface.hpp:259:259"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:259:259
--8<--
```

**SceneModuleManagerInterface::updateIdleModuleInstance()**では`idle_module_ptr_`のインスタンス化（初回，または前サイクルでモジュールが _isExecutionReady_ に昇格した場合）または既存の`idle_module_ptr_`の更新を行う．

```cpp title="behavior_path_planner_common/include/scene_module_manager_interface.hpp:60:68"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:60:68
--8<--
```

```cpp title="behavior_path_planner_common/include/scene_module_interface.hpp:171:177"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_interface.hpp:171:177
--8<--
```

**SceneModuleInterface::processOnEntry()**はいくつかのモジュールではoverrideされている．

```cpp title="behavior_path_start_planner_module/src/start_planner_module.cpp:161:163"
--8<--
planning/behavior_path_start_planner_module/src/start_planner_module.cpp:161:163
--8<--
```

**SceneModuleInterface::createNewSceneModuleInstance()**は純粋仮想関数で，各サブモジュールが共変値として自身をインスタンス化して返す．

```cpp title="behavior_path_goal_planner_module/include/manager.hpp:37:42"
--8<--
planning/behavior_path_goal_planner_module/include/behavior_path_goal_planner_module/manager.hpp:37:42
--8<--
```

RUNNINGできる**SceneModule**の個数には各モジュールで制限があり，それらを**SceneModuleManagerInterface::observers\_**として管理している．

```cpp title="behavior_path_planner_common/interface/scene_module_manager_interface.hpp:198:199"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:198:199
--8<--
```

### IDLEからの昇格以降のモジュール数とobserver

_observer_ は生成した`idle_module_ptr_`で外部に保有されている個数を管理しており，**PlannerManager::runRequestModules()**（**PlannerManager::getRequestModules()**で求めたものから次の _candidate_ を求める関数）でexecutableになった`idle_module_ptr_`が**SceneModuleManagerInterface::registerNewModule()**で _observer_ として登録される．

```cpp title="behavior_path_planner/src/planner_manager.cpp:559:568@runRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:559:568
--8<--
```

```cpp title="behavior_path_planner_common/interface/scene_module_manager_interface.hpp:79:91"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:79:91
--8<--
```

またそのうち _candidate_/_approved_ としての計算ですでに _FAILURE_ / _SUCCESS_ であるモジュールは _expired module_ として _observer_ からも削除する．

```cpp title="planning/behavior_path_planner/src/planner_manager.cpp:574:593@runRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:574:593
--8<--
```

```cpp title="planning/behavior_path_planner/src/planner_manager.cpp:751:768@runApprovedModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:751:768
--8<--
```

### モジュールの状態遷移とRTCによる承認・isWaitingApprovalについて

#### IDLEからRUNNING

**getRequestModules**で生成された`module_ptr`はIDLE状態であり，**runRequestModules**でprivateの方の**PlannerManager::run**内で**updateCurrentStatus**を呼ばれることで`RUNNING`状態に映る．

```cpp title="behavior_path_planner_common/scene_module_interface.hpp:444:444"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_interface.hpp:444:444
--8<--
```

```cpp title="planning/behavior_path_planner/src/planner_manager.cpp:567:567@runRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:567:567
--8<--
```

```cpp title="behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:276:291"
--8<--
planning/behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:276:291
--8<--
```

**SceneModuleInterface::updateCurrentStatus**，**SceneModuleInterface::updateState**
，**SceneModuleInterface::setInitState**を追うと分かるように，`IDLE`状態でこの関数が呼ばれると必ず`RUNNING`に遷移するようになっている．
todo: start_plannerはisWaitingApprovalになっていそう

```cpp title="behavior_path_planner_common/scene_module_interface.hpp:156:164"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_interface.hpp:156:164
--8<--
```

```cpp title="behavior_path_planner_common/scene_module_interface.hpp:376:386"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_interface.hpp:376:385
--8<--
```

```cpp title="behavior_path_planner_common/scene_module_interface.hpp:468:468"
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_interface.hpp:468:468
--8<--
```

todo: 基本的にupdateRTCStatus()はmanagerからは呼ばれないのでRTCを使うモジュールは自発的に呼ぶ必要がある．
todo: rtc_interface_map_ptrが空だとデフォルトでそのモジュールは`isActivated() = true`になるっぽい
