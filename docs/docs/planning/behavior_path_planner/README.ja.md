# behavior_path_planner(PlannerManager)

## 概要

登録されたそれぞれのモジュールの元は`manager_ptrs_`であり，その中から

1. `request_modules`
2. `candidate_module_ptrs_`
3. `approved_module_ptrs_`

へと昇格していく．`candidate_module_ptrs_`の中で一番優先度が高く`isWaitingApproval`でないものがcandidate moduleをすっ飛ばして`approved_module_ptrs_`に入る．それ以外のものは一旦`candidate_module_ptrs_`に残り，次のサイクルで`approved_module_ptrs_`に追加できるかどうか判断される．

## 詳細

## manager_ptrs\_の初期化

BehaviorPathPlannerが初期化の際にプラグインのリストを読み込むので，そこで`manager_ptrs_`が初期化されて以降は`manager_ptrs_`は不変

```cpp title="behavior_path_planner/src/behavior_path_planner_node.cpp:138:143@BehaviorPathPlannerNode"
--8<--
planning/behavior_path_planner/src/behavior_path_planner_node.cpp:138:143
--8<--
```

```cpp title="behavior_path_planner/src/planner_manager.cpp:44:59"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:44:59
--8<--
```

## 各フラグの実際の値

defaultの値はそれぞれ以下のようになっている．

```cpp
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_manager_interface.hpp:209:227
--8<--
```

| Property                                  | start            | avoidance        | lane_change      | dynamic_avoidance | goal                                | external_lane_change | side_shift       |
| ----------------------------------------- | ---------------- | ---------------- | ---------------- | ----------------- | ----------------------------------- | -------------------- | ---------------- |
| isAlwaysExecutable                        | `false`(default) | `false`(default) | `false`(default) | `true`            | fixedなら`true`                     | `false`(default)     | `false`(default) |
| isSimultaneousExecutableAsApprovedModule  | 状況により変化   | `true`           | `true`           | `true`            | fixedならtrue,そうでなければ`true`  | `false`              | `false`          |
| isSimultaneousExecutableAsCandidateModule | 状況により変化   | `false`          | `true`           | `true`            | fixedならtrue,そうでなければ`false` | `true`               | `false`          |
| isKeepLast(virtualではない)               | `false`          | `false`          | `false`          | `true`            | `true`                              | `false`              | `false`          |

*start_planner*の設定は以下のようになっている．

```cpp title="planning/behavior_path_start_planner_module/src/manager.cpp:736:763"
--8<--
planning/behavior_path_start_planner_module/src/manager.cpp:736:763
--8<--
```

```cpp title="planning/behavior_path_start_planner_module/src/manager.cpp:765:791"
--8<--
planning/behavior_path_start_planner_module/src/manager.cpp:765:791
--8<--
```

## getRequestModules()

基本的に今`candidate_module_ptrs_`にいないモジュールのうち，現在の`approved_module_ptrs_`と同時実行可能なモジュールを求めている．またほとんどのモジュールは`$max_size == 1`になっているので，結局`approved_module_ptrs_`と`candidate_module_ptrs_`にいないモジュールからrequest modulesが求められる．

/// tip | 注意
ただし（おそらく実装都合で）`candidate_module_ptrs_`の中に同じ名前のモジュールがあれば`request_modules`の中に`shared_ptr`のコピーが追加され，この関数を呼んだ直後では同じインスタンスを共有した状態になる．**runRequestModules()**の中の**updateCandidateModules()**でこの共有状態は解消される．
///

moduleをrunさせることはないのでmoduleの状態遷移などには全く影響しない．

### 副作用

なし

### 処理

conditionsの両方に`not getManager(m)->isAlwaysExecutable() &&`が記述されているので，それよりsuppが大きい**hasNonAlwaysExecutableApproveModules()**（「どれかのapproved modulesが**not isAlwaysExecutable**である」）は不要ではある．

```cpp title="behavior_path_planner/src/planner_manager.cpp:271:271@getRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:271:271
--8<--
```

`manager_ptrs_`の中の各`manager_ptr`について

- もし`manager_ptr`が**isAlwaysExecutable**でない場合
  - 自身は**isSimultaneousExecutableAsApprovedModule()**でなく，かつ`approved_modules_ptrs_`の中に「**isAlwaysExecutable()**でないが**isSimultaneousExecutable()**である」要素が一つでも存在するのであればこの`manager_ptr`は`request_modules`にはせずスキップ，continue
  - あるいは`approved_modules_ptrs_`の中に「**isAlwaysExecutable()**でないかつ**isSimultaneousExecutable()**でもない」要素が一つでも存在するのであればこの`manager_ptr`は`request_modules`にはせずスキップ，continue
- もし`candidate_module_ptrs_`に同名のモジュールがない場合，`manager_ptr->canLaunchNewModule()`かつ`manager_ptr->isExecutionRequested()`であれば`request_modules`に`manager_ptr->getIdleModule()`を追加してcontinue
  - もし同名のモジュールのインスタンスが`approved_module_ptrs_`にある場合`module_size >= 1`であるので，`$max_module_size == 1`の設定の下では`manager_ptr->canLaunchNewModule()`で弾かれるので`request_modules`には追加されない（ここで暗黙的に`approved_modules`となっているモジュールを`request_modules`に入れないように処理されている．かなり気付きにくい）
- もし`candidate_module_ptrs_`に同名のモジュール(itr)がおりそのモジュールが**isLockedNewModuleLaunch()**である場合，`request_modules`にはそのitrだけを入れ直してbreakする(`request_modules`が1個だけになる)(この`manager_ptr`は無視)
- もし`candidate_module_ptrs_`に同名のモジュール(itr)がおりそのモジュールが**isLockedNewModuleLaunch()**でないなら，`request_modules`にはそのitrを追加してcontinue

つまり

- `approved_module_ptrs_`が空なら任意のモジュールが`request_modules`にjoinできる
- **isAlwaysExecutable()**なモジュールは`request_modules`に通常のチェックのみで自由に出入りできるし，他のモジュールのjoinも邪魔しない
- **isAlwaysExecutable()**なモジュールを除外した場合，`request_modules`の中には
  - **not isSimultaneousExecutable()**なモジュールが1個
  - **isSimultaneousExecutable()**なモジュールが複数個
  - のどちらかしかあり得ない

前回からのサイクルの残りで`candidate_module_ptrs_`に存在しているが現在のコンテキストでは`request_modules`にならないモジュールが存在する可能性があるので，登録されている全てのモジュールについて立ち上げの可否を判断していると思われる．

## runKeepLastModules()

approvedなkeep last moduleをbootstrap実行する．逆に**runApprovedModules**ではkeep last moduleは実行されないようになっている．

### 副作用

なし

### 処理

`approved_module_ptr_`のうち**isKeepLast()**であるものを実行する．

```cpp title="behavior_path_planner/src/planner_manager.cpp:405:412"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:405:412
--8<--
```

## runRequestModules()

/// tip | 注意
`candidate_module_ptrs_`と`request_modules`は`shared_ptr`のモジュールを共有しているので，`request_module`に対してrun()を呼んでいるように見えるがきちんと`candidate_module_ptrs_`に対してrun()を呼べている．
///

### 副作用

- `candidate_module_ptrs_`の要素数が変化する

### 前提

`request_modules`は

- **not isSimultaneousExecutableAsApprovedModule()**が1個だけ OR
- **isSimultaneousExecutableAsApprovedModule()**が複数個 のどちらか

に加えて

- **isAlwaysExecutable()**が複数個

### 処理

まず`request_modules`を**sortByPriority()**する．これは`priority`の値だけに基づく（昇順，priorityが小さいほど重要で配列の先頭に来る）．

`sorted_request_modules`の中の各`request_module`について

- もし**isAlwaysExecutable()**ならば`executable_modules`に追加してcontinue
- もし`executable_modules`が空，あるいは全て**isAlwaysExecutable()**ならば`executable_modules`に追加してcontinue
- もし自身は**isSimultaneousExecutableAsCandidateModule()**かつ，`executable_modules`の中に「**not isAlwaysExecutable()**かつ**isSimultaneousExecutableAsCandidateModule()**」のものがあれば`executale_modules`に追加してcontinue

つまり入力の`request_modules`のうち

- **isAlwaysExecutable()**は全て

と，それらを除いた内で

- 一番初めの要素が**not isSimultaneousExecutableAsCandidateModule()**であった場合その要素のみ OR
- 全ての**isSimultaneousExecutableAsCandidateModule()** のどちらか

の組み合わせが`executable_modules`になる．_start_planner_ のように**not isSimultaneousExecutableAsCandidateModule()**であるモジュールは`sorted_request_modules`の先頭に入ることが重要であるため，`$priority = 0`のように優先度を高めて**sortByPriority()**させることでそれを達成している．

次に`executable_modules`の出力を得る（ここではbootstrapはせず独立に出力を格納する）．**executbale_modulesと**`candidate_module_ptrs_`**は重複しているので，ここで**`candidate_module`**を間接的に実行していることになる**．

```cpp title="behavior_path_planner/src/planner_manager.cpp:559:567@runRequetsModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:559:567
--8<--
```

```cpp title="behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:276:301"
--8<--
planning/behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:276:301
--8<--
```

run()すると各モジュールの結果が分かるので，`getCurrentStauts() == ModuleStatus::FAILURE`または`getCurrentStauts() == ModuleStatus::SUCCESS`のモジュールは**deleteExpiredModules()**したうえで`executable_modules`からも削除する．

もし`executable_modules`が空であれば**clearCandidateModule()**をしてreturn．この時点で`executable_modules`は`IDLE`か`RUNNING`か`WAITING_APPROVAL`のどれかである（todo: `IDLE`状態で**updateCurrentStatus()**を呼ぶと必ず`RUNNING`になるので，`RUNNING`のはず）．

```cpp
--8<--
planning/behavior_path_planner_common/include/behavior_path_planner_common/interface/scene_module_interface.hpp:74:80
--8<--
```

`waiting_approved_modules`と`already_approved_modules`に分類する．

```cpp title="behavior_path_planner/src/planner_manager.cpp:607:613@runRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:607:613
--8<--
```

`already_approved_modules`の中での**selectHighestPriorityModule()**，その次に`waiting_approved_modules`の中での**selectHighestPriorityModule()**の順で返す．

**updateCandidateModules()**してから上で**selectHighestPriorityModule()**の結果とそれに対応する出力をペアで返す．

## updateCandidateModules

全てのモジュールで`$max_size == 1`であるため，事実上`candidate_module_ptrs_`と`request_modules`のモジュール名に重複はない（実装都合で両者で重複はあるが，それはshared_ptrを共有しているだけ）．

/// tip | 注意点
`highest_priority_module`が`isWaitingApproval`でない場合，そのモジュールはcandidateを経ずにapprovedに入る．
///

### 副作用

- `candidate_module_ptrs_`のいくつかの要素が消えたり増えたりする
- `candidate_module_ptrs_`はその後**sortByPriority()**されている

### 処理

- 削除
  - `candidate_module_ptrs_`のうち`request_modules`にないものを削除(前回のサイクルから立ち上がる必要がなくなったもの)
  - `candidate_module_ptrs_`のうち`highest_priority_module`と同一であり，かつそれがまだ（or もはや）**isWaitingApproval()**ではないものは削除（このモジュールは即座にapproved_moduleにするからcandidate_moduleから消さないといけない）
- 追加
  - `request_modules`のうち`candidate_module_ptrs_`にないものを追加(今回のサイクルで新たに追加する必要があるもの)
    - ただし`request_modules`のうち`highest_priority_module`と同一であり，かつそれがまだ（or もはや）**isWaitingApproval()**ではないものはスキップ（このモジュールは即座にapproved_moduleにするからcandidate_moduleには追加しないため）

`highest_priority_module`かつ**not isWaitingApproval()**であるモジュールはcandidateを経ずに直接`approved_module_ptrs_`に入るようになっている．そのため既存の`candidate_module_ptrs_`から削除し，追加もされないようにしている．

```cpp title="behavior_path_planner/src/planner_manager.cpp:855:870@runRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:855:870
--8<--
```

```cpp title="behavior_path_planner/src/planner_manager.cpp:876:891@runRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:876:891
--8<--
```

```cpp title="behavior_path_planner/src/planner_manager.cpp:170:178@runRequestModules"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:170:178
--8<--
```

## runApprovedModules

/// tip | 注意点
ここで動いているモジュールは`WAITING_APPROVAL`ではないというだけで，RTCでは承認されているとは限らない．どちらかというとmanagerがどのモジュールのRTCを送信するかを選択することでRTC介入を可能にするモジュールを選択している
///

### 副作用

- isWaitingApproval()に戻った`approved_module`がいた場合その分`approved_module`が1つ減り，`cadidate_module`はそれ1つだけになる
- FAILUREだったapprove_moduleがいた場合，それ以降の`approved_modules`全てと全ての`candidate_modules`が削除される
- SUCCESSの`approved_modules`も全て削除される

### 処理

出力の初期値は以下である．

```cpp title="behavior_path_planner/src/planner_manager.cpp:641:641"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:641:641
--8<--
```

`approved_module_ptrs_`が空であればこの初期値をそのまま返す．

そうでない場合，まず`approved_module_ptrs_`のうち`is_keep_last`のものを昇順で最後尾に移し(low -> high)，`is_keep_last`でないものをbootstrapで実行して結果を保持する．

```cpp title="behavior_path_planner/src/planner_manager.cpp:701:706"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:701:706
--8<--
```

次に，`is_keep_last`でないもののうち**isWaitingApproval()**に戻ったものがいた場合，**clearCandidateModules()**してから再度そのモジュールだけを`candidate_module_ptrs_`に戻して`approve_module_ptrs_`からは削除し，`results`からはそのモジュール以降の全てのモジュールの結果を削除する．

```cpp title="behavior_path_planner/src/planner_manager.cpp:730:740"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:730:740
--8<--
```

次に`FAILURE`だったものがいた場合，それとそれ以降を全て**deleteExpiredModules()**し，**clearCandidateModules()**し，そのモジュール以降を`approve_module_ptrs_`から削除する．

`result`に結果が残っているものがapprovedかつvalidなモジュールであるので，`approved_module_ptrs_`を逆向きにiterateして`result`に結果が残っているものを見つけたらそれを求める．

`SUCCESS`のものを`approved_module_ptrs_`の後尾に移す．

```cpp title="behavior_path_planner/src/planner_manager.cpp:793:815"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:793:815
--8<--
```

successしたモジュールは末尾に揃えられているので，それらを`approved_module_ptrs_`から削除する.

```cpp title="behavior_path_planner/src/planner_manager.cpp:817:834"
--8<--
planning/behavior_path_planner/src/planner_manager.cpp:817:834
--8<--
```

/// tip | 注意
succcessしたモジュールを削除しているだけなので，ここであえて末尾に揃えてから削除する必要はない．
///

## run()

以下をloopする

- `approved_modules_output = runApprovedModules()`
  - `output = getReferencePath()`を初期値とする
  - `approved_modules_`が空であればそれをそのままoutputを返してbreak
- `approved_modules_output`の経路を元に`request_modules = getRequestModules()`を求める
- `request_modules`がなければ`runKeepLastModules(approved_modules_output)`を返してbreak
- `request_modules`があれば`[highest_priority_module, candidate_modules_output] = runRequestModules(request_modules, approved_modules_output)`を求める．
- もし`highest_priority_module`がnullならば`runKeepLastModules(approved_modules_output)`を返してbreak
- もし`highest_priority_module->isWaitingApproval()`ならば`runKeepLastModules(candidate_modules_output)`を返してbreak
- もし`highest_priority_module`が承認されていたら`approved_module_ptrs_`に追加
- 一番上に戻る

candidateがapproveされるとapproved modulesの一番最後に入る

```cpp title="behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:344:351"
--8<--
planning/behavior_path_planner/include/behavior_path_planner/planner_manager.hpp:344:351
--8<--
```
