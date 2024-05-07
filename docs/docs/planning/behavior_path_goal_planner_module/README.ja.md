# behavior_path_goal_planner_module

rvizで普通にゴールを置いた時に，ゴールが通常の道路の上だと指定した位置にきっちりと向かうようになっている．もしゴールを路肩レーンの上に置くと前後の路駐禁止エリアや路駐車両を考慮してゴール位置が修正されるようになっている．またrvizのmodified goal pluginを使うと，通常レーンの上に置いたゴールも路肩に自動的に寄せられるようになる．

まだ幅寄せの経路が生成されていない段階で経路が黄色くなっておりおそらく減速(deceleratePath)指示が出されている．すぐには反映されないが経路が見つかったら

```cpp title="planning/behavior_path_goal_planner_module/src/goal_planner_module.cpp:1191:1198"
--8<--
planning/behavior_path_goal_planner_module/src/goal_planner_module.cpp:1191:1198
--8<--
```

の処理がされている．

またそのときすでにfoundPullOverであれば

```cpp title="planning/behavior_path_goal_planner_module/src/goal_planner_module.cpp:919:926"
--8<--
planning/behavior_path_goal_planner_module/src/goal_planner_module.cpp:919:926
--8<--
```

が実行される．これは障害物などがない場合である．またapprovedでなくかつhasDecidedPathでない場合でも幅寄せの経路は出る．hasNotDecidedPathの場合はneedPathUpdateで定期的に経路の更新を試みている
っぽい．

## 経路を確定させるタイミング

始めに路肩にゴールを置いたとして，初めからシフトする幅寄せの経路が出る訳ではない(todo: 初めのあの状態がどこでそうなってるか書く)．基本的に

- **_NOT_DECIDED_**：まだゴールのずっと手前にいるので
  - 本当に何もしない(todo: idle)．ただしバックグランドで候補経路の生成は開始されている
  - 幅寄せ経路の候補の生成が終わっているが，暫定候補経路の幅寄せ開始地点からまだ遠いので何もしない(todo: candidateで出力するタイミング)
- **_DECIDING_**：**GoalPlanneModule::isSafePath** == `true`または **isActivated** == `true`で，暫定候補経路の幅寄せ開始地点の手前`$pull_over.decide_path_distance`にまで近づいたら実際にplann
  ingnの出力として幅寄せ経路への追従を開始する（がしばらく直線区間が続くのでまだ出力経路は変わりうる）
  - **GoalSearcher::isSafeGoalWithMarginScaleFactor**でゴール地点が危険と判断されたり**GoalPlannerModule::checkObjectsCollision**で危険と判定されたら**_NOT_DECIDED_**に戻る
- **_DECIDED_**：**_DECIDING_**が1.0秒以上続いたらこの状態になり，以降はずっとこの状態になる．

**GoalPlannerModule::checkDecidingPathStatus**で
