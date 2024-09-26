# motion_velocity_out_of_lane_module

## 基本的なデータ

### calculate_trajectory_footprints

自車の現在位置から`std::max(params.slow_dist_threshold, params.stop_dist_threshold)`の範囲で上段から与えられたtrajectory沿いにfootprintポリゴンのvectorを作る．

```cpp title="autoware_motion_velocity_out_of_lane_module/src/footprint.cpp:56:79"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/footprint.cpp:56:79
--8<--
```

### calculate_trajectory_lanelets

trajectory全体を線分として繋げてそれらを含んでいる全てのlaneletを求めている．

```cpp title="autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:66:83"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:66:83
--8<--
```

Lanelet2の実装によると

```cpp title="lanelet2_core/geometry/LaneletMap.h"
/**
 * @brief Returns all elements that are closer than maxDist to a geometry in 2d
 * @param layer for the check (a layer of LaneletMap)
 * @param geometry to check (any 2d geometry)
 * @param maxDist maximum distance to the input geometry. If zero, only primitives containing the element are returned.
 * Be aware that rounding errors can affect the result for primitives directly on (or very close to) the boundary.
 * @return vector of pairs: <actual distance, element> of all elements of a layer which are closer than maxDist to the
 * geometry. The return type differs depending on if the layer is const or not. Result sorted in ascending distance.
 * @see findNearest
 */
template <typename LayerT, typename GeometryT>
auto findWithin2d(LayerT& layer, const GeometryT& geometry, double maxDist = 0.)
    -> std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>>;
```

とある.

もし上段の経路がlane changeしているものだった場合，これだとその前後の近傍のlaneletが欠落してしまうためそれを求める処理を**get_missing_lane_change_lanelets**で行っていると思われる．

### get_missing_lane_change_lanelets

与えられたtrajectoryがlane changeしない場合この関数の戻り値は空であると考えられる．`trajectory_lanlets`の前後のlaneletの集合和を`consecutives`，横の隣接laneletの集合和を`adjacents`として求めている．

```cpp title="autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:38:55"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:38:55
--8<--
```

```
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
|   A   |   B   |   C   |   D   | /- E >|-- F ->|   G   |       |       |
+-------+-------+---_____~~~~~~~~~-----+-------+-------+-------+-------+
|-- H ->|-- I ->|--/ J  |   K   |    L  |   M   |   N   |       |       |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+
```

よって上のようなtrajectoryの場合は

- `trajectory_lanelets`
  - H, I, J, K, D, E, F(K/DでLC)
- `consecutives`
  - H, I, J, K, L, C, D, E, F, G
- `adjacents`
  - A, B, C, K, D, L, M, N

となるためCとLが`missing_lane_change_lanelets`として得られる．

```cpp title="autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:56:63"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:56:63
--8<--
```

### calculate_ignored_lanelets

egoから`$rear_offset`後方の点が載っているlaneletのうち`trajectory_lanelets`に属していないものを求めている．

```cpp title="autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:86:102"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:86:102
--8<--
```

### calculate_other_lanelets

egon中心に`std::max(params.slow_dist_threshold, params.stop_dist_threshold) + params.front_offset + params.extra_front_offset)`以内の距離にあるlaneletのうち，`trajectory_lanelets`と`ignore_lanelets`の両方に属していないものを求めている．

```cpp title="autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:105:124"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/lanelets_selection.cpp:105:124
--8<--
```

## オブジェクトのフィルタリング

### filter_predicted_objects

各オブジェクトのうちラベルが歩行者であるものは除外し，次にconfidenceが低いものと自車位置に対する法線距離がパラメーター以下であるものは`is_crossing_ego`であるとして弾く．すでに自車と交差しているpredicted pathは弾いているので，直接自車と交差はしないがぎりぎり近いものが対象になるということ．

```cpp title="autoware_motion_velocity_out_of_lane_module/src/filter_predicted_objects.cpp:114:127"
--8<--
planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/src/filter_predicted_objects.cpp:114:127
--8<--
```
