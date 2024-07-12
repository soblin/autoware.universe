# map_loader

## lanelet2_map_loader

_map_projector_loader_ ノードが生成時に一度だけ`/map/map_projector_info`をpublishするので，それをsubしたこのノードが`/map/vector_map`をpublishsする．

```cpp title="src/map_loader/lanelet2_map_loader_node.cpp:68:90"
--8<--
map/map_loader/src/lanelet2_map_loader/lanelet2_map_loader_node.cpp:68:90
--8<--
```

QoSは`transient_local`なので，vector mapを利用するノードでvector mapに対するcallbackが何度も実行されるということはない．
