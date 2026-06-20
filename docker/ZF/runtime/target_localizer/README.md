# ZF Target Localizer 说明

ZF 使用 `base_link`，并约定车体前方为 `-X`。

`target_localizer.yaml` 中的关键参数：

- `forward_sign: -1`
  - 前方采样位置为 `x = -front_sample_distance_m`
  - 后方采样位置为 `x = +rear_sample_distance_m`
- `left_sign`
  - 这是旧配置兼容项。
  - 当前代码会读取它，但实际左右判定固定为 `+Y=left`、`-Y=right`，所以它不会改变输出。

当前设备几何代码固定把 `+Y` 标成 left，把 `-Y` 标成 right。ZF 车体前方为
`-X` 时，真实物理左侧是 `-Y`，因此如果后续要用 `/equipment_state`
做左右距离审计，需要在代码层面对 left/right 标签做重映射。

默认 ZF Docker compose 已禁用 `target_localizer`。只有启动时使用
`enable_target_localizer:=true`，这个目录下的配置才会影响运行。
