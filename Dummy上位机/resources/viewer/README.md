# 离线三维视图

运行时没有 CDN 或网络依赖。Three.js 0.170.0（MIT）与 URDFLoader
0.12.6（Apache-2.0）源文件、许可证和 SHA-256 清单都在 `vendor/` 中。
依赖源：<https://github.com/mrdoob/three.js/tree/r170>、
<https://github.com/gkjohnson/urdf-loaders>。

`RobotView` 通过随机端口和随机 token 的只读 `127.0.0.1` 服务读取
`resources/viewer/` 与 `resources/models/`，不开放其他目录，不提供目录列表，
拒绝跨源请求；WebEngine 拦截全部外部网络请求。Qt WebChannel 在内存中传递
显示状态。关闭窗口会释放服务、WebChannel 和三维资源。

`ready` 仅在 URDF 和全部 10 个 STL 可视网格成功加载后发出。仅加载
visual，不加载重叠的 collision。模型、网格、脚本或 GPU 错误显示在视图中，
并发出 `status_changed`；模型失败不发出 `ready`。初始化超时是 25 秒。

## 显示映射和需要实机验证的事实

六轴输入单位为度，地轨输入单位为毫米。J1/J2/J3/J6 同号转换为弧度，
J4/J5 取反；`rail` 转为米。夹爪百分比 0 表示闭合、100 表示全开，两个
指关节都设为 `0.026 * (1-p/100)` 米，关节轴本身方向相反。

J4/J5 符号和夹爪方向来自离线数学与网格对照，尚未验证实机方向。
以上变换只用于显示，绝不修改发送到固件的角度与位移。

目标预览按当前固件/old 助手范围限制：J1 ±175°、J2 -75~90°、
J3 0~180°、J4 ±270°、J5 ±100°、J6 ±180°、地轨 ±250 mm。
实际回读不裁剪到 URDF 的近似弧度限位，避免 180° 被截为 3.1415 rad。
URDFLoader 的 `ignoreLimits` 对实际和目标都开启，目标在 Python 层按精确
固件范围限制。机械限位仍由固件负责。

固件正解采用 165 mm 基准高度，URDF 为 227.3 mm。MoveL 目标标记把基座
局部 XYZ 转为模型世界坐标，Z 增加 62.3 mm，Y 增加目标地轨位移。
其姿态轴采用输入 Rx/Ry/Rz 的 ZYX Euler 显示约定，只作为预览；本版本
没有本地逆运动学、碰撞检测或路径可达性判定。标记不会改变机械臂姿态。

实际模式在未收到位置回读时不显示伪造的实际机械臂。断连/超时保留最近
姿态并提示数据过期。固件的 `#GETJPOS` 不含夹爪实际位置，因此显示夹爪
目标开度并明确标注“无实际回读”。目标模式可叠加最近实际姿态的淡色轮廓。

## 可调用接口

`RobotView(parent=None)`：`ready` 和 `status_changed(str)` 信号；
`set_target_pose(list7, gripper_pct=100)`、`set_actual_pose(list7, gripper_pct=None)`、
`set_display_mode('preview'|'actual')`、`set_target_cartesian(list6|None)`、
`set_theme('light'|'dark')`、`set_stale(bool)`、`set_actual_known(bool)`、
`focus_joint(1..8|'J1'..'J8'|'Joint1'..'Joint6'|'rail'|'gripper')`、
`reset_view()`、`set_camera_view('front'|'side'|'top'|'isometric')`、`cleanup()`。
网格和坐标轴设置：`set_grid_visible(bool)`、`set_axes_visible(bool)`；
兼容别名 `set_show_grid` 与 `set_show_axes`。

UI 兼容别名：`set_cartesian_target`、`set_preview_visible`、`set_follow_actual`、
`set_camera`、`reset_camera`。
