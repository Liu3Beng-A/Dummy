# Dummy Robot 上位机

独立的 Windows / PySide6 机械臂控制台，以开发开始时 `串口助手_old.py` 的协议与业务行为为基线，带离线 URDF 三维显示、示教、参数、保护/校准、RGB 和终端页面。

## 启动

在本目录运行：

```powershell
python -m pip install -r requirements.txt
python main.py
```

启动不会连接设备或自动执行运动。串口波特率为固件定义的115200，连接类型需选择USB或UART4；界面根据当前固件接口开放功能。默认配置与日志存放于 `%LOCALAPPDATA%\DummyRobot\`，可以用 `--data-dir` 指定便携数据目录。

```powershell
python main.py --data-dir .\build\preview-data --size 1366x768
python main.py --smoke-test --snapshot .\build\preview.png --data-dir .\build\smoke-data
python -m unittest discover -s tests -v
```

## 操作说明

- 预览和实际位置独立。拖动关节默认只更新预览；启用拖动发送后遵守原助手模式与状态限制。
- 地轨位置单位为mm，关节为°，速度档位1～100，100对应电机轴30r/s。
- STOP是固件软停命令。回执状态及未知/超时分别显示。
- PID保存通过当前`#SET_PID`直接写EEPROM；加速度、电流限制及保护参数按页面区分临时应用与持久保存。
- 示教顺序执行等待模式确认和匹配运动编号完成，不按固定延时猜测到位。示教点的夹爪开度用于预览，原顺序轨迹命令只包含J1～J7。
- 本地偏好和示教文件只保存在电脑，加载文件不会自动写入设备。

## 三维显示边界

所有模型与前端脚本来自本目录，支持离线加载。六轴、地轨和左右夹指联动；相机操作不会发运动指令。

显示映射来自URDF与固件正解的离线数学对照：J4/J5取反，地轨mm转m，夹爪开度反向映射至0～26mm。实机方向尚待逐轴确认。URDF基准比固件基座高62.3mm。

当前位置回包包含六关节和地轨，夹爪实际开度未提供，视图只显示夹爪目标值。STOP/校准门禁期间暂停位置轮询并显示过期/暂停状态。MoveL目标以位置标记展示，未实现本地逆运动学、碰撞检测或物理仿真。

## EXE 构建

**2026-10-09 状态：按用户要求暂停打包验证。** 首轮文件夹包已构建，但启动时 QtCore DLL 加载失败，尚未修复及验收。现有 `dist/` 为排查产物，请通过上面的 Python 入口继续开发和测试；下列命令留作后续构建说明。

```powershell
python -m pip install -r packaging\requirements.txt
powershell -ExecutionPolicy Bypass -File packaging\build.ps1
# 文件夹包验证后再生成单文件版
powershell -ExecutionPolicy Bypass -File packaging\build.ps1 -OneFile
```

输出位于 `dist/`。文件夹版须分发整个`dist/DummyRobot/`目录；单文件版为`DummyRobotPortable.exe`。资源、模型、Qt运行时随包携带，使用端不需要安装Python。USB串口设备驱动按实际硬件安装。

程序依赖PySide6和pyserial，前端依赖Three.js及URDFLoader；对应第三方许可证随资源保留。项目详细方案及验证结果见仓库`docs/Dummy上位机开发方案.md`。

开发期间，另一处工作在原助手及固件中新增了 `#MOTOR_DEFAULT` 电机恢复默认事务。本轮独立程序尚未接入该后发功能；需要在其协议稳定后同步专用操作、状态门禁和回读验证。
