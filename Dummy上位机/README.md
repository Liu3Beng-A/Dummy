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
- 运动页右下的夹爪区固定显示，切换 MoveJ/MoveL 不影响位置输入、滑块、拖发开关和电流开合；与夹爪设置页同步。
- 启动、失能、回零、休息、校准及模式控件位于“运动控制”标题右侧。速度和发送/读取紧接滑块区，窗口空间不足时仅轴区滚动。
- 串口日志默认显示在夹爪下方；点击“展开”或顶部“终端”打开独立窗口，保留筛选、搜索、暂停、导出、清空及自定义指令/历史。关闭独立窗口后内嵌日志继续显示，展开不会挤占主界面。
- 地轨位置单位为mm，关节为°，速度档位1～100，100对应电机轴30r/s。
- STOP是固件软停命令。回执状态及未知/超时分别显示。
- PID保存通过当前`#SET_PID`直接写EEPROM；加速度、电流限制及保护参数按页面区分临时应用与持久保存。
- 示教编辑器可切换 MoveJ（六轴+地轨）与 MoveL（X/Y/Z、A/B/C），支持滑块、数值和显式拖发。选择点位、切换类型及同步实际位置只更新编辑/预览，不自动运动。
- 示教列表可混合两类点位；顺序执行先确认空闲 SEQ，再等待匹配运动类型及编号的完成回执。夹爪开度用于预览，不随点位轨迹自动发送；MoveL 不移动地轨。
- 点位导出使用版本 2，旧版本 1 或无版本的 MoveJ 文件仍可导入；旧版软件不支持新的 MoveL 点位文件。
- 本地偏好和示教文件只保存在电脑，加载文件不会自动写入设备。

位置轮询是上位机主动发送 `#GETJPOS`，默认 5Hz，可在设置中调整为 1～10Hz 或关闭。主控返回已有的七轴缓存，不因该指令新增 CAN 查询；按正常数值长度估算，5Hz 在 115200/8N1 UART 上占接收带宽约 1.9～2.5%、发送约 0.4%。USB CDC 的波特率设置不是实际 USB 吞吐上限。轮询回包及时仅表示主控回答及时，协议未提供各电机缓存年龄。

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
