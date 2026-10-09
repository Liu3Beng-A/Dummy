# SESSION STATUS（2026-10-09）

## 当前完成：选中电机恢复默认参数（2026-10-09，源码与全部固件已验证）

- **用户决定/范围**：用户强调最初测试正常、优先排除参数设置影响，明确恢复此前已同意的默认参数功能。本节取代下方历史分析的“继续暂停”。已修改 `串口助手_old.py`、`firmware_new/protection_protocol.h`，主控 `Robot/instances/protection_coordinator.h/.cpp` 与 `UserApp/protocols/ascii_protocol.cpp`，电机 `Ctrl/Motor/motor.h/.cpp`、`Ctrl/Motor/stall_protection.cpp`、`UserApp/main.cpp`、`UserApp/protection_config.cpp`。用户随后明确授权提交本工作改动；只提交本功能与本工作文档，不烧录、不操作实机，未改另一工作负责的独立 `Dummy上位机/` 工程。
- **功能与操作**：old 助手电机参数区新增“恢复默认参数（选中节点）”，只支持 1～8，ALL 禁用；先 `!DISABLE` 并等待 `Disabled ok`，再确认恢复。清空所选电机最后 1KiB 用户参数页，恢复当前型号源码默认 PID、速度、加速度、电流、STALL/CAL 和零点偏移，保留独立磁编码器校准表；电机重启保持失能，之后须重新置零校准。J7 PID 默认 350/320/25/50，其余型号 200/80/300/250；基础速度/加速度均为 30r/s、30r/s²，电流上限 2.5A。
- **完成语义**：USB 新命令 `#MOTOR_DEFAULT <1～8> <非零uint32请求编号>`；主控拒绝非失能/忙态，电机再次核验实际失能及无校准/保存任务。CONFIG op=5 先回 PENDING，主循环擦写并逐字节核验，再软件复位；BKP DR2～5 保存请求编号、事务和结果，重复 CAN 请求不会反复擦写。启动强制重建默认基础参数并核验整个 Flash 参数页后才回 OK；主控再取得新的失能状态证明，才打印 `ok MOTOR_DEFAULT [n] req=x restored`。失败/超时明确报告，STOP/DISABLE 取消等待时提示已受理擦写可能继续。
- **界面回读与一致性**：成功后清除所选节点旧 STALL/CAL 缓存，依次回读 PID、ACC、I_LIMIT，再按节点回读 STALL/CAL；全部完成才提示“恢复及回读完成”。期间阻止运动、设置和校准，保留 STOP/DISABLE 与只读诊断；断线/超时取消未发队列与延迟回调，节点/请求编号不符的迟到回执不匹配。J8 同时恢复主控 HAND_I 校准电流上限及助手输入框至 1.0A。
- **默认值遗漏已修正**：`stall_protection.cpp::InitProtection` 原先在 Defaults 后硬编码覆盖 CAL 值，本轮改为各自默认值与基础限幅取较小值。恢复后 J7 SEEK/RETURN/RETOUCH/ACCEL 为 5/5/0.5/3，J8 为 2/2/0.5/5；CAL 电流分别 2.5A/1.0A，CONTACT_RATIO 分别 0.3/0.2。正常启动已有有效 EEPROM 仍优先，升级不会自动重置参数。为适配原有 95KiB 应用区，仅对低频配置 Validate 使用 Os，20kHz 控制/检测函数优化未改，未扩大 Flash 区侵入编码器表。
- **验证**：实际生产逻辑本机回归通过：主控 24810 项断言，35/42/57 各 3545，gripper 3546，覆盖门禁、重试、写入失败、部分启动写入重试、完成回执及原有保护/校准流程。Tkinter 界面 65 项检查通过，含取消/超时/迟到回执/自动回读和两种窗口尺寸全部参数控件边界；old 助手语法和 Git diff 空白检查通过。`串口助手.py` SHA256 仍为 `D07D3E90968F42706C107EC038AFD5109E6A67FC0A12F3C2BC3D98D13FF0D7E4`，未修改。测试在 Git 忽略的 `firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/`。
- **构建与产物**：主控及 35/42/57/gripper Ninja 全部成功，两处 `ninja -n` 均无待构建。主控 BIN 448652B；电机 35/42 各 96436B、57 96428B、gripper 96388B，距 `0x08017C00` 编码器表分别余 844/844/852/892B。BIN/HEX 内容、HEX 校验和及应用区边界通过；SHA256 清单为上述分析目录 `defaults_restore_artifacts.json`，日志为两个 build 下的 `defaults_restore_build.log`。原有非 void 空函数/RWX 告警保留。
- **实机边界/待验证**：须使用配套新版主控和对应型号电机固件，再运行 old 助手验证真实恢复；旧电机不支持新 op，不能显示为成功。尚未验证实机擦写/重启或恢复后校准结果，未证明碰限位 BAD_TASK 已解决，唯一复位原因未确认。恢复的是当前源码默认值，不是历史 EEPROM 快照；断电/备用域丢失时不保证事务结果可追溯，超时后应重新失能并回读。

## 当前分析：校准碰限位后 J8 丢失任务，参数风险与 STOP 等待逻辑（2026-10-09）

- **用户要求/范围**：用户提供 20:43～20:45 新日志，要求主要结合逻辑和参数分析校准为何触发堵转；用户补充确认 20:44:08 异常时夹爪刚碰到物理限位。恢复默认参数功能继续暂停。本轮只读源码并同步本文，未改固件或助手、未下发参数、未编译/烧录/提交。
- **本次实机链路**：HOME/RESET 成功，DISABLE 确认后开始 CAL；J1～J6 已完成坐标捕获，J7/J8 在 20:44:05.290 开始 SEEK。J8 在 20:44:08.448 从 phase=9 直接报告 phase=0/NONE，20:44:08.463 主控报 BAD_TASK，尚未出现 BACKOFF/RETOUCH/RETURN。20:44:30.273 回读 J8 task=0、at_ms=21888，校准电流/接触阈值/最大搜索行程为零；LAST=no_failure_record。按首条回包时间减 HAL_GetTick 粗算启动点约 20:44:08.385，包含通信延迟。与任务/阶段/失败快照丢失合并判断，强烈指向碰限位时 J8 重启或同等程度的运行状态丢失；不能由当前日志区分供电、外部复位或软件原因。前一日志 J7 BAD_TASK 也有类似时间证据。
- **普通保护已排除为本次直接原因**：电机 `Ctrl/Motor/stall_protection.cpp:590` 在非 IDLE 阶段直接跳过普通堵转检测，J8 在 IDLE 也被 EXCLUDED。校准寻限位通过 `BeginSearch` 设置专用接触电流/速度/误差/时间，接触成立应在同文件第847～866行减速判稳，再由第787～805行依次 BACKOFF、RETOUCH、RETURN；正常接触不主动重启。超时/异常接触/再次堵转走 FailProtection 或 BeginLocalProtection，都会先 FreezeCalFailure，普通 STOP/DISABLE 不抹掉该失败快照。现有 HardFault 是死循环；看门狗 HAL 模块未启用，正常接触分支未发现主动系统复位调用。不能据此排除所有软件或硬件复位。
- **实际参数及耦合**：J7 回读 SEEK=5、RETURN=20、RETOUCH=0.5、ACCEL=30、CURRENT=2.5、RATIO=0.3、BACKOFF=0.5；J8 回读 2/5/0.3/5/1/0.2/0.3。专用触碰条件为滤波电流命令达到 current×ratio、滤波速度≤0.02r/s、参考-实际误差绝对值≥0.02r，连续100ms；J7 约0.75A，J8 在主控 HAND_I 保持默认1A时约0.2A。主控 `protection_coordinator.cpp:699` 还会传 HAND_I 作为 J8 校准电流上限，电机第712～713行取较小值，因此 GET_CAL 单独不能证明实际 cap。`focCurrent` 是限幅后的控制电流命令（motor.cpp:373～378），不是实测电流。J8 参数是在疑似重启后回读的，不能证明异常前临时 RAM 配置完全相同。
- **可诱发正常校准失败的参数风险**：校准 BACKOFF/RETURN 使用 CAL_RETURN_RPS、CAL_ACCEL、calibrationCurrent；这些阶段的再次堵转检测仍用低门槛 CAL 接触阈值，持续时间改用普通参数 REBLOCK_MS，启动屏蔽改用普通 STARTUP_MS（stall_protection.cpp:671、791、805、829～871）。J7 RETURN=20/ACCEL=30 高于 Defaults 表的5/3；短0.5r回退在理想梯形下峰值约3.87r/s，不能直接当成20r/s瞬时回退。J8 短0.3r回退峰值约1.22r/s。J8 SEEK=2r/s 对16:1输出约45°/s，理想持续100ms期间参考可继续推进约4.5°；这只是参考轨迹数量级，不是实测压入行程或力量。上述风险若触发正常分支应产生 EARLY_CONTACT/REBLOCKED 等记录，不能直接解释本次 BAD_TASK。
- **已确认的后续等待问题**：主控 Error 对 BAD_TASK 调用 BeginStop(false, wasCal)；夹爪 `UserApp/protocols/interface_can.cpp:40～45` 只在仍有校准上下文时接受 STOP。重启后上下文已丢失，J8 拒绝该 STOP 且不回0x0A，符合 STOP wait=0x100/no reply；DISABLE 始终允许，符合用户随后 Disabled ok。此链路是校准任务丢失后的恢复逻辑问题，不是 J8 普通堵转开启。
- **建议与未完成**：先以未保存的参数对照降低碰限位冲击（J8 SEEK 可单项降至0.5；J7 CAL_ACCEL/RETURN 回到3/5），其他检测阈值保持，以区分运动负载影响；这不是已验证修复。若继续修复，需启动时记录 RCC_CSR 复位原因、启动会话与软件复位入口，并明确报告校准任务丢失；在夹爪上下文丢失后转八轴 DISABLE 并核验确认。重新编译没有重置 EEPROM，已保存参数仍优先；InitProtection 硬编码覆盖 Defaults 的遗漏已在历史第83行附近记录，尚未修改。本次唯一复位原因与降速对照实机结果均未确认。

## 当前完成：firmware_new 全部固件完整重编译（2026-10-09 20:30）

- **用户要求/范围**：用户要求重新编译 `firmware_new` 全部固件。先在主控、电机各自 `build/` 执行 `ninja -t clean`，再按现有 Release 配置执行 `ninja -j 8`；未修改固件源码、配置或串口助手，不提交、不烧录、不操作设备。本节仅同步工作状态。
- **构建结果**：主控 199/199、电机 213/213 全部成功，两个构建进程退出码均为 0；最终两处 `ninja -n` 均为 `no work to do`。主控 BIN 于 20:30:54 生成，四类电机 BIN 于 20:30:44 生成；对应 ELF/HEX 同时重建。保留既有非 void 空函数缺少返回值和 RWX 链接告警。
- **产物与核验**：主控 `firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin` 446776B，SHA256 `8b65c47995760a30c010eeaba44b49b0b7205471c7c2e5051e38c6c4028cb632`；电机产物在 `firmware_new/dummy-motor-fw/build/`：35/42 各 97044B，SHA256 均为 `e75e36ef9decab103e9045bbf70f86ecdcb481bf0040451221d2ccfa3e7521a7`；57 为 97044B，SHA256 `a1e79690164ac62b6a75890f42d32a778ab358c6e00c5c70f9b8d7dde1fd03ca`；gripper 为 96996B，SHA256 `f2c69eb4bb1c030aa0fca426a06da3d0072ffd6560efbf9993f9e37aae3cf08e`。五份 BIN 的 SHA256 均与重编译前一致；HEX 校验和、显式数据与 BIN 对应字节、BIN 间隙的 objcopy 默认零填充及应用区边界核验通过。
- **内存**：主控 RAM59528B、CCMRAM64KiB、FLASH446776B；四类电机 RAM10704B。35/42/57 距编码器表起始地址 `0x08017C00` 各剩 236B，gripper 剩 284B，均未越界。构建日志分别为两处 `build/rebuild_20261009.log`。
- **暂停工作**：“恢复默认参数”按钮及配套协议已获用户同意，但尚未改代码，用户后续明确要求暂停，当前继续保持暂停。20:23 日志的 STALL_SET 拒绝问题仅完成初步源码核对：电机 `UserApp/protection_config.cpp:86` 限定 BACK_CURRENT_A 不超过实际 `boardConfig.currentLimit / 1000`；主控 `Robot/instances/protection_coordinator.cpp:858` 把任意非 OK 回包统一打印为 REJECTED，并通过 `Error()` 锁定/STOP。尚缺本次 J1 的 `#GETI 1` 回读和具体拒绝状态/事务阶段，不能断言实机电流上限或唯一拒绝原因。未实施修复，本次重编译不代表上述问题已解决。

## 当前完成：在回退基线上单独恢复J7默认PID（2026-10-08，已编译）

- **用户决定/范围**：完成整体回退后，用户要求地轨默认PID采用回退前决策的版本，即KP=350、KV=320、KI=25、KD=50。本轮只修改电机模块`firmware_new/dummy-motor-fw/UserApp/main.cpp`的MOTOR_TYPE=57默认表及说明注释，文档同步本文、地轨电机加入方案和PID说明状态；其他固件/助手代码维持87288fb，PID事务与顶部模式提示仍未恢复。不提交、不烧录、不操作设备。
- **已确认事实**：57固件固定节点为J7，CMake用MOTOR_TYPE=57构建motor-57目标。默认表仅在BoardConfig.configStatus不为CONFIG_OK时初始化；已有有效EEPROM优先生效，本次没有覆盖或清除实机已保存PID、编码器表及坐标数据，也不等同于实机参数已变成这四项值。
- **修改/验证**：相对87288fb的生产代码diff只有main.cpp中两行（注释与四项默认值）。四类电机Ninja编译成功；主控和35/42/gripper BIN的SHA256与回退后记录一致；57 BIN/HEX对应字节及HEX校验和检查通过，未越过95KiB应用区。未新增控制逻辑或恢复其他撤销功能，旧构建告警保留。
- **当前地轨产物**：`firmware_new/dummy-motor-fw/build/motor-57.bin`为97044B，SHA256 `a1e79690164ac62b6a75890f42d32a778ab358c6e00c5c70f9b8d7dde1fd03ca`；同目录motor-57.hex可供烧录，应用区剩余236B。核验记录`firmware_new/dummy-motor-fw/build/analysis/rail_pid_defaults_20261008.json`；先前rollback_artifacts.json保留为纯基线回退时的历史产物记录。
- **未完成/边界**：仅完成源码默认值和固件产物更新；未刷J7、未更改实际EEPROM、未证明此前摆动或通信故障已解决。本次不延续已结束的通信故障调查。

## 历史回退记录：已放弃本轮改动并回退至87288fb（2026-10-08）

- **用户决定/范围**：用户提供16:54旧主控对照日志，表示问题仍在，明确要求“直接放弃改动退回到87288fb23b20e1f35513e5a044fa25099d4d7122，保留文档记录”。已按此决定结束本轮修复尝试，恢复全部未提交固件代码及串口助手_old.py；HEAD原本就是该提交，未改写提交历史、未提交新commit。
- **已回退功能**：主控/电机PID事务、失能RAM应用、独立SAVE_PID、保存互斥等本轮修改均撤销；old助手PID界面改动与右上角主控模式提示一并撤销；J7源码默认恢复KP=300、KV=50、KI=30、KD=100。有效EEPROM仍优先生效，代码回退不改变实机EEPROM中的参数。
- **最新测试事实**：16:54:29.343受理HOME id=1，约1177ms取消；J2 `PROTECTION_STATUS_TIMEOUT`，age rx=989/probe=1004ms，随后STOP locked。运动快照仍是七轴首次目标下发/READY成功（send=7、txok=0FE、ready=0FE、reject=0、各try=1/fail=0），计划4171.7ms，moving=02C。J2 LAST仍no_trigger_record，LIVE为STOP_LOCKED/trigger_reason=NONE。用户报告旧主控对照仍失败，这不能当作整个旧主控+旧电机组合的验收，也未证明唯一根因；本轮不再继续调整或要求补测。RGB仍按用户要求排除。
- **文档/日志保留**：既有全部文档及本轮新增PID说明保留；PID说明、查询端补齐方案、地轨电机加入方案新增“已撤销/历史记录”标识，避免照已撤销协议操作当前版本。完整原始日志逐字节保存为[16:54旧主控对照日志](测试记录/2026-10-08_165425_旧主控对照_HOME通信超时.txt)，SHA256 `ab294ce1bcaa18ca0f26dee66856e20bd8aae942d6b7dfaecada3b6edc400b42`。以下分析和“完成”记录均为回退前历史，不能视为当前代码功能或尚待继续执行的任务。
- **编译/核验**：主控及35/42/57/gripper在各自常用build目录重新Ninja编译成功，五份BIN与87288fb独立归档产物SHA256完全一致：主控446776B；35/42各97044B；57为97036B；gripper为96996B。常用BIN/HEX已恢复基线，分析目录中的旧记录保留。核验清单为`firmware_new/dummy-motor-fw/build/analysis/git_audit_87288/rollback_artifacts.json`；生产代码相对目标提交无内容diff，old助手语法核验通过。保留原有构建告警，没有通过修改基线源码消除告警。
- **未完成/实际设备边界**：此次完成的是工作区与构建产物回退，没有连接或烧录设备；若设备电机端仍为本轮新版，需要用户将对应旧版固件刷回后才是设备全套回退。通信故障根因未解决，不宣称回退后实机已恢复正常，不延续之前的修复或测试计划。

## 回退前分析记录：以87288fb为基线定位普通运动通信回归（2026-10-08）

- **当前问题/范围**：用户确认`87288fb23b20e1f35513e5a044fa25099d4d7122`为上一个HOME正常的版本，要求结合Git改动分析本文中的故障。当前HEAD正是该提交，本轮PID与模式提示为未提交改动。用户明确RGB忽略，不再把RGB列入本次问题、修复建议或测试前置条件；下节RGB分析仅作历史记录。这里只更新本文及Git忽略的独立分析/构建产物，未修改生产源码、现有发布产物或实机，不提交。
- **最新日志事实**：已读16:22～16:24附件，补齐了此前要求的`#MOTION_STATUS`。16:22的RESET id=4也由J5保护快照超时取消，不能把故障限定为J2或HOME按钮。随后用户执行ENC_CAL 2和完整CALIBRATION成功，RESET id=5成功；16:23:49.029的HOME id=6仍在约1.17秒时由J2 `PROTECTION_STATUS_TIMEOUT`取消，`age rx=974 probe=1003 flags=03`，继而电机报`COMMUNICATION_TIMEOUT`并联停。重新校准没有消除该现象，不建议继续反复校准或先调PID/电流/加速度。
- **运动下发已确认**：id=6的`send=7 txok=0FE ready=0FE pending=000 reject=0`，七轴均`try=1 fail=0`，READY在10～40ms内接受。计划4181.7ms，1169ms取消，尚未执行到计划终点。实际明显运动轴为J2/J3（moving=00C）；结合查询时的age与日志时间，最后接受的运动回复约在发布后0.19～0.20秒，而其他轴约1.11～1.15秒仍有回复。该时间换算含USB打印延迟，不是精确CAN波形。real仍是查询时的姿态，取消后的confirmed=000不能改读为从未收到READY。
- **Git已排除的直接改动**：主控HOME规划、普通运动完成握手/回调、健康检查，以及电机`CloseLoopControlTick`、`TickProtection`、`TickInternalProtection`、`TickPositionCompletion`函数体相对基线未变；主控interface_can.cpp/.hpp及dummy_robot.cpp虽显示工作区M，但实际内容diff为空，保留用户已回退状态。新日志在故障前没有SET_PID/SAVE_PID或配置保存命令，因此不能继续按在线PID写Flash导致NO_ENCODER解释此次故障。电机CAN队列/TXOK/有界重试也已在基线存在。
- **独立编译与汇编对照**：基线从Git归档到`firmware_new/dummy-motor-fw/build/analysis/git_audit_87288/src`，独立Release/Ninja构建，不覆盖当前固件。35电机的旧/新CloseLoopControlTick均1612B/524条静态指令，TickProtection均1500B/516条，TickInternalProtection均3064B/1023条，主要调用结构一致；字段偏移与链接地址有变化，不能称机器码完全一致。普通检测滤波旧/新均为两条32位SDIV，不能声称本轮新增了20kHz PID计算或64位滤波除法。静态对照不替代实机最坏耗时测量。
- **尚需验证的差异**：电机`UserApp/protocols/interface_can.cpp:22–26`新增整函数Os属性，实际影响CAN RX中断里的普通0x07目标/0x23查询/STOP，不只是配置命令；该函数旧版2636B，当前1956B。`Ctrl/Motor/stall_protection.cpp:963–1016`增加主循环配置提交检查及每轮PID暂存超时IRQ保护；未发现普通无PID情况下不恢复PRIMASK或无条件写Flash的路径。这些是需要对照的运行路径差异，目前均未证明为唯一根因。
- **Os试验的实际限制**：仅在上述忽略目录复制当前CAN源码、去掉Os并编译替代对象，OnCanCmd变为2956B。只替换该对象独立链接，Flash97776B，超过95KiB应用区496B，链接失败；未生成可烧录试验固件，未改现有对象/BIN/HEX。因此不能直接删除Os当作已验证修复，也不能扩大应用区侵入编码器表。若后续证明该路径有关，需保留普通收发的速度优化，将体积优化限制在配置分支。
- **时间相关线索的边界**：丢回复约0.2秒与普通检测默认STARTUP_MS=200接近；20kHz TIM4优先级0、CAN优先级3，电机主循环负责真正送出回复，因此中断占用或主循环无法及时服务CAN仍是候选。实机STARTUP_MS本轮未回读，且上述20kHz生成代码结构未显著改变，不能仅凭该时间相关就宣布CPU超时根因。需要固件交叉对照，必要时再增加DWT耗时和电机CAN收发/主循环计数。
- **可复现的基线产物**：已独立重建旧主控`build/analysis/git_audit_87288/core_out/Core-STM32F4-fw.bin`，446776B，SHA256 `8b65c47995760a30c010eeaba44b49b0b7205471c7c2e5051e38c6c4028cb632`；旧35电机`build/analysis/git_audit_87288/out/motor-35.bin`，97044B，SHA256 `e75e36ef9decab103e9045bbf70f86ecdcb481bf0040451221d2ccfa3e7521a7`。其余42/57/gripper也已独立编译；五组BIN hash均与本文旧版归档完全一致，HEX校验和、HEX/BIN对应字节和Flash区域检查通过，核验记录`build/analysis/git_audit_87288/audit_result.json`（路径均以firmware_new/dummy-motor-fw为根）。当前五份BIN及生产固件源码hash仍与PID构建清单一致，未发生发布产物漂移。
- **下一次最小对照**：先只刷上述旧主控，保留当前八电机固件、EEPROM及编码器校准数据；使用同一助手及相同初始姿态，等待START完成，依次RESET、HOME并保留完整日志。成功后查询MOTION_STATUS；失败后DISABLE确认再查询，期间不再新发运动。此阶段不使用新PID事务/保存功能，不重新校准或改参数。若旧主控+当前电机正常，重点定位当前主控改动；若仍失败，尚不能直接断言电机根因，需旧主控+旧电机再对照，以确认同一硬件/配置下旧版是否仍正常。再依据结果做小范围修复或诊断，源代码修改另按4.1说明文件/作用并取得确认。

## 上一轮日志分析：HOME的保护快照超时（历史记录，RGB已按用户要求排除）

本节原“最小待补数据”和RGB后续建议已由上节替代：运动快照已经收到，RGB不属于当前调查范围。

- **当前问题/范围**：用户提供16:04及16:08两份日志，要求先分析、核对本文及最近文档，再说明需要补测什么。用户确认主控和全部八个电机都已刷最新固件；期间另试修一个bug，因同样出现故障已回退。按本项目5.2只同步本文，不修改固件、助手或其他报告，不编译烧录固件、不提交、不操作设备。
- **已确认的直接触发点**：两次HOME均由主控`firmware_new/dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:482–518`的健康检查触发`PROTECTION_STATUS_TIMEOUT`后取消。16:08:52.167受理HOME id=3，53.224报J2失败，`age rx=854 probe=1004 flags=03`；16:04对应919/1003ms。probe是距上次匹配有效0x36快照的时间，rx是距上次被在线逻辑记录的节点通信的时间；不能描述成普通位置一直正常刷新、仅保护回复缺失。flags03是最后接受快照的反馈/范围有效位，不是故障时新数据。queued/rx/retry及CAN alst均为累计计数，不能用1668-1654直接认定本条HOME丢了14帧；主控terr=0/full=0也不排除电机侧或请求/回复路径问题。
- **新日志排除的误读**：本次53.563已收到`STOP locked`，距通信故障约340ms；上一轮2秒快照`wait=020`表示J5已回应但仍DECELERATING，不是J5离线，新日志没有复现该长等待。J2/J5运行快照均`phase=0 gate=STOP_LOCKED fb=1 timer_ms=0 trigger_reason=NONE`，LAST均no_trigger_record，因此此次没有这两轴本地机械STALL触发记录，不能再按此前在线改PID导致NO_ENCODER的同一原因解释。使能后的RGB保存WARN来自助手门禁，未发送保存指令，也没有PID写入指令。
- **快照速度的解释边界**：J2 raw=-0.0156641、filt=2.53068，J5 raw=0、filt=0.129141。`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:655–685`在STOP锁定门禁提前返回，停止更新普通检测滤波器；因此filt可保留停前历史，不是当前仍以该速度运动的证据。快照采于故障后，不还原故障前的完整编码器/通信波形。
- **代码与历史修复对照**：顶部模式提示只复用START后一次查询，日志均正确回读INT=2；本轮未发现重复模式查询。PID事务分支在此次日志无设置/保存时未被请求，不能据此证明所有PID改动绝对无关，但没有Flash保存触发证据。`docs/堵转误报修复与实机测试流程.md`第9节及当前电机`Core/Src/can.c`已有保护回复TXOK确认、100ms/16次上限及1–4ms错相重试，不能重复套用旧版“HAL入邮箱就出队、完全不重试”的根因。当前主控interface_can.cpp/.hpp及dummy_robot.cpp相对HEAD无内容diff，尊重用户已经回退的状态，不恢复其他试修。
- **独立确认的RGB软件缺陷**：两份日志frames/errors分别2148/2148及13358/13358、timeouts=0。`firmware_new/dummy-ref-core-fw/Bsp/gpio/rgb.cpp:306–309`在正常完成回调也调用`HAL_TIM_PWM_Stop_DMA`。仓库HAL DMA_IRQHandler先置READY（stm32f4xx_hal_dma.c:912）再调用完成回调；Stop_DMA经HAL_DMA_Abort_IT遇到READY设置NO_XFER=0x80（同文件:581–586）；回到同次DMA_IRQHandler的错误尾段又触发TIM_DMAError及RGB::Interrupt(2)，导致错误计数加1、wireValid=false和后续重复发帧。主控RGB源码相对HEAD没有本轮PID/UI新增修改，这不是本轮顶部模式文字逻辑；其与J2通信故障的因果尚未证实，不能宣称已找到HOME唯一根因。
- **RGB复现与旧测试遗漏**：从当前源码提取完整HAL_DMA_IRQHandler、HAL_DMA_Abort_IT、HAL_TIM_PWM_Stop_DMA、TIM完成/错误函数、主控回调及RGB::Interrupt，在系统临时目录用MSVC编译执行；寄存器/外设以替身提供，未改生产源码。仅直接执行TIM完成回调时errors=0而DMA ErrorCode已为0x80；完整DMA IRQ链注入200次正常TC、没有TE/FE/DME，结果errors=200、每次NO_XFER且wireValid=false。旧`build/analysis/rgb/run_native.py`只提取TIM回调、Stop_DMA为空替身，未覆盖DMA_IRQHandler收尾；此前文档的200帧errors=0仅对该夹具成立，不能证明完整HAL路径正常。临时复现不是实机DMA/总线测试。
- **最小待补数据**：新附件实际没有#MOTION_STATUS。如16:09:03 Disabled ok后未再次START/运动，直接失能查询#MOTION_STATUS，提供从MOTION id到J7的完整输出；否则下一次故障后先按需取诊断，再DISABLE确认、在任何新START/运动之前查询。`dummy_robot.cpp:1539–1548`取消保留上一任务txok、ready、分轴时刻/尝试/失败/规划快照，只清部分确认/在途位，不能把取消后的confirmed=000视为从未收到回复；real是查询时当前姿态，age也不是冻结的故障时年龄。静止阶段与首次HOME期间J2是否先卡顿/抖动仍待用户观察回答，保持PID/加速度/电流及保护参数不变，不要求反复校准/撞限位或拔线。
- **未完成/建议后续**：J2探针缺失究竟在主控请求发送、电机接收/主循环回复、总线发送还是主控接收匹配，现有计数不能区分；先看上一条HOME分轴诊断，必要时再提出小范围CAN链路计数方案。RGB宜先修正常完成清理重复abort并补完整HAL IRQ回归，之后验证errors不再随成功帧增长；尚未获本轮源代码修改授权，未执行修复，不放宽通信看门狗或反馈/堵转阈值。

## 当前完成：old助手顶部主控模式提示（2026-10-08）

- **当前问题/授权**：用户直接要求在串口助手old右上角STOP和电机状态之间加入主控模式，复用使能读取与切换回执同步。只修改串口助手_old.py和本文，保留此前PID改动；不修改固件、不提交、不连接硬件。
- **已确认事实**：串口助手_old.py的Started ok处理已调用_query_command_mode，查询回执含req并拒绝旧请求号；模式按钮通过_set_cmdmode等待ok CMDMODE mode确认。顶部使用pack(side=RIGHT)，按电机状态、模式提示、STOP的创建顺序得到从左到右STOP→模式→电机状态。
- **最新改动**：新增lbl_cmdmode和_update_cmdmode_display，显示顺序(SEQ)/打断(INT)/连续(TRJ)/调参(TUN)。初始及断线显示未知，查询/切换中显示等待提示，实际确认后更新；查询或设置超时显示未知，拒绝切换时保留上次已确认模式。手动发送#CMDMODE的成功回执也更新显示，已有等待切换时仍拒绝不匹配的确认。UI更新沿用RX主线程处理，不新增周期查询。
- **验证/已排除方向**：Python语法通过；实际隐藏Tk检查使能回读、错请求号、切换等待/确认、手动回执、拒绝、查询/设置超时、断线；1680×1000和1560×940实际渲染验证三个控件顺序及连接状态无重叠。既有motion_ui_test.py共410项真实Tk/分片/顺序发送/模式竞争检查通过。修改后重新读取实际文件，git diff --check通过；未把点击模式按钮当主控已确认，也未扩大到固件协议修改。
- **未完成/待实测**：重启old助手后，在实机Started ok和切换模式时确认顶部文字自动更新。本轮无固件变化，不需因本次提示新增而重新烧录；此前PID固件刷写要求仍按下节执行。

## 当前完成：PID RAM应用/失能保存与J7新默认（2026-10-08，已编译，待实机测试）

- **当前问题/授权**：用户反馈在线SET_PID后J7 NO_ENCODER及七轴联停，明确授权执行此前方案，并指定57/J7默认350/320/25/50。只修改firmware_new、目标串口助手_old.py及相关文档；不提交、不烧录/操作实机，不改串口助手.py、参考目录或esp32。本文原有12行未提交核查记录保留。
- **已确认事实**：旧主控每个PID字段CAN保存位固定1，电机主循环使能时也写Flash。SPI/采样停顿与位置合理性判据存在触发NO_ENCODER的路径，但本次实机故障唯一根因尚未由波形证明；其他轴phase6为联停而非各自机械堵转。实际DISABLE完成会清电机软停锁，失能后设置流程可用。
- **最终实现**：复用0x30..0x33带事务参数通道，PID字段64..67及操作PID_BEGIN5/PID_COMMIT6/PID_SAVE7，保护PARAMETER_COUNT/存储版本不改。失能后按声明字段mask暂存、全部ACK再原子应用，COMMIT幂等，主控回读四项后才RAM成功；SAVE以Flash已保存BoardConfig为基础只更新PID，物理核验整个1KiB重写页，保留其他已保存字段及保护扩展。运行PID与BoardConfig已保存PID分离，CAL/STALL保存不持久化试调PID，PID保存也不顺带保存未持久化加速度/电流。START/保存互斥，STOP/DISABLE取消任务并重试CONFIG ABORT（含J8），电机收到取消后清理暂存/尚未开始的PID保存；已开始的Flash写入不能回滚。旧PID直接CAN仅允许失能RAM且拒绝保存位1；旧配置保存、实体按钮、旧使能/校准/恢复/重启入口不能穿过配置保存门禁。
- **已改文件**：共享protection_protocol.h；电机motor.cpp/.h、stall_protection.cpp、configurations.h、main.cpp、protection_config.cpp、interface_can.cpp；主控ctrl_step.cpp、protection_coordinator.cpp/.h、ascii_protocol.cpp；串口助手_old.py；本文、查询端补齐方案、地轨电机加入方案及新增PID调参与保存协议及测试说明.md。GET/SET/SAVE经协调器事务，USB/UART4共用，旧立即打印PID成功分支已移除；助手增加应用/保存已应用值及等待回执，RX通过root.after，超时报告未知，断线/入队失败清等待。ACC/I_LIMIT的显式&保存使能时明确拒绝，仍使用旧回执。
- **软件验证/已排除**：原生主控24992项、电机1693项断言通过，含PID专项及旧STOP/保护退让/校准回归；实际Tk离线50项通过，含回包、禁用、坏值、取消、断线、入队失败、线程投递及1680×1000/1560×940布局。旧motor夹具补CONFIG_OK、旧core夹具补现有RGB类型/入口测试替身，与真实启动状态对齐，不为迁就测试修改生产RGB。断言多数来自既有周期/传输回归，不能替代实机。未改NO_ENCODER阈值、位置跳变容差、20kHz控制算法或Flash/编码器表分区。配置Validate及CAN分发/新保存函数用Os解决容量，不改控制tick优化。
- **最终编译/产物**：Ninja五目标成功；主控FLASH448540B、RAM59528B、CCM65536B；电机35/42/57/gripper FLASH96776/96776/96768/96744B、RAM均10736B，95KiB代码区最少余504B。BIN/HEX逐字节一致、校验和有效，主控最高0x0806D81B低于0x080E0000，电机最高0x08017A07低于0x08017C00，未覆盖编码器表或EEPROM。保留既有RWX及编码器基类缺返回告警。主控BIN SHA256=41ce1ae2daddf703e54e8e3ab8d9021f3b60900d1cac3bb0c2f7b8ac97829c05；J7 BIN SHA256=23c3b80977e98d62bb52331517ef44905a8c92693a8ed808d3d49aa8d6f4e2db。全部hash、最终源hash、脚本及结果在忽略目录firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/pid_artifacts.json等文件。串口助手.py SHA256未变，参考目录/esp32未改，git diff --check通过。
- **待实测/交付**：配套更新主控及相应电机固件，J7用motor-57；全节点PID能力需要相应节点均更新。有效EEPROM继续优先；实际Disabled ok后SET_PID 7 350 320 25 50，等四项回读及ok PID_APPLY [7] RAM；START后验证到位±0.03mm及往返，再失能等Disabled ok，SAVE_PID 7等saved，重新上电GET_PID 7核对。NO_ENCODER唯一实机原因及新参数能否消除摆动仍待用户验证。本轮未提交、未烧录、未连接设备。


## 已关闭核查：J1～J6校准原点捕获与结束保存时机（2026-10-08，未改固件）

- **用户最终确认**：用户明确说明“程序没问题，是我测试的问题，我校准之后没有发送reset，以为没校准成功”。本次校准失败怀疑按测试判断问题关闭，不据此启动固件修复。`!CALIBRATION`只把六轴捕获位置定义为REST_POSE，不主动移动六轴；`!RESET`是独立的回REST运动，见`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:801–806`，不是校准保存或生效的必要步骤。
- **当前问题/边界**：用户要求先核查`!CALIBRATION`是否错误地等J7/J8返回后才采集J1～J6的位置；期望六轴在J7/J8开始前固定原点，必要时先存RAM，全部成功后保存EEPROM。本轮基于HEAD `87288fb`只读代码；按项目工作状态同步要求只更新本文，不改代码、配置、上位机或其他文档，不编译、不提交、不连接硬件。
- **已确认事实：先捕获后运动**：主控`firmware_new/dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:574–588、649–667`先验证八轴实际失能及反馈，再开始校准上下文；`CAL_ZERO`向J1～J6发送`SET_CAPTURE_COORD`，只有本任务六轴全部`CAL_DONE`确认收齐，才等待1000ms并启动J7/J8并行自动校准。电机`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:639–653、744–757`在有效编码器反馈连续稳定50ms后，将当时`realPosition - Steps(motorR)`写入运行时`encoderHomeOffset`（RAM）；六轴标签为本地0，主控`initPose`映射到REST_POSE。此操作保持六轴失能、不中途写Flash、不重置实际编码器连续位置。
- **已确认事实：结束保存没有重新采样六轴原点**：电机同文件`429–453`中，同任务已完成的捕获重试不再取当前位置；`SAVE_COORD`只进入待保存。`PollProtectionMain:898–915`读取已经存在的RAM `encoderHomeOffset`并传给`SaveProtectionSettings`，没有根据保存时`realPosition`再计算原点。`firmware_new/dummy-motor-fw/UserApp/protection_config.cpp:141–158`将传入原点写扩展记录，并取其单圈余数写旧BoardConfig，随后Flash提交/回读。主控`protection_coordinator.cpp:703–730`等J7/J8全部返回判稳、八轴失能后，依次提交八节点并收齐结束确认，才返回成功。因此“J7/J8返回后重新取六轴实际位置作为原点保存”不符合当前正常校准调用链；已有实现正是开始存RAM、末尾统一提交。
- **采样时刻的精确边界**：当前捕获不是USB收到字符串的同一瞬间，也不是六轴同时采样：入口验证、逐节点CAN命令及各轴50ms稳定确认会产生短暂差异（主控`PollGroup:415–438`）。它保证全部捕获完成在J7/J8启动之前。若用户另需严格锁存命令到达瞬间，属于新的时序要求，不能把这一差异说成末尾才捕获。
- **已确认的收尾缓存不一致**：`protection_coordinator.cpp:726`在`CAL_FINISH`把`robot.currentJoints`和`robot.targetJoints`再次硬写为REST_POSE；此时六轴若因失能被移动，这次缓存赋值与实际角度不同。它没有改电机`encoderHomeOffset`、没有发送重新捕获，不能认定为EEPROM保存错时刻。正常反馈回调`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:629–639`会根据电机角度加原有`initPose`更新实际角度。推荐后续保留开始捕获的原点，校准收尾同步六轴最新实际反馈及保持目标，避免硬写REST；本轮尚未获准实施。
- **已排除方向/未确认**：正常校准入口由保护协调器处理，不走旧0x15 `ApplyPositionAsHome`末尾置零；自动DISABLE、SAVE_COORD和CAL_END未发现重新计算J1～J6原点。此前曾询问异常发生阶段及校准期间六轴有无移动；用户现已自行确认测试判断问题，不再要求补测。收尾缓存问题没有被证实导致本次现象，不作为本次修复任务；旧单圈启动无法恢复掉电多圈移动的限制仍保留。
- **独立已确认遗漏（记录、不修改）**：`stall_protection.cpp:53–63`在调用`ProtectionSettings::Defaults`之后、载入EEPROM之前，仍把CAL_ACCEL写为min(5,额定上限)、SEEK/RETURN写为min(0.5,额定上限)、RETOUCH写为min(0.25,额定上限)。这会覆盖前轮在`protection_config.cpp:52–66`更新的J7/J8部分默认值：空白存储启动时不一定得到已验收默认速度/加速度；有效EEPROM仍在其后加载，所以用户已保存配置不受这处覆盖影响。这是前轮默认值集成核查遗漏，与本次原点捕获时机无关；后续若授权，应改为限制Defaults给出的值而非重写常数，并验证真实InitProtection路径。
- **最新改动/验证**：只更新本文分析及用户最终确认记录。未改生产源码，未重跑之前通过的编译/测试，未提交；调用链与赋值点已逐项读取。本次核查已关闭，收尾缓存及默认值初始化的独立发现仅留档，任何后续修改由用户另行决定。

## 当前归档：主控RGB功能、独立AUTO颜色及old助手提交（2026-10-08）

- **当前问题/授权**：用户明确要求“可以，先提交吧”，本次获准提交下列RGB工作。此前各节“不提交”是当时边界，已由本次授权更新；未授权推送或烧录。
- **已确认事实**：当前分支为`主控固件同步功能`，仓库本地身份为`Liu3Beng-A <liu3beng@icloud.com>`。待提交共14个文件，全部属于本轮主控RGB、old助手及三份相关文档；无电机、参考firmware、串口助手.py、esp32或构建产物变更。
- **提交内容**：24灯珠整帧DMA、亮度/开关/渐变与动画、运行/保护状态联动、Flash显式保存及v1→v2迁移、SEQ/INT/TRJ独立AUTO颜色和同色常亮/旋转、DMA完成回调与回包/查询修复、old助手完整RGB模块及中栏页面切换。本文与RGB方案/STOP查询规则说明一起归档。
- **验证**：提交前重新核对产物及测试源码hash，BIN仍为446776B、SHA256`8b65c47995760a30c010eeaba44b49b0b7205471c7c2e5051e38c6c4028cb632`；BIN/HEX一致，四电机BIN和参考助手hash未变，git diff --check通过。此前主控Ninja编译、原生380320项断言、实际Tk离线148项检查通过，未重复执行已通过且源码未变的测试。
- **未完成/待实测**：Git提交归档不代表实机验收。用户继续测试灯珠电气/动画观感、自动模式待机/运动同色、保存重启及运动并发。测试脚本/记录和BIN/HEX保留在本地Git忽略的build/analysis/rgb及build目录；提交编号以Git日志和本轮回复为准。

## 当前完成：SEQ/INT/TRJ独立AUTO颜色及同色常亮/旋转（2026-10-07，已编译）

- **当前问题/授权**：用户直接要求调整RGB自动联动，三种普通指令模式各有独立单色预设，待机常亮和运动旋转使用同色，与手动预设0/1/2分开；继续此前允许直接执行、同步old助手、编译由用户测试的工作。本轮修改主控`Bsp/gpio/rgb.hpp/.cpp`、`UserApp/protocols/ascii_protocol.cpp`、`串口助手_old.py`及RGB方案/本文；保留既有未提交工作，不提交/烧录/连接硬件。电机、参考firmware、串口助手.py、esp32未改。
- **已确认事实**：原`RGB::Defaults/SelectScene`中READY_SEQ取colors[1]，RUN_SEQ取effects[8].first，INT待机/运行各有独立固定覆盖色，RUN_TRJ是彩虹；改手动预设不会同步对应旋转色。现有Flash v1保存整个Config和CRC，直接扩大结构会令旧记录被当作无效，因此本轮加入兼容迁移。
- **最新改动**：新增autoColors三槽和常亮模式10/11/12（SEQ/INT/TRJ），0～9编号不变，动画参数仍只允许3～9。默认READY分别绑定10/11/12、RUN均为8旋转且颜色来源2/3/4动态引用相应AUTO槽；手动预设/动画主色修改隔离，TUN和其他状态规则保留。新增!RGB_AUTO_COLOR 1..3 R G B及对应全量回包；!RGB_SET_STATE支持AUTO_SEQ/AUTO_INT/AUTO_TRJ来源，原无颜色/显式RGB形式兼容。设置继续直接回包，无额外查询。
- **保存兼容**：版本升为2，autoColors附加在v1结构后，布局静态断言。合法v1保留亮度/开关/过渡/手动颜色/动画和其他状态绑定，取原三个READY有效颜色作为AUTO色，再配对六个READY/RUN为独立常亮/同色旋转。仅RAM迁移，storage=1/dirty=1，仍需实际失能后!RGB_SAVE写v2；不自动擦写。坏CRC/旧版本越界来源被拒绝，新版本保存重载保持全部AUTO槽。
- **old助手**：中栏RGB/电机切换保留；新增“自动颜色”子页独立编辑三槽、常亮预览/恢复自动。状态绑定支持13个灯效及五种颜色来源，动画页保留7种动画，收包/cache验证及重连清理同步扩展。实际文件布局已回读，Tk主线程更新流程保留。
- **验证/已排除**：原生380320项断言通过，新增三组常亮/旋转颜色相同、修改手动预设及动画8颜色不影响AUTO、修改AUTO同时影响配对、10～12预览/报警优先、命令/回包长度/坏参数、v1迁移和v2保存重载；此前真实HAL连续200帧无超时回归保留。实际Tk离线148项检查通过，含四子页、独立颜色/来源回包、单条指令、坏行及1680×1000/1560×940布局（小窗口页高276，最高内容269）。未通过改运动状态机或复制旧效果实现此功能，未以软件测试代替灯珠实测。
- **编译/产物**：主控Ninja成功，RAM59528B/128KiB、CCM64KiB/64KiB、FLASH446776B/896KiB，保留既有RWX链接告警。BIN `firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`446776B，SHA256`8b65c47995760a30c010eeaba44b49b0b7205471c7c2e5051e38c6c4028cb632`；同目录HEX SHA256`1829765be04db543e9896de95d8e61eb2081f77458185ef02e760a05ad4da41d`。BIN/HEX逐字节一致、校验和有效、最高地址0x0806D137低于预留扇区0x080E0000；源码hash匹配回归，电机BIN/参考助手hash未变，git diff --check通过。
- **未完成/待实测**：用户只更新主控并重启old助手；在自动颜色页设三种易区分颜色，修改0/1/2后开启!RGB_AUTO 1，验证对应模式待机/运动同色及旋转。失能确认后保存、重启回读检验六个颜色槽和绑定。实机LED电气/动画观感/运动并发尚未验证；本地忽略目录build/analysis/rgb保留测试与产物记录。

## 当前完成：RGB DMA完成回调、模式回包及重复查询修复（2026-10-07，已编译）

- **当前问题/授权**：用户提供21:52～21:56串口日志，表示仍有此前问题并明确要求直接修复后编译。本轮继续已有未提交RGB工作；修改主控`Core/Src/tim.c`、`Bsp/gpio/rgb.hpp/.cpp`、`UserApp/protocols/ascii_protocol.cpp`、`串口助手_old.py`及RGB方案/本文。不提交、不烧录、不连接设备；电机、参考firmware、串口助手.py、esp32不改。
- **已确认事实**：日志默认BOOT=3、RUN_INT=8、auto=1，正确绑定已在设备生效，不能继续归因于旧保存配置。RGB_DMA frames=199/timeouts=198持续升到frames=12410/timeouts=12409。`Core/Src/tim.c`把同一DMA句柄同时绑定CC2/CC4；仓库HAL `TIM_DMADelayPulseCplt`按指针先匹配CC2，`UserApp/main.cpp`回调只接受CH4，因此完成通知被丢弃。主控设置后立即回包并且old助手立即再查，模式/source仍来自上一渲染帧，造成重复输出和模式显示落后一拍。
- **最新改动**：DMA只绑定/释放CC4；`RGB::GetStatus`根据已受理配置和指定机械臂状态解析mode/source，保留实际渲染亮度与DMA计数，协议传入实时`dummy.GetRGBState()`，不从通信线程渲染。old助手所有RGB设置只发原设置指令，简单设置由固件直接回短状态，颜色/时间/动画/绑定/RESET由固件直接回完整配置；只连接与主动查询发送#RGB_GET。中栏RGB/电机切换、上电彩虹、INT旋转及显式失能保存语义保留。
- **已排除/验证**：不是新绑定未下发，也不通过放宽DMA超时或删超时计数掩盖问题。新增回归执行实际TIM2绑定语句、仓库真实HAL完成/错误函数和主控实际回调；修复前复现busy无法释放，修复后连续200帧全部完成、timeouts=0/errors=0，并验证错误恢复。原生328800项断言及实际Tk离线110项检查通过，含尚未渲染时模式/联动状态回包、完整配置回包、助手单条发送和既有布局/门禁。多数断言为重复帧/周期检查，不能当作实机验证。
- **编译/产物**：主控Ninja成功，RAM59512B/128KiB、CCM64KiB/64KiB、FLASH445704B/896KiB，保留既有RWX链接告警。BIN `firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`445704B，SHA256`b95b1f6cdf220b0cc6bea97801504908c462d5497c858f7cea94687e7cf1cbc1`；同目录HEX SHA256`1f18b6b161136ab64fb8acf0600f310e5860f54b237289492e825398f7a96051`。BIN/HEX逐字节一致、校验和有效，最高地址0x0806CD07低于预留扇区0x080E0000；测试源码hash一致，四电机BIN与参考助手hash未变，git diff --check通过。忽略目录`build/analysis/rgb`保留测试及产物核对记录。
- **未完成/待实测**：用户只更新主控并重启old助手；确认亮度/自动联动/模式操作没有追加查询和重复状态，旋转/彩虹动画流畅，分时主动#RGB_STATUS查看timeouts不再逐帧增长。灯珠电气、实际波形及运动并发仍待实机验证。此前日志storage=2/dirty=1表示存储记录无效/未保存，三单色仍可用!RGB_COLOR设定，实际DISABLE确认后!RGB_SAVE整组保存；本轮不自动清空Flash。

## 当前完成：上电彩虹、INT旋转核对及RGB/电机页面切换（2026-10-07，已重新编译）

- **当前问题/授权**：用户直接要求上电初始化改彩虹、INT运动旋转、old助手RGB与电机参数放在同一栏切换，并指出亮度应用和自动联动触发#RGB_GET。本轮在上一轮未提交RGB改动上继续，修改主控`Bsp/gpio/rgb.cpp`、`串口助手_old.py`及RGB方案/本文；保留既有工作，不提交或烧录。
- **已确认事实**：`firmware_new/dummy-ref-core-fw/Bsp/gpio/rgb.cpp`原RUN_INT绑定已为8号青色旋转，与RUN_SEQ共用旋转渲染；`dummy_robot.cpp:GetRGBState`按motionActive选择RUN_INT。未将用户观察直接归为运动状态错误；实际设备是否自动联动/保存了其他绑定尚未回读。原old助手`_rgb_send`对全部动作追加#RGB_GET，已找到重复全量回读来源。
- **最新改动**：主控默认BOOT绑定改为3号彩虹，保留2秒上电时段及已保存配置优先；INT默认青色旋转保持。old助手中栏新增“RGB / 电机参数”Notebook，右栏恢复三页运动控制；RGB状态文字适配中栏宽度。亮度、自动、开关、模式、保存回读#RGB_STATUS，颜色/渐变/动画/绑定/RESET及主动全量查询保留#RGB_GET。原串口助手.py只读参考。
- **验证**：主控Ninja重新编译成功，RAM59512B、CCM64KiB、FLASH445424B/896KiB；既有RWX告警保留。原生160029项检查通过，新增上电彩虹帧变化、INT运动旋转及启动退出验证；实际Tk离线110项通过，覆盖中栏两页互切、原电机参数可访问、亮度及自动仅短查询，以及两种窗口尺寸的三个RGB子页布局。BIN/HEX逐字节一致，最高地址0x0806CBEF；源hash一致、电机产物及串口助手.py未变，git diff --check通过。
- **最新产物**：`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`445424B，SHA256`493b389137975f7a0f682144b8540e9759d556333f4500dbe5e95707f7f27442`；同目录HEX SHA256`5c039099bc6dfe12e65589a218b794007ea73e94ad74acc9908ef9a653235608`。本地验证记录仍在Git忽略的`build/analysis/rgb`。
- **已排除/待实测**：不自动清空已有RGB保存配置，不修改其他运动模式灯效或运动完成逻辑。若设备已保存旧绑定，用中栏状态绑定设置BOOT=3、RUN_INT=8并开启自动，再实际失能确认后保存；用户实测上电、INT运行、页面和发送日志。尚未连接设备验证物理灯效。

## 当前完成：24灯珠RGB复刻及old助手同步（2026-10-07，已编译，待实机测试）

- **授权/范围**：用户认可RGB分析方案，明确允许直接执行，要求同步`串口助手_old.py`功能UI并在完成后编译，由用户实机测试。开始时Git干净。本轮修改主控RGB驱动、TIM2时序/配置存储边界、状态只读映射及协议；同步old助手、RGB方案、STOP文档查询说明与本文。电机、参考固件、`串口助手.py`、esp32未改；未提交、烧录或连接设备。
- **原实现事实**：参考`firmware/ref_core_f405/Bsp/gpio/rgb.hpp`为24灯、三组颜色、0～9效果；参考ASCII配置会即时保存。本轮修改前的新版`Bsp/gpio/rgb.hpp`为12灯、逐灯发送/忙等，`rgb.cpp`未处理ALLOff。新版控制模式仍只实现1～4，不将TRQ/SRV显示成已实现。
- **已完成驱动与功能**：`firmware_new/dummy-ref-core-fw/Bsp/gpio/rgb.hpp/.cpp`整帧24灯GRB、唯一DMA缓存、50Hz毫秒相位、静态跳帧、DMA忙/错误/超时恢复；独立0～100%亮度与开关，关灯记忆亮度，渐变可中途反向；三组颜色及彩虹、潮汐、呼吸、赛博、双心跳、旋转、闪烁。`Core/Src/tim.c`调整TIM2 ARR为104，代码计算800kHz、尾部复位约320us；实际电气待实测。
- **状态/协议**：`Robot/instances/dummy_robot.h/.cpp`和`protection_coordinator.h/.cpp`只读既有状态，`UserApp/main.cpp`由RGB线程刷新；自动关联启动、失能、1～4模式待机/运行、校准、STOP、保护报警，手动效果结束覆盖后恢复。FAULT固定红色；J8不加入普通运动完成。`UserApp/protocols/ascii_protocol.cpp`USB/UART4共用严格RGB解析器，查询分行满足62字节响应限制；STOP/校准允许精确RGB只读查询，设置/保存仍拒绝。新增RGB_SAVE互斥门禁，保存期间START/校准不能进入，原运动握手和CAN协议保留。
- **存储**：RGB显式保存仅允许实际失能、无运动/队列、无保护/校准任务/故障及pending START。`STM32F405RGTx_FLASH.ld`代码区限定896KiB，预留最后扇区11；0x080FFC00记录包含版本/CRC/范围校验，写后回读，无变化不擦写，失效回默认。单Bank擦写会停顿，断电可能丢失旧RGB配置，未实现双扇区保留。
- **old助手/文档**：`串口助手_old.py`新增RGB标签及亮度灯效、动画参数、状态绑定三个子页，支持选色、开关、自动、渐变、查询、保存和恢复默认；MCU回读更新缓存，Tk主线程更新UI，复用STOP/校准发送门禁。`docs/RGB灯环复刻方案与测试.md`记录功能、协议和实测流程，`docs/STOP软停实现与测试报告.md`同步RGB只读例外。原`串口助手.py`SHA256保持`d07d3e90968f42706c107ec038afd5109e6a67fc0a12f3c2bc3d98d13ff0d7e4`。
- **已排除方向**：不复制旧头文件内的大量效果函数；不把亮度0作为关灯记忆；不通过新任务声称消除单Bank Flash擦写停顿；不把TRQ/SRV预留灯效当作已实现控制模式。
- **软件验证**：MSVC原生回归直接编译生产RGB.cpp及提取当前协议/状态/保存/RequestStart，154180项检查通过（多数为周期断言），覆盖GRB、渐变/反向/回绕、全部动画、DMA错误/丢完成恢复、严格解析/响应长度、状态优先级、STOP查询门禁、保存互斥、实际映射Flash内存加载/CRC及写失败。实际Tk离线UI103项通过，覆盖命令、回读、门禁、坏行容错及1680×1000/1560×940状态填满后的布局；Python语法通过。Fake HAL及离线UI不能替代灯珠电气/电机并发实测。源hash与最终生产源一致，BIN/HEX校验和及逐字节一致，最高地址0x0806CBDF低于预留扇区0x080E0000；git diff --check通过。
- **编译/产物**：主控Ninja编译链接成功，RAM59512B/128KiB，CCM64KiB/64KiB（既有整体堆分配），FLASH445408B/896KiB，保留既有RWX链接告警。`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`445408B，SHA256`ec2c18dd031780f4d3c06d73def43643c9aff1f12b09f4e1bf7d57294ee7e0cd`；同目录HEX SHA256`64f1d28729f119c122bffb1e525a7776c65d481481b04e7f57d2346883b8f616`。四电机BIN哈希与上一轮已验收默认参数产物一致。本地验证记录在Git忽略的`firmware_new/dummy-ref-core-fw/build/analysis/rgb/`。
- **待实测/交付**：本轮只需更新主控，使用old助手RGB页；先核对24颗纯色/通道、40%→关→60%→开和0/1/100%、渐变反向及3～9动画，再看正常START/运动/STOP/失能/校准状态，最后失能确认后保存并重新上电回读。灯环型号、低亮度观感、实际波形及运动并发稳定性待用户确认，未宣称实机验收。


## 当前归档：校准修复及已验收默认参数提交（2026-10-07）

- **用户授权/范围**：用户明确要求“提交”，并询问firmware_new根目录新增文件用途及是否保留。本次归档目前11个待提交文件：电机校准搜索参考边界、接触确认期限、首次失败冻结快照；主控六轴坐标确认后1000ms等待及J7/J8并行；CAL_STATUS只读诊断；最终J7/J8默认、old助手和3份文档。未增加新的固件行为、未删除文件、未烧录或操作设备、未推送远端。
- **提交前状态**：五目标构建/产物以及默认配置、EEPROM与old助手核对已在上一节完成，当前提交只补本文归档说明，不重复运行固件测试或修改参数。仓库本地身份为Liu3Beng-A / liu3beng@icloud.com；不使用agent身份。提交标题围绕校准实际行为和最终验收参数，具体提交哈希以本轮git log/回复为准，避免在提交自身的文档中写不可预知的哈希。
- **根目录文件用途（均须保留）**：`firmware_new/stop_protocol.h`定义0x0A软停/解锁/保持使能/失能动作、phase和token回执；`firmware_new/motion_completion_protocol.h`定义0x23目标确认、实际到位/判稳标志、nonce及节点容差；`firmware_new/protection_protocol.h`定义堵转/回退/校准的CAN事件、参数、事务、阶段、原因和诊断快照。它们都是已经Git跟踪的共享生产头文件，放在两个固件项目的共同父目录，避免主控/电机复制协议产生不一致，不是构建临时文件，也无需单独烧录。
- **引用已验证**：电机`dummy-motor-fw/Ctrl/Motor/motor.h:7–8`及主控`dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.hpp:6–7`直接包含stop和motion_completion；电机`dummy-motor-fw/UserApp/protection_config.h:3`及主控`dummy-ref-core-fw/Robot/instances/protection_coordinator.h:4`包含protection。删除任一共享头会导致相关源码编译缺依赖；复制或迁移firmware_new时应保留父目录与两个项目的相对结构。
- **后续边界**：本轮只按用户授权归档已完成校准工作和说明共享头文件用途。已记录的退让/复触超时耦合、错误名混用、LIVE标签残留及最少Flash余236B仍按前节保留，不因为提交而宣称已修复。未来新增功能由用户另行决定。

## 当前完成：J7/J8校准默认更新为用户最终实测值（2026-10-07，已编译，未提交）

- **授权/证据**：用户20:03:20反馈CALIBRATION ok、Disabled ok及自动保存完成，随后START、HOME约4395ms、RESET约4402ms正常；20:04:22～25回读完整CAL_GET7/8，明确要求“把固件中默认参数改成最后查询的值，测试已经没有问题了”。此组还调整速度/加速度/退让/复触容差，不作为上一轮单参数试验结果；整组通过是用户实测反馈，不宣称此前位置收敛偶发超时唯一根因已经找到。
- **最终默认（电机轴单位）**：J7 SEEK/RETURN=5/5r/s、RETOUCH=.5r/s、ACCEL=3r/s²、CURRENT=2.5A、CONTACT_RATIO=.3、SPEED/ERROR=.02/.02、TRIGGER/STARTUP=100/100ms、MAX_TRAVEL=101r、SEEK/RETURN/RETOUCH_TIMEOUT=220000/120000/5000ms、BACKOFF=1r、REPEAT/POSITION_TOL=.05/.005r、CONTACT_WINDOW=1r。J8对应2/2、.5、5、1A、.2、.02/.02、100/100、6r、30000/15000/5000ms、.5r、.1/.01r、5×16/360r（回读.222222）。完整单位/范围表已同步方案第13.3节。
- **生产改动与作用**：`firmware_new/dummy-motor-fw/UserApp/protection_config.cpp:52–66`只改J7/J8校准Defaults，保留J1～J6既有初值、普通堵转、Set/Validate/Load/Save及存储布局。`串口助手_old.py:49–69/1271–1278`CAL编辑初值对应J7表和J8覆盖表；既有节点回读缓存优先，发送/保护锁/线程和界面布局不变。更新`docs/堵转保护与自动回退及校准方案.md`当前首节及参数表、`docs/堵转误报修复与实机测试流程.md`当前首节和本文；没有改主控、STOP状态机、普通完成/同步、PID、总电流上限、参考目录、esp32或串口助手.py。
- **约束与常规细化**：J7校准默认电流取min(2.5A,节点总限流)，J8取min(1A,节点总限流)，避免旧总限流较低时默认整组校验不合法，未提升总限流。J8 CONTACT_WINDOW保留原精确5×16/360换算，不将串口显示的六位小数当作重新量化标准。接触三条件AND/连续100ms、参考结束后1000+100=1100ms有限窗口、真实到位与50ms稳定、六轴后1000ms再并行、START busy不挂起、STOP/DISABLE可取消、全成功后失能保存均未改。
- **配置优先级/保存**：合法EEPROM仍优先于新默认，不迁移覆盖旧值、不自动连接设备或写参数。`Ctrl/Motor/stall_protection.cpp:898–911`成功校准保存当前整组参数和坐标，未单独CAL_SAVE仍可能保存试验值；新固件不会强制恢复本轮默认。用户实机当前已完成保存，正常情况下不需为默认值更新反复校准或整片擦除。J7 RETOUCH_TIMEOUT也作用首次退让、错误名RETURN_TIMEOUT混用，及此前LIVE标签残留问题继续记录，未顺手修复。
- **软件验证**：复用实际生产配置/EEPROM测试，更新已变更默认期望，209项断言通过，覆盖18项J7/18项J8配置、1～8节点合法性、低总限流电流封顶、NaN/Inf/范围/整数拒绝、保存真实Flash回读及旧合法EEPROM值仍优先。实际old助手刷新函数40项通过，18键分别与回读值一致、缓存回读优先及全文件语法通过；普通STALL编辑初值未变。未把本轮软件核查称为新的物理限位测试。
- **编译与产物**：四电机Ninja重新编译链接成功；主控Ninja无待构建工作，主控BIN哈希保持此前值。五组BIN/HEX校验和与逐字节一致、95KiB电机应用区、原串口助手哈希、限定修改范围、配置校验/EEPROM加载保存源未改、git diff --check通过。四电机RAM10704B；FLASH97044/97044/97036/96996B，最少余236B。已有RWX链接告警保留，资源仍在限制内；未为余量改无关优化或分区。检查记录在Git忽略的`firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/cal_defaults_20261007_artifacts.json`及配置测试结果。
- **当前状态/下一步**：用户已认可最终参数组实测，默认落地与构建核对完成；按需烧录J7 motor-57及J8 motor-gripper，主控无需因本轮默认修改更新，保留EEPROM和编码器表。未烧录/连接设备、未提交commit，等待用户后续指令；旧参数建议与待测试文字按历史阅读，以本节为当前状态。

| 对象 | BIN（同目录有HEX） | 字节数 | SHA256 |
|---|---|---:|---|
| 主控（未变） | `firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin` | 434760 | `2a1fa48ea689cfc26dd4f7274196312f082a6c60b975c7485c92943c94588eee` |
| motor-35 | `firmware_new/dummy-motor-fw/build/motor-35.bin` | 97044 | `e75e36ef9decab103e9045bbf70f86ecdcb481bf0040451221d2ccfa3e7521a7` |
| motor-42 | `firmware_new/dummy-motor-fw/build/motor-42.bin` | 97044 | `e75e36ef9decab103e9045bbf70f86ecdcb481bf0040451221d2ccfa3e7521a7` |
| motor-57 | `firmware_new/dummy-motor-fw/build/motor-57.bin` | 97036 | `41dc86381bf340cf49eeb1e7239f53f2f3329eface874e2bb7b9cf8ef2689ecc` |
| motor-gripper | `firmware_new/dummy-motor-fw/build/motor-gripper.bin` | 96996 | `f2c69eb4bb1c030aa0fca426a06da3d0072ffd6560efbf9993f9e37aae3cf08e` |

## 当前建议：用户要求参数验证，单独试J7位置容差（2026-10-07，尚未实测）

- **范围**：用户询问应调整什么参数继续测试；本轮只读校准完成/参数验证/保存路径，按AGENTS 5.2更新本文。未改固件或配置文件、未写实机参数/运行设备、未编译/提交。下列为用户手动验证建议，不能记作已经执行或通过。
- **单变量**：保持既定校准姿态，确认八轴失能后，仅`#CAL_SET 7 POSITION_TOL_R 0.005`，等确认再`#CAL_GET 7`核对。J7从.001r=.005mm放宽为.005r=.025mm，可容纳前次约.0179mm参考跟随残差；此项同时影响CAL_BACKOFF和CAL_RETURN实际到位，不能称为仅退让容差。接触电流比例/速度/误差/持续时间、真实50ms稳定窗口、重复接触点容差、速度/电流/加速度和超时全部保留；不保证一定通过。
- **目的/流程**：只进行一次受控CALIBRATION，记录J7 phase10首次退让到phase11复触的时间、完整阶段输出及最终成功/失败，检验容差放宽下的表现。先前.001已出现成功，一次新成功不能证明根因或长期重复性；不自动要求多次硬限位重试。失败/异常先DISABLE等确认，然后读取CAL_STATUS 7 LAST；若第一失败来自J8再读8 LAST，各等完整end。本轮失败后不继续盲目加大容差/延长限位停留。
- **保存语义已核实**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:898–911`校准坐标保存复制整个当前protectionSettings，`UserApp/protection_config.cpp:137–154`与坐标一起写Flash；主控`Robot/instances/protection_coordinator.cpp:703–720`在全部实际完成/失能后自动请求。因此不发CAL_SAVE不等于试验仅RAM：成功会自动保存.005，失败未保存时本轮RAM值仍存在。需要恢复时，在失能状态SET7 POSITION_TOL_R .001并CAL_SAVE7，等保存确认再GET7核对，不必为恢复参数重新跑校准。
- **后续边界**：当前试验是用户主动选择参数验证后的推荐，不撤销上一节“现有证据不能认定.001必须改变”的归因边界。若.005仍首次退让超时，应增加目标误差/稳定窗口进度等取证，另提文件用途和授权；不在本轮擅自修改STOP、CAL完成或超时逻辑。

## 当前分析：19:50同参数校准成功，J7退让确认存在重复性差异（2026-10-07，仅分析）

- **范围/授权**：用户补充19:48:46～19:50:41完整成功日志及CAL_GET7/8，询问未改参数为何成功。本轮只读当前源码和前次失败附件，按AGENTS 5.2更新本文；未修改固件、配置、其他文档或助手，未提交/编译/操作设备。保留已有未提交改动。
- **实机回读回答前次待确认**：J7 POSITION_TOL_R仍.001、BACKOFF_R=1、RETURN_RPS=1、ACCEL=10、RETOUCH_TIMEOUT_MS=5000、RETURN_TIMEOUT_MS=120000；J8 POSITION_TOL_R=.006，保持上一轮用户已设置并CAL_SAVE的值。用户本轮没有调整参数，不能把这次成功归因于此前建议的J7 .005；该建议未被执行。J7 CAL阶段只更新坐标、保护位置范围和方向，源码未自动改CAL_POSITION_TOL_R，见`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:639–653/783–805`。
- **两次J7首次退让不同**：前次phase10 19:47:16.661→phase14 21.666，约5.005秒失败；本次phase10 19:49:02.663→phase11 04.788，约2.125秒通过。前次LAST phase10、v_ref0、跟随差-.00357422r≈-.0178711mm，绝对值超.001r=.005mm位置容差；本次phase11只能证明真实完成门槛曾通过，没有成功时的目标误差/稳定窗口数值，不能编造误差0或确认整个两次过程的波形。前次快照也不能证明其此前从未短暂进入过容差。
- **完成机制未改变**：`stall_protection.cpp:888–894`要求参考速度0、参考位置准确到内部目标、真实编码器误差不超过CAL_POSITION_TOL_R，并通过`:479–488`连续50ms稳定（实际位置峰峰≤8内部单位、平均有符号估计速度≤.01电机r/s、反馈有效）。J7峰峰8单位约.00078125mm；任一资格失效或窗口位移超限会重新计稳定窗口。`:762–769`退让仍使用复触5秒期限，并错误共用RETURN_TIMEOUT名称；本次2.125秒内通过即可进入复触，前次到期没有通过则失败。不是电流/接触TRIGGER参数自动变化，也不是规划器走完被直接算完成。
- **同设置不等于同反馈过程**：`:783–791`每次第一接触减速判稳后，从当次实际编码器位置捕获firstContact并生成退让目标；`:664–670`规划器以当次realPosition/estVelocity起步；`Ctrl/Motor/motor.cpp:558–569`捕获时清积分后用当时focCurrent初始化保持积分。这些运行状态未由相同参数强制固定。负载、摩擦、回差、接触预紧、控制状态或编码器微动均可能令末端收敛/稳定资格不同，但现有两份阶段日志和一个失败快照不能确定唯一原因，不断言机械问题或PID/积分实现存在已证实bug。
- **整轮确实完成**：本次J8 phase12 19:48:56.018→phase13 19:49:05.121约9.103秒；J7 phase12 19:49:07.128→phase13 59.764约52.636秒。随后八轴失能、八节点phase16保存确认、phase0结束，19:50:01.013返回CALIBRATION ok。全部pending到最终ok约74.234秒。成功是既有严格门槛实际通过并完成保存，不是超时放行；J8已在首次退让/复触/返回后等待J7，没有新J8失败。
- **修正归因/当前建议**：前轮失败证据支持“失败时尚未满足完成门槛”，不足以证明J7 .001必然不可达或必须改成.005。本次证明.001可通过，但单次成功不能证明重复性问题已解决。先保留当前参数，不因一个失败/成功继续调电流、PID、位置容差或要求反复顶限位。若后续决定定位，应先明确增加退让目标误差、参考结束时刻、稳定进度/峰峰/清零原因的受限取证，另列文件用途获授权；独立退让超时及准确错误名仍是已记录、未实施的软件方向。

以下章节为之前轮次当时的分析；前次J7参数待回读及.005临时试验建议，以本节新增证据和当前建议为准。

## 当前分析：19:47 J8到位通过，J7首次退让phase10超时（2026-10-07，仅分析）

- **证据/范围**：用户提供`C:/Users/Liu3Beng/.codex/attachments/136c81bb-d139-4004-85cd-a305dae237cf/已粘贴的文本.txt`，描述J7回退完成时出现问题。本轮读取完整日志及实际源，仅按AGENTS 5.2更新本文，不修改固件/参数/其他文档/助手、不提交/编译/操作设备。用户的肉眼完成描述和软件资格需分别核对，不能凭参考速度0判定校准成功。
- **J8阶段已通过**：用户19:46:41.860将J8 POSITION_TOL_R设.006并在43.766 CAL_SAVE成功保存参数（此前建议只临时试，实际用户选择了保存，须如实记录）。本轮J8 phase9→10→11→12于19:46:59.925→phase13于19:47:09.002，返回约9.077秒，已完成并保持等待J7。此次改善与单容差调整一致，不能证明所有条件下都已验收，也不能把它当PID优化试验。CAL_SAVE参数保存与整轮校准坐标结果保存不同：随后整轮失败saved/pending000，未进入CAL_SAVING。
- **J7实际失败阶段**：19:47:16.661从CAL_SEEK9进入CAL_BACKOFF10，21.666约5.005秒后CAL_FAILED14/RETURN_TIMEOUT；没有phase11/12，不是最终从+250返回0的失败，也不是普通STALL自动回退。J7 LAST task57935/phase10、fb/enabled/intent均1、v_ref0，说明首次接触后退离限位的内部动作未通过实际到位稳定确认。
- **本次已确认快照**：J7 LAST err=-.00357422r≈-183内部单位；actual_travel=-22.952r、ref_travel=-22.9556r，实际位置比参考点沿正电机退让方向更远约.00357422r×5=.0178711mm。J7寻端电机方向负、退让方向正，见`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:689/787–791`；本次应解释为退让略超参考点，不能说尚欠退约.018mm。跟随误差是相对参考的反馈差，不是原始总退让距离；相对行程仍以首次searchStart计，不把22.952r当作退让了114.76mm。
- **到位/判稳门槛**：`:888–894`CAL_BACKOFF和CAL_RETURN共用实际CAL_POSITION_TOL_R以及`:479–488`50ms稳定。J7本轮尚无CAL_GET7，默认/此前设置.001r约51单位=.005mm，若仍是该值，失败点残差约容差3.57倍，actualArrival无法满足（参考若没准确到internalTarget也不满足）。稳定另要求实际峰峰≤8单位，J7对应约.00078125mm=.78125µm、平均有符号实际速度≤.01r/s；本快照没有稳定窗口峰峰/进度，不能证明只要放宽位置容差就必然通过，也不能拿单个v_raw0证明已连续50ms稳定。
- **超时耦合及错误命名已确认**：`:762–769`CAL_BACKOFF使用CAL_RETOUCH_TIMEOUT_MS，而CAL_RETURN才用CAL_RETURN_TIMEOUT_MS；非搜索的CAL阶段超时统一返回RETURN_TIMEOUT。日志phase10后5.005秒与此前J7复触5秒期限一致，具体值仍需回读。改RETURN_TIMEOUT_MS不会延长此次退让期限；延长RETOUCH_TIMEOUT会同时影响退让和复触，不能把它说成独立退让参数。这两个设计问题已有历史记录，本轮未擅自实现独立字段/原因号。
- **不是再次受阻或命令限流饱和**：LAST I_cmd=.242/I_filt=.25<I_thr=.75A，err绝对值.00357422<.02r，unmet5，timer0、peak/last23.2ms<再次受阻100ms，原因RETURN_TIMEOUT不是REBLOCKED。命令上限2.5A、当时输出约.242A，快照无输出饱和证据，不据此提高电流/改接触TRIGGER。detect0表示非寻端接触阶段，并不排除退让内部再次受阻保护；need100是REBLOCK_MS，非到位时长。
- **LAST与LIVE不能混用**：随后LIVE phase14/err0、参考与实际行程相同，是失败STOP将当前编码器位置捕获为新的保持目标，见`Ctrl/Motor/motor.cpp:557–571`及软停处理；不说明原退让目标已按要求完成，更不意味着校准失败解除。LIVE failure_reason=NONE为运行诊断的reason入参，不覆盖LAST冻结RETURN_TIMEOUT；根因以首次失败LAST为准。
- **待回读及建议**：已异步请求失能状态只读CAL_GET7，不重新校准，确认POSITION_TOL、BACKOFF、RETURN_RPS和RETOUCH_TIMEOUT。若实际J7容差仍.001且用户确认退让已完成，建议由用户只临时`#CAL_SET 7 POSITION_TOL_R 0.005`并回读，先不CAL_SAVE、不同时改PID/电流/速度/超时；.005r约.025mm，可容纳本次约.018mm残差，但也作用最终返回且50ms稳定仍保留，效果待实测。若本就已≥该容差，则不能沿用位置容差不足解释，应看参考到目标/实际稳定及反馈过程；不盲目加宽或反复顶限位。
- **后续软件方向（未获本项改动授权）**：退让应有清楚的独立超时语义及阶段错误名，CAL到位容差/稳定门槛需按地轨与夹爪实际单位评估；如需增加target误差/稳定进度的取证，另列文件用途并获授权。当前只记录问题，不更改既有STOP判稳或普通运动完成流程。

## 当前分析：19:43回读确认J8严格到位容差与低增益PID，建议单参数验证（2026-10-07，仅分析）

- **范围/新增证据**：用户补充19:43 CAL_GET 8和GET_PID 8；本轮只读实际设置/输出路径，按AGENTS 5.2更新本文，不改固件/配置/其他文档/助手，不提交、编译或写设备参数。用户尚未说明19:24返回时是否基本全开或中途停止，也尚未反馈POSITION_TOL_R=.006的试验结果，不把建议当成已执行。
- **实际参数已确认**：J8 SEEK1、RETURN.5、RETOUCH.5电机r/s，ACCEL5r/s²、CURRENT1A、CONTACT_RATIO.2、SPEED/ERROR .02/.02、TRIGGER100、STARTUP100、MAX_TRAVEL6、SEEK_TIMEOUT30000、RETURN_TIMEOUT15000、RETOUCH_TIMEOUT5000ms、BACKOFF.3、REPEAT_TOL.01、POSITION_TOL.001、CONTACT_WINDOW.222222r。返回速度/15秒总期限及.001容差现在有实机回读，不再仅引用默认或历史设置。
- **19:24失败与容差核对**：LAST跟随残差绝对值.005r=256内部单位；实际容差.001经Steps整数截断为51单位，约5倍差距。源码`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:888–894`完成要求参考到内部目标且真实误差进入容差，再通过50ms稳定；本失败周期不具备完成资格，但没有历史完整波形证明此前从未进入容差或说明稳定窗口哪项反复失效。按静止起点、全开100映射4.44444圈、RETURN.5/ACCEL5估计参考单程约8.99秒，15秒总期限名义上留约6秒收敛/判稳余量；这是理想规划估算，不是测得参考实际停止时刻，也不证明延长超时一定无效。
- **PID路径与差异**：实机GET_PID 8为kp50/kv50/ki3/kd2；当前gripper源码默认`firmware_new/dummy-motor-fw/UserApp/main.cpp:66–71`为200/80/300/250，`:106–109`启动复制至DCE实际参数。`UserApp/protocols/interface_can.cpp:217–238/334–371`设置同时更新运行DCE与boardConfig，回读取boardConfig；校准返回MODE_COMMAND_POSITION走`Ctrl/Motor/motor.cpp:141–179/419–447`DCE位置闭环，没有独立CAL PID覆盖。当前Kp为默认四分之一、Ki为默认百分之一，可作为小误差收敛能力较弱的候选方向；不把增益比例称为实机速度比例，不据此判定PID是唯一原因。默认值不等于已针对这套实物调好的值，EEPROM有效时优先于默认。
- **建议的单参数试验（由用户操作，未实施）**：若本次返回已基本全开，保持实际PID、RETURN.5、限流1A、TRIGGER100及15秒超时不变，只临时`#CAL_SET 8 POSITION_TOL_R 0.006`并回读，不先CAL_SAVE。容差约307内部单位/.135模型开度，可容纳本次.005r残差；该参数同时用于CAL_BACKOFF与CAL_RETURN实际到位，50ms稳定、参考到目标、寻端/复触接触判据和重复性门槛不改。不能保证通过；若中途明显未返回，先不要靠放宽容差掩盖，应另核对实际运动与闭环。若仍失败，失能后取CAL_STATUS 8 LAST及完整阶段日志，不自动继续加电流/速度或反复放宽条件。
- **PID写入语义需保持准确**：当前主控`UserApp/protocols/ascii_protocol.cpp:399–417`的SET_PID调用四个SetDce接口，而`Robot/actuators/ctrl_step/ctrl_step.cpp:422–477`四帧data[4]=1要求保存；并非提供无保存的PID试调入口。本轮不建议直接发送整组默认PID，也未调用任何PID写工具。后续如决定PID试验必须明确这个已有持久化行为，避免把它描述为仅RAM试调。
- **待确认**：用户实际开度观察和单容差验证结果；先将本次参数与证据留档，不修改完成流程/STOP状态机。若未来需要分别配置退让与返回容差或扩充稳定窗口快照，先另提文件用途并获授权，不因本轮分析自行扩展实现。

## 当前分析：19:24 J8复触已通过，返回阶段15秒到期失败（2026-10-07，仅分析）

- **证据/范围**：用户贴出19:23～19:24完整CAL日志及J7/J8 LAST。本轮按实际源码核对返回完成条件与故障分类，仅按AGENTS 5.2更新本文，不改生产代码/参数/其他文档/助手，不提交、编译或操作硬件。上轮确认窗口修复已编译交付，此次日志是后续实机反馈，不能继续把新RETURN_TIMEOUT当成复触CONTACT_NOT_CONFIRMED。
- **阶段已推进**：J8 phase9 19:24:02.003→phase10 07.197→phase11 08.523→phase12 09.900，说明本轮首次接触、退让、第二次接触、接触后实际稳定及重复点检查均已通过，闭合坐标已在RAM生效并启动返回100；见`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:783–805`。随后phase14/RETURN_TIMEOUT 24.880，返回阶段约14.980秒，与此前CAL_RETURN_TIMEOUT_MS=15000相符；不是复触时间不足。仅证明此次复触已通过，不推定所有其他参数下均已验收。
- **返回真实完成条件**：`:888–894`要求参考速度0、参考位置等于internalTarget、实际编码器到目标误差≤CAL_POSITION_TOL_R，再通过`:479–488`实际50ms稳定（位置峰峰≤8内部单位、平均有符号速度≤.01电机r/s且反馈有效），才捕获固定保持点并到CAL_DONE。`:762–769`返回总期限包含规划、实际跟随收敛及稳定判定，超时先失败；没有把参考停止直接认定为到位。
- **本轮LAST关键事实**：J8 task51084/phase12/intent/fb/enabled均1、v_ref0，err=-.005电机圈；actual_travel=-4.13992r、ref_travel=-4.14492r相差.005r。跟随残差对应1.8°电机轴、.1125模型开度单位（减速16），不能表述为.1125mm或物理测得开度。此前回读/默认CAL_POSITION_TOL_R=.001r对应.0225模型开度；若实机本项未改，本周期不能满足actualArrival：参考若已到目标则实际残差为容差5倍，参考若未到目标则参考条件本身不成立。快照没有internalTarget/实际目标误差或稳定窗口峰峰值，不把v_ref0单独视为参考已准确到目标，也不证明15秒期间完全没有进入过位置容差。
- **不是再次受阻计时触发**：LAST I_cmd=-.063A/I_filt=.061<I_thr=.2，err绝对值.005<接触/再次受阻ERROR.02，v_filt=.00283203≤.02，unmet5；timer0、peak/last15.5ms<need100，故障原因明确RETURN_TIMEOUT而非REBLOCKED。`:175–193`detect0只表示当前非SEEK/RETOUCH接触检测阶段，RETURN仍执行再次受阻保护；need_ms100在此为REBLOCK_MS，不是返回完成时长，更不能通过继续改CAL_TRIGGER来修复返回到位。返回判稳使用独立50ms窗口，单个v_raw=-.00703125或v_filt低不能证明该窗口已通过。
- **电流/PID归因边界**：实际返回继续由`Ctrl/Motor/motor.cpp:141–179/419–447`正常位置闭环DCE输出，限流不会强制输出指定电流。失败瞬间电流命令幅值.063A远低于1A命令上限，不能据此认定限流不足或要求提高电流；这也不是电源/绕组实测值。PID收敛、摩擦/回差/微动和完成容差是否适配均待核对，不能从一个快照断言唯一硬件原因或编造PID参数。RETURN_RPS/当前POSITION_TOL/实际PID本轮未回读，不能假设还在用默认速度或据规划模型保证多少秒能实际到位。
- **J7与保存**：J7 LAST仍phase9/v_ref=-1、failure_reason=CANCELLED_STOP，因J8失败取消整轮，不是J7自身限位故障。主控`firmware_new/dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:669–705`先要求两轴CAL_DONE才进入DISABLING/SAVING；本轮saved/pending均000、没有CAL_SAVING，未启动不完整校准保存，DISABLE37.099正常完成。两节点at_ms为独立上电计时，不跨节点相减分析异常。
- **待补充/下一步**：已通过异步问题请求用户保持失能，只回读CAL_GET 8及GET_PID 8，并说明本次返回时是否基本全开或中途停住；无需重新跑CAL。待确认实际RETURN_RPS、POSITION_TOL_R、RETURN_TIMEOUT_MS与PID后，再评估返回时间预算、末端实际收敛和夹爪容差/稳定要求是否适配。当前不盲目延长超时或放宽容差，不擅自修改完成判定；若要修复，先列文件与作用并获授权。原退让共用复触超时/错误命名另项继续保留。

## 当前完成：校准接触确认期限修复，编译及离线回归通过（2026-10-07，待烧录）

- **授权/范围**：用户在18:58失败重新分析后明确“执行修复”，授权上轮1000ms独立收敛余量 + 连续触发时间方案。生产源码本轮只修改`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp`；同步`docs/堵转保护与自动回退及校准方案.md`、`docs/堵转误报修复与实机测试流程.md`、本文。其余已有未提交改动保留，按本轮前置hash核对未变；不改主控、默认配置、协议、EEPROM格式、助手、参考firmware或esp32，不提交、不连接/烧录/写硬件参数。
- **已确认事实/改动**：电机`:875–884`仅在CAL_SEEK/CAL_RETOUCH的参考停止固定后，把确认期限由startup+trigger+5×filters+50ms改为HZ+contactTriggerTicks，即1000ms+CAL_TRIGGER_MS（20kHz）。TRIGGER100对应1100ms，默认150对应1150ms；持续判定`:825–847`、失效清零、接触后的减速/实际稳定/复触点比较、实际返回、总阶段超时`:762–769`、行程/反馈/租约保护及STOP/DISABLE均保持。候选重启不会刷新参考结束期限；条件满触发时长即接受，不强制等待期限。
- **实施决策/与前案一致性**：1000ms用于单独的收敛余量，再加当前触发时长；不是把全部等待固定成1000ms，不新增可调参数/CAN/EEPROM字段。这是上轮“预留约1000ms、再加连续触发时间”的具体落地。修复只提供有限等待机会，若速度一直不合格仍失败；不继续缩短TRIGGER、不放宽速度/误差/电流、不额外推进参考或扩大搜索行程。退让与复触期限共用/RETURN_TIMEOUT语义歧义仍是另项记录，未扩大范围。
- **旧版本复现与新版本对照**：使用真实motor.cpp/stall_protection.cpp/motion_planner.cpp、受控编码器/电流命令输入及配置，构造参考结束后330.85ms开始最后一段连续有效候选。旧代码在400.05ms时冻结69.2ms/100ms并CONTACT_NOT_CONFIRMED，22项断言通过，贴合实机失败点69.15ms的时序关系；新代码同输入在430.85ms连续满100ms立即接受，不等1100ms。此为依据日志设计的输入，不是声称复现未采样的真实机械/速度波形。
- **验证**：专用CAL确认期限3192594项断言通过，覆盖J7/J8、SEEK/RETOUCH、TRIGGER20/100/150/2000ms、提前有效立即接受、连续重计、反复5ms短候选有限失败、完全无接触失败、固定参考/无额外实际行程、实际越界、总阶段超时先到、反馈门禁及STOP/DISABLE在延长窗口内取消。原CAL边界与首次快照798869、电机保护/完整理想校准7040、主控STOP/坐标/并行/取消/诊断24732项通过；大量数字来自周期断言，不能作为实机验收。测试生成器、案例、源hash、构建日志和产物校验在忽略目录`firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/cal_confirmation_wait_*`以及两build的cal_confirmation_wait_build.log；没有新增生产测试依赖。
- **构建与产物**：主控Ninja无待构建工作、BIN与前轮完全一致；四电机Ninja均重编译链接成功，RAM10704B、原95KiB区最少剩452B，原RWX警告未顺手改。五组BIN/HEX校验逐字节相同，原`串口助手.py`SHA256仍D07D3E90968F42706C107EC038AFD5109E6A67FC0A12F3C2BC3D98D13FF0D7E4；测试源hash与最终源一致，git diff --check及生产修改范围通过。
  - 主控`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin` 434760B / SHA256 `2a1fa48ea689cfc26dd4f7274196312f082a6c60b975c7485c92943c94588eee`。
  - 35/42 `firmware_new/dummy-motor-fw/build/motor-35.bin`、`motor-42.bin`各96828B / SHA256 `6c66e13dd5f258a0d8297af198804162c15516f87023e794a096e84b0fbed14d`。
  - 57 `firmware_new/dummy-motor-fw/build/motor-57.bin` 96820B / SHA256 `22ec81465ac3b0cdb718ad6258442f4d014ff0e761367a160a41bda8d1dfe9ad`。
  - gripper `firmware_new/dummy-motor-fw/build/motor-gripper.bin` 96780B / SHA256 `874fa3f47ae2db2a382a4d3c6bae706da5105d33bab88fdd96b51a3a7fe05867`。
- **待用户实测**：本轮只需烧J7 motor-57及J8 motor-gripper，保留EEPROM/磁编码器表，已有配套主控不必重烧；先失能回读CAL_GET 7/8、保持此次100ms和其余参数，执行一次受控CAL。重点J8复触phase11能否到phase12/13，以及两轴均返回后全八失能保存/CALok。校准中START busy不挂起；若失败先DISABLE，再各读CAL_STATUS 8 LAST/7 LAST完整冻结记录，取证前不重启/重跑。生产测试尚未烧录，不能宣称实际接触已验收。

## 当前分析：18:58 TRIGGER=100ms已生效，复触连续候选仍被有限确认期限截断（2026-10-07，仅分析）

- **范围/证据**：用户提供`C:/Users/Liu3Beng/.codex/attachments/8598fa36-f169-4109-99fa-49d517efacc0/已粘贴的文本.txt`，要求重新分析降低TRIGGER后仍失败。本轮只读完整日志与实际电机源，按AGENTS 5.2仅更新本文；不改固件、参数、助手或其他文档，不提交/编译/连接设备，不要求再次撞限位。上一节提出的确认窗口修复尚未实施，用户此次仅修改实机参数。
- **100ms明确已生效**：J8在18:57:38.812 SET成功、40.338 SAVE成功；J7也SET/SAVE成功。失败冻结快照J8和取消快照J7均显示need_ms=100，排除未保存/仍使用150ms的解释。J8 phase9于18:58:18.217开始，23.419到phase10、24.657到phase11、25.844报CONTACT_NOT_CONFIRMED，复触约1.187秒；首次接触和退让已通过，本轮没有进入phase12。STOP locked26.086，DISABLE31.985正常完成，saved/pending仍000。
- **连续计时证据**：J8 LAST task49227/phase11/gateELIGIBLE，intent/fb/enabled/detect均1，unmet0；I_filt=.894≥I_thr=.2A，v_filt=.0179883≤.02r/s，err=.0423828≥.02r，在失败周期三个条件均满足。当前连续timer69.15ms<100ms，阶段最长peak75.6ms<100ms；clearCONDITIONS/detail2说明此前非零候选因速度条件失败清零。peak大于当前timer证明此前有更长候选已中断；last_ms=.65仅是最近一次被清零的非零候选长度，不是最长候选，也不能据此声称只发生一次清零。最后仍差30.85ms，但仅在后续条件持续成立时才可能补足，不能保证再等31ms就成功。
- **源代码与本轮结果一致**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:199–205/825–847`严格三条件AND、失败立即清零且不累加间歇候选，当前快照没有证明计时器不运行。`:875–884`在固定参考softPosition==internalTarget、softVelocity==0后，独立计数确认期限，即使当前候选仍有效也会因到期报CONTACT_NOT_CONFIRMED。本轮ref_travel=.34r、actual_travel=.297617r、v_ref0，说明参考已结束、实际仍有.0423828r跟随残差。没有完整速度波形，不能在真实微动、估计速度波动、机械弹性之间认定唯一原因；只确认速度条件曾失效。
- **为什么缩短TRIGGER没解决**：源码期限换算为CAL_STARTUP_MS+CAL_TRIGGER_MS+5×(CURRENT_FILTER_MS+SPEED_FILTER_MS)+50ms（20kHz，末项1000周期）。按原有STARTUP100及默认共享滤波10/20ms，TRIGGER150时期限450ms，改100后期限400ms，要求和等待截止同时缩50ms，两者相差的收敛余量仍300ms。滤波值本轮未回读，400/450ms是按上述配置计算；“缩短TRIGGER使期限等量缩短”不依赖滤波具体值。本轮69.15ms与上一轮107.25ms来自两次不同反馈轨迹，不能按相减50ms预测。按继承RETOUCH=.5、ACCEL5、参考.34r估计参考运动约.78秒，再加.4秒约1.18秒，与1.187秒日志接近，属模型对照，不是物理触碰的精确计时。
- **结论与推荐修正**：本次继续支持“参考结束后的确认期限对实际收敛/速度条件反复失效不够适配”，而不是100ms没落地或当前电流/误差条件不足。存在两个关联因素：速度失效令连续候选重新开始、有限期限又在最后有效候选满100ms之前终止。上一节至少1秒确认窗口建议仍未实施；进一步推荐把实际收敛余量单独定义，初步取1000ms，再留足配置的连续触发时间，阶段总超时继续作为硬上限，避免只改TRIGGER去补收敛时间。参考保持固定、不额外推进搜索，不改变电流/速度/误差判据、不累计间歇候选、不允许候选反复续期；连续满TRIGGER后立即进入接触减速/实际稳定/重复点检查，不固定等待整段期限。延长窗口仅提供机会，不保证速度必能持续满足，也不代表整轮已验证成功。当前不优先放宽SPEED或继续缩短TRIGGER；后续若仍受速度判据影响，再依据快照/必要波形评估迟滞，不擅自改普通堵转检测。
- **J7联动与未完成**：J7 LAST phase9、v_ref=-1r/s，failure_reason=CANCELLED_STOP，因J8失败被整轮取消，不是J7自身限位检测失败。两个at_ms属于独立节点时钟，不能跨节点相减归因。后续拟改电机`stall_protection.cpp`的校准确认期限并同步既有方案/测试/状态文档，需先列文件用途并获用户确认；无需新增CAN、EEPROM或串口参数。退让与复触期限复用/RETURN_TIMEOUT歧义仍另行记录，不扩大本项范围。离线验证应覆盖有效候选完成、速度失效清零、完全无接触/反复短候选有限到期、阶段总超时及STOP/DISABLE打断；本轮未执行源代码修改或构建。

## 当前分析：18:50复触已有有效候选107.25ms，有限证明窗口先到期（2026-10-07，仅分析）

- **范围/证据**：用户贴出18:49～18:50完整串口日志及J7/J8 LAST；本轮读实际电机源、当前默认/参数换算、已有状态文档，按AGENTS 5.2仅更新本文，不改固件、配置、助手或其他文档，不提交、编译、烧录、连接设备或要求再次撞限位。此次证据足够定位确认窗口与连续计时的关系，不再用上轮缺快照的结论代替当前快照。
- **本轮参数与阶段**：J7成功SET/SAVE SEEK/RETURN1、RETOUCH.5；J8成功SET/SAVE SEEK1、BACKOFF.3。其他值依据前轮保存历史，而非本轮GET确认；LAST直接证明J8接触门槛.2A/.02r/s/.02r、连续150ms、限流1A、实际行程上限6r、复触参考总长.34r。18:50:33.836 phase9→39.076 phase10→40.396 phase11→41.612 phase14 CONTACT_NOT_CONFIRMED（复触约1.216s）；退让本轮已完成，未进入phase12，随后STOP locked/全轮无保存，DISABLE47.537成功。
- **首次决定性复触证据**：J8 LAST task49343/phase11/gateELIGIBLE/intent/fb/enabled/detect均1、unmet0。I_cmd.938/I_filt.928≥I_thr.2A；v_filt.0175391≤v_thr.02r/s；err.0412109≥.02r；三个滤波条件在失败点都通过，但timer/peak107.25ms<need150ms，当前连续候选还需至少42.75ms且必须继续满足条件。v_raw.0216406略高于.02不等于此周期判据未通过，生产代码使用滤波绝对速度。clearCONDITIONS/detail2、last.45ms明确记录此前一个非零候选因速度条件失败清零；不是把间歇候选累加，更不能声称此次缺电流或缺误差。没有完整波形，不推定只有一次速度清零或以后一定能连续通过150ms。
- **已确认参考/窗口时序**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:680–696`复触目标长BACKOFF+.02×2=.34r（初次6圈不会改变该参考），LAST ref_travel.34、actual_travel.298789、v_ref0证明有界参考已结束但实际编码器受阻/跟随落后。`:825–847`连续计时尚未完成；`:875–884`referenceFinishedTicks从参考停止开始独立计数，超过contactStartup+contactTrigger+5×(currentFilter+velocityFilter)+50ms后无条件Fail CONTACT_NOT_CONFIRMED，不检查已有候选还差多少。默认过滤10/20ms下为450ms；按继承RETOUCH.5、ACCEL5估算参考.34/.5+.5/5=.78s，再加.45s约1.23s，与1.216s日志接近（接收调度及起始实际速度有差异），不能当物理接触时间。失败快照证明截止在当前有效连续确认完成之前触发；属于接触证明窗口设计对实际收敛/微动不够适配，而非计时器算错或检测没有执行。
- **参数调法不能混淆**：缩短TRIGGER150→100同时把证明窗口450→400ms，要求和截止一起缩50ms；在相同候选形成时刻下，不会补回当前约42.75ms缺口，不能保证通过。加大TRIGGER同样延长两者；只调RETOUCH_TIMEOUT只改变阶段总截止，本次却触发独立CONTACT_NOT_CONFIRMED。提高电流或放宽ERROR不针对失败时已经通过的条件；提高SPEED可能减少此前清零但未经波形证明，不优先用它放宽物理接触判据。物理微动/速度估计抖动究竟哪种令速度短时不满足仍未测，不宣称唯一机械原因。
- **推荐的针对性固件方案（尚未实施）**：只在校准SEEK/RETOUCH参考固定后给实际收敛及连续候选足够但有限的接触证明期限；建议把该等待上限设为至少1000ms（或独立定义确认窗口），保持与触发持续时间的合理裕量、阶段总超时照常生效，不累计间歇候选，不让重复候选无限续期，不增加参考/实际搜索行程、不改变电流/速度/误差/150ms判据，STOP/DISABLE随时打断。条件连续满150ms立即进入接触停稳/复触点比较，不固定等满1秒；本例若现有条件保持，当前候选仅需补足约43ms，但后续仍须实际停稳/接触重复性，不保证整轮一改即过。此项可先在既有电机stall_protection.cpp内修复，不必新增CAN/EEPROM/串口参数；同步保护方案/测试流程/状态文档，改前列文件用途并获用户确认。退让/复触超时共用及RETURN_TIMEOUT歧义作为另一个已记录问题，不擅自扩大当前修复范围。
- **J7与保护联动**：J7 LAST reason=CANCELLED_STOP、phase9，v_ref-1、v_raw约-.93，因J8失败被取消，不是J7自身堵转。两个at_ms是独立节点时钟，不用其差值推断跨节点因果。没有部分校准保存或旧目标自动恢复证据。
- **未完成**：尚未获本项源代码改动授权，未执行修复或测试；推荐离线先复现“截止时候选107.25ms仍通过、应允许在有限窗口内连续到150ms”及速度清零重计、无候选/反复候选截止、整体阶段超时、STOP/DISABLE，再编译交用户实测。当前不需再要求硬限位测试来确认软件时序路径。

## 当前分析：18:41夹爪退开实际到位未完成，2秒共享期限触发RETURN_TIMEOUT（2026-10-07，仅分析）

- **证据/范围**：用户提供`C:/Users/Liu3Beng/.codex/attachments/9015c98f-049f-4ef7-b1d4-70b37245b68f/已粘贴的文本.txt`，追加18:41失败及J7/J8 LAST。本轮只核对日志/代码，按AGENTS 5.2更新本文；无固件、配置、助手或其他文档修改，无提交/编译/设备操作。接续上一轮分析，不把两次不同阶段故障混为一项。
- **实际参数变化**：18:40:55将J8 RETOUCH_TIMEOUT_MS由5000改2000（是缩短到2秒，并非延长），BACKOFF_R由1改.5并保存；J8 SEEK/RETURN仍2、RETOUCH仍.5、ACCEL5、CURRENT1、CONTACT_RATIO.2、ERROR.02、MAX_TRAVEL6、POSITION_TOL.001等未改。J7本轮GET确认SEEK/RETURN2、RETOUCH.5、ACCEL10、CURRENT2.5、CONTACT_RATIO.3、BACKOFF1、ERROR/SPEED.02、默认各阶段超时等，随后SEEK/RETURN升5并保存；J7并行速度变化不作为J8自身超时的直接证据。
- **实际阶段定位**：18:41:54.573 J8 CAL_SEEK9→57.500 CAL_BACKOFF10（已确认首次接触并停稳捕获）→59.494 CAL_FAILED14/RETURN_TIMEOUT（距phase10约1.994s），无CAL_RETOUCH11、无CAL_RETURN12。用户说复触张开，实际对应复触前退让张开；本轮没有进入第二次闭合或最终开到100。随后全轮error/saved000/pending000/STOP locked，两个只读LAST分包均完整。
- **已确认固件耦合及错误命名**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:762–769`在CAL_BACKOFF使用CAL_RETOUCH_TIMEOUT_MS，CAL_RETURN才用CAL_RETURN_TIMEOUT_MS；BACKOFF阶段超时也笼统返回RETURN_TIMEOUT，容易误以为最后返回100。用户缩短复触期限同时缩短了退让完成期限，时间包含整个规划运动、实际误差收敛及实际50ms稳定。此复用/命名不清是已确认设计问题，未擅改；后续应独立定义退让期限并区分BACKOFF/RETURN错误，但新增字段/协议/EEPROM兼容及具体修改先提案获授权。
- **本次LAST直接证据**：J8 task61052/phase10、mode1、fb/enabled/intent1，v_ref0、v_raw.000722656r/s，实际目标误差.00546875r=280内部单位≈1.96875°电机轴≈.123047模型开度；完成门槛CAL_POSITION_TOL_R仍.001r≈.0225模型开度，失败时误差约门槛5.47倍。`:888–894`规划停止且参考到内部目标之外，还要求实际误差≤该门槛并通过`:479–488`50ms真实稳定窗口，才能启动BeginSearch(true)进入phase11。本次参考已停但实际到位资格不满足，2秒到期先终止；不能仅凭肉眼张开/实际速度低认定到位或自动放宽容差。实际收敛迟缓的唯一PID/摩擦等原因未测，不凭I_cmd.022A推定总限流不足。
- **不是再次受阻触发**：J8 LAST I_cmd.022/I_filt.021<I_thr.2A、滤波速度.00236328≤.02、误差.00546875<接触ERROR.02，unmet5、timer0、peak/last11.5ms、clearCONDITIONS/detail2；退让内部再次受阻门槛100ms，未达到，实际故障是RETURN_TIMEOUT而非REBLOCKED。诊断detect0仅表示搜索接触检测标志在BACKOFF为假，退让仍有再次受阻保护，见`:175–193/825–871`；need_ms100不是本阶段超时。实际/参考行程4.20434/4.2098r仍相对于首次SEEK的searchStart，不是声称退开4.2圈；目标误差两者相差.00546875有效。
- **规划与实际的时间区别**：J8退开.5r在速度上限2r/s、加速度5r/s²下，静止起点理想参考为三角速度（峰值约1.581r/s），耗时约2×sqrt(.5/5)=.63246s；不能因此认为2秒必然足够，真实闭环收敛和50ms稳定也占总期限。上轮成功J8返回100规划约8.99s、日志实际阶段约11.986s，也显示完成不只取决于参考时间；仅作模型/日志对照，不测得机械收敛波形。
- **J7联动证据**：J7 LAST reason=CANCELLED_STOP、phase9、v_ref-5/v_raw约-5.03，未触发自身接触；被J8失败整轮取消。两个节点at_ms是各自电机上电时钟，不跨节点比较为事件顺序。没有本轮J7独立堵转故障证据。
- **建议/界限**：在失能后将J8 RETOUCH_TIMEOUT_MS恢复5000可以解除本次人为缩短的退让期限，但不能宣称必通过，也不能修复18:39 phase11 CONTACT_NOT_CONFIRMED。优先恢复完整实测成功J8基线：SEEK/RETURN=.5、RETOUCH=.25、BACKOFF=.0888889、RETOUCH_TIMEOUT5000，MAX_TRAVEL6/CONTACT_RATIO.2/CURRENT1/ERROR.02/TRIGGER150等保留；先不同时提高速度、改退让/缩时，不靠放宽POSITION_TOL或加电流掩盖本次未到位。未通过工具设置任何参数或擅自保存；完整成功配置已在上一节记录。
- **未完成**：18:39复触闭合的首次失败快照已被新CAL_BEGIN覆盖，本次phase10 LAST不能反推上一轮未通过哪一接触条件。后续优化需独立处理复触接触证明和退让/返回实际收敛及超时语义；用户尚未授权源代码修改，本轮仅解释并记录。

## 当前分析：18:39复触参考结束但接触未确认，不是复触阶段超时（2026-10-07，仅分析）

- **证据/范围**：用户提供`C:/Users/Liu3Beng/.codex/attachments/d0d1389f-d07e-4de7-9052-07edcee01dfe/已粘贴的文本.txt`，包含18:34成功与18:39调整后失败，询问为何在夹爪复触闭合失败。本轮读实际源及完整日志，按AGENTS 5.2仅更新本文；未修改生产代码/配置/其他文档/助手、未提交、未连接设备、未执行运动。
- **本次实际变化**：上轮成功J8 SEEK/RETURN=.5、RETOUCH=.25、BACKOFF=.0888889r；本轮改J8 SEEK/RETURN=2、RETOUCH=.5、BACKOFF=1r并保存，其余已知CURRENT1/CONTACT_RATIO.2/ERROR.02/MAX_TRAVEL6/TRIGGER150/STARTUP100及RET超时5s未改。J7也同时改BACKOFF1、SEEK/RETURN2、RETOUCH.5、ACCEL10、CURRENT2.5、CONTACT_RATIO.3、SPEED/ERROR.02并保存；J8本轮ACCEL仍5，不把J7的10套到J8。一次修改多个参数，不能独立归因某一个变化。
- **阶段定位**：18:39:21.929 J8 phase9→24.705 phase10（已接受第一次接触并减速捕获）→25.943 phase11（退让实际到位后复触）→28.571 phase14 CONTACT_NOT_CONFIRMED；复触仅2.628s，无phase12，未接受第二次接触、更未进入两次接触点比较后返回。saved/pending000、STOP locked28.853；DISABLE38.096正常。J7停止仍为整轮失败取消，无独立J7异常证据。
- **单位及代码已确认**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:787–791`CAL_BACKOFF_R是电机圈，且退让速度使用CAL_RETURN_RPS。J8从.0888889→1r，相当于模型退开2→22.5开度（11.25倍），不是1mm/1开度；J7同样1r则是5mm。`:680–696`复触从实际退让后位置生成参考长度CAL_BACKOFF_R+2×CAL_ERROR_R，本轮1+.04=1.04r；首次寻端的MAX_TRAVEL6仍作实际行程保护，但不会把复触参考目标也变6r。理想退让准确时只比第一次捕获接触点向闭合方向额外约.04r=.9模型开度，这项余量没有随着退让距离增大而增加。
- **失败分支已确认**：`:875–884`只有复触参考已经固定到内部目标且速度0、有限接触证明窗口耗尽，才返回CONTACT_NOT_CONFIRMED；阶段超时`:762–769`在CAL_RETOUCH会返回SEARCH_TIMEOUT，本日志不是该错误。按照静止起点、1.04r参考、.5r/s、5r/s²和默认滤波10/20ms，参考耗时约1.04/.5+.5/5=2.18s，加100+150+5×(10+20)+50=450ms窗口约2.63s，与2.628s相符；是参考模型解释，不能用接收时间代替物理触碰时刻。只调CAL_RETOUCH_TIMEOUT_MS不会延长这个证明窗口，增大MAX_TRAVEL也不改变复触参考长度。
- **未证明的触发条件**：`:825–847`复触仍需电流命令≥.2A、滤波估计实际速度≤.02r/s、绝对参考/实际误差≥.02r连续150ms，任一不满足清零。附件没有本次phase11的LAST，不能用旧phase9快照判定这次缺电流、缺误差、速度抖动或连续计时不足，也不能说本次必为间歇累计bug。高速初次接触/较长高速退让/复触速度变化可能改变接触载荷、背隙、弹性或电流命令，但仅是待快照验证方向，不宣称唯一机械故障。CONTACT_UNSTABLE比较在`:792–794`第二次接受并实际停稳后才执行，本次未到那里，放宽REPEAT_TOL不针对本错。
- **建议/用户新输入**：已请求仅在失能、未新CAL/重启时读取`#CAL_STATUS 8 LAST`完整输出，不重新触碰。用户回复“重新测试，可能是复触超时，调整时间后测试”；已立即说明2.628s/CONTACT_NOT_CONFIRMED不同于5s/SEARCH_TIMEOUT，延长阶段超时不能解决本分支，不要求再次撞限位验证。建议先保留失败证据，再恢复J8上一轮成功的四项速度/退让值作为基线，CURRENT1/CONTACT_RATIO.2/ERROR.02/MAX_TRAVEL6等不变；不据此擅改节点参数或保存。若优化需独立设置有界复触余量、确认时间与合理速度的组合，并保留真实反馈、连续检测、复触一致性、行程/时间约束；具体数值/文件须后续证据和授权。
- **未完成**：本次复触LAST未提供，尚不能确定哪项连续判据导致不接受接触；用户新测试结果待反馈。软件单项范围校验通过不等于整组物理接触参数已经实测适配。最新日志证明首次搜索6圈方案在本组高速下能确认接触，不证明复触也受其6圈余量保障。之前成功配置与所有未提交改动保留。

## 最新实测：18:34整轮校准成功，J8搜索预算改为6圈（2026-10-07）

- **用户反馈/范围**：用户贴出18:34～18:37完整日志并确认“这次成功了”。按AGENTS 5.2仅记录本文，不擅自改默认参数、代码、配置、其他文档或助手、不提交、不连接设备或执行测试。固件默认MAX_TRAVEL_R仍4.66667；用户本次实际EEPROM参数已保存为6，二者须区分。
- **本次J8已知有效设置**：SEEK_RPS=.5、RETURN_RPS=.5、RETOUCH_RPS=.25、ACCEL5、CURRENT_A1、CONTACT_RATIO=.2、SPEED_RPS=.02、ERROR_R=.02、TRIGGER150、STARTUP100、MAX_TRAVEL_R6、SEEK_TIMEOUT30000、RETURN_TIMEOUT15000、RETOUCH_TIMEOUT5000、BACKOFF_R=.0888889、REPEAT_TOL_R=.01、POSITION_TOL_R=.001、CONTACT_WINDOW_R=.222222。用户先恢复速度.5并把最大搜索行程改6，随后重新设置寻端超时30000并保存；无ERROR_R变更。J7实际参数本轮未回读，不推定全部等于默认。
- **完整成功证据**：18:34:45.935六轴CAL_DONE齐全并WAIT1000ms；46.941 J7/J8 SEEK并行。J8 phase9 46.972→phase10 57.061（10.089s）→phase11 58.334→phase12 59.458→phase13 18:35:11.444。J7 phase9 18:34:46.952→phase10 18:35:44.775→phase11 46.009→phase12 47.110→phase13 18:37:27.896。随后CAL DISABLING、Disabled ok、CAL SAVING，J1～J8分别phase16/NONE、八节点phase0/NONE结束，18:37:29.169 !CALIBRATION ok并Disabled ok。软件确认寻端、退让、复触一致性、RAM标坐标、实际返回、全部失能、八节点保存及结束回执齐全；没有失败或部分保存日志。
- **最新分析/纠正方向**：这次在ERROR_R仍.02的情况下能通过首次和二次接触，证明既有三条件检测及后续流程能运行；不需要为了通过而将误差降到末端保持残差以下。结合前次编码器几乎跟完4.66667圈却缺误差，本次最大搜索预算增至6后在约10.089s已进入退让（理论6圈/.5rps加减速及证明窗口约12.55s才到参考结束失败），强支持原搜索参考在有效接触证明形成前已经结束、余量不足这一方向，不再把降低CONTACT_RATIO或延长总超时当作唯一解法。此次也恢复了速度，未做完全同条件单变量对照，不能宣称已独立证明全部机械/反馈根因或真实全开闭合圈数。
- **参数语义**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:680–695`的6圈是从实际起点向闭合方向的有界搜索预算，不是实测机械行程，也不改变开度0～100映射。`:787–805`接触成立后提前停止搜索并退让/复触，返回100仍按100×16/360=4.44444电机圈计算；不把6圈换算135模型开度后误当成实际爪指走过135。本轮成功日志证明当前参数一次成功，不证明任意初始开度、负载、重复性均通过。
- **未完成/后续**：先保留这一实测成功配置，固件默认值是否改6/.2、返回100是否对应机械正常全开及重复性由用户后续决定/反馈，不擅自设置。上一轮失能后LIVE位置变化约4.69圈及无活动CAL时gate/unmet残留问题尚无补充观察；成功本轮不证明已解释或修复这两项，不要求为此再次顶限位。

## 当前分析：18:21校准仍未确认接触，仅误差条件不足（2026-10-07，仅分析）

- **范围/证据**：用户提供附件`C:/Users/Liu3Beng/.codex/attachments/09115373-8613-420a-a51c-e4a12e6cf5ad/已粘贴的文本.txt`，询问为何反复无法校准、应调参数还是固件问题。本轮读实际固件和本轮完整日志，按AGENTS 5.2仅更新本文；不改固件/配置/其他文档/助手、不提交、不连接设备或请求重复顶限位。
- **本轮实际参数**：CAL_GET 8原CONTACT_RATIO=.3、SEEK/RETURN=.5、SEEK_TIMEOUT15000、ERROR_R=.02、TRIGGER150、STARTUP100、MAX_TRAVEL4.66667、ACCEL5；随后成功SET CONTACT_RATIO=.2、SEEK_TIMEOUT30000、SEEK/RETURN=1并CAL_SAVE成功。没有ERROR_R或最大搜索行程变更，不能再说本轮电流门槛仍30%或速度仍.5。
- **当前失败，不是本次已证明死机**：18:21:17.347 J8 phase9，22.664 phase14/CONTACT_NOT_CONFIRMED，约5.317秒，随后saved/pending000、STOP locked22.904。18:21:29 DISABLE在约84ms回复Disabled ok；LIVE及LAST分包完整end。证明本次失败后的USB/CAN查询仍可工作，不把历史USB卡住归因于本次这条失败。J7因整轮校准取消停止，没有独立J7失败证据。
- **冻结证据**：LAST task50573/phase9、gateELIGIBLE、intent/fb/enabled/detect均1；I_cmd=.212、I_filt=.210≥I_thr=.2A；v_raw0、v_filt=.00275391≤v_thr=.02r/s；err=.00425781<.02r，unmet4仅误差不满足，timer/peak/last均0、clearNONE。当前缺的是误差条件，不能沿用上一轮“电流和误差均不足”；峰值0只证明该搜索阶段没有一个可计数周期三条件同时成立，不证明误差独立全程均小。失败点误差约218内部单位/1.5328°电机轴/0.0958模型开度，门槛1024单位/7.2°/0.45模型开度。
- **时间与搜索预算对应**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:680–696`从实际起点生成固定相对位置参考；`:809–817`每周期独立推进内部规划器，仅参考到界时固定；`:875–884`固定参考到达后有限接触证明窗口失败，与CAL_SEEK_TIMEOUT独立。规定静止起点、1r/s及5r/s²的梯形参考估算4.66666/1+1/5+.450=5.31666s，与本次5.317s相符；上一轮.5r/s时估算9.88332s，也与日志相符。属参考模型吻合，不是测得物理触碰时间；延长SEEK_TIMEOUT30s不延长这450ms证明窗，加快SEEK只更早走到同一个参考终点，不增加搜索距离。
- **核心线索/归因边界**：LAST ref_travel4.66666r、actual_travel4.6624r，编码器几乎跟完105模型开度搜索（实走按模型约104.904单位），故仅留下很小残差；`:825–847`的三条件AND运行与日志一致，未发现门禁关闭、间歇累计或用规划位置冒充编码器位置的新证据。`motor.cpp:83–116`从有效角度包独立解包累加实际位置；`:136–195`CAL控制权期间普通规划器不覆盖其参考。仍不能证明机械实物确走104.904开度、真实0～100行程必为4.44444圈或是哪一项硬件故障；固定行程/搜索余量与真实传动匹配未确认，编码器至爪指运动关系也待核对。
- **参数与方案建议**：不通过再次加电流/速度/超时试探，也不把ERROR_R=.003当修复（失败点该值在数值上可通过，但端点保持电流/静止/小残差也可能符合而伪造限位；单个快照不证明能连续150ms）。仅降低CONTACT_RATIO已经不再针对本次未满足项，缩短TRIGGER也无法让峰值0的候选开始计数。先核对实际开闭电机圈数及反馈/传动，若真实全行程与4.44444r模型不符须校正夹爪开度映射/搜索预算；若编码器几乎抵达目标才形成受阻，应在机械范围内重新设计受控搜索余量/接触证明，保留实际编码器、连续判据、行程/时间界限、复触、STOP/DISABLE及失败不保存。此前修复的是参考微越界与取证，不宣称已解决实际接触识别；当前不能二选一断言只需降阈值或唯一硬件原因。
- **附加LIVE变化待解释**：失能后LIVE task0/enabled0/detect0，err4.69229r、actual_travel=-.0295117r；与LAST actual4.6624相差约4.69191电机圈。同任务取消/DisableForStop不重置realPosition或searchStart，`:175–193`LIVE为查询时实际位置/保留参考及CAL标签，不能拿失能后的大误差证明触发前达标；可能是失能后实际打开或反馈异常等，先询问用户这次失能后是否自行/手动打开、闭合后是否直接观察电机轴继续转，只回忆已有测试，不要求重跑。gate/unmet残留显示问题仍仅记录，未改。
- **尚未完成**：等上述已有观察区分位置变化，唯一机械/反馈原因及最终校准参数尚未确认；后续若需要代码改动先列文件用途、获得授权。其余未提交工作保留，本轮无构建或实机操作。

## 当前分析：解释J7/J8完整校准流程及用户最新观察（2026-10-07，仅分析）

- **最新用户观察**：本次开始时夹爪处于打开状态，闭合后夹爪保持闭合、地轨也完全停住。用户未确认“打开”是否为物理全开100，也未单独说明电机轴/联轴器是否仍转；不把这个描述扩大为传动没有滑动等结论。本轮先解释现有流程，不新增机械测试要求，不改生产文件、实机参数或提交。
- **主控准备**：`firmware_new/dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:788–797/574–588`要求主控处于DISABLED且无保护忙状态，清旧运动/队列，锁定并查询全部八节点有效、新鲜反馈及实际失能状态；全部CAL_BEGIN/CAL_READY后，`:649–668`仅给J1～J6修正当前位置标签为REST_POSE，不使能运动，六轴CAL_DONE确认齐全后等待1000ms，再给J7/J8分别发送同任务CAL_RUN。两轴并行独立推进，不要求同时结束；J8默认HAND电流1A也作为其校准电流上限约束。
- **各轴完整顺序**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:680–696/783–805/888–894`为CAL_SEEK(9)第一次寻端→接触成立后减速、实际50ms稳定并捕获→CAL_BACKOFF(10)退开→实际到位稳定后CAL_RETOUCH(11)低速复触→第二次接触点与第一次比较→RAM标限位坐标→CAL_RETURN(12)返回→实际到位稳定后CAL_DONE(13)固定保持。J7寻用户+250mm端（电机方向-），将该端标+250后返回0；J8寻闭合端（电机方向+），将该端标开度0后返回100。上电只是把夹爪当前位置假定为100，不主动打开或寻端，见`Ctrl/Motor/motor.cpp:68–72`。
- **搜索与接触判据**：搜索从当前实际编码器位置生成相对、有界位置目标，不是直接下发用户+250/0绝对目标。第一次默认搜索J7最多101电机圈=505mm、J8最多4.66667圈=105模型开度；参考到边界固定，不视作触碰。`:819–866`在CAL启动豁免后要求滤波绝对电流命令≥calibrationCurrent×CONTACT_RATIO、滤波绝对实际估计速度≤CAL_SPEED、参考/实际误差绝对值≥CAL_ERROR连续CAL_TRIGGER；任一条件失效清零。calibrationCurrent来自CAL_CURRENT并对J8取HAND上限较小值；实际输出还受电机总限流/板型限制，focCurrent不是实测电流。已有实测坐标时首次接触还必须落在CONTACT_WINDOW内；假定上电坐标不凭该窗口证明端点。固定参考结束后仍无接触证明，在有限窗口结束报CONTACT_NOT_CONFIRMED。
- **默认值及耗时边界**：`UserApp/protection_config.cpp:52–62`默认SEEK/RETURN=.5电机r/s、RETOUCH=.25、ACCEL5r/s²、CONTACT_RATIO=.3、SPEED/ERROR=.02r/s/.02r、TRIGGER150ms、STARTUP100ms。J7退让.2圈=1mm、J8退让.0888889圈=2模型开度，与普通堵转BACK_MM=5/20无关；二次接触差≤.01电机圈，返回实际误差≤.001圈并连续50ms稳定。J7默认寻端/返回速度2.5mm/s；若真实从0到+250，首次寻端约100s、返回另约100s，不含加减速/确认。J8按模型全行程100对应4.44444圈、.5r/s单程约8.9s。代码默认不覆盖有效EEPROM；J8本次回读值已知，J7现值未回读，不宣称默认就是实机参数。
- **阶段超时**：J7默认SEEK220s、RETURN120s，J8两项15s，BACKOFF及RETOUCH均使用5s。实际最大搜索行程、无效反馈、续租/通信失效、再次受阻及重复接触点不一致都能终止流程；返回和退让均要求实际编码器到位，不能只按规划器结束。
- **共同成功收尾**：主控`:669–730`要求两轴本任务CAL_DONE/NONE及新鲜有限实际位置，先完成者固定保持、另一轴继续；两轴完成后全八轴DISABLE，确认失能再逐节点SAVE_COORD，八个SAVED确认及CAL_END完成后才返回!CALIBRATION ok和Disabled ok，保持失能。保存是八节点顺序写入，不是跨节点原子事务；如果保存中途失败，可能已有部分节点写入，saved/pending记录实际进度且不返回成功。寻端/返回之前失败则不启动保存；J1～J6已经生效的RAM标签不自动回滚。
- **失败和用户命令**：主控`:345–364`在任一轴校准失败后取消全八节点任务，停止包括J8在内的本轮校准，锁定且不继续复触/返回/保存；J7特殊EARLY_CONTACT/返回受阻可转入普通保护回退，取消整轮校准。CAL活动/等待/保存期间START返回busy、不挂起，STOP取消并保持锁定，DISABLE取消并使全八轴失能。当前日志J8由phase9直接到14/CONTACT_NOT_CONFIRMED，尚未进入退让、复触、坐标捕获或返回；J7一起停止与整轮失败联动一致，不证明J7自身限位或故障。
- **当前结论/未完成**：此次用户观察补足开始为打开、闭合后两机构停止，但不改变LAST三条件未同时成立的已确认事实。尚未确定真实夹爪行程/电机反馈与模型差异的唯一原因；不以规划终点或肉眼闭合作为校准成功。本轮只更新本文记录流程，未编译、烧录、连接设备、修改参数或提交。

## 当前分析：18:04实机CAL失败，电流命令和误差同时不足（2026-10-07，仅分析）

- **范围**：用户贴出新CAL失败及LAST/LIVE，要求详细解释物理限位判定与普通堵转的关系。本轮只读固件、按AGENTS 5.2更新本文；不改固件/配置/助手/其他文档，不提交或要求再次顶限位。
- **本次事实**：J1～J6 CAL_DONE齐全后WAIT1000ms、J7/J8并行SEEK；J8 phase9到CONTACT_NOT_CONFIRMED约9.883秒，随后整轮error、saved/pending均000、STOP locked。DISABLE及J8诊断全部正常响应。LAST在故障前phase9、mode1、intent/fb/enabled/detect全1、gateELIGIBLE；stop_lock1是内部校准预期锁，并不屏蔽内部接触检测。I_cmd=.174/I_filt=.173/I_thr=.3A，实际命令上限1A；v_raw0/v_filt=.00109375≤.02r/s；err=.0025<.02r。unmet=5=电流1+误差4，速度条件满足；timer/peak/last均0，意味着该SEEK阶段从未出现一个计数周期三条件同时满足，不证明电流或误差各自全程都不满足。clearNONE仅在没有非零候选清零时正常出现。
- **已确认代码判据**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:680–696`准备CAL专用阈值/方向/有界位置参考，启动豁免来自CAL_STARTUP_MS；`:825–847`复用普通CURRENT_FILTER_MS/SPEED_FILTER_MS对abs(focCurrent)/abs(estVelocity)滤波，与abs(softPosition-realPosition)一起AND连续计数，任一失效清零，不累计间歇条件。focCurrent为控制输出命令，不是测得绕组或电源电流。CAL_CURRENT_A及HAND任务限流决定CAL_CONTACT_RATIO门槛，普通STALL_CURRENT_A不参与这条判据。`:848–866`还要求已测得坐标情况下的预计限位位置窗口；接受后减速并实际50ms稳定，再退让/复触，比对两次实际接触点，才标限位原点返回。phase9直接失败意味着尚未接受第一次接触，未进入后续判稳/复触，不能归因于后续STOP停稳太严格。
- **与普通堵转的区别**：同样三条件AND、滤波及连续计时思路，但CAL内部状态/阈值/动作独立。普通`:590–630`仅J1～J7、运动意图/反馈/模式/到位门禁通过后使用CURRENT_A/SPEED_RPS/ERROR_R/TRIGGER_MS；J8排除普通检测但CAL寻找闭合端允许检测。CAL中预期接触→退让复触→标坐标→实际返回→全部成功后失能保存；普通STALL→全机联动STOP和内部回退→锁定等待START/DISABLE。
- **搜索失败已与旧bug区别开**：`:811–817/875–884`固定搜索参考及有限证明窗口这次走到了CONTACT_NOT_CONFIRMED分支，未再因参考1单位越界抢先SEARCH_TRAVEL。当前约9.88秒与默认0.5r/s、4.66666r加450ms证明窗口一致，日志接收时间不是精确电机触碰时刻。本轮反馈证明前轮边界修复/冻结查询生效，不能证明物理接触全部已经识别。
- **关键新线索及单位边界**：ref_travel4.66666r、actual_travel4.66416r，相差.0025r=128内部单位=.9°电机轴。`dummy-ref-core-fw/Robot/instances/dummy_robot.h:47–79`按16减速、100开度映射100°，标准0～100对应4.44444电机圈；本次参考105开度搜索，编码器实走按该模型折算104.9436开度单位，离参考仅.05625，而误差门槛.02r对应.45单位。此为模型换算，不是测得机械真实开度超过100。代码没有直接限位开关输入；肉眼闭合与电机编码器不能明显早于参考停止并非同一证据。若起始确实物理全开、传动及比例成立，通常应在约4.4444圈停止、剩余约.2222圈误差；本次几乎完整跟随105搜索，需优先核实初始实物开度、真实开闭电机圈数、编码器至爪指传动是否继续运动，以及当前搜索余量，不认定是哪种机械缺陷。
- **不得据此盲调**：只降低电流比例仍有误差门槛未过；只缩短150ms也无用，连续候选峰值原本0。同时降到小保持残差容易把正常规划终点停下误当物理端点，应先核实真实行程/传动，再决定限位检测配置或更适合的局部搜索策略，不扩大不受约束搜索、不把规划到达直接算接触。
- **J7停止/未保存**：主控`Robot/instances/protection_coordinator.cpp:669–678`发现J8失败，`:345–364`取消整轮八节点任务并启动包括校准J8的STOP。J7停止是失败联动，当前没有J7自身失败证据；无最终CAL ok，Flash保存计数000，已完成的J1～J6RAM标记不能据此说全部回滚。
- **附带诊断显示问题（记录，未改）**：DISABLE后的LIVE task0/mode0/enabled0/detect0正确；gateELIGIBLE/unmet5是calDetectionGate/Unmet保留了上个活动阶段，`FillCalDiagnostic:175–193`未在无活动任务时重算这些标签，不能把LIVE这些残留解释为失能后仍在检测。LAST为冻结证据，未受这个LIVE显示问题影响。后续获授权可独立修正LIVE标签，不擅自顺手改。
- **最新补充/待确认边界**：用户已说明开始夹爪打开，闭合后夹爪保持闭合、地轨完全停住；没有进一步确认物理全开100或单独观察电机轴/传动件。先按新首节解释完整流程，不重复请求撞限位测试；尚未确定唯一物理根因。

## 当前完成：校准搜索边界修复及失败冻结快照（2026-10-07，待烧录实测）

- **授权/范围**：用户在确认30%已实际生效及搜索参考微过冲缺陷后要求“修复”。完成校准SEEK/RETOUCH有界参考与CAL失败快照；保留本轮开始时已有30%默认、六轴完成后等待1000ms、J7/J8并行及文档/old助手改动。未提交commit，未连接设备、写实机参数、烧录或执行物理运动。
- **已确认事实/修复**：电机`Ctrl/Motor/stall_protection.cpp:778–817`先以64位差值检查实际编码器行程，包含接触后的减速阶段；仅SEEK/RETOUCH参考到达或越过既定internalTarget时固定于该目标，并以NewTask(target,0)清该内部规划器的位置/速度及积分。未改共享普通motion_planner.cpp。`:875–884`保留原有限接触证明窗口，默认100+150+5×(10+20)+50=450ms；参考走完不当作物理接触，无接触证明返回新增CONTACT_NOT_CONFIRMED(26)，实际越界/搜索目标整数溢出仍SEARCH_TRAVEL(13)，阶段总超时也仍有效。接触仍需电流命令/实际低速/误差连续150ms，不放宽30%/速度/误差/行程或位置判稳。
- **新证据通道**：电机`:174–239`使用独立96B RAM首次失败快照，`:497/524/541/916`在校准转普通保护、失败、取消、保存失败前捕获，先记录再清检测。记录模式/phase/任务/门禁/反馈、原始及滤波电流命令/速度、误差、实际门槛和限流、连续计时/本阶段峰值/最近非零候选清零原因、实际/参考搜索行程与失败原因；普通STALL触发快照不覆盖。LAST在STOP/START/DISABLE后保留，直到新任务CAL_BEGIN真正接受或重启；同任务BEGIN/同request重传不重新捕获、不清LAST。阶段峰值在下一内部子阶段启动时重置，条件/无效反馈失效清连续候选，间歇条件不累计。
- **命令/协议**：`#CAL_STATUS 7`或8读取LIVE，追加`LAST`读取首次失败；无记录no_failure_record，不会拼造停机后的触发值。共享`protection_protocol.h`保留0x37/0x38、DIAG_VERSION=2及24字段/8字节分包，新增selector=3/4、无失败record=5，J7/J8限定；普通selector0/1及J8普通堵转排除不变，EEPROM VERSION=1/格式不改。字段的CAL专用映射见该头部注释和方案。主控`Robot/instances/protection_coordinator.cpp:70–150/761–794`复用只读事务，按节点/request/version收齐、100ms重发、1500ms超时，控制任务打印，CAN ISR仅复制，无打印或任务临界区调用。只读事务不贡献校准/运动busy，不改变保护锁或参数。
- **必要配套与自主决策**：实施读串口入口发现只允许旧STALL_STATUS经过锁，因此增加已向用户说明的`UserApp/protocols/ascii_protocol.cpp:32–34`，只扩展CAL_STATUS读命令入口；严格语法/节点仍由协调器检查，参数写入/运动门禁不扩展。协调器.h沿用先前并行改动，本次诊断通过既有selector区分，不另增状态成员。新增失败reason26区分分支；默认450ms窗口不新加可调参数，受既有接触/滤波设置和阶段超时限制。压缩原文件内启动/参数缓存更新、CAN/main查询、单次故障/坐标捕获/子阶段启动函数为Os，周期20kHz控制/检测/判稳函数编译选项不改；Flash95KiB、编码器表/EEPROM保留区不改。常规设计均在用户授权快照与边界修复内，未顺手更改PID等其他问题。
- **old助手**：`串口助手_old.py:1216–1220/1284–1300`CAL页增加“读取校准快照/读取失败快照”；选7/8生成只读命令，发送入口和发送线程仅对严格CAL_STATUS/STALL_STATUS诊断白名单放行。CAL_STATUS回包不会伪装成CAL或STALL事件、不会解锁/自动运动；Tk隐藏检查包含原顺序模式。串口助手.py校验未变，禁止修改的参考目录未改。
- **验证**：真实生产代码受控探针770841项（含每周期边界断言）通过：低电流/高速条件不足时参考固定238933单位、实际227555未越界，在9429.4ms达到参考上界，450ms后9879.4ms CONTACT_NOT_CONFIRMED；之前同输入9430.55ms参考238934误报SEARCH_TRAVEL。有效接触仍9153.1ms接受；参考结束后再让条件成立可在9583.4ms接受。两搜索方向、短复触、积分清零、实际行程越界、连续150ms重计、反馈失效、首次快照保留/新task清除及普通J8排除均检查。此为规定输入，不能证明用户实机哪项门槛未满足。
- **回归/编译**：生产主控/STOP/理想CAN同行24732项（含真实ASCII锁入口、CAL诊断完整/缺包/错节点/无记录及打印）通过；真实电机/保护/规划器与六种理想CAL起点7040项、配置/EEPROM170项、隐藏真实Tk诊断106项及原顺序/分包/模式410项通过。五目标Ninja及BIN/HEX逐字节一致、应用区大小、旧助手哈希、git diff --check通过。主控FLASH434760B/RAM56384B，四电机FLASH96844/96844/96836/96796B、RAM10704B，最少剩436B；既有RWX及编码器虚函数无返回值告警记录而未扩大修改范围。测试数含循环，不是对应数量物理试验。
- **未完成/实机边界**：软件缺陷已修复；实机闭合能否连续符合CURRENT/SPEED/ERROR仍待本轮快照确认，不宣称所有夹爪接触问题已消失。先配套烧录主控、J7/57、J8/gripper，保留EEPROM/编码器表；J1～J6逻辑未为此新增校准运动，如希望全节点同一构建可用已编译35/42更新。失能后LIVE/LAST先查询无动作，再支撑好六轴进行一次受控校准；闭合后仍无退让或啸叫立即DISABLE，不连续重试硬限位。失败后分别CAL_STATUS 8 LAST、7 LAST，每项等end，取到日志后再开始新CAL/重启（会清记录）。详细顺序见测试流程新首节。

| 对象 | BIN（同目录有HEX） | 字节数 | SHA256 |
|---|---|---:|---|
| 主控 | `firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin` | 434760 | `2a1fa48ea689cfc26dd4f7274196312f082a6c60b975c7485c92943c94588eee` |
| motor-35 | `firmware_new/dummy-motor-fw/build/motor-35.bin` | 96844 | `4be94d2f0c244ba4334f820975ea18b2654adc89980879694bae74a490cac030` |
| motor-42 | `firmware_new/dummy-motor-fw/build/motor-42.bin` | 96844 | `4be94d2f0c244ba4334f820975ea18b2654adc89980879694bae74a490cac030` |
| motor-57 | `firmware_new/dummy-motor-fw/build/motor-57.bin` | 96836 | `c7c5350da7f8d3495793ae06c07fdac11b687291f7e0523def75196d73cbc542` |
| motor-gripper | `firmware_new/dummy-motor-fw/build/motor-gripper.bin` | 96796 | `b1fd09dd1960b386c26f11d61caffa927f4e31ff2c2f67faa8362863f1952c4f` |

以下“待确认/未修复”描述是各历史轮次当时状态，本节为当前状态。

## 当前分析：30%已生效，J8搜索失败取消并行地轨（2026-10-07，17:26实机日志）

- **本轮范围**：用户报告并行校准仍失败；只读生产代码，在ignored分析目录构建真实生产代码的受控输入探针，按AGENTS 5.2更新本文。未改生产固件/配置/助手、未写实际节点参数、未重新发布固件或提交。本次拟修复内容须确认后实施，此前并行及30%改动保留。
- **实机已确认**：17:26:33.163六轴CAL_DONE、33.164 CAL WAIT 1000ms；34.161两轴SEEK，34.182 J7/34.203 J8 phase9；43.640 J8 phase14 SEARCH_TRAVEL，J8阶段9到失败9.437秒；随后error J8、saved=000 pending=000、STOP locked。17:26:57 DISABLE成功。并行及一秒延时已工作；J8没有接受接触并进入phase10退让/phase11复触/phase12返回，不能把本次当已成功找限位。无Flash保存不等于六轴RAM坐标未变。
- **参数补充与排除方向**：17:29 CAL_GET 8回读CONTACT_RATIO=.3、CURRENT_A=1、速度.5/.5/.25、ACCEL5、SPEED/ERROR=.02/.02、TRIGGER150、STARTUP100、MAX_TRAVEL4.66667、SEEK/RETURN/RETOUCH超时15/15/5秒、BACKOFF.0888889、REPEAT/POSITION容差.01/.001、窗口.222222。明确排除80%未应用；不能仅凭CAL_CURRENT_A=1确定实际总/HAND限流，但实际生效上限不超过该值。
- **地轨停止是已确认的失败联动**：主控`Robot/instances/protection_coordinator.cpp:659–668`逐轴检查本任务失败，J8 CAL_FAILED调用Error；`:335–354`取消八节点任务，包含校准中的J8，BeginStop停止/保持整轮。不证明J7自己SEARCH_TRAVEL或堵转。任一校准失败禁止完成/保存仍按既定方案，不应删掉联动来掩盖夹爪失败。
- **两层问题**：电机`Ctrl/Motor/stall_protection.cpp:751–754`先检查实际或参考行程严格越界，再`:758–787`检查接触；`:797–805`规划精确终点后有有限接触证明窗口。直接参考检查会在规划器微过冲时先结束，不能获得证明窗口。接触必须滤波abs(focCurrent)达到实际校准限流×.3、滤波abs(estVelocity)≤.02电机r/s、abs参考/实际误差≥.02电机圈连续150ms；肉眼已闭合不等于三条件连续成立。无失败前快照，不能断言哪项条件唯一未满足，更不能认定电流条件已通过。
- **真实生产源离线复现**：ignored `generate_cal_contact_failure_probe.py/run_cal_contact_failure_probe.cmd`直接包含生产motor.cpp、stall_protection.cpp、motion_planner.cpp和当前protection_config.cpp，硬件/CAN边界使用已有替身，手动受控输入不代表实机电流或速度。在J8全开到闭合的理想实际位置边界下，默认30%/1A校准上限；模拟接触电流命令.2A（电流不满足）或命令.5A但实际估计速度.03r/s（速度不满足），均在9430.550ms CAL_FAILED/SEARCH_TRAVEL，参考行程238934单位超过238933一单位，实际行程227555未越界，候选峰值0。模拟.5A及实际0速度时在9153.100ms接受连续接触、未触发该失败。与实机9.437秒接近支持参考分支，但探针不能证明实机是哪项接触条件未满足或唯一根因。1单位约.000439开度；不是实际超行程的证据。
- **诊断缺口**：`FailProtection:479–490`在失败后ResetDetection，丢连续候选证据；现有STALL_STATUS只覆盖普通J1～J7，J8不支持，字段也是普通门槛而非CAL接触门槛。需要失败前冻结校准电流/速度/误差、实际有效门槛、连续峰值/清零原因、实际/参考行程及具体失败分支，失能后经主控USB读取，不要求ST-Link或再次长时间顶限位。
- **推荐修复、等待确认**：仅校准内部SEEK/RETOUCH把参考约束于既定有界目标，并同步重置该内部规划器位置/速度/积分；保留实际编码器最大行程、接触三条件及有限证明窗口，无接触仍明确失败。新增CAL_STATUS 7/8失败冻结快照并区分实际行程越界与到参考终点仍未确认接触，复用CAN诊断分包通道；普通堵转/J8排除不变。拟改motor.h、stall_protection.cpp、共享protection_protocol.h、主控protection_coordinator.cpp/.h、串口助手_old.py及三份保护文档；不改共享普通规划器、PID、机械范围、Flash分区或持续时间。构建需核对原95KiB电机应用区，诊断/校准启动等非周期函数可在同一已列文件内按体积优化，不改变20kHz周期函数编译选项；不能扩大编码器/EEPROM保留区。本轮不再要求撞限位实测。

## 当前改动：六轴坐标确认后等待1秒，J7/J8并行校准（2026-10-07）

- **用户授权与计时起点**：用户要求“把j7和j8改成同步校准”，补充“j1-6校准后一秒再启动j7和j8”。按同轮并行理解，不要求两个CAN单播同一毫秒启动或同时完成；1秒从本任务J1～J6的SET_CAPTURE_COORD/CAL_DONE确认全部收齐开始，不从收到CALIBRATION命令计时。
- **最新改动**：主控`Robot/instances/protection_coordinator.cpp/.h`用CAL_WAIT、CAL_MOVE替换CAL_HAND/CAL_RAIL串行阶段；等待1000ms后J7/J8独立幂等CAL_RUN、分别100ms重试、轮转避免TX忙饿死同伴，分别续约。两轴本任务CAL_DONE/NONE/有限实际位置及300ms内新鲜回执都有效才进入八轴失能/顺序EEPROM保存；先完成轴保持，不再次寻端。`串口助手_old.py`校准确认弹窗同步等待与并行说明；本文、方案、测试流程更新。上轮CONTACT_RATIO默认30%保留，不改电机内部校准动作或协议。
- **已确认事实**：电机`Ctrl/Motor/stall_protection.cpp:398–404`同任务CAL_RUN在运动及CAL_DONE阶段幂等，只回当前快照，不再次搜索；`:358–369`独立task/version/nonce续约，500ms租约未变。主控原Cancel/Error覆盖全部八节点并停止校准中的J8；异常J7普通堵转事件仍取消校准、接管普通保护回退。CAL等待/运动/失能/保存期间START busy不挂起，STOP/DISABLE取消后不能晚到启动或保存成功。
- **常规决策及排除方向**：两轴实际动作顺序、搜索方向、退让/复触、返回0/100和接触比例不改；取消不保存不完整流程。新增逐轴完成掩码和独立重试，不用ALL广播CAL_RUN（J8载荷有HAND电流限制），不改普通堵转串行回退。先完成轴继续用同任务CAL_RUN刷新完成证明，另一轴完成时不能用几分钟前的旧完成快照收尾；异常、拒绝、1.5秒无本任务回执任一轴均终止整轮。
- **未完成/限制**：之前确认的搜索参考越边界缺陷仍待单独修复，不能认为并行或30%已解决所有SEARCH_TRAVEL。合法EEPROM优先加载，实际已保存80%仍需失能SET/GET/SAVE变30%。本轮不连接设备、写实际参数、烧录或commit。
- **验证完成**：主控Ninja成功，FLASH434360B（比上一轮+344B）、RAM56384B，原有RWX链接告警保留；四电机Ninja无需重编译。真实主控协调器/STOP离线24572断言通过，包含999/1000ms边界、两个完成顺序、新鲜/错误任务/NaN回执、只完成一轴不保存、CAN单次TX忙及两轴续约丢ACK、两轴续约间隔与单帧预算、任一轴失败/完成后掉线、等待/运行中的STOP/DISABLE和START busy；隐藏真实Tk72项通过，确认弹窗写明等待1秒和并行、WAIT及两轴SEEK保持busy。断言数含周期循环，端点耗时由理想CAN同行模拟，不是实际接触/动态测试。五BIN/HEX、原应用区大小、参考助手校验及git diff --check通过。
- **最新产物**：`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`（同目录HEX），434360B / SHA256 e3480d0085c7badff5701566e9aa54340ed96ab4090c55f66da504aed28b14ad。电机BIN及SHA仍为下一节30%版本，协议及电机端无需为并行重新改动/烧录；实际CONTACT_RATIO以节点EEPROM回读为准。ignored测试文件及产物不入库，本轮未commit或烧录。

## 当前改动：校准接触电流比例80%→30%（2026-10-07，用户明确授权）

- **当前问题与决策**：用户反馈夹爪闭合时80%电流门槛过高，明确要求改为30%。仅调整J7/J8共享CAL_CONTACT_RATIO默认值；普通堵转CURRENT_A仍为初始化限流40%，其余校准参数和流程保持原实现。此前80%回读及讨论保留为历史证据，以下各节中的“本轮未改CAL/只读分析”属于对应历史轮次。
- **最新改动**：电机端`firmware_new/dummy-motor-fw/UserApp/protection_config.cpp:54`默认.8→.3；`串口助手_old.py:55`校准编辑初值同步.3；本文、方案及测试流程记录生效方式与未修复事项。不改主控、参考固件、EEPROM格式或普通堵转控制逻辑。
- **已确认事实**：`Ctrl/Motor/stall_protection.cpp:628`使用实际校准电流命令上限乘CONTACT_RATIO判接触；例如实际校准上限1A时新门槛.3A，仍需实际低速/跟随误差连续150ms及有效编码器反馈。不是整机电源实测电流比例。合法EEPROM记录仍优先覆盖新默认，不能仅靠重新烧录让已有.8生效为.3。
- **参数应用**：失能并等Disabled ok后，逐条发送`#CAL_SET 8 CONTACT_RATIO 0.3`、等最终成功、`#CAL_GET 8`回读确认，再`#CAL_SAVE 8`等saved；J7需要更新已有保存值时同样用节点7。old助手回读值优先于编辑初值，不自动覆盖实际节点参数。
- **未完成与已排除方向**：上一节已确认的搜索参考越边界问题另行待修，本轮不扩大MAX_TRAVEL、不改触发时间/速度/误差、不以规划结束代替接触。降低比例不保证该边界缺陷消失，也没有证据证明电流是唯一阻塞条件。不要求继续长时间顶限位测试；未连接串口、写真实参数、烧录或新增commit。
- **验证完成**：35/42/57/gripper四种Ninja构建通过，主控Ninja确认无需重编译；配置/EEPROM170项（含J7/J8新默认30%、已保存80%优先加载）、隐藏真实Tk66项（两节点初值30%、已回读80%仍显示80%）、真实电机/保护/规划器及理想编码器校准模拟7040项通过，git diff --check通过。循环断言及理想模拟不能代替实机接触/空驶误触发验收。既有RWX告警保留，BIN大小与此前相同，参考串口助手.py校验未变。
- **最新产物**：四电机位于`firmware_new/dummy-motor-fw/build/`，35/42各97260B / SHA256 66cc5e2886981b36ce67def470445e97806bc7f6cd83afe60296236ac8f62276；57为97252B / 9f71b6b41ffd799dbda3cc302196b41f0348359a3bc592f30cbc468b85dc31ae；gripper为97212B / 7993e5d39d577c84c5a0e1f6b23a8f4858b4e23a13325251dba4bc4b59c0e67f，均有对应HEX。主控434016B / cd1ca6e158b9a557cb05f59d51e5ac6e8b7fb2aaa8e53e7df840605439f9819d未变。ignored测试及产物不加入版本库，本轮未commit。

## 当前分析：CALIBRATION夹爪闭合后SEARCH_TRAVEL，地轨尚未启动（2026-10-07，16:38日志）

- **本轮范围**：用户要求分析，J1～J6留待之后讨论。只读生产代码，在原ignored build分析目录运行真实规划器探针，按AGENTS 5.2记录本文；未改固件、配置、old助手、实际EEPROM参数或发布构建，未烧录、未新增commit。保护基线仍424c9ed，唯一tracked未提交文件为本文。
- **日志事实**：16:38:15.388 CAL pending；八轴phase8/CAL_READY；J1～J6 phase13/CAL_DONE；15.886 CAL J8 SEEK、15.958 phase9/CAL_SEEK；25.400 phase14/CAL_FAILED reason=SEARCH_TRAVEL，距报告SEEK约9.442秒。没有J8 phase10退让/11复触/12返回，也没有CAL J7 SEEK。saved=000 pending=000证明未发送或确认本次Flash保存，随后STOP locked；38:45 DISABLE成功。J1～J6捕获坐标已在RAM生效，不能把无Flash保存解释为没有发生任何RAM坐标变化。
- **J7不动原因已确认**：`dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:649–656`串行先J8，只有本任务新鲜CAL_DONE/NONE/有效位置才切J7。J8失败分支终止整轮、包括夹爪的停止/保持，J7本次未获CAL_RUN；不能据此判地轨控制故障或堵转。
- **SEARCH_TRAVEL路径**：`dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:628–644`构造有界相对搜索目标；`:753–754`实际或规划参考相对搜索起点超过searchMaxTravel直接失败（在接触判定之前）；`:797–805`参考已精确到终点/速度0后，等待startup+trigger+5*(电流/速度滤波)+50ms仍无有效接触，也报同原因。目标整数溢出还可在开始时报同码，但本次先正常运动约9秒，不能用立即构造失败解释。没有接受接触时才继续这些分支；接受后internalStopping在`:725`独占，稳定后进退让。
- **接触条件及未证实项**：`:758–768`有效反馈时，滤波abs(focCurrent)≥实际校准限流×CONTACT_RATIO、滤波abs(estVelocity)≤CAL_SPEED_RPS、abs(softPosition-realPosition)≥CAL_ERROR_R连续CAL_TRIGGER_MS；任一失效立即清零。默认ratio=.8、低速.02电机r/s、误差.02电机圈（夹爪约.45开度）、连续150ms、启动100ms，滤波共享10/20ms。实际限流为min(CAL_CURRENT_A,正值HAND运行限流,总限流)；默认CAL/HAND各1A时门槛.8A，不能由整机电源电流推断是否通过。EEPROM可覆盖各值。本次没有接触快照，不能确认80%门槛、电机微振的速度或计时哪一项阻塞，也不能据肉眼闭合证明全部三条件达标。
- **已确认边界缺陷与真实源复现**：`Ctrl/Motor/motion_planner.cpp:190–329`离散积分在近终点允许小参考过冲；搜索目标恰好按最大距离设置，却在上述严格参考检查中不给规划器收敛机会。忽略目录`cal_search_planner_probe.cpp`直接包含该生产cpp，默认J8 MAX_TRAVEL=105*16/360圈量化238933单位、速度.5电机r/s、加速度5、初速0，正负方向均在9430.550ms出现参考238934/-238934（越过1单位、速度±730单位/s），至9439.450ms才精确终点及速度0。当前搜索守卫会在首次超出时先退出，不能进入之后的默认450ms证明窗口。1单位约.000439开度；此为参考数值边界问题，不证明物理实际越行程。9.43055秒与日志9.442秒接近，支持该路径，但尚未回读实际参数/冻结分支，不认定唯一实机根因。
- **推荐修复、未授权**：在校准寻端/复触内部链路把参考约束在既定有界目标，并一致停止内部规划参考，保留实际编码器的最大行程检查及有限接触证明窗口；不扩大机械搜索行程，也不能把规划结束自动当作接触成功。失败应区分实际行程越界/参考到终点未接触，同时保留接触时电流/速度/误差/最长连续计时及失效条件，避免再靠长时间顶限位定位。CAL接触参数单独评估，不擅自将普通40%/100复制过去；不扩改J1～J6或地轨并行顺序。
- **用户已补充**：确认空夹至自身闭合限位；16:44 `#CAL_GET 8`回读SEEK/RETURN/RETOUCH=.5/.5/.25、ACCEL5、CURRENT1、CONTACT_RATIO.8、SPEED/ERROR=.02/.02、TRIGGER150、STARTUP100、MAX_TRAVEL4.66667、SEEK/RETURN/RETOUCH超时15000/15000/5000、BACKOFF.0888889、REPEAT/POSITION容差.01/.001、CONTACT_WINDOW.222222，均与上述默认探针输入相符。排除本次夹物及CAL保存了另一组阈值的方向；尚未给具体闭合时刻、HAND运行限流或触发瞬间电流/速度/计时证据，仍不能断言某一条件唯一失效。不要求再次运行校准或持续顶限位。

## 当前节点：普通末端修复已获实机反馈，日常默认值已确认（2026-10-07）

- **用户反馈/授权**：用户“我测试没有什么问题”，随后确认日常默认方案并要求“改完后提交，提交完后我们开始讨论!CALIBRATION”。本轮明确获准将此前已验收保护改动及本次默认值一起提交，使用仓库本地身份Liu3Beng-A / liu3beng@icloud.com；不连接串口、写实际节点参数或烧录。
- **当前改动**：`dummy-motor-fw/UserApp/protection_config.cpp:36`仅普通TRIGGER_MS默认300→100ms、补误差电机圈单位注释；`串口助手_old.py:32`同步100ms编辑初值。本文、方案和测试流程同步；CAL默认参数/流程、PID、总限流、控制热路径、Flash/EEPROM格式和参考固件未改。
- **已确认默认**：J1～J7检测开；电流命令门槛初始化限流40%（最低.1A）、实际速度.02电机r/s、关节跟随误差.5°/地轨.5mm、连续100ms、启动200ms、滤波10/20ms；回退5°/5mm、速度3/.5电机r/s、加速度5r/s²、回退及保持上限=初始化总限流、实际容差.001电机圈、超时5秒、REBLOCK100ms、GRANT_WAIT500ms。J8排除、主控START沿现流程设置实际位置边界。
- **事实及排除**：`Ctrl/Motor/stall_protection.cpp:53–65`先生成默认再优先加载合法EEPROM，`UserApp/protection_config.cpp:121`验证记录。重新烧录不覆盖保存的敏感试验参数；`OnMotionLimitChange`提高总限流不自动按40%提高已有门槛。本轮不迁移EEPROM、不擅自提高J7总限流。100ms仅缩短连续候选200ms，不能承诺碰限位总延迟100ms；新默认尚无各轴带载实测。
- **验证完成**：四电机Ninja重新链接成功，主控Ninja确认最新产物无需重编译；配置/EEPROM165项（含1/2.5/3A的40%、低电流下限、J8排除、CAL保持150ms及保存的50ms优先于新100ms默认）、末端/快照主控123/电机182717、隐藏真实Tk默认/诊断63及顺序/分包/模式410均通过；git diff --check通过。断言数含周期循环，不能当实机用例。保留既有RWX告警；五BIN/HEX都在原应用区，35/42最小Flash余量20B。
- **最新产物**：主控434016B / SHA256 cd1ca6e158b9a557cb05f59d51e5ac6e8b7fb2aaa8e53e7df840605439f9819d；35/42均97260B / f9ccfd8fb44c177420a25919fb72410f1dffb1f4cdcc06fe1909d504ffb4a6f0；57为97252B / 79331da4908c9a1718267b888f95a5a577eeaa9cd82ee2292a26bab061e0e4ab；gripper为97212B / f87a1fdd9fe8e6266569725a35d77860ba004990f6a0984ae9893234b8fb3284。路径仍各原build目录，ignored清单endpoint_fix_artifacts.json已更新，原串口助手.py校验未改。
- **提交完成**：`424c9ed fix(protection): 修复堵转接管与末端误报并确定日常默认值`，19个生产/文档文件，作者及提交者均仓库本地Liu3Beng-A / liu3beng@icloud.com。包含此前已验收的目标确认与保护接管、回退入口及STOP幂等、末端误报与LIVE/LAST、40%阈值和本次100ms默认；提交后已核对工作区干净。ignored测试脚本/构建产物未加入版本库。新默认实机验收与此前修复反馈仍分开记录。此提交之后仅本文记录校准讨论，暂不再提交或修改校准代码。
- **下一步**：提交完成后只讨论CALIBRATION，先读实际实现；用户已确认失能发起、J1～J6重新标REST_POSE、J7正向限位250→0、J8负向限位0→100、全部成功后自动失能保存、START busy不挂起、失败取消不保存不完整校准。校准代码修改仍须另列文件/用途获确认。

## CALIBRATION重新讨论：提交后只读初核（2026-10-07）

- **已实现流程**：`dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:748/561`入口检查主控失能，带本次probe逐轴核验有效反馈及实际DISABLED，再逐节点CAL_BEGIN再次核验。`:638–692`六关节静止捕获坐标（主控把零电机位映射REST_POSE），J8闭合寻端/返回100，再J7逻辑正端寻端/返回0，随后八轴失能确认、顺序逐节点Flash保存及CAL_END，全部成功才`!CALIBRATION ok`。
- **实际端点采集**：`dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:628/729–747`初次寻端、停稳、退让、慢速复触，复触位置与首次比较后才标J7电机-50圈（主控反向映射+250mm）或J8电机0，回到实际反馈目标。J7退让1mm、J8退让2开度。不是只看规划器完成；返回需`:809`实际位置容差及50ms稳定窗。
- **操作兼容**：`ascii_protocol.cpp:14`校准期间START直接busy不挂起；`protection_coordinator.cpp:308/335`STOP/DISABLE取消并禁止后续未开始保存/成功回执。Flash已开始写入不能撤销，已有设计明确用saved/pending掩码报告部分保存，不能承诺八节点原子回滚。old助手`:1703`已有失能提示、确认弹窗与阶段显示，不自动入口DISABLE。
- **接触参数仍需单独讨论**：`UserApp/protection_config.cpp:52–64`CAL_CONTACT_RATIO=.8、CAL_TRIGGER_MS=150、CAL_STARTUP_MS=100、实际低速.02电机r/s、误差.02电机圈；寻端/返回/复触速度.5/.5/.25r/s、加速度5。J7校准限流默认初始化总限流，J8默认min(1A,总限流)并受主控HAND电流再次限制。普通CURRENT_A=40%和TRIGGER100没有改动这些CAL值；滤波仍共享现有普通10/20ms设置。
- **推荐推进、未决策**：先在失能时分别读取`#CAL_GET 7`和`#CAL_GET 8`，每条等完整输出再发下一条，以实际EEPROM为准；不先要求机械寻端。之后独立确定校准低能量限流、接触比例与持续时间，保留复触/实际返回及失败边界，不能根据整机电源电流直接推断focCurrent门槛。不得自动把普通参数复制到CAL。本轮仅初核，不声称现有校准已实机验收。

## 当前交付：普通运动末端误触发与首次堵转快照已修复（2026-10-07，待用户烧录）

- **授权**：用户确认前节分析后明确“同意，开始修改”。本轮仅七个生产文件及下述三份文档，保留此前全部未提交工作；未连接串口、改实际参数、烧录或commit。
- **最新改动**：电机`Ctrl/Motor/motor.cpp/.h`新增共享普通目标范围判断，SetPositionSetPoint使用普通容差；TickPositionCompletion通过原50ms实际完成后ResetDetection并标记COMPLETED，旧目标/PID/积分和完成/OLED锁存保留。`Ctrl/Motor/stall_protection.cpp:556`实际及参考均在普通目标范围时ARRIVAL，清连续候选而保留意图；参考与实际只到一方、目标范围外或坏反馈不能通过。退出范围从零重新计时。目标启动与退休不借用回退容差，本轮只绑定主控COMMAND_POSITION链路，直接底层Trajectory/PWM/STEP_DIR旧退出保持原行为。
- **冻结证据**：`stall_protection.cpp:148/174/466`首次本地普通STALL在STOP改参考/清计时前一次性捕获模式/意图/反馈/命令电流/速度/soft-real及goal-real误差/计时/阈值/事件/方向。RAM原始记录与查询副本分开，相同请求重试保持第一份一致数据，新的事件不能撕裂在途查询。只冻结普通STALL；同伴STOP、回退失败、校准接触或其他故障不覆盖该普通记录。START/DISABLE/新运动后保留至下一次本地普通STALL或重启，无记录明确no_trigger_record。
- **主控/助手/协议**：共享`firmware_new/protection_protocol.h`诊断版2、24字段、LIVE/LAST选择及ARRIVAL/COMPLETED枚举，原控制协议/EEPROM版1不变。主控`Robot/instances/protection_coordinator.cpp/.h`严格接收`#STALL_STATUS <1..7> [LAST]`，选择对应只读快照、收齐后打印来源/事件/参考速度/目标误差/方向/原因；不改保护接管或解锁。`串口助手_old.py:1213/1280/1287`新增“读取触发快照”，允许正确LAST经过现有保护门禁，拒绝0/8/附加参数；收到冻结日志不伪造实时STALL或解锁状态。隐藏真实Tk实际顶部请求宽587px，适合原800px最小窗口；读取验证了实际文件与控件。`串口助手.py`SHA仍D07D3E...0D7E4，未改。
- **常规决策/边界**：末端范围暂停候选允许参考尚有最后少量运动，必须参考及实际同时进入普通允许范围；完整完成仍等参考精确终点/速度0及原反馈稳定窗口。这比只在规划结束时暂停能覆盖最后几个周期，未放宽完成或STOP。新快照聚焦首次普通STALL，不扩修内部REBLOCK、校准接触或其他故障快照。原始直接电机非COMMAND_POSITION模式退出保留；不能把本轮验收推广到这些未专项验证模式。参数默认、EEPROM、PID、限流、回退速度/距离/加速度、5秒超时、连续条件复位、全七轴联动、方向/空间约束和提前START决策不变。
- **编译处理**：新增记录最初使电机超95KiB区164B，仅低频CAN SetPositionSetPoint沿用局部Os，新诊断/一次性捕获沿用已有Os；连续20kHz闭环、检测及稳定窗优化未变，不改CMake、Flash分区/校准表/EEPROM区域。最终主控BIN434016B、普通RAM56384B；四电机RAM10592B，35/42/57/gripper BIN97260/97260/97252/97212B，最小应用余量20B，均成功生成ELF/HEX/BIN。既有RWX与编码器基类缺return告警保留。
- **目标离线证据**：从最终生产源码提取的`run_endpoint_fix.cmd`，主控123、电机182717项断言通过；覆盖各轴/正负容差边界、敏感参数、实际或参考未到位、末端未规划结束不能完成、出入范围计时清零、振荡/坏反馈不能完成、完成后保持不重触发与新目标重新武装；LAST无记录/计时与方向捕获/失能使能与取消同伴不覆盖/新事件不撕裂旧查询/协议与语法。新增隐藏真实Tk54项，原顺序/模式/分包410项通过。断言数含周期循环，不是独立机械用例数量。
- **兼容回归**：最终回退入口与STOP重试41815主控/45817电机；普通完成组合38830/45501；保护接管161797主控、57采纳/限幅172；CAN5817主控/54657电机；配置/EEPROM132，均通过。保留全七轴确认、串行实际回退及早START/STOP/DISABLE优先兼容；本机反馈/传输模型不能冒充实机。脚本与日志在忽略的`firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/`，清单`endpoint_fix_artifacts.json`记录五产物SHA及七个生产源文件SHA。
- **文档更新**：本文及`docs/堵转保护与自动回退及校准方案.md`记录实现细化/边界，`docs/堵转误报修复与实机测试流程.md`首节给最新完整步骤，不用旧轮次推荐替换本轮控制变量。
- **用户测试**：主控+J1～J7配套烧录（35/42按实际类型，J7用57），J8可一并更新；不整片擦除。先DISABLE/GETI/GET_PID/STALL_GET记录现值和LAST无记录，保留上一轮J7 1A/.2A/.001圈/50ms/10ms设置；自由行程内35档SEQ HOME/RESET/正负3～5轮，每条ok后观察1～2秒无STALL，再验证STOP/START保持及新目标。异常结束/失能后查询LAST与LIVE，不再持续撞硬限位取证。正常通过后如需验证真实阻挡，用可立即撤开的低能量软阻挡及5mm反向空间。没有实机测试，不宣称误报全部消失或REBLOCK/机械问题已解决。

| 对象 | BIN路径 | SHA256 |
|---|---|---|
| 主控 | `E:\Dummy-code\firmware_new\dummy-ref-core-fw\build\Core-STM32F4-fw.bin` | `cd1ca6e158b9a557cb05f59d51e5ac6e8b7fb2aaa8e53e7df840605439f9819d` |
| 35/42 | `E:\Dummy-code\firmware_new\dummy-motor-fw\build\motor-35.bin` / `motor-42.bin` | `4a1533292323c643fbdd5682356f603037db5e0add18533a4f17da7f81636067` |
| J7/57 | `E:\Dummy-code\firmware_new\dummy-motor-fw\build\motor-57.bin` | `07e446292b12043d438c2154a9599da838c45ee3673be9db009b3e31e2da1786` |
| J8 | `E:\Dummy-code\firmware_new\dummy-motor-fw\build\motor-gripper.bin` | `e5e59c19141ce76e961d443001ba85ae3584ab1ee88b858591b9ea0e6ab03c4d` |

## 当前分析：回退入口已验收，普通 MOVEJ 末端误触发 J7 堵转（2026-10-07，14:58～15:06 日志）

- **本轮范围**：用户确认J7碰限位回退反应变快，要求根据新日志与固件详细分析普通运动末端触发STALL。只按AGENTS 5.2更新本文，并在既有忽略build分析目录生成/运行离线验证；未改生产固件、默认配置、EEPROM、串口助手或其他项目文档，未重新编译发布产物、烧录或commit。
- **回退入口实机已改善**：前两次硬限位测试仍使用ERROR_R=.1/TRIGGER_MS=300/STARTUP_MS=200，J7 phase1→phase3分别为15:00:13.350→13.493（143ms）和15:00:25.753→25.918（165ms），随后均phase4成功。与此前481ms相比，日志事件至回退入口减少约316～338ms；不是测得物理接触总反应时间。此项用户已确认解决，暂不继续缩短入口确认或改STOP判稳。
- **本地发起节点**：整份附件共有11次phase1/STALL，全部J7，其中3次250mm硬限位测试、8次普通MOVEJ/RESET测试。J1～J6的phase6/7为联动停止/保持，不是各轴独立发生堵转。8次普通运动的J7最终均NO_DIRECTION；其中MOVEJ id16在15:04:11.946返回ok，12.472才报告STALL（526ms后）；RESET id20在15:05:22.122返回ok，22.682才报告STALL（560ms后）。其他多次在正负往返末端约3.9秒触发，取消尚未返回ok的任务。
- **实际参数变化及失败设置**：14:59:26的CURRENT_A=.4，以及当时ERROR_R=.001/TRIGGER50/STARTUP10设置均被require DISABLE拒绝；CURRENT_A全程仍为.2，总限流1A。15:00:56/15:01:00/15:01:01失能后才成功改ERROR_R=.001（5mm/圈对应.005mm，原.1为.5mm）、STARTUP10（原200）、TRIGGER50（原300），并保存；15:01:33 GET确认。15:05:12～13试图恢复ERROR_R=.01/TRIGGER300/STARTUP200再次全部被拒绝；15:05:24失能后直接SAVE，未重新SET，故后段仍为极敏感设置。SPEED_RPS=.02对应电机轴.02圈/秒、地轨.1mm/s；滤波10/20ms、BACK_POS_TOL_R=.001（约.005mm）、REBLOCK100未改。
- **已确认代码缺陷：普通完成与检测退出使用不同容差**：`firmware_new/motion_completion_protocol.h:30–37`普通J7目标容差1024内部单位=.1mm，稳定范围204单位≈.02mm；`dummy-motor-fw/Ctrl/Motor/motor.cpp:740–781`规划器抵达且实际编码器在该容差内，连续50ms稳定并满足平均速度后允许SETTLED。主控`dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:663–666/1470–1474`要求七轴当前目标、实际位置、新鲜SETTLED证明后才完成，不是仅规划器走完或沿用旧目标ok。与此同时`Ctrl/Motor/stall_protection.cpp:92/102`把positionTolerance绑定BACK_POS_TOL_R；`:557`只有softVelocity=0且真实误差≤该约.005mm容差才清motionIntent；`motor.cpp:722–725`普通目标启动也复用这个回退容差。普通完成并未通知检测退出，存在“.005mm<残余误差≤.1mm，运动已完成但堵转检测仍开启”的明确区间。
- **为何集中在末端**：`stall_protection.cpp:528–554`只检查位置模式和motionIntent，再对abs(focCurrent)、abs(estVelocity)、softPosition-realPosition分别滤波/判门槛；没有把普通实际到位证明或末端容差纳入检测门禁。末端规划速度为零、实际速度很低、位置保持仍有非零电流命令，残余误差仅需≥.005mm，连续50ms就可能满足全部条件。CURRENT_A=.2只是1A上限的20%，不是此前尝试的40%；focCurrent是控制输出命令，不是电源表电流或实测绕组电流。STARTUP10只影响新运动开始，不能直接解释几秒后的触发；间歇条件失效仍由ClearDetectionTimer清零，本次没有证明计时累计bug。低电流/机械/PID可能影响真实收敛，但不能据现有日志认定这些是唯一根因。
- **NO_DIRECTION的意义及证据边界**：`stall_protection.cpp:446–450`取已有规划方向，并要求方向已持续至少5ms、误差方向与其一致；误差翻转（例如末端越过目标、保持反向修正）会将方向置0。`:644`内部GO拒绝无方向并保持保护，避免乱退。末端误判会因此表现为STALL→STOP→NO_DIRECTION，而非退开；不能据此认定回退模块本身未执行或应删除方向保护。日志未冻结每次触发误差/方向，故不能声称实机八次均已证明超调；已证明这一代码路径与现象相符。
- **真实源离线复现**：`build/analysis/protection_regression/generate_endpoint_analysis.py`直接包含当前motor.cpp/stall_protection.cpp及真实完成函数，以J7有效静止反馈、±200内部单位（±.019531mm）残差、已持续正向运动的历史方向作为规定输入。先以.1A命令使连续50ms普通SETTLED通过并确认motionIntent仍真、0x23返回SETTLED；再给.3A命令，ERROR_R=.001/TRIGGER50两种残差均50ms后LOCAL_STOPPING/STALL，未到目标侧方向+1、越过目标侧方向0。对同样输入恢复ERROR_R=.1/TRIGGER300后1秒无STALL；共26项断言通过。结果`endpoint_analysis_result.txt`。此模型证明固件允许误判路径，不是实机PID/摩擦/抖动波形测量，也不能替代触发瞬间反馈取证。
- **回退再次受阻的关联**：15:01:15.810开始回退，17.992报REBLOCKED（约2.182秒，5mm/.5rps理想匀速约2秒）。`stall_protection.cpp:730–760`回退再次受阻复用普通检测ERROR_R/SPEED_RPS，电流门槛min(detectorCurrent,.8*backCurrent)=.2A，REBLOCK_MS=100；极小普通误差阈值也会影响回退末端。可能是末端收敛误判，也可能实际再次受阻，现有日志不能区分，不直接判定机械再次堵住或超时。15:01:42快照在DISABLE后采样（`:148–169`），不是触发瞬间快照，不能用intent=0/电流0推翻之前真实50ms计时。
- **推荐后续，尚未执行**：先将极敏感诊断参数与常规运行参数分开；恢复此前成功的ERROR_R=.1/TRIGGER300/STARTUP200作为对照，CURRENT_A=.2与改成.4的作用单独验证，不自动写EEPROM。代码侧把普通运动检测退出接入本节点真实到位证明，普通起始/末端门禁使用普通目标容差，回退容差只服务内部保护到位；末端处于普通允许误差内的小修正不能判作普通堵转，未实际到位仍须检测，不能仅凭softVelocity=0或规划器完成关闭检测。保留连续条件失效清零、真实编码器证明、联动STOP、限位及方向保护；增加首次触发瞬间的冻结诊断以区分末端残余误差/超调/真实阻挡，再决定回退再次受阻门槛是否需独立容差。普通完成时即清检测计时/运动意图与旧方向；新的实际运动任务重新建立意图和方向。具体方案、文件和边界仍应向用户列出后获得改动授权。
- **已排除/未证明**：本轮本地触发节点是J7而非此前J6，日志没有TARGET_READY_TIMEOUT；尚未证明硬件负载、摩擦或抖动的具体原因。日志的CURRENT_CMD_LIMITED仅表示输出命令持续限幅，两个90档全行程均最终ok无STALL，不能与机械堵转等同。前一轮入口调度优化未改普通检测阈值/PID/完成判据，参数变更后暴露了已有边界不一致；目前不宣称每次真实末端反馈或全部机械问题已证明。用户无需再用持续撞硬限位验证这条软件路径。

## 已完成：回退入口确认调度优化及失败后 STOP 兼容修复（2026-10-07，待用户烧录）

- **本轮授权**：用户在确认逐项优化方案后要求“执行改动吧，完成后我来烧录测试”。只改本节列出的八个STOP/保护源文件及本文；未获commit授权，不提交；未连接真实串口、写节点参数、烧录或执行实机运动。保留本轮开始时已有的其他未提交改动，包括独立磁编码器校准入口。
- **主控职责与实际改动**：`firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp/.hpp`将SendSoftStop改为返回实际CAN提交结果；`Robot/instances/dummy_robot.cpp:816/1045`只在新操作/新确认阶段初始化游标，每个时隙公平发送一个未确认节点，QUERY同步回包不再从J1重发全组。实际提交失败不更新成功发送时间、不占协调器已用发送预算，仍保留待发送；按游标轮转重试，缺低ID节点不阻塞健康节点。保留首帧ALL DISABLE、后续未确认节点重试，以及锁定后持续位置查询；未确认/FAULT/非法位置/旧token不能授权回退。发送前在临界区校验epoch/token/action，防止用户STOP/DISABLE切换后残留发送。
- **保护时隙与首次GO**：`Robot/instances/protection_coordinator.cpp/.h:489`仅PROTECT_STOP开放每四个5ms周期中的三个给STOP确认，另一个留PREPARE/续租；普通STOP/START/DISABLE维持原常规时隙。主控每周期最多接受一个STOP/协调保护帧，已确认节点不继续占确认轮询。首个GO在任务与WAIT_GRANT匹配时获得一次额外发送机会，失败后回归常规20ms重试，不无限抢占续租/事件ACK；保留七轴全部STOP确认及串行回退。未新增CAN命令、字段或独立解锁入口。
- **电机固定保持接管**：`firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp:493`只在WAIT_GRANT/PEER_HOLD、已有HOLDING、无待处理STOP/GO或内部运动、正常使能位置模式、反馈有效、无配置/驱动故障、当前位置距固定捕获点不超过8内部单位且估计速度不超过.01电机r/s时，内部STOP_PROTECTION新token直接绑定已有保持；不重启减速、不清积分、不重新捕获位置。其他情况仍按原加速度软停并通过原50ms实际编码器窗口，未放宽首停证明或回退实际完成判据。
- **失败后重复STOP/CANCEL**：`Ctrl/Motor/motor.cpp/.h`保存动作类型+token作为幂等键，在取消保护之前识别相同STOP重试，避免反复清50ms稳定窗。类型纳入键，使与内部STOP_PROTECTION相同token的用户STOP仍能取消回退；DISABLE优先不变。`Ctrl/Motor/stall_protection.cpp:324`相同任务CANCEL重试不重复启动软停。失败不自动执行之前挂起START；新的显式START须完成停稳及现有反馈/位置同步，保持新捕获点，不恢复旧目标。
- **终态原因保留**：`stall_protection.cpp:407/471`失败后同任务迟到GO/PREPARE返回实际失败原因，普通STOP/CANCEL保留最初终态原因。主控Node分别保存EVENT原因与CONTEXT响应原因，EVENT日志/保护失败使用EVENT原因，避免RETREAT_TIMEOUT被BAD_TASK覆盖；CAL/配置事务仍使用其响应原因。新的事件/任务可以记录新原因，不将旧故障永久带到下一次操作。
- **常规决策与方案偏差**：没有改检测电流/速度/误差、连续时间、滤波、PID、总限流、回退距离/速度/加速度/5秒默认超时及EEPROM格式，也未改普通MOVEJ同步/finish、J8排除、RGB、串口助手或参考版。主控`dummy_robot.h`本轮实际无需改动。落实了普通CANCEL重试同样幂等，这是STOP兼容修复的补充。首次GO优先预算限定为一次机会，而不是原分析副本中可持续WAIT_GRANT抢占；这样避免丢GO包时饿死续租。电机应用区最初超出284B，仅对低频RequestSoftStop、BuildProtectionEvent/Context及CancelInternal沿用局部Os体积优化，20kHz控制/检测/停稳循环编译选项和Flash分区未改；最终35/42最小余量372B，保护编码器表/EEPROM保留区。
- **最新离线证据**：从最终生产源重新生成临时本机测试，`build/analysis/protection_regression/run_retreat_entry.cmd`通过，主控41800、电机45792项断言。同旧模型（七轴已保持、无丢包、邮箱占一个5ms周期，未计电机重复50ms窗口）八种任务对齐首个GO105～110ms，STOP/QUERY14帧（QUERY7），旧模型450ms/42帧。真实motor+0x0A处理路径中，IDLE/PEER_HOLD/HOLD_FAILED三类状态每70ms重试同一STOP30次均HOLDING30/30、重启0；旧版后两类重启30/30。验证固定目标/积分保持、移动/偏移/坏反馈/待GO不能跳过软停、同token用户STOP取消、丢CANCEL确认、回退失败直接显式START保持解锁、编码器无效仍可DISABLE；主控验证CAN提交拒绝、缺节点不GO、首次GO拒绝后续租间隔≤120ms、错误原因不覆盖、失败清挂起START。以上为规定反馈/传输模型，不是实机耗时或机械验证。
- **兼容回归**：真实源码普通运动/STOP/保护/校准组合主控38815项、电机45476项；保护接管ISR/晚到事件/用户取消主控161774项、57饱和/目标采纳172项；主控CAN5817、电机CAN54657项，均通过。最后对ENABLE坏反馈动作键的小修和新增断言已重跑41800/45792目标回归并重新编译四电机；各回归并非相互独立，不把断言数相加宣传为用例数。无新增生产测试文件，脚本与结果在忽略的build分析目录。
- **编译与交付**：主控、35/42/57/gripper五目标Ninja成功；主控FLASH/BIN433176B、普通RAM56384B，四电机RAM10472B、BIN96908/96908/96900/96860B，全在原区域内。既有RWX LOAD及编码器基类缺return告警没有扩修。BIN/HEX均在原build目录，详见下表；SHA/临时源码清单位于`firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/retreat_entry_artifacts.json`及`retreat_entry_source_manifest.json`。本轮源文件SHA对照只改变上述八文件，`串口助手.py`、old助手、默认配置、协议头及其他源文件未变。

| 对象 | BIN路径 | SHA256 |
|---|---|---|
| 主控 | `E:\Dummy-code\firmware_new\dummy-ref-core-fw\build\Core-STM32F4-fw.bin` | `cd63d4d2bb23ae21a0c968ba378f8a8179fd46a3b930e730c2d9634c65ccf7ca` |
| 35 | `E:\Dummy-code\firmware_new\dummy-motor-fw\build\motor-35.bin` | `40f0fc1db2300565f3e01e1a8fe7438a253f79ba9f55e7712f6c9ff2d2c623d5` |
| 42 | `E:\Dummy-code\firmware_new\dummy-motor-fw\build\motor-42.bin` | 同35 |
| J7/57 | `E:\Dummy-code\firmware_new\dummy-motor-fw\build\motor-57.bin` | `340cfe3a1ce1c47ceef17169159cf992be881712db26d3347e47f20d9c64058e` |
| J8/夹爪 | `E:\Dummy-code\firmware_new\dummy-motor-fw\build\motor-gripper.bin` | `bdd3e74a845289689471642b29a8c8ace2169fbf814d32a00239a649edbc866a` |

- **用户烧录范围**：主控和J1～J7配套更新（J1～J6按实际35/42类型，J7用57），J8可一并烧录新gripper以验证共用STOP/CANCEL的校准兼容；J8仍不参与普通堵转或七轴软停。保留现有EEPROM与磁编码器校准数据，不进行整片擦除。合法EEPROM优先，本轮烧录不更改用户实际参数。
- **实机测试顺序**：①失能后回读GETI/GET_PID/STALL_GET 7记录现值，确认BACK_MM=5、BACK_RPS=.5、BACK_TIMEOUT_MS=5000；维持本轮成功配置及TRIGGER_MS=300，先不降时间。②START→HOME、SEQ下原35档正负/REST点位3～5轮，逐条等ok，确认无本地STALL、同步及OLED完成正常；普通移动中STOP→STOP locked→START应保持停点。③仅在上述正常后，在安全行程内进行一次低能量可让开的软阻挡（J7维持已验证1A、低速小行程、有5mm反向空间），出现STALL后解除阻挡，观察实际反向约5mm及phase3→4、STALL locked；不撞硬限位、不用20mm/5秒组合、不通过持续啸叫取证。④后续分轮验证回退中提前START（全部成功才Started ok且不恢复旧目标）、回退中STOP（取消并保持，新START解锁保持）、回退中DISABLE（全八轴失能）。⑤若自然发生回退失败，保留原因后直接显式START验证，不先DISABLE绕过；失败前已挂起的START不应自动使能。每轮有异常停止继续受阻。保留毫秒日志，以首次J7 phase1到phase3与旧481ms比较；总反应仍包含300ms连续判定，串口间隔不等于物理接触时间。
- **尚未完成/已保留问题**：实机烧录、实际回退入口耗时及机械反馈由用户验收；本轮不增设全机停稳总体截止、不放宽抖动判稳，尚未证明停稳/掉线异常时一定能进入回退。USB历史异常仍单独观察，未据本轮模型宣称修复。没有保存测试参数或commit。

## 当前任务：串口助手增加指定节点磁编码器校准（2026-10-07）

- **授权范围**：用户确认执行此前列出的三个文件改动：主控 `UserApp/protocols/ascii_protocol.cpp`、根目录 `串口助手_old.py`、本状态文档；随后明确授权将刚才的改动单独 commit。仅按片段提交本功能及本节状态记录，保留原有未提交改动；未授权烧录或实机运动。
- **已确认事实**：电机 `UserApp/protocols/interface_can.cpp:80` 的 `0x02` 设置 `encoderCalibrator.isTriggered`；`UserApp/main.cpp:143` 上电时双按钮按下设置同一标志。StdId 为 `(node << 7) | 0x02`，接收入口要求 DLC 至少4；本次发8字节全零标准数据帧。`Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp` 正反转采样、写入校准表，成功后 `HAL_NVIC_SystemReset()`；没有校准完成 CAN 回执。
- **已排除方向**：不复用 `!CALIBRATION`，该指令是整机坐标置零及 J7/J8 寻限位。电机固件已有磁编码器校准支持，无需修改；Fibre 的 `do_calibration` 定义不等于现有 ASCII 串口入口。节点0是广播，拒绝磁编码器校准 ALL。
- **最新改动**：主控 USB 新增严格匹配的 `#ENC_CAL <1..8>`，仅在失能且无运动/保护/队列任务时提交 CAN `0x02`；通过现有 `TrySendProtection` 使用局部全零帧并检查提交结果，回复 `ok ENC_CAL [n] dispatched` 或发送错误，UART4 明确 forbidden。上位机电机参数区增加“磁编码器校准（选中节点）”，沿用节点选择并在0禁用；校准前确认节点/运动/写表/重启，按钮及手输命令均要求已收到失能状态且无活动任务。回复仅提示 CAN 帧提交，不宣称校准完成。
- **验证结果**：主控 `dummy-ref-core-fw/build` 中 Ninja 编译成功，生成 ELF/HEX/BIN，FLASH 433632 B、普通 RAM 56384 B；链接器报告 LOAD RWX 警告，本次未改链接脚本。使用真实隐藏 Tk 控件及模拟串口，验证节点1～8发送、0禁用、非法节点拒绝、取消确认不发送、使能/未知/锁定/任务忙拒绝、弹窗后状态再次检查、发送线程实际写出 `#ENC_CAL n\n`、dispatched回复仅打印发送提示；初次测试发现确认分支写反，已修正并通过复测。提取真实 ASCII 解析函数及真实 `CtrlStepMotor::TrySendProtection` 做临时本机编译测试，验证节点路由、StdId/DLC/全零数据、空参数/ALL/符号/溢出/多余参数拒绝、门禁、CAN失败/空节点错误、UART4 forbidden、不吞无关指令；全部通过。临时本机测试产物已清理，不连接真实串口或机械臂。
- **未完成/待确认**：改动与离线验证完成，按用户授权单独提交本功能。尚未烧录或实机验证；使用前需烧录本次主控 HEX/BIN，重新启动 `串口助手_old.py`，失能并等 `Disabled ok`，选择单个节点后点击磁编码器校准并确认。主控 USB 接口支持此命令，UART4仍禁止配置校准。校准完成与成功需观察电机；当前协议没有完成状态，未添加计时猜测或自动 START。

## 恢复分析：J6 重装后普通运动复测通过，继续回退入口延迟方案（2026-10-07，13:59～14:00 日志）

- **最新用户反馈与范围**：用户此前反馈J6啸叫/疑似松动并准备重新装配，本轮反馈测试没有问题，要求重新开始之前的分析。HOME id1完成4363ms，SEQ MOVEJ id2～6分别2210/3932/3870/3925/2396ms，均正常最终ok，日志没有STALL或保护失败。支持装配问题这一方向，但没有触发瞬间快照，不据此宣称唯一根因已证明或所有保护异常路径均正常。恢复的是此前未完成的“每项回退入口优化作用及预计节省时间”分析；未授权生产修改、参数写入、发布编译或commit。
- **实机基线仍采用12:30成功回退**：J7 phase1到phase3为481ms；本地WAIT_GRANT到回退439ms；最后同伴PEER_HOLD到主控STOP locked352ms；STOP locked到打印RETREATING22ms。碰限位时刻没有记录，且串口打印有调度延迟，因此不能把这些间隔当作完整物理反应或精确CAN发送时间。连续检测TRIGGER_MS=300在事件前，GRANT_WAIT_MS=500是截止而非固定等待。
- **逐项离线对照**：只在忽略的`firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/`生成`latency_baseline/fair/pending/fast_slots/go_priority.cpp`临时副本并用MSVC本机编译执行，未修改ARM生产源码。抽取方法与当前真实`DummyRobot::PollSoftStop`、`CtrlStepMotor::SendSoftStop`、`ProtectionCoordinator::Poll`一致；模型规定七轴起初均HOLDING、回包有效、无丢包，主控发送令牌占用一个5ms控制周期，电机再次STOP的真实50ms稳定窗未建模。基线首个GO450ms、接受STOP/QUERY42帧（QUERY7）；公平单节点游标且不因同步回包从J1重来170ms/14帧（QUERY7）；追加跳过已经确认节点仍170ms/14帧；再把PROTECT_STOP中的发送机会改为四周期三次、间隔5ms且为协调上下文留时隙，110ms/14帧；追加首个GO优先仍110ms。对应成功模型断言335/275/275/263/263项通过。这是规定模型的单路径调度对照，不是完整故障回归或已发布新固件。
- **预期作用与量化边界**：公平发送在该模型减少280ms；跳过已确认节点在该理想场景额外减少0ms，作用主要是少用总线/不拖累剩余节点；保护专用时隙再减少60ms。首个GO优先在本次对齐场景额外0ms，不声称一定省20ms，主要约束其他报文竞争导致的最坏等待；实施时必须限制优先级预算，保留续租、事件ACK、PREPARE及STOP/DISABLE，不以一直WAIT_GRANT无限抢占。真实发送提交失败须立即保留该节点待发送并在后续控制周期重试，不能将失败算成功；忙/丢包下节省时间不作固定承诺。将轮询10ms单独改为5ms但仍只开放原隔周期时隙不会达到这一效果。
- **已停稳节点避免重复软停**：`dummy-motor-fw/Ctrl/Motor/motor.cpp:479–492/605–629`在新token的STOP_PROTECTION接管时仍重新进入减速并清稳定窗；主控`dummy_robot.cpp:908–915`QUERY后绑定token+1。建议仅对同一保护过程、反馈有效且已经捕获固定保持位置的节点绑定新确认token，保留位置目标及保持电流，不重新软停。实际尚在运动/无可靠停稳证明的节点仍执行原减速/50ms编码器证明；普通用户STOP必须能取消回退。理论可免除每轴一次额外50ms稳定窗，轴间并行且与通信重叠，总回退入口不能按七轴乘50ms，静止正常场景整体额外收益估计0～50ms，未做实机测量。
- **参数调优独立进行**：先保留本轮已能正常运行的PID、限流、电流/速度/误差判据及300ms连续检测，验证调度优化；以后TRIGGER_MS 300→50可在同一连续条件起点下减少250ms，但会增加短暂受阻触发风险，需要单独误触发测试。条件失败清零及真实编码器停稳确认不删除，不自动改用户EEPROM或默认参数；STARTUP200ms不作为每次碰限位必减的固定时间。
- **失败解锁与原因覆盖另列**：前节已复现的普通STOP取消内部保护时相同token重试不断重启稳定窗，以及终态故障原因被迟到CONTEXT拒绝覆盖，仍属于待修复兼容性缺陷，不因这轮普通运动成功而排除。它们修复正常首次回退入口的预计收益为0ms，作用是失败后STOP可完成、显式START可保持解锁及诊断保留真实原因；失败不自动执行先前挂起START。前节J1/J4停稳门槛卡住属于STOP_PROTECTION，不能混同普通STOP重试bug。
- **拟实施文件与用途（待用户确认）**：主控`Robot/instances/dummy_robot.cpp/.h`维护一次操作的公平游标/未确认节点/失败重试；`Robot/actuators/ctrl_step/ctrl_step.cpp/.hpp`向上返回STOP实际提交结果；`Robot/instances/protection_coordinator.cpp/.h`调整保护专用时隙并保证必要上下文/续租/首个GO发送。电机`Ctrl/Motor/motor.cpp/.h`实现已证明保持的内部接管与普通STOP幂等取消；主控协调器与电机`Ctrl/Motor/stall_protection.cpp`按最终设计处理终态原因及迟到请求。不改普通MOVEJ同步/finish、J8排除规则、RGB或参考固件；本轮只更新状态文档，无固件修改或提交。

## 前序问题：普通 MOVEJ 五次均由 J6 发起堵转保护（2026-10-07，12:35～12:37 日志）

- **最新用户反馈**：用户确认J6啸叫，怀疑松动，准备重新装配。啸叫与当前控制命令饱和/低反馈速度的现象相容，但尚未证明机械松动是根因，也不能仅据此宣称保护误报。先暂停运动，装配完成后保持PID/限流/堵转参数不变：失能静态查询J6参数→START静态观察→只改J6目标的小范围低速双向运动→确认无异常后恢复原35档整机点位。其他轴目标使用当时OLED实际位置，不凭默认REST假定当前姿态；本轮无需碰物理限位。SEQ下每条运动等最终ok，再查询MOTION_STATUS/STALL_STATUS 6，出现啸叫或STALL立即结束当轮，不重复同一目标/反复START。留意承重支撑后DISABLE。需要完整带毫秒日志和J6实际运动观察；没有授权新固件修改或提交。
- **用户新的优先级**：明确要求把回退入口延迟记入文档，先跳过这项讨论，改为分析 MOVEJ 触发STALL。前节保留481ms拆分、450ms基线离线证据和拟优化方向，当前暂缓延迟优化、参数降时讨论及实现。被用户打断的“逐项改动可缩短多少”分析尚未完成，只做了源码读取，未生成新原型、执行改动或编译；没有得到逐项节省时间的测量数据。只把轮询10ms改成5ms而保留隔周期发送时隙，并不会加快STOP确认，这是已读代码约束，不能擅自宣传节省一半时间。
- **新日志与本轮权限**：附件 `C:/Users/Liu3Beng/.codex/attachments/b9ab38fd-27c7-40bc-9333-59acd1beefc4/已粘贴的文本.txt`。本轮用户要求分析及记录问题，未授权固件修改、参数写入、编译发布或commit；仅更新本状态文档。不要求继续反复运动或碰限位。
- **已确认源轴**：MOVEJ id2/4/5/6/7各次均为J6 `phase=1 reason=STALL` 后进入 `phase=2`；其中id2为INT、其余为SEQ。J1～J5/J7均phase6/7，是收到J6事件后联动停止/保持，不代表这些轴都本地堵转。日志总计5次（不是6次）：id2、4、5、6、7；id3回REST无STALL记录，最后RESET id8没有保护终态打印。`dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:545`是本地 `BeginLocalProtection(STALL,true)`，`:470–475`同伴通知调用isLocal=false并进入PEER_STOPPING；模式SEQ/INT是主控队列模式，电机MOVEJ检测均在位置控制中。
- **时间特征**：id2 accepted12:36:23.250 →J6触发24.372约1122ms；id4 queued12:37:06.844 →J6触发09.175约2331ms；id5/6/7 queued后分别538/543/554ms出现J6本地STALL，MOVEJ CANCELLED分别约521/510/510ms先返回。若J6仍为默认200ms启动豁免+300ms连续检测，这些后续约0.5秒触发与“每次新运动很快进入持续条件满足”相符，但本轮尚无J6实际参数，不以时间反推替代参数回读或触发快照。
- **实际触发判据**：`stall_protection.cpp:511–529`须J1～J7、检测使能、未STOP锁定、已使能、反馈有效、位置类实际模式、有运动意图并过豁免；`:533–545`滤波abs(focCurrent)>=detectorCurrent、滤波abs(estVelocity)<=detectorSpeed、abs(softPosition-realPosition)>=detectorError，三条件连续成立达到detectorTriggerTicks才以reasonSTALL触发，任一条件失效`:540`清零。CURRENT_CMD_LIMITED、收到MOVEJ、进入SEQ或普通运动完成状态本身不直接触发机械STALL。软件判断成立不等于已经证明外部机械干涉，编码器反馈/跟踪控制及判据适配仍需区分。
- **默认量级与事实边界**：`UserApp/protection_config.cpp:39–46`初始电流命令阈值为当时限流40%、速度.02电机r/s、J6误差.04166667电机圈（减速比30对应关节.5°）、连续300ms、豁免200ms。若这些是J6实际值，速度阈值对应关节.24°/s，明显低于正常运动速度；电流门槛低单独不能触发，仍须低速+大误差连续成立。J7的.2A/.1圈参数不能套用于J6，EEPROM合法设置优先，烧录不强制覆盖；后续增大限流也不会自动按40%提高已经保存的CURRENT_A（`stall_protection.cpp:106–121`）。目前只知道J6有多次本地事件，不能直接断言其电流阈值过低或机械/反馈正常。
- **电流饱和先兆**：id4 12:37:08.011 J6 CURRENT_CMD_LIMITED、08.117 J3 CURRENT_CMD_LIMITED，09.170 MOTOR_DRIVE_BLOCKED、09.172 J6 f=EC，09.175 J6本地STALL打印。`motor.cpp:317–329`前者表示电流输出命令连续顶到额定限流约1秒，不是实测过流，不是机械STALL条件的替代入口；J3未出现本地phase1。f=EC含回包标识80、CURRENT_CMD_LIMITED40、DRIVE_BLOCKED20、ENCODER_READY08、PLANNER_READY04；没有TARGET_READY02、POSITION_READY10或SETTLED01。位置模式切软停、锁定会置DRIVE_BLOCKED并使旧目标确认不适用，实际EVENT/0x23及任务打印顺序有先后，因此这行不是证明主控先制造了堵转；也不能从电流命令饱和推导实测相电流足够或硬件一定无故障。
- **另一个已确认问题：本轮回退被全机停稳门槛阻塞**：id2 STOP wait=010缺J4；id4/5/6/7 wait=012缺J1与J4，rx=0FE、token均匹配、badpos=0，缺的是HOLD确认，不是节点无回复。日志没有J6 phase3/4或STALL locked，J6自己phase2停稳但主控仍PROTECT_STOP等待其他轴。`protection_coordinator.cpp:582–590`不在七轴STOPPED前GO；`motor.cpp:638–659`减速后要求50ms内8内部单位峰峰值范围、平均速度<=.01电机r/s，抖动/实际缓慢运动可能一直清计时，`:634`无保护停稳截止。主控此阶段也无整体截止。这里明确是 **STOP_PROTECTION**，电机`:479–485`cancelInternal=false，同token重试已返回，不得错误归因于前节“普通STOP取消保护时每次重启”的缺陷；新token内部接管可能额外软停一次，但不是每次相同token都重启。尚无J1/J4实际反馈窗口，不认定唯一为抖动。
- **待区分原因**：①J6真实启动/跟踪困难：机械/线缆干涉、限流不足、PID调节或电气驱动问题；②编码器/磁耦合等导致软件低速，与实际轴运动不符；③正常末端纠偏/微动满足当前三条件，检测判据缺少实际运动进展的区分。缺少触发时I_filt/v_filt/err/位置及轨迹阶段，不能选定唯一原因，也不建议通过再降低TRIGGER_MS或盲目加电流来掩盖。
- **已向用户询问安全补充**：J6触发前肉眼是在正常转动、停止、抖动还是啸叫，有无机械/线缆干涉；仅在失能静止时回读 `#STALL_GET 6`、`#GETI 6`、`#GET_PID 6`，无需新运动/碰限位。现有 `CaptureStallDiagnostic:148–170`只在查询时捕获，触发后的`BeginLocalProtection:446`已ResetDetection清滤波，后续DISABLE也改变模式/电流/误差，不能把故障后即时快照当触发瞬间数据。必要的后续方案为触发瞬间冻结RAM快照，包含规划参考/实际反馈与现有判据，再决定阈值或进展判据；未实施。
- **USB尾部异常单独保留**：RESET id8 pending后最后为ClearCommError/设备不识别此命令，没有对应STALL现场或FPS信息。本日志不能证明该USB异常原因，先前ISR任务临界区缺陷已在上一轮修复，不能再次当作未修复根因。

## 最新实测：5mm 回退及 START 成功，继续分析回退入口延迟（2026-10-07，12:30 日志）

- **用户新实测**：12:30:41 将 J7 BACK_MM 从20改为5并保存，其他回读参数仍为 CURRENT_A=.2、SPEED_RPS=.02、ERROR_R=.1、TRIGGER_MS=300、STARTUP_MS=200、滤波10/20ms、BACK_RPS=.5、BACK_ACCEL=10、回退/保持电流1A、BACK_TIMEOUT_MS=5000。12:30:54.977进入RETREATING，57.954进入HOLD_SUCCESS、57.955打印`STALL locked`；12:31:01.389 START pending →02.623 Started ok，未先DISABLE。此次证明5mm的成功流程及随后START有效；不推翻前节已复现的失败后普通STOP重试缺陷。
- **延迟精确拆分（PC收日志时间）**：J7本地堵转54.496 → WAIT_GRANT54.538为42ms；J1～J6最后一个PEER_HOLD在54.603；主控STOP locked54.955；J7 RETREATING54.977。堵转报告后至回退共481ms，J7自身停稳后仍等439ms，其他六轴也报告保持后仍等352ms，STOP locked到开始回退22ms。碰物理限位时刻未记录，不能把481ms当碰限位总响应。触发前连续条件300ms另计，200ms启动豁免不是每次受阻必加的延迟。
- **解释与代码依据**：`dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:533–545` 三条件同时连续成立才触发；`dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:582–610` 等七轴STOP确认后GO；确认不是复用phase7打印便可立即通过，仍有各节点QUERY/token同步/内部STOP/0x0A确认。`dummy_robot.cpp:908–915/1054–1103`同步回包重置发送、首次发送从J1开始、已确认轴仍轮询；`actuators/ctrl_step/ctrl_step.cpp:34–40`不反馈实际发送是否成功。前节真实主控代码离线模型（七轴初始均HOLD、无丢包、模拟邮箱占用一个5ms周期）仍450ms发首个GO，与本轮额外等待量级一致。实机各帧具体CAN排队时间尚未测得，不把整个352ms精确归给某一函数。
- **推荐优先级**：先修主控待确认节点调度、保留公平游标/发送失败重试，避免同步回包又全轴从J1开始，已确认节点退出此次STOP确认轮询；保护内部接管时已实际停稳并固定捕获的节点只绑定新的STOP确认，不再重启软停/稳定窗，未停稳节点仍按自己的加速度停稳。保留七轴联动停止与真实编码器证明，GO/事件/必要续租保证及时发送。再独立调优连续检测时间：建议从300降至50ms做误触发验证，保持条件失效即清零；不自动改运行参数或固件默认，不把GRANT_WAIT500当延时或为加速删除停稳判断。
- **本轮工作**：用户仍要求解释及方案，未执行代码/参数修改、发布编译或commit。仅按AGENTS 5.2更新本状态文档；拟实施文件沿前节列表，具体执行仍待用户授权，不要求再碰限位取证。

## 当前分析：堵转回退启动慢及失败后 START 卡住（2026-10-07，12:11 实机日志）

- **本轮授权与边界**：用户要求分析两项实机问题，未要求执行修改。仅依据 AGENTS 5.2 更新本状态文档；在已忽略的 `firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/` 生成并运行临时离线验证产物。未修改生产代码、配置、运行参数或其他文档，未重新构建发布固件，未 commit。不需要再次碰物理限位。
- **已确认实机触发**：两次 J7 均出现 `phase=1 STALL → phase=2 WAIT_GRANT → phase=3 RETREATING`，用户确认肉眼真实退开。补充的 J7 快照 `peak_ms=300、last_ms=300、clear=TRIGGERED、bad_total=0` 证实连续堵转计时达到 300ms。本日志不再属于上一轮 TARGET_READY_TIMEOUT 抢占回退的情况。其他轴的 phase6/7 表示联动停止/保持，不代表这些轴分别发生机械堵转。
- **实际参数已回读**：J7 总电流上限 1A；检测 CURRENT_A=.2、SPEED_RPS=.02、ERROR_R=.1、TRIGGER_MS=300、STARTUP_MS=200、滤波10/20ms；BACK_MM=20、BACK_RPS=.5、BACK_ACCEL=10、BACK_CURRENT_A=1、HOLD_CURRENT_A=1、BACK_POS_TOL_R=.001、BACK_TIMEOUT_MS=5000、REBLOCK_MS=100、GRANT_WAIT_MS=500；主控绑定位置范围仍为 -50～50 电机圈（±250mm）。这些是本轮用户实际设置，不是新推荐默认值。
- **回退未成功、20mm 超时已确认**：第一次 12:12:47.648 开始回退，52.654 报 RETREAT_TIMEOUT；第二次 12:13:43.385 开始，48.384 报 BAD_TASK，均约5秒结束，没有 phase4/HOLD_SUCCESS 或 `STALL locked`。5mm/电机圈、.5圈/秒仅2.5mm/s，完整20mm仅匀速路程至少8秒，另需加减速/判稳；5秒配置不可能完整完成。建议恢复已决策5mm/.5r/s/5秒，本轮未修改参数。当前参数校验只逐项验证范围，尚无距离/速度/加速度/超时联合可行性检查（`dummy-motor-fw/UserApp/protection_config.cpp:111`）。
- **回退前等待分解**：第一次 J7 phase1 47.163 → phase3 47.648 为485ms，其中自身 WAIT_GRANT 47.209 → GO/RETREATING 为439ms；第二次 phase1 42.712 → phase3 43.385 为673ms，其中 WAIT_GRANT42.790 → RETREATING 为595ms，J1直到43.208/43.270才上报联动停止/保持。以上为PC收到日志的间隔，不是直接测得碰限位时刻。当前检测连续300ms另计，STARTUP200ms只在相应新运动启动豁免期发生，不是每次碰限位必加200ms。GRANT_WAIT_MS=500是未获接管/续租的截止，**不是固定等待500ms再回退**（`Ctrl/Motor/stall_protection.cpp:503`）。
- **主控额外延迟已离线复现**：`Robot/instances/protection_coordinator.cpp:587` 只在七轴 STOPPED 后发 GO；`dummy_robot.cpp:908` 各节点先 QUERY 同步token，`:913`使用token+1、`:915`再次置 stopNeedsSend；`:1059`10ms轮询，`:1077`首次/重置发送重新从J1开始循环所有节点；`actuators/ctrl_step/ctrl_step.cpp:34–40` 丢弃 TryCanSendMessage 的失败返回；`Bsp/communication/interface_can.cpp:278` 使用单发送令牌、非阻塞申请。同周期后续节点可能未发出，新的 QUERY 回包又触发从J1发送，已经确认的节点仍被轮询/重发，放大七轴确认时间。直接执行当前主控源码、模拟邮箱占用一个5ms周期/无丢包/七轴初始均HOLDING：第一次GO仍需450ms，共42个接受的STOP/QUERY帧，其中QUERY7帧。此结果证明调度本身可造成量级相符的额外延迟；不是实测总线时延的精确拆分，也不能据此断言第二次J1晚报的物理原因。
- **START 卡住的确定缺陷：保护状态下普通 STOP 重试不幂等**：回退失败 `protection_coordinator.cpp:337–339` 清提前START并执行普通STOP；电机 `motor.cpp:479–485` 先检查非IDLE保护并 CancelInternal，再因 cancelInternal=true 跳过同token重复请求保护，`:490–492`每次设DECELERATING/重发STOP请求；`stall_protection.cpp:461–468`取消后仍是HOLD_FAILED，而不是IDLE，故每次重试继续满足取消条件。`:605–609`每次重新初始化减速/50ms稳定窗口。CAN `UserApp/protocols/interface_can.cpp:48–58`先处理请求，再立即取phase回包，因此即使两次请求之间已达到HOLDING，下次STOP请求也先将其改回DECELERATING再回执。主控 `dummy_robot.cpp:943–945` 只认可HOLDING/DISABLED，`:855–858` STOPPING时只挂起START，形成持续等待。这不是CAN节点离线：实机 rx=0x0FE、badpos=0、七轴token均与expect一致。
- **该缺陷已用真实电机函数复现**：调用当前 motor.cpp、stall_protection.cpp、真实CAN STOP处理代码，以完全静止、有效的编码器每70ms重复相同token STOP共30次。普通IDLE情形30/30回包HOLDING、0次重启；起始PEER_HOLD或HOLD_FAILED情形，两个场景均0/30 HOLDING、30/30重新DECELERATING；请求间均已真实满足HOLDING。最后只发送QUERY立即得到HOLDING。无需物理受阻即可复现。现有STOP的50ms/8内部单位稳定窗仍很严格，但不能把本次全部等待仅归给实际抖动。
- **DISABLE为何能绕过**：`motor.cpp:567–575/517–536`直接失能并清保护到IDLE，不要求先得到HOLDING；主控随后走全新坐标/限位/使能确认，所以实机DISABLE约94～100ms成功，之后START约1.28秒成功。此绕过符合代码，但保护失败后应允许停稳后的明确START正常保持解锁，不能要求必须先断电失能。
- **BAD_TASK 原因覆盖缺陷已离线复现**：电机 `stall_protection.cpp:360–365` 在RETREATING/HOLD_SUCCESS对同任务GO重试幂等，但HOLD_FAILED收到晚到GO回CONTEX_REPLY/BAD_TASK；`:404–405`实际EVENT仍携带原终态RETREAT_TIMEOUT。主控 `protection_coordinator.cpp:189` 的真实EVENT原因与`:233`的请求回复原因共用 Node.reason，`:508/595–608`可能打印/判断被覆盖的原因。真实电机代码验证晚GO：CONTEXT原因19/BAD_TASK，EVENT及内部原因8/RETREAT_TIMEOUT；真实主控代码先收到超时EVENT再收到BAD_TASK回复，eventPhase仍5但共享reason变19。证明存在该覆盖路径；本日志缺原始CAN帧，第二次BAD_TASK不能据此绝对认定原始原因，但5秒/20mm配置和终态时间高度支持超时后晚GO的解释。
- **与参考版差异**：`firmware/motor_fw_f103_57/Ctrl/Motor/motor.cpp:289` 连续门槛30ms，`:324–334`在电机本地直接切回退，不经过当前七轴停稳确认与主控GO许可，所以更快。推荐保留当前联动停止、真实反馈证明和连续计时，先消除不必要的调度/重启；若需缩短检测反应，之后再把300ms有依据地调低并验证误报，不能把旧版的直接回退或间歇累计逻辑整体复制过来。
- **推荐修复，尚未实施**：①STOP按token与动作种类记录重复请求，普通STOP只取消保护一次，同一请求后续只能回当前状态，不重启；STOP_PROTECTION→用户STOP即使token重合仍须能打断；终态原因不被重复取消覆盖。②主控优先轮询尚未确认节点、保留公平游标、记录实际发送成功，避免每次同步回包又从J1重发；保护内部STOP确认已有固定HOLD状态时不额外重启减速。③分离EVENT终态原因与CONTEXT请求拒绝原因，同任务晚GO在失败终态返回该终态，不再次执行回退。④恢复5mm回退并增加配置可行性提示/校验，保留5秒用户决策，不擅自延长。提前START仍只在全部回退成功后自动解锁，失败后清挂起；用户失败后新发START在真实停止并捕获位置后允许保持解锁，旧目标/队列不能复活。
- **拟涉及文件，待执行授权**：电机 `Ctrl/Motor/motor.cpp/.h`（STOP动作/token幂等、固定保持）、`Ctrl/Motor/stall_protection.cpp`（失败终态与晚GO处理），主控 `Robot/instances/dummy_robot.cpp`（STOP待确认节点/发送调度）、`Robot/actuators/ctrl_step/ctrl_step.cpp/.hpp`（透传STOP发送结果）、`Robot/instances/protection_coordinator.cpp/.h`（分离原因、保护确认调度）；若采用参数联合校验再涉及电机`UserApp/protection_config.cpp`及上位机`串口助手_old.py`。同步既有堵转方案/测试文档；禁止修改`串口助手.py`，不提交commit。最终实现具体范围仍需向用户列清并获得确认。
- **离线证据文件**：忽略目录中的 `analysis_stop_retry_motor_result.txt`（198断言）、`analysis_stop_retry_core_result.txt`（6断言）、`analysis_retreat_wait_result.txt`（335断言）；模拟编码器/传输不能替代后续实机验证。


## 当前交付：目标确认与保护接管修复已编译，待用户烧录（2026-10-07）

- **授权**：用户要求执行修改并给测试步骤；另通过异步问答明确“本轮一起修复，完成保护接管”，解除此前首个堵转事件ISR任务临界区问题的暂缓。修改前已列文件/用途；无commit授权，本轮不提交、不烧录。
- **已确认改动**：电机`Ctrl/Motor/motor.cpp`普通0x23采纳目标确认不再混入overload，新增DRIVE_BLOCKED/CURRENT_CMD_LIMITED分别证明实时阻断/命令饱和；主控`dummy_robot.cpp/.h`已采纳目标不再因运行状态重发旧目标，运行阻断/目标变化明确分类，失败冻结flags、任务自动提示饱和；`protection_coordinator.cpp/.h`CAN ISR只锁定/记录，任务原子接管；内部确认失败STOP_PROTECTION保留真实晚到STALL，用户STOP/DISABLE取消语义不变；共享`motion_completion_protocol.h`flags扩展，无新命令/GUI或回退触发入口。实际到位、连续堵转条件、同步运动、PID/EEPROM及校准动作未改。
- **实现决策/限制**：新增状态最初使35/42应用区超76B，仅GetPositionCompletionStatus函数局部Os并缓存反馈有效性解决，95KiB区/校准表/用户区布局不动，20kHz控制环优化不变。最小应用余量28B。没有真实STALL事件时内部失败只停止锁定，不伪造方向/回退；已验证先事件/后事件与用户取消竞争。未声称1A命令饱和是11:38日志唯一原因；未声称所有USB超时归于ISR。绝对速度微振/稀疏坏反馈/停稳截止/参数联合校验仍未修改，检测敏感性还需实机证据。
- **验证**：五个Ninja Release目标通过；现有真实源码普通完成/STOP/保护/校准及新接管回归通过。主控含ISR禁用API检查158114项、电机45476项、新57保护采纳172项、主控CAN5817项、电机CAN54657项；仅离线规定输入/硬件替身，不是实机验收。git diff --check通过，参考助手SHA保持D07D3E...0D7E4，未修改firmware/esp32/串口助手.py。新增脚本`build/analysis/protection_regression/run_handoff.cmd`和产物清单`build/analysis/motion_protection_handoff_artifacts.json`均在忽略build目录。
- **产物**：主控432184B SHA256 f6e5bacc918d76b5f8a7ac6f6b15bb862c1113bb0203c42bcd003629b99ee27d；35/42 97252B b064e2f68395bdbd03ce5a55e88129465bdd49c294374f059d2d0befdebd3253；57 97244B e78d31c72f9bf9f17127371b6c7f9b73b7b9e44dbf1ef9ce5d4c572ec42fb5f8；gripper97204B 1533024aeecfff60ad54c43a8cdd509732cf5e4f984acf7c9e67076e9c9eab5d。主控RAM56384B、四电机RAM10472B。保留既有RWX/编码器虚函数缺return警告。
- **实机后续**：主控+J1～J7配套烧录，J8本轮可保留/一并更新；不整片擦除。最新详细流程在`docs/堵转误报修复与实机测试流程.md`首节，先正常HOME/RESET/小行程SEQ和STOP/START/DISABLE；再仅J7 1A、档1、RAM CURRENT .2/SPEED .2/ERROR .01/TRIGGER100/STARTUP200、BACK5/.5rps/5acc/1A/5秒，一次行程中部软阻挡，不再撞硬限位，约1秒无事件就结束受阻，不要求故障后强行查询。早START和STOP/DISABLE回退中断测试以首次保护成功为前提。运行KD=300、已存BACK20不会被烧录覆盖，测试控制变量/参数恢复规则已写入文档。

## 最新实机：目标确认超时可复现，本轮OLED与DISABLE正常（2026-10-07 11:38）

- **已确认事实**：11:38:06 J7运行DCE为300/50/30/**300**，加速度30r/s²，总限流1.00A。11:38:21.844地轨230mm/35档MOVEJ获INT accepted，约1.830秒后再次TARGET_READY_TIMEOUT、wait=080 txok=07E ready=07E；11:38:23.893 STOP locked。11:38:31.182 DISABLE→31.187 pending→31.291 Disabled ok；用户明确本轮OLED正常刷新。失能后MOVEJ明确error DISABLED；新START完成后往0的MOVEJ id=2获accepted，日志没有该运动后续实际完成证明。
- **结论更新**：目标确认失败和此前USB无响应必须分别记录，本轮主控没有持续卡死，普通失败→STOP→DISABLE路径仍能运行。已确认ISR临界区违规不因此消失，但不能将本轮故障归给断言死循环；缺少STALL阶段/事件回执，可能未接管事件或普通STOP先取消/抑制，仍不能据无回退唯一认定检测未触发。旧overload→撤销TARGET_READY仍是源码已确认且符合低限流工况的候选，需要自动故障证据而不是再碰限位。
- **基线差异**：当前KD=300不同于前轮消除高速拖尾的已验证300/50/30/100；`dummy-motor-fw/UserApp/main.cpp:62`默认仍100，`:34–35`及`:106–109`合法EEPROM优先，烧录默认不会覆盖已存运行值。实际KD差异会影响速度误差项、电流命令/微动，但本轮不能据此唯一归因或认定烧错固件；后续验收须明确控制变量，不继续未经修复的碰限位PID试验。
- **快照覆盖边界**：本日志STALL_STATUS/STALL_GET查询的是J1，非J7；J1失能、fb=1、bad_total=0、session=0属于该轴本次静态状态，不能排除J7受阻微动或稀疏坏反馈，也不能据J1参数推定J7本轮检测阈值。当前无需用户重复有损测试或故障后查询。
- **后续与本轮修改**：沿前节已确认缺陷修复方向推进讨论，实施仍待列具体文件用途并获确认。仅按AGENTS 5.2同步本文；无固件/运行参数修改、编译或commit。此前“受阻后无法查询”的方法继续停用，不要求ST-Link。

## 当前审计：堵转检测及保护接管的已确认缺陷，停止有损实测（2026-10-07）

- **授权与结论边界**：用户说明BACK_MM=20只为看清回退，要求从固件审计堵转bug，担心57板与电机持续受阻损坏。本轮仅分析，按AGENTS 5.2同步本文，无生产代码/配置/其他文档修改、无发布编译或commit。20mm在5秒内回不完只能解释回退后的超时，不能解释完全未见回退阶段；不再要求碰限位、受阻后手动查询或ST-Link操作。实测说明当前完整保护未达到要求，但无回退不等于三条件必定未触发。
- **P0 事件ISR违规（已确认，非实机唯一归因）**：主控`Bsp/communication/interface_can.cpp:170–183`CAN Rx ISR→OnCanMessage/robot.Receive→`Robot/instances/protection_coordinator.cpp:194–200`在首个LOCAL_STALL调用CancelMotion→`dummy_robot.cpp:1469–1479`任务临界区与CAN任务发送清理。FreeRTOS `port.c:403–415`在ISR、临界嵌套为1时断言，`Core/Inc/FreeRTOSConfig.h:154`屏蔽中断并无限循环，USB/控制任务不可继续。前轮真实源码ISR模型7/7已经验证，当前路径仍在；不能用更低阈值绕过，也不能删断言。先前用户暂缓修复决定仍有效，本轮未改；用户此前某轮FPS持续刷新与该死循环矛盾，不据此宣称所有USB超时原因已找到。
- **P1 旧过载与目标确认混用（已确认）**：`motor.cpp:317–329`命令持续顶到ratedCurrent约1秒置overloadFlag；`:780–784`把该标志混入TARGET_READY，主控`dummy_robot.cpp:674–678`撤销已确认ready/txok，`:1434–1464`即使在运动中也报TARGET_READY_TIMEOUT并走普通STOP。当前日志符合该候选路径，但未取到overload现场值。目标接收/采用应独立于实时故障和完成判稳；原过载须明确报告与统一处理，不能简单删掉或把所有过载无条件当机械堵转。
- **P1 普通失败STOP取消堵转/抑制晚到事件（已确认竞争路径）**：`dummy_robot.cpp:819–842`普通RequestSoftStop先`protection->Cancel(false)`；`protection_coordinator.cpp:294–310`置suppressEpisodes=true并清trigger/job；`:194`之后不接管晚到LOCAL_STALL。电机`motor.cpp:479–481`收到普通STOP会CancelInternal，`stall_protection.cpp:461–468`把已有保护置HOLD_FAILED/CANCELLED_STOP。因此本地检测即使已触发、事件尚在路上，也可能被普通失败STOP打断且不授权回退。用户主动STOP/DISABLE仍必须打断保护；内部投递失败与用户取消应分原因处理，已证实STALL应统一交接，未证实堵转不能因通信失败猜方向回退。
- **P1 稀疏坏反馈的保护空档（逻辑已确认，实机未证实发生）**：`motor.cpp:83–99`每个有效采样立即清invalidFeedbackTicks；`stall_protection.cpp:515–516`一次坏样本即清堵转计时，`:488`编码器故障要求连续100周期（5ms）无效。每隔20ms出现一个坏周期可使连续50/300ms堵转永远不成立，也永远不到连续5ms反馈故障，仍输出纠偏命令。前轮真实源码规定输入对照已重现最长19.95ms、不触发。保持“无效反馈不能当有效堵转证据”的原则，但须增加有界反馈质量故障兜底，不能恢复把坏读数当有效的旧实现或跨间歇累计堵转时间。
- **P1 检测速度与有效运动进展不等价（设计缺口，实机幅度未知）**：`motor.cpp:121–127`已平滑有符号估计速度；`stall_protection.cpp:533–540`再滤波绝对速度，往返微振不会相互抵消，可以形成非零速度底，使无净进展的受阻始终不满足低速条件。前轮真实闭环规定±12内部单位/100Hz摆动得到.065～.069r/s，阻止.02阈值；±40模型得到.21～.234r/s，超过允许配置最大.2。这些不是硬件实测。建议按指令方向的编码器位移窗口/实际进展配合误差判断，并以正常低速、反向、负载震动验证误报；不能盲目提高速度上限或删除条件。
- **P1 本地软停/等待全机停稳缺少保护阶段截止（已确认）**：`stall_protection.cpp:497–501`LOCAL_STOPPING/PEER_STOPPING只等HOLDING/DISABLED或FAULT，无phaseTicks截止；`:503–505`GRANT_WAIT仅进入WAIT_GRANT后才生效；`:666–676`BACK_TIMEOUT仅MovingPhase阶段。主控`protection_coordinator.cpp:572–582`PROTECT_STOP无总截止，只等robot STOPPED/已有节点HOLD_FAILED，且此作业不跑普通Health。`motor.cpp:634–650`STOP稳定窗口抖动可不断复位，通信丢失也可让七轴确认一直缺少，从而永久等待回退入口而不报终态。需给减速/稳定证明和全机停稳设有界失败策略，仍不得未经实际停稳就开始回退或默认断电承重轴。
- **非根因与配置校验缺口**：20mm/.5电机r/s（2.5mm/s）完整回退至少8秒，5秒超时不相容；`UserApp/protection_config.cpp:111–119`只校验单参数范围与位置上下限，没有距离/速度/加速度/超时联合可行性。应拒绝或明确警告不可能完成的组合，但此缺口不阻止进入RETREATING。本轮也确认检测实际在20kHz中执行（motor.cpp:136）、SET提交发布整数运行值；检测不是未接入。`PollProtectionMain:807–812`事件20ms重发/250ms周期、EVENT_ACK比对事件与阶段；`Core/Src/can.c:264–289/323–355`有保护队列、TXOK/重试/超时，不宣称事件只发送一次或完全无确认。CAN拥塞条件仍待专门验证。
- **取证缺口**：当前快照只在DIAG_QUERY时捕获（`:148–173`），没有冻结overload/目标就绪失效或STALL触发的完整现场；Stop/参数/下条运动可改变即时字段。这是诊断设计缺口，非直接触发原因。建议在关键节点自动冻结RAM、主控任务打印简短故障记录，正常查询时可读历史，避免故障后再发查询；新增数据不写带载EEPROM。
- **建议修复顺序和拟影响模块（未授权实施）**：先首事件ISR/任务边界与普通失败/堵转接管竞争，分离目标接收确认、实时故障、实际完成；再反馈质量兜底及指令方向进展窗口；补保护停稳阶段截止和自动证据；最后联合校验参数。候选文件为电机`Ctrl/Motor/motor.cpp/.h`、`stall_protection.cpp`、`UserApp/protection_config.cpp`，主控`Robot/instances/protection_coordinator.cpp/.h`、`dummy_robot.cpp/.h`，共享`firmware_new/protection_protocol.h`；若扩诊断协议/命令同步对应主控can/ascii与目标`串口助手_old.py`，不改参考助手/firmware/esp32。具体修改前仍逐项列文件用途待用户确认；57应用区仅余20B，必须核查空间且未经授权不变更Flash/EEPROM布局。验证先真实源码无动力故障输入/ISR模型，覆盖饱和但正常运动、固定编码器、往返微振、稀疏坏读、迟到STALL与内部失败/用户STOP竞争、停稳超时和旧目标不复活，不把离线模型冒充机械实测。

## 当前分析：J7碰限位后TARGET_READY_TIMEOUT，仅普通STOP、未确认堵转回退（2026-10-07 10:57）

- **实机事实与取证限制**：用户发送J7限流1A保存请求；STALL_GET为CURRENT=.4、SPEED=.02、ERROR=.1、TRIGGER=300、STARTUP=200、滤波10/20ms、BACK_MM=20、BACK_RPS=.5、BACK_ACCEL=10、BACK/HOLD_CURRENT=1、超时5000ms；随后CURRENT=.2并获SET/SAVE成功。START完成后MOVEJ地轨230mm/35档，获INT accepted；约1.85秒后MOTION fail reason=TARGET_READY_TIMEOUT、wait=080 txok=07E ready=07E、J7 target not confirmed，随后STOP locked。用户明确本次J7有实际移动且触碰物理限位，受阻后无法查询，DISABLE也不行。撤回再次要求故障后STALL_STATUS/MOTION_STATUS查询，不把独立回读或ST-Link当用户本轮必须操作项；不再要求反复碰限位。
- **日志可确认的链路**：主控`dummy_robot.cpp:1434–1438`检测当前目标确认缺失→`:1454–1466`FailMotionDispatch调用RequestSoftStop→`:819–842`取消保护协调、普通STOP锁定七轴并清旧运动，不发RETREAT_GO。080为J7位、07E为J1～J6；txok是当前CAN发送状态，不是实际到位或当前电流。TARGET_READY是目标匹配及运行条件通过，不等同已到达目标。该日志证明主控因普通目标确认失败发STOP，不能单独证明或否定电机曾本地触发STALL；无STALL事件/阶段回执支持回退已启动。当前目标确认会在运行中撤销，不能把J7 actual motion与TARGET_READY_TIMEOUT视为矛盾。
- **已确认软件竞争路径、实机原因仍待证实**：电机`motor.cpp:317–329`在非电流模式且abs(focCurrent)==ratedCurrent连续约1秒时置overloadFlag，只有命令脱离饱和才清。它不是实测过流，也没有自动产生新的STALL/授权回退事件。`:780–784`在overloadFlag为真时不返回TARGET_READY；主控`:674–678`收到不READY回包清该节点ready和txok并安排重发；`:1434`使用从本次运动开始计的300ms门槛，运行中失去READY也会立即进入目标确认超时；随后普通STOP会取消内部保护动作（电机`:479–481`）。因此1A限流下持续饱和/受阻可以先走旧过载→确认失败→STOP路径，绕过新的正常堵转回退。此路径已由源码确认，是兼容性问题；本轮没有现场overloadFlag/计时快照，不能认定唯一根因，也不能把USB无响应直接归给该标志。
- **参数差异**：本次仅降低电流门槛至.2A，其余仍为SPEED=.02r/s=地轨.1mm/s、ERROR=.1r=.5mm、连续300ms、启动200ms、滤波10/20ms，并不是上一轮建议的.2r/s、.0002r、20ms、0、1/1ms。电流易通过不代表速度/误差/反馈和连续计时也通过；尚无受阻快照定位具体阻塞条件。
- **另一个已确认配置冲突**：20mm回退在.5电机r/s、5mm/r下最高2.5mm/s，完整回退仅匀速段最低需8秒，必然超过5秒超时（加减速/判稳还需时间）。即使正常启动回退，完整20mm也无法在当前超时内成功；受机械限位裁剪后的短距离情况须另看。此前用户确认5mm、.5r/s、5秒的组合应恢复用于验证，不为本轮擅自延长已确认超时。
- **建议下一步、未实施**：将运动目标接收/采用确认与实时故障状态、实际到位判稳分开；目标曾确认后发生过载/反馈异常应明确报告故障与统一保护策略，不能仅改为忽略故障或无条件自动回退，也不能因为普通确认失效取消正在接管的堵转保护。保留完整触发/确认失败前RAM证据（实际模式、目标匹配、overload/饱和时长、反馈、候选及事件阶段），采用事前/自动采集，不依赖受阻后手动查询；USB失效仍独立定位，已知ISR临界区缺陷仍按用户先前决定暂不修。生产改动前列文件和用途待用户确认。本轮仅分析并按AGENTS 5.2同步本文，无固件/参数修改、编译或commit。

## 当前测试建议：现有固件J7低电流、极敏感检测参数（2026-10-07）

- **用户选择**：不使用ST-Link查询电机，先实测当前固件能否触发；J7总限流设1A，检测电流命令门槛指定40%=0.4A。本轮核对参数/动作，只提供命令建议，未连接设备、实际修改运行参数、改固件、编译或commit。按AGENTS 5.2更新本文。
- **拟用临时参数**：仅J7，ENABLE=1、CURRENT_A=.4、SPEED_RPS=.2（合法最大、地轨1mm/s）、ERROR_R=.0002（合法最小、约.001mm）、TRIGGER_MS=20（合法最小）、STARTUP_MS=0、CURRENT_FILTER_MS=1、SPEED_FILTER_MS=1（两滤波合法最小）。这些是仅验证触发的极敏感值，不是正常使用推荐；可能启动/低速误报。实际有效反馈、位置模式和运动意图门控仍生效，不能保证20ms内必触发。
- **回退保持设置**：保留既定BACK_MM=5、BACK_RPS=.5、BACK_ACCEL=5、BACK_POS_TOL_R=.001、BACK_TIMEOUT_MS=5000、REBLOCK_MS=100、GRANT_WAIT_MS=500；BACK_CURRENT_A=HOLD_CURRENT_A=1以配合J7总限流。检测的电流/速度/误差/滤波也用于回退再次受阻，极敏感值可能引起REBLOCKED，不能将回退失败等同没有检测触发；1A也可能不足以克服实际负载。
- **设置顺序**：先DISABLE完成，临时`#I_LIMIT_J 7 1.0`不带&，GETI确认，再逐条STALL_SET并等每条最终回执，最后STALL_GET回读。不要STALL_SAVE，不把试验门槛写EEPROM；限流降低不会自动按40%改既有阈值（`stall_protection.cpp:106–121`），故CURRENT_A必须显式设.4。先记录原GETI/STALL_GET，测试后失能恢复。仅J7限流降低，其他承重关节参数不改。
- **应观察行为与边界**：正常链路为J7本地保护软停/事件→主控锁定清旧目标并J1～J7联动软停→J7获授权、按捕获方向反向退离最多5mm→实际编码器到位判稳固定位置保持→STALL locked；J8排除。无方向/空间/反馈、再次受阻、超时或授权通信失败会锁定报原因，不能承诺必回退。当前USB失效未定位且已知ISR缺陷仍未修；不能把DISABLE作本轮唯一停止手段。先行程内低速短移动观察，避免为取得触发而持续顶住硬限位；若受阻啸叫无退离或USB无响应即用独立驱动断电结束，不持续受力。暂不发提前START，保持锁定便于观察。参考源码检测回放与ST-Link方案仍未实施。

## 当前讨论：堵转保护的用途、参考检测基线与ST-Link取证（2026-10-07）

- **本轮范围**：用户讨论没有软件电流环时是否需要堵转保护，以及为何不先复现参考检测。本轮只分析；按AGENTS 5.2记录本文，不改生产代码、参数、配置或其他文档，不编译、不烧录、不commit。以下为建议，尚未获实现授权。
- **电流含义已核查**：新版`dummy-motor-fw/Ctrl/Motor/motor.cpp:373–386`和参考57`firmware/motor_fw_f103_57/Ctrl/Motor/motor.cpp:438–448`均把控制器输出赋给focCurrent再下发驱动，它是电流命令，均不是MCU实测相电流。新版`Ctrl/Driver/tb67h450_base.cpp:44–52`通过Vref设置两相电流；Toshiba官方TB67H450FNG数据手册第10页说明芯片支持PWM恒流控制，不能把“MCU没有采样电流环”扩大成“驱动芯片没有恒流能力”。芯片能力与本机实际相电流跟踪效果仍须区分。官方来源：https://toshiba.semicon-storage.com/info/TB67H450FNG_datasheet_en_20201126.pdf?did=65346&prodName=TB67H450FNG 。
- **用途建议**：保留以运动需求、编码器实际进展和跟随误差为依据的机械受阻保护；电流命令仅是控制努力的辅助信号，不能当实测转矩或碰撞力。没有软件相电流闭环并不使受阻检测失去用途，但当前受阻后USB通路失效尚未定位，不能称保护已可靠交付；也不能把电流环增加等同于解决所有受阻问题。
- **参考基线与推进纠正**：原需求明确连续判定、固定捕获保持、实际反馈完成回退、机械边界约束，因此不整体复制旧保护是有依据的；但首次实现同时改变阈值、附加滤波、误差基准和门控，未先建立实机等价检测对照，造成灵敏度差异难以定位。建议在现有STOP/回退/确认框架内独立对照检测层，先用失能无动力信号回放复现旧检测原貌，再每次改一项。已有真实源码离线对照不是实机灵敏度验证。旧版低速条件失效时保留计时只能作为对照基线，正式逻辑仍推荐连续判定；若用户要求生产原样恢复，须明确这与此前连续堵转要求冲突。旧版LOCKED动态追随estPosition、规划完成即回退完成等缺陷不恢复。
- **阈值事实**：参考57`motor.cpp:286–298`为初始化限流40%、速度50内部单位/s=.0009765625r/s、误差15内部单位=.00029296875r、30ms；J7每圈5mm，误差约.00146484mm。最新用户J7为1A、.2r/s、.01r=.05mm、50ms、滤波10/10ms。旧误差门槛小约34倍，但旧速度门槛反而更严，不能笼统声称旧版全部门槛宽松。`:364–370`只在估计速度>150内部单位/s时清计时，新版`stall_protection.cpp:533–545`附加滤波绝对电流/速度，任一条件失效立即清计时。尚无本次受阻快照证实具体卡在哪条条件。
- **工具已确认与下一步建议**：用户有ST-Link、没有USB-CAN；现有主控USB→CAN可以用于正常查询，但USB失效时不是独立取证路径。优先利用ST-Link读取主控/电机PC、调用栈及RAM，配合全部驱动失能且动力隔离的受控软件回放检查检测到事件/锁定链路，不再撞物理限位。带载时暂停电机MCU可能停位置环，故只建议在机械固定且无动力条件下做断点/暂停。已确认ISR任务临界区缺陷仍按用户先前决定暂不修，不将它认定为实机USB故障根因。

## 当前分析：受阻后快照无pending，停止撞物理限位，改用无机械受力诊断（2026-10-07）

- **新实机证据**：10:25:30 J1、10:25:39 J7静止快照完整，gate=NO_MOTION、intent=0、fb=1、enabled=1、detect=1，符合未运动状态。10:25:54向J7=90mm/35档发送MOVEJ，10:25:55快照完整：mode=1、phase=0、gate=ELIGIBLE、intent/fb/enabled/detect均1；I_cmd=-.959A、I_filt=.897A、I_thr=1A；v_raw=-10.251r/s、v_filt=10.2236r/s、v_thr=.2r/s；err=-.0120508r、阈值.01r；unmet=3、连续/峰值0、无坏反馈。此时是在正常运动，地轨速度约51mm/s；电流和速度未通过、误差已通过，不能拿这份数据代替后来的受阻状态。
- **故障边界**：10:26:24向250mm/35档发送MOVEJ并获accepted；10:26:27 STALL_STATUS无pending/结果；10:26:32 DISABLE无回执，随后Write timeout/ClearCommError。用户观察地轨受阻啸叫、OLED J7为“-”，本次未注意FPS。该字符不代表屏幕还在刷新或主控未卡住。既不能确认受阻时未触发检测，也不能确认保护事件已到主控；无pending属于指令/回复通路失效，不是诊断CAN缺字段超时的已确认表现。
- **测试决策**：用户明确丝杆损伤风险，不再请求撞硬限位、手挡地轨或持续啸叫测试。上一轮“受阻后再发查询”的方法在此USB失效工况无法取证，停止使用；已失效串口不能作为停止保障。当前仅分析，不改固件、参数或新增commit；按AGENTS 5.2记录本文。
- **推荐替代**：先在全部驱动失能且动力隔离、机械不会动作的条件下，用独立CAN注入一个保护事件或软件受控回放，验证主控首次事件ISR/任务边界及USB是否继续工作，不依赖真实机械堵转；现有真实源码ISR模型已确认7/7任务临界区违规，但仍不把它认定为本次硬件故障根因。可用SWD获取PC/调用栈，或用独立CAN绕过主控读电机诊断；不要求复现机械受阻。后续若授权代码修改，考虑普通检测非零候选清零/触发瞬间冻结RAM证据、独立读取、测试入口仅在动力隔离条件下可用，并继续严格连续判定。注入只验证软件链路，不能代替物理检测灵敏度与实际回退验收；RAM断电丢失，不在带载运动时写EEPROM。
- **待确认**：已异步询问可用ST-Link/USB-CAN设备，以选择具体无机械受力方法。USB无pending仍需独立定位；本次没有新受阻快照支持放宽速度判据，保持当前检测逻辑。

## 前序交付：只读堵转检测运行快照已编译（2026-10-07）

- **授权与边界**：用户先做运行快照，暂不修主控保护事件ISR调用任务临界区的已确认缺陷；FPS持续刷新时不能认定该缺陷是此次USB超时根因。修改前已列文件用途，按此次授权执行。无commit授权，未提交；此前40%默认阈值五文件修改继续保留，不混称本轮新增阈值修改。
- **修改文件**：`firmware_new/protection_protocol.h`增加只读0x37/0x38及字段/原因；电机`dummy-motor-fw/Ctrl/Motor/motor.h`、`stall_protection.cpp`记录门控/计时/反馈、缓存一致快照并分帧；主控`dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp/.h`独立查询/重试/超时/打印，`UserApp/protocols/can_protocol.cpp`路由、`Bsp/communication/interface_can.cpp`发送白名单、`UserApp/protocols/ascii_protocol.cpp`只读锁定例外；`串口助手_old.py`参数窗口按钮及发送入口/线程严格只读例外；测试流程与本文。参考串口助手.py、firmware、esp32、Flash布局和EEPROM格式未改。
- **诊断行为**：`#STALL_STATUS 1..7`单节点，无ALL/J8；不贡献保护Locked/Busy、不改运动/队列/参数/计时/EEPROM，不充当通信看门狗/保护租约。电机新请求号捕获19字段，同号重试保持同一时刻；主循环每毫秒最多一个普通队列回复，保护优先。主控100ms重试，1500ms缺字段只报诊断超时，不锁机；CAN ISR只复制数据，打印任务每轮最多4行，完整16行到end。用空闲取消时隙，原STOP/租约/取消/健康先处理，一帧预算保持。
- **字段与历史**：实际/请求电机模式、保护阶段/门控、运动意图、完整反馈有效性、软件使能/检测/STOP锁、未满足位图、原始有符号及滤波abs FOC命令/速度、softPosition-realPosition、阈值、当前/最长连续计时、最近非零清零原因/时长、无效反馈周期/连续坏采样峰值、启动豁免剩余时间。focCurrent仍为输出命令，不是实测电流。原三条件、门控、滤波及严格连续判据保持。原ResetDetection(true)开始运动会话时重置历史；STOP/DISABLE/查询保留，INT中途覆盖且motionIntent=true不重开会话，内部回退/校准计时不混入普通峰值。当前值可被原失能复位，须受阻期间查询。
- **体积决策**：初次57构建超95KiB应用区1220B，已说明后只对ProcessProtectionCan及新增捕获/发送函数使用GCC局部Os；控制/检测20kHz编译选项保持。四目标在原分区内，35/42余12B、57余20B、夹爪余60B。后续电机功能需重新评估空间，不借用校准表/EEPROM。
- **验证**：五目标Ninja成功。真实源码快照/保护基线电机7015/主控11174断言，普通完成扩展38995/45476，主控CAN5817、电机CAN54657，配置/EEPROM132，真实隐藏Tk及串口替身65项全部通过，含一致缓存、分帧/分片、错号/节点/版本、缺字段重试/超时不锁机、诊断与START/DISABLE并行、ASCII及UI锁定只读例外、清零/峰值/会话和原回退/校准/完成流程。原EVENT分支和CancelMotion待修缺陷保留。未连接硬件或实际串口，未烧录。
- **产物**：新主控BIN430920B、SHA=`DB922D44224DA05459F47F63082239DF85A3342B160D21E2B0E037FAB5A39C24`；57 BIN97260B、SHA=`1D27085F99DB0CEAD4E1722EB66D0848C3CF3F7CAEAC2182E0ED28C0CBAC27E3`。35/42为97268B，gripper为97220B，其余SHA及全部路径见测试流程第10.4节；同名hex均生成。主控RAM56368B/CCM64KiB，电机RAM10472B。原MT6816缺return及RWX告警不扩修，参考串口助手.py SHA保持。
- **待用户实测**：先更新主控和J7（57），维持保存参数/PID，START静止/正常运动/受阻分别读；受阻时约200～300ms间隔手动取3～5份完整`#STALL_STATUS 7`，等end再下一次，保留START→MOVEJ→快照→DISABLE日志及实际受阻时间/OLED FPS。DISABLE后可读历史，先不要新运动。查询J1～J6须对应节点新35/42，J8不必为快照更新。根据gate/unmet/clear/detail/peak/滤波速度再决定速度判据，保留连续堵转要求；USB无pending仍独立定位，不宣称已解决。详细字段与流程见`docs/堵转误报修复与实机测试流程.md`第10节。

## 前序分析：新版撞地轨限位无堵转回执、USB失能无回复；与参考版触发逻辑对照（2026-10-07）

- **本轮授权与边界**：用户要求分析当前堵转是否落地及对比`firmware`实测更灵敏的触发逻辑。本轮未修改生产固件、配置、助手或方案参数、未重编发布固件、未烧录、未commit。按AGENTS 5.2同步本文；模拟及源码抽取文件仅位于已忽略的`firmware_new/dummy-ref-core-fw/build/analysis/protection_regression/`。工作区原有五文件修改属于上一轮40%默认门槛，不能混称本轮新增改动。
- **实机证据**：用户确认发送MOVEJ让J7撞物理限位，受阻时OLED J7坐标不变，总限流3A，排除仅静态保持推轨或TRQ测试。09:34快照为CURRENT=1.5A、SPEED=.05r/s、ERROR=.05r、TRIGGER=200ms、STARTUP=100ms。09:39最终设置为CURRENT=1A、SPEED=.2r/s、ERROR=.01r、TRIGGER=50ms、STARTUP=10ms、滤波10/10ms、BACK/HOLD电流2A；MOVEJ目标200mm/35档已accepted，后两次DISABLE无pending，随后USB Write timeout。用户明确补充当时**OLED FPS持续刷新**；不得把坐标不变当屏幕停住。未获得电机运行时FOC命令、滤波速度、误差、motionIntent、有效样本及候选计时。
- **落地调用链已确认**：电机`UserApp/main.cpp:198`TIM4 20kHz→`Ctrl/Motor/motor.cpp:7`更新编码器及闭环→`:136`TickProtection；参数COMMIT会更新运行整数快照并同步开关，GET非仅GUI缓存。`stall_protection.cpp:440`要求J1～J7、适用位置模式、已使能、反馈有效、未锁定且motionIntent；`:455`滤波abs(focCurrent)、滤波abs(estVelocity)、未限幅softPosition-realPosition同时满足，任一失效立即清候选。触发`:366`先本地保护软停，`:721`主循环事件重发。回退方向不确定仅影响随后回退，不会阻止最初STALL触发。focCurrent始终是输出电流命令，不能用电源端整机电流代替。
- **参考版实际常量**：35/42/57三份`Ctrl/Motor/motor.cpp`同类触发结构，57`:273`仅位置模式且STALL_IDLE，无motionIntent门控；`:278`进入位置模式100ms豁免；`:286`CURRENT为ratedCurrent×40%，速度50内部单位/s，误差15内部单位，累计30000us=30ms。每电机圈51200内部单位、地轨5mm/r，所以50/s=.00097656r/s=.0048828mm/s，15=.000292969r=.00146484mm；这是内部细分单位，不是15个整步。新版J7原默认ERROR=.1r=.5mm是参考门槛341.3倍；最新ERROR=.01r=.05mm仍34.1倍，但最新速度.2r/s=1mm/s已比参考允许速度大204.8倍，CURRENT1A低于假定3A参考上限时的1.2A，不能笼统称所有新版门槛更严格。参考源码“200ms/400ms/75%”注释与实际30ms/60ms/40%常量矛盾，以代码为准。
- **计时语义差异**：参考57`:364`条件失效时只有abs(estVelocity)>150内部单位/s才清上升沿及累计时间；低速时电流或误差条件间歇满足会累计，不是连续30ms。`:317`二次验证处于三条件已成立的分支，高电流>原门槛必然>60%门槛，误差>15必然>12；二次验证必定通过，后面的速度放宽及双倍时间兜底不可达，不应继承。新版严格连续语义符合此前用户决策；不能为了重现灵敏度复制间歇累计bug。
- **其他检测差异与待证实风险**：参考只直接检查abs(estVelocity)、abs(focCurrent)、estError；新版对绝对速度再一阶滤波，可使来回微动产生持续非零速度底，即便OLED坐标不变。参考estError=soft-est含速度前馈，新版误差soft-real；真静止时前馈趋零，不能独立解释数秒不触发。新版`IsFeedbackValid`/`:440`一次无效采样即清计时，`motor.cpp:83`及`Ctrl/Sensor/Encoder/mt6816_base.cpp:22`检查SPI/奇偶/磁铁/突跳；连续100周期无效才NO_ENCODER，稀疏无效可能长期清计时而无故障打印。参考编码器坏读后保留旧角度且检测不校验当次样本；不能照搬旧读数当有效反馈。以上速度微动/稀疏坏采样均未被实测证实。
- **真实源码对照验证**：`generate_stall_compare_probe.py`抽取参考57原检测块（仅回退执行边界stub），与当前真实TickProtection接受相同规定输入。持续高电流/固定位置：参考30ms、新版最新阈值50ms；高5ms低5ms交替：参考55ms触发，新版1秒不触发（候选最长28.9ms）；两版数值门槛对齐仍参考55ms、新版不触发；固定误差20内部单位：参考30ms、新版最新误差门槛不触发；每20ms有一次无效反馈：参考30ms、新版不触发（候选最长19.95ms）。5项比较断言通过，结果`stall_compare_probe_result.txt`。完整新闭环/规划器固定编码器模型（300/50/30/100、3A）按第一组用户门槛307.15ms触发，最新门槛82.95ms触发并发出STALL事件；100Hz±12内部单位摆动可使过滤速度.0651～.06859r/s阻止第一组触发，±40可使.21053～.23379r/s阻止最新.2门槛触发。8项断言通过，结果`stall_trigger_probe_result.txt`。这些是规定物理模型及信号，绝不是本次硬件电流、噪声或实际耗时测量。
- **主控已确认独立缺陷及推断纠正**：CAN RX中断`Bsp/communication/interface_can.cpp:180`→`UserApp/protocols/can_protocol.cpp:29`→`dummy_robot.cpp:764`→`protection_coordinator.cpp:119`在首次LOCAL_STALL调用CancelMotion；`dummy_robot.cpp:1471`及其PollCanMotionTransmit使用任务临界区。FreeRTOS `port.c:403–415`禁止ISR调用此函数，`Core/Inc/FreeRTOSConfig.h:154`断言后BASEPRI屏蔽并死循环。ISR模型7节点均重现，结果`stall_isr_probe_result.txt`；已发布主控ELF反汇编确认该断言及死循环实际存在（vPortEnterCritical=0800e2b4）。但是用户FPS持续刷新与全主控卡死矛盾，**撤回将此缺陷直接认定为09:39 USB超时实机原因的推断**。现证据不能断言该轮收到过STALL事件；此缺陷仍必须修，且也影响其他带LOCAL_STALL的异常事件。USB `Bsp/communication/interface_usb.cpp:170`在ASCII处理返回后才重新开放RX，USB任务局部堵塞/端点或PC驱动异常仍可伴随OLED刷新；DISABLE连pending都没有与等待电机ACK不同，原因待诊断。
- **推荐下一步/未执行**：先修首次保护事件的ISR/任务边界：ISR只保存事件、立即锁定与撤销普通发送权限，队列清理、CancelMotion/CAN邮箱处理在5ms控制任务执行；保持已有STOP/START/DISABLE/回退设计，不删除FreeRTOS断言。增加受限频率查询的检测快照：实际模式/门控原因、motionIntent、当前及滤波FOC命令、当前及滤波速度、跟随误差、连续时间峰值/清零原因、无效采样计数，并为USB健康/命令处理记录证据；目标UI仅old助手。取得实测后才决定速度按编码器时间窗口位移/振幅评估及短暂坏采样质量故障策略，保留有效阻塞条件连续成立与承重保持，不盲目降阈值或复制旧计时。生产修改前仍须列文件用途并由用户确认；目前仅分析，无新修复授权。因已出现DISABLE无响应，建议暂停重复撞硬限位试验，恢复停止通路后再验收。

## 已完成改动：按用户要求将普通堵转初始化电流门槛改为40%（2026-10-07）

- **授权与范围**：用户观察J7总限流3A时受阻电源端整机电流约1.6A，明确要求默认CURRENT_A从初始化上限90%改为40%。已在修改前说明将改电机端`firmware_new/dummy-motor-fw/UserApp/protection_config.cpp`、`串口助手_old.py`及堵转方案/测试流程/本文；本轮未获新commit授权，不提交。电源端整机电流与FOC电流命令无法直接换算，将40%作为用户指定试验门槛，效果待实机验证。
- **实现与常规边界决策**：默认CURRENT_A按40%初始化；为兼容既有0.1A合法下限，额定电流低于0.25A时取0.1A，不改变合法范围，已向用户说明。2.5A→1.0A、3A→1.2A、2A→0.8A、1.5A→0.6A。助手建议初值从2.25改1.0A（典型2.5A），仍应以实际节点GET回包为准；J8普通检测继续排除。
- **保持与影响**：三条件连续判定、误差/速度/滤波/300ms/200ms参数、回退/保持电流和距离/速度/5秒超时、已验证J7 PID、CAL接触参数及STOP融合不改。较低CURRENT_A同时参与普通回退再受阻的min(CURRENT_A, 0.8×BACK_CURRENT_A)门槛；不声称仅影响第一次触发。既有降低普通限流使阈值超限时的80%修正规则不改，为本轮边界，GET后可显式SET新门槛。
- **EEPROM与操作**：合法已存配置优先，烧录新默认不能自动覆盖旧值。J7实际3A时，DISABLE完成→STALL_SET 7 CURRENT_A 1.2最终确认→STALL_GET 7核对→START后低速实测；确认合适后DISABLE及STALL_SAVE 7保存。其他节点按各自回读上限计算40%，不盲目ALL设置同一A值。现有固件已有SET支持，也可直接按此参数测试。
- **验证与交付**：四电机Release/Ninja构建49/49通过。现有真实配置/EEPROM回归按新默认更新期望后重新从当前生产源码生成并编译，132项断言通过；助手Python源码编译和git diff --check通过。测试生成/结果仍位于忽略的build目录，未增加生产测试文件。35/42 BIN 97124B，SHA256=`FECBE1CD01752B13FC83B14604EDC8C79A8E697A6142C21EC54273D249E5D63E`；57 BIN 97116B，SHA256=`7090CA5FCCB21B8D4A04D67584E7420F2F657BA35FAD06930A02F11E3F285FB1`；gripper BIN 97076B，SHA256=`06C01F6F4A6BD16F779178D433CF610F737E55BC54B9FF5AF15113FACA6D7D7D`，电机RAM均10368B。95KiB应用区最小剩余156B，全部链接仍在界内，既有MT6816缺return及RWX告警不扩修。产物位于`firmware_new/dummy-motor-fw/build/motor-{35,42,57,gripper}.{bin,hex}`。主控BIN SHA仍`19FF2DEE49DF98D9D65107C82B7CE0BAC1FF888CD18428A613AFD810C87BB695`、参考串口助手.py SHA仍`D07D3E90968F42706C107EC038AFD5109E6A67FC0A12F3C2BC3D98D13FF0D7E4`。未连接设备、未烧录、未commit；本次无协议变更，J8继续排除普通保护，无需为本次普通阈值试验重刷J8。

## 前序讨论：普通堵转参数解释与实机调参（2026-10-07）

- **本轮范围**：用户要求解释新增堵转参数及调整方法，由用户实测。只读核查当前代码，按AGENTS 5.2更新本文；未修改固件、实际节点参数、配置或助手，未编译、烧录或commit。上一轮默认PID及相关改动已经以仓库本地身份提交`5148016`，本轮开始时工作区干净。
- **已确认触发机制**：`firmware_new/dummy-motor-fw/Ctrl/Motor/stall_protection.cpp:440`普通检测只适用于J1～J7、已使能且编码器有效、未STOP/内部保护锁定、支持的位置模式及motionIntent成立；TRQ电流模式不走这组三条件。`:455`使用滤波后的abs(focCurrent)、滤波后的abs(estVelocity)、abs(softPosition-realPosition)，分别满足CURRENT_A以上、SPEED_RPS以下、ERROR_R以上并连续TRIGGER_MS才触发STALL，任一条件失败清候选计时，参考方向改变也清计时。focCurrent是控制器输出命令，不是实测绕组电流；softPosition是当前规划参考，不是最终goalPosition。没有新增独立迟滞参数。
- **时间与启动细节**：滤波为`:29`一阶固定点滤波，CURRENT_FILTER_MS/SPEED_FILTER_MS是响应时间尺度，非固定额外等待。`motor.cpp:703`仅在没有既有运动意图时启用新的STARTUP_MS豁免，连续新目标不保证每条重置豁免；`:463`只有softVelocity为0且实际位置达到内部positionTolerance才清motionIntent，不能把肉眼静止等同于检测关闭。模式变化、STOP/START/DISABLE等通过ResetDetection清候选/滤波/方向状态；参数生效UpdateProtectionLimits清候选与滤波。
- **默认、单位与运行值**：`UserApp/protection_config.cpp:36`默认CURRENT_A为初始化电流上限×0.9（如2.5A→2.25A、3A→2.7A），SPEED_RPS=.02，TRIGGER_MS=300、STARTUP_MS=200、滤波10/20ms；ERROR_R对应0.5°/0.5mm，J1～J5=.06944444r、J6=.04166667r、J7=.1r。J1～J5减速比50，J6=30，地轨5mm/r；.02r/s分别对应.144°/s、.24°/s、.1mm/s。范围见同文件`:68`，各毫秒参数整数。合法EEPROM优先，不以源码默认/GUI初值冒充当前运行值。`stall_protection.cpp:99`提高电流限制不自动提高阈值；降低上限到阈值以下会将阈值调到新上限×.8，并裁剪回退/保持电流、速度、加速度。
- **回退与机械边界**：BACK_DEG=5（J1～J6）、BACK_MM=5（J7），BACK_RPS=3/.5为减速箱前电机轴，BACK_ACCEL=5、BACK_TIMEOUT_MS=5000、REBLOCK_MS=100、GRANT_WAIT_MS=500。回退/保持电流默认取初始化上限且受运行总限流约束，不建议降低承重轴保持电流。BACK_POS_TOL_R=.001r对应J1～J5 .0072°、J6 .012°、J7 .005mm，仍须实际编码器及固定50ms稳定窗口证明完成。`:636`普通回退再受阻电流门槛=min(CURRENT_A,0.8×BACK_CURRENT_A)，速度/误差/滤波使用普通参数，持续时间改用REBLOCK_MS，STARTUP_MS也参与内部动作豁免；CAL接触参数独立。MIN_POSITION_R/MAX_POSITION_R/DIRECTION_MASK只供普通串口GET，SET入口拒绝；由主控按当前坐标范围下发，J7正常为±50电机圈（±250mm），方向位定义在电机坐标。
- **串口规则及调参建议**：`dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:626`设置/保存必须在已完成DISABLE、协调器空闲且未保护锁定时；GET可在普通空闲态查询。STALL_SET经过确认/事务提交仅RAM生效，保存另用STALL_SAVE并实际失能后写Flash回读；单节点1～7，0展开ALL，8排除。ALL普通参数是相同电机单位值，ERROR_R建议分轴换算；BACK_DEG 0仅J1～J6、BACK_MM 0仅J7。逐条等最终回执，不把pending当完成。实测先GETI/STALL_GET并记录现值，保留已验证PID和回退设置，先正常静止/启停/往返，后受控低速单轴受阻；每次只改一项。正常启动误报优先STARTUP_MS 200→300；非启动短暂误报考虑TRIGGER_MS 300→400或按轴ERROR增至.75°/.75mm；真实持续受阻未触发则逐项降低CURRENT_A比例、ERROR或TRIGGER，不同时改变多个条件。SPEED_RPS和滤波先保留.02/10/20，5秒回退超时沿用户已确认决策。PROTECT通信超时及NO_ENCODER等不靠机械阈值调参解决，按phase/reason区分并保留完整日志。待用户实测，当前没有新的故障日志或确定某轴最终阈值。

## 已交付：用户确认J7新DCE参数改善本轮高速拖尾，默认值已提交5148016（2026-10-06 22:37实机反馈）

- **最新授权**：用户已实测反馈最终参数下未再出现问题，明确要求把`KP=300、KV=50、KI=30、KD=100`改为固件默认PID，同步文档后commit。修改前已说明文件和用途；本次新增生产改动仅电机端`firmware_new/dummy-motor-fw/UserApp/main.cpp`的MOTOR_TYPE==57默认DCE表及其注释，并同步`docs/地轨电机加入方案.md`与本文。既有未提交的57电流设置上限及目标助手查询按钮一并纳入本次用户授权提交，使用仓库本地Git身份。
- **实机对照**：22:35:49回读原运行参数`kp=250 kv=300 ki=30 kd=300`。22:36:32仅将KD降为100（其他为250/300/30）后，SEQ任务3/4正2302、负2849ms，差547ms，仍有方向差。22:37:04设置最终300/50/30/100后，SEQ任务5/6正2286、负2270，任务7/8正2290、负2315，任务9/10正2350、负2327ms；三对绝对方向差16/25/23ms，正向均值2308.7、负向2304.0ms，全部位于2270～2350ms。INT任务1/2仅accepted，不拿它们当正式完成样本。
- **结论边界**：同一晚100档工况下，最终DCE参数明显消除了此前约500ms方向差，支持闭环参数匹配是当前现象的重要影响因素；最终步骤同时改变KP与KV，不能唯一归因于KV，也不能据此证明其他工况或硬件完全无问题。本轮没有新的MOTION_STATUS阶段采样，不声称分别测到了pos/settled改善。保留已经正确的同步投递与实际完成判据，不通过放宽finish阈值掩盖问题。
- **生效方式**：启动`UserApp/main.cpp:34`先读取EEPROM，只有configStatus!=CONFIG_OK才初始化默认表，随后`:106`将boardConfig的DCE载入控制器。新值是57初始默认值，35/42/夹爪默认不变，已有有效EEPROM的用户设置不自动覆盖。用户本轮SET_PID请求保存EEPROM，但回显本身不能独立证明Flash写成功；如确认重启后运行值，用GET_PID 7回读，而不是为应用默认值擦除全配置。57默认电流保持2.5A，3.0A设置上限及此前用户已回读的运行3.0A保留。
- **验证及交付**：默认表读回确认57为300/50/30/100、35/42/夹爪保持200/80/300/250，EEPROM优先级未改。四目标Ninja 8/8编译通过；57 BIN为97052B、RAM10368B，SHA256为`E62D56F5D2C20FF99C598A5A5E4AD3560075112667B124E5AF347694C826F63F`，产物`firmware_new/dummy-motor-fw/build/motor-57.bin`及同名hex。35/42 BIN SHA仍`A32AEA43BCB023809A51D77D84CC10A982DAADC0838D1CA3C6AD17951331C29E`，夹爪仍`EAF0B3A126657402B56B4160DFDFD64967CA8FF5A359C590F2C36587737C21AB`。编译仅有既有MT6816虚函数缺返回及RWX链接警告，本轮不扩大修复。`git diff --check`和助手Python源码编译通过；参考串口助手.py SHA保持`D07D3E90968F42706C107EC038AFD5109E6A67FC0A12F3C2BC3D98D13FF0D7E4`。本次用户授权提交包含main.cpp、既有电流上限interface_can.cpp、查询按钮串口助手_old.py及两份文档；提交号见本次Git日志。主控、控制算法、STOP/堵转/校准、EEPROM优先级、参考目录均未改，未连接设备或烧录。

## 前序分析：J7的100档方向差位于实际跟随及最终稳定，尚未排除电机侧固件（2026-10-06 22:14实机快照）

- **最新证据**：附件`C:\Users\Liu3Beng\.codex\attachments\7fc6dab9-de68-498b-a22c-844eff86174b\已粘贴的文本.txt`中，SEQ任务9/11为用户负向全行程，J7电机目标为+18r（地轨inverseDirection=true）；总耗时2802/2781ms，tx=3/2、try=1、fail=0，plan=2203/2217、pos=2450/2427、settled=2798/2777ms。任务10/12为用户正向全行程，电机目标-18r；总耗时2281/2288ms，tx=2/4、try=1、fail=0，plan=2207/2214、pos=2172/2179、settled=2277/2284ms。四次均v约30r/s、a30r/s²、d约36r，其他轴目标不变且已很早判稳。总方向差约507ms来自J7，不能再归因于其他轴或普通目标下发晚500ms。
- **分阶段结论与边界**：负向规划完成后210/247ms才首次观察位置进入0.1mm容差，再经350/348ms首次观察SETTLED；正向pos可早于规划结束，plan至settled约70ms。plan/pos/settled为主控第一次收到相应状态的观察时间（`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:663`），有轮询粒度，不是电机内部精确时刻。首次进入位置容差不代表已经静止；后段350ms还不能区分持续爬行、往返微动、反馈噪声或其他判稳前置条件复位。真实跟随迟到已存在，不能通过直接放宽finish掩盖全部方向差。
- **90档对照更新**：本附件任务6/7总耗时2358/2331ms，J7 plan=2264/2235、settled=2334/2302ms，方向差仅27ms。本轮90档没有复现此前约500ms差，100档仍稳定复现；尚需确认测试之间是否改过PID、供电、负载或编码器电气校准。任务8重复相同+90目标、40ms完成是同目标查询流程，不作为提前完成错误证据。
- **固件核查没有结束**：真实规划器镜像验证与本轮tx/plan数据降低了目标投递/正负规划差作为主因的可能性，不能据此认定整个固件正常。`firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp:419`的DCE中KI位置误差与KV速度误差共同进入积分，KD为速度误差项；积分及总输出均限幅，普通新位置目标（`:691`）不直接清积分，清积分发生于使能/模式切换等NewCurve流程（`:221`）。保留积分本身并非已确认错误，不能直接改为每次清零。高速提前角补偿`:890`的注释明确需要按TLE5012/MT6816测量更新；这些与编码器匹配、命令电流饱和恢复均为候选，机械负载/高速电压余量也未排除。
- **判稳事实**：`firmware_new/motion_completion_protocol.h:21`及`dummy-motor-fw/Ctrl/Motor/motor.cpp:721`要求规划完成、有效编码器/正常使能等前提、实际位置误差≤0.1mm、50ms窗口位置跨度≤约0.02mm、窗口平均估计速度≤0.05mm/s；条件不满足会复位窗口，未设置固定350ms等待。当前快照不足以确定具体哪条条件导致负向窗口反复复位。
- **PID对照建议及待确认**：先取得`#GET_PID 7`完整kp/kv/ki/kd回包，不用编译默认值代替实际EEPROM运行参数，也不直接给未知基准上调四项。设置入口`dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:422`及`:436/:450/:464`均置保存标志；`dummy-motor-fw/UserApp/protocols/interface_can.cpp:217`接收后请求CONFIG_COMMIT，`UserApp/main.cpp:154`执行eeprom.put。因此应记录原值、DISABLE后一次只改一项，回读确认，再保持3.00A/a30及相同行程做SEQ正负对照，比较pos/settled而非只比较ok；不是RAM临时调参。具体调整项与幅度待实际PID和末端现象决定。已异步询问供电电压、末端爬行/微动/静止表现及本轮设置是否变化，等待用户回答。
- **本轮范围**：按AGENTS 5.2只同步本文分析记录；当前已有电流上限和助手查询按钮改动保留，未新增固件改动、编译或commit。以下22:03记录为前序对照；其“尚无MOTION_STATUS”待办已经由本附件完成，当前只缺PID及工况补充。

## 前序对照：J7运行3.00A已确认，单地轨100档仍有约0.55秒方向差（2026-10-06 22:03实机反馈）

- **采集入口已按用户要求完成**：用户要求在目标串口助手增加发送MOTION_STATUS的按钮。已修改`串口助手_old.py:331`，在左侧“查询”区、获取位姿下方增加“查询运动状态”，点击通过既有send_cmd发送`#MOTION_STATUS`，回包仍走原线程/日志入口；没有绕过STOP/校准门禁，也不干扰SEQ任务状态。Python语法及真实隐藏Tk界面创建/按钮调用验证通过，确认实际入队一次对应命令、SEQ状态保持、断连/STOP门禁生效；使用串口模块替身，未连接设备。读回布局源码验证，按钮请求宽92px适合现有320px左栏。参考`串口助手.py`SHA256仍`D07D3E90968F42706C107EC038AFD5109E6A67FC0A12F3C2BC3D98D13FF0D7E4`。用户重启助手后使用；固件没有新增改动或编译，无commit。下一步仍为正/负各一份运动完成后的快照及GET_PID 7。
- **最新已确认事实**：22:02:40 `#GETI 7 = 3.00A`，此前缺少设置后的运行回读这一项已解决，不再要求重复验证电流值；仍不能仅据此单独证明Flash存储结果。当前SEQ、关节保持REST、只将地轨在±90mm切换。任务20/22全幅正向2298/2261ms，均值2279.5；任务19/21/23全幅负向2819/2791/2866ms，均值2825.3；相邻配对差493与605ms。任务18起点日志未给出，不拿2111ms当作180mm全幅基准；任务24到0为半行程，亦不混入全幅比较。
- **解释修正**：此前联动100档正负237ms是整机差，不能当作J7自身差；其他轴较晚完成可能掩盖更早到位的地轨方向差。当前单地轨负向仍晚约0.55s，说明提高限流明显改善但未消除方向差。J6加速度缓存疑点不能解释“J6目标保持不变”的这组残余差，暂缓要求GETJACC 6，不为此扩大固件改动；其他静止轴是否受到机械振动影响判稳，仍需分轴快照确认。
- **代码检查范围与边界**：主控`dummy_robot.cpp:66～113`取距离绝对值，电机`motion_planner.cpp:190～345`正负规划分支镜像，已有144组真实源码逐周期对称测试；DCE `motor.cpp:419～449`正负输出采用同一个EffectiveCurrentLimit，没有单独负向限流或延时。3.00A是允许输出的电流命令上限，不是实测电流，不能据此确认高速时实际绕组电流足够。剩余候选为方向机械负载、高速电压/转矩余量、闭环与编码器标定匹配、到位后的稳定过程，目前不能唯一定位。若仍为a30且v30、180mm，参考预算约2200ms，与正向约2280ms接近；这只是当前既有参数下的计算，不冒充本轮实机计划采样。
- **下一步只收阶段数据，不再重复一般往返**：最新日志仍无正负分别保存的MOTION_STATUS。已异步请求保持参数，SEQ100档正负全幅各一次，每条自己的ok后、新目标前保存完整快照，再GET_PID 7回读实际DCE参数；暂不改PID/加速度/判稳窗口或超过已确认3.0A上限。计划相同而负向pos晚则优先跟随，pos接近而settled负晚则优先最终稳定；结合所有轴settled确认实际最后一轴。若需动态soft/real/velocity/foc诊断，先列新增文件与用途征求用户确认，不擅加固件日志。
- **本轮改动**：按用户最新要求修改目标助手的查询按钮，按AGENTS 5.2更新本文；固件源代码、构建产物及原电流上限试验版本保持，无新固件编译、烧录或commit。以下21:59记录作为前序对照保留，缺GETI/要求先查J6的待办以本节为准。

## 前序对照：提高J7电流后联动高速拖尾改善（2026-10-06 21:59实机反馈）

- **最新用户反馈与范围**：用户烧录后先GETI确认原运行值2.8A，21:58:20发送`#I_LIMIT_J 7 3.0 &`，随后反馈有所改善但正负方向仍不同。本轮分析并更新本文，不新增固件/配置/助手改动或commit；现有单文件57上限3.0A改动与21:55:46产物保持。
- **日志量化**：只使用SEQ正式完成回执（任务6～15），85档正2540/负2665ms，差125ms；90档正2526/负2645ms，差119ms；95档正2517/负2690ms，差173ms；100档正2472/2490ms（均值2481），负2730/2705ms（均值2717.5），平均差236.5ms。此前100档联动正约3376.5、负4037ms，现显著缩短；90负此前约3036.5，现2645ms。85/90/95各一对，100两对，不能用有限样本认定所有工况已解决。INT任务2～5只有accepted，不能把接受时刻当到位时刻。
- **当前解释及限制**：改善强烈支持限流提高后高速跟随/转矩余量有所改善，但不证明只有电流一个原因，FOC仍是电流命令而非实测绕组电流。新日志没有设置3.0之后的GETI回读，电机新版允许3.0不等于该帧/Flash必然成功，仍需运行回读；已有(saved)假成功语义缺口本轮没有修。残余119～237ms是整机任务差，还不能全部归给J7；本轮没有最新分轴快照确认谁最后判稳。
- **独立疑点沿用前次实机证据**：前次任务53 J6用主控a=200规划、实际plan=2410，而真实规划器按a=30得到2400.5ms（TX5ms、轮询粒度后吻合）；按a200只有2201.6ms。启动异步回读丢失后可使用J6=200兜底，电机默认30，因此可能存在加速度缓存不一致，必须GETJACC 6核实。该疑点可能造成约200ms基线迟到，不能直接解释J7负向差，不能用旧快照断言本次重启后的缓存仍相同。暂不改加速度/PID/判稳窗口来掩盖残差。
- **最少待补证据**：已异步请求先GETI 7确认3.00A，然后SEQ100档±90各一次，每条自己的ok之后、下一目标之前分别MOTION_STATUS保留完整34行；两份快照保存后再GETJACC 6（回读会更新缓存，避免提前改变对照输入）。比较各轴v/a/d、tx/plan/pos/settled及重试：计划相近但负向pos晚优先实际跟随；pos接近而settled负晚优先最终稳定/速度反馈；J6缓存与实读不符则先用已有SYNC_ACC完整回读复测，不自动下发新的加速度。无新固件需要烧录。

## 已交付：仅开放57电机3.0A设置，用户实机已反馈明显改善（2026-10-06 21:56）

- **最新授权与边界**：用户确定35/42最大设置电流2.5A、57最大3.0A，并要求先提高电流做实机对照，若无改善再深入分析。已在修改前说明仅修改电机`UserApp/protocols/interface_can.cpp`及本文；不自行提交、烧录或连接串口。当前35/42与夹爪仍接受0.1～2.5A，57接受0.1～3.0A；这是参数设置上限，默认值及EEPROM已有值不自动提高。用户应在烧录后显式设置J7 3.0A并GET回读，否则仍可能使用已保存的2.8A。
- **最新日志已经确认的事实**：21:41～21:44附件中，任务53的整机预算2200ms，J7 tx=5、plan=2205、pos=3080、settled=3430，整机3439ms；延后主要是规划完成之后实际位置/判稳迟到，不能再解释为500～800ms串口回执固定等待。pos是首次进入实际位置容差，settled是首次观察实际判稳，均受轮询粒度影响；查询发生在完成后11.6s，age=11661ms并非运动中离线。21:42:52回读J7加速度30r/s²，21:42:54回读运行限流2.8A。J7计划输入也为30r/s²，没有本次缓存/回读矛盾。
- **单地轨对照**：固定REST关节目标只动J7时，85档正向2361/2362ms、负向2420/2390ms；90档正向2322/2360ms、负向2981/2955ms；100档正向3499ms、负向3973ms。无需其他关节同时运动也能复现高速退化，因此不能以联动加速/整机共同供电峰值作为必要原因。仍不排除J7自身电压/转矩余量、方向负载、闭环/编码器匹配；提高2.8到3.0是否解决必须实测。
- **已实施的最小改动**：`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:169`给0x12添加局部作用域，以构建MOTOR_TYPE==57选择3.0A，其余2.5A；沿用有限值/0.1A下限、mA换算、ratedCurrent及boardConfig更新、OnMotionLimitChange和原持久化标记。没有更改规划、PID、加速度、完成判据、STOP/堵转/校准逻辑或上位机。主控仍立即打印接受/保存文本的既有缺口本轮只记录，因此测试必须以#GETI 7的3.00A回读为运行生效证据，不用(saved)文本代替实际保存验证。
- **本轮软件验证**：从真实0x12分支提取到现有真实Motor控制器/保护设置测试环境，35/42/57/8分别196/196/200/196项（合计784）断言通过；包括上下边界、2.5/2.8/3.0、超限、NaN/Inf、写入标记、普通有效限流及正负FOC电流命令限幅，57升到3.0后不擅自提高检测/回退/保持阈值。测试检查写入请求标记，并非实机Flash写入结果；FOC是电流命令不是实测电流。源码SHA和测试在忽略的`build/analysis/protection_regression/current_limit_source_manifest.json`及`current_limit_*_result.txt`。
- **编译与产物**：四目标Release/Ninja 8/8成功，最终dry-run无待编译；既有MT6816空虚函数无返回/RWX警告未改。57 BIN 97052B、RAM10368B，2026-10-06 21:55:46生成，SHA256 `8A42D7EBFE3ECE91C47F2F1E22D1768D9E072B945EBBFF177A0C0EBF76345C3B`。35/42 BIN仍`A32AEA43BCB023809A51D77D84CC10A982DAADC0838D1CA3C6AD17951331C29E`，gripper仍`EAF0B3A126657402B56B4160DFDFD64967CA8FF5A359C590F2C36587737C21AB`，三者与改前逐字节一致；主控BIN/SHA仍`19FF2DEE49DF98D9D65107C82B7CE0BAC1FF888CD18428A613AFD810C87BB695`。本轮只需烧录J7的`firmware_new/dummy-motor-fw/build/motor-57.bin`（或同目录hex）。
- **实机流程**：烧录J7后发送!DISABLE并等Disabled ok；#I_LIMIT_J 7 3.0（先仅RAM）、#GETI 7确认3.00A，再!START等Started ok。同一已验证±90点位、当前加速度30r/s²，85/90/100档各双向2轮，每条ok后、新目标前#MOTION_STATUS保存完整记录，比较J7 plan/pos/settled与拖尾。先不同时调整PID/加速度/保护阈值；若改善再DISABLE后用3.0 &保存，并在重新上电后GETI核实Flash生效。此操作由用户完成，代理没有下发命令或更改设备EEPROM。
- **其他问题仅记录，暂缓深入修改**：任务53 J6规划用a=200、v=6.92722、d=15，tx=5、plan=2410；真实规划器同输入a200结束2201.6ms，a30结束2400.5ms，加TX约5ms吻合本次观察。任务41 J6低速v1.23075/d7.5、预算6100，plan=6146；同源码a30规划6134ms，也与轮询/TX粒度相符。主控`dummy_robot.cpp:153,181～186`启动仅异步一次查询，未收到时:463～471使用fallback（J6=200，`dummy_robot.h:22`），与电机默认30可能不一致。这里尚未GETJACC 6，不能宣称其实际a一定为30；建议后续回读J6/用已有#SYNC_ACC确认，但按用户最新要求优先只做电流试验，不扩大固件修改。离线输出在rail_j6_acc_result.txt。

## 前序分析：同步实机通过；用户更正为J7高速拖尾，非已到位后的ok等待（2026-10-06）

- **提交与授权**：按用户要求，已使用本地身份 Liu3Beng-A / liu3beng@icloud.com 提交`6a7cdc4`，包含四个主控同步发送文件、两份既有报告更新及`handcode/ascii_protocol.cpp`、`can_protocol.cpp`、`dummy_robot.cpp`、`dummy_robot.h`四文件删除。提交说明同步修复及待分析回执延迟。本轮随后仅分析，未修改固件/配置/助手、未编译/烧录、未再次提交；按AGENTS.md 5.2更新本文。
- **用户最新纠正，取代之前描述**：20:55～20:58实机确认同步已无问题。最初报告停后300～800ms才ok并称七个*已齐；用户随后撤回该观察，确认J7后段仍慢慢移动且OLED的J7为未到位。因而撤回“七个*已齐但同轮确认迟滞”为本次主因的分析，保留当前等待J7实际到位才成功的判据，不放宽完成条件。现象称高速跟随滞后/拖尾；是否严格失步还缺参考轨迹/编码器的动态对比。
- **新线索与完整日志**：用户此前保存J7 2.8A后100档正常，当前80档正常，补充21:25～21:30同一轮85/90/100档往返记录。排除HOME出发的半行程任务2/25及同目标任务18后，85档正向2次均值2534.5ms、负向2次2580.5ms；90档正向11次2512.5ms、负向12次3039.1ms；100档正向3次3484ms、负向4次4062.5ms。100档双向均明显变慢，不能继续概括为“仅负向异常”。用户目测90开始提前减速，100更明显。ms是整段任务时间，不能当作停后回执延迟；日志仍无分轴MOTION_STATUS或本轮加速度回读。
- **确定的电流设置兼容性缺口**：电机`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:169～175`的0x12仅接受0.1～2.5A，超过2.5直接break，没有拒绝回执；主控`dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:376～387`仍直接返回ok/(saved)，不确认电机接受或Flash完成。历史092f5ed无该上限，5e5eddc加入保护时已经出现，本轮主控同步修改没有新加。新发送2.8A可能根本没生效，但不能据此认定当前一定为2.5A。
- **限流边界核实**：电机`UserApp/main.cpp:34～35,92`优先读取旧EEPROM，把boardConfig.currentLimit赋给ratedCurrent；正常`Ctrl/Motor/stall_protection.cpp:359～364`不夹到2.5A，仅保护回退/保持再取较小限制。因此早已有效保存的2.8A仍可加载，不会因最新主控升级自动降低。`#GETI 7`返回boardConfig的运行限流值，不是实测电流，也不单独证明Flash已保存。之前15:24的J7 CURRENT_A=2.52/BACK_CURRENT_A=2.8同样不能当作本次回读。
- **最新电流回读已回答**：21:26:27的`#GETI 7 = 2.80A`确认本轮运行配置确实为2.8A，不能再把本次直接归因为已降到2.5A。此前发送2.8本身可能被0x12拒绝而旧值恰为2.8；21:26:30的3.0A `(saved)`同样会被当前电机代码拒绝，不能证明提高或保存成功。后续没有3.0回读；不得把之后运动当作已有效增加电流的对照。这个兼容性缺口仍应单独修复，但尚未获本轮修改授权。
- **最新主控变化及排除**：76619f8→6a7cdc4的timeTrapezoid/ComputeSyncSpeeds源码逐字一致，普通帧仍目标+速度；电机/ctrl_step电气参数路径无差异。当前motor-57.bin SHA仍`6761481238E5F68A3E150CDF6CFDA6877915523EAAF106D7AF0DBFD3CF4A9CCC`，与上一轮17:36产物一致。单发0x07已经完整写入速度/目标，不能凭取消周期性目标重发认定缺少持续驱动。当前主要变化是实际目标投递与起步时间更集中。
- **优先待验证的解释**：先核对电机实际限流；若仍2.8A，比较真实规划v/a及地轨单独运动/七轴联动。同步起动改变各轴加速重叠，可能暴露供电压降、负载或高速转矩余量不足，但没有电压/实测电流数据，未认定电源问题。以J7直接丝杆5mm/r、示例a=35、距离180mm计算，80/90/100档上限分别24/27/30r/s（120/135/150mm/s），90对80峰速高12.5%、同质量动能高26.6%；实际同步反解可能低于上限，以MOTION_STATUS的J7 v为准。
- **后续候选，仅记录**：`dummy-motor-fw/Ctrl/Motor/motor.cpp:890～916`高速相位补偿说明为DPS传感器曲线，需对TLE5012/MT6816实测更新；1300000内部速度约25.390625r/s切换分段，位于24与27r/s之间。分段数值近似连续且本轮未改，不能据80/90差异直接判定补偿表错误。只在限流及联动负载排除后结合实际速度/跟随误差继续审查，不擅改补偿、PID、电流或加速度。
- **规划方向与提前减速核查**：主控`dummy_robot.cpp:66～113`按距离绝对值反解正数限速，J7逆向映射在:127、:517～520；用户坐标朝-90对应电机正方向。电机`motion_planner.cpp:173～178,190～345`使用同加减速度，制动距离为内部规划速度平方除以2a，判断基于trackPosition而非实际编码器位置。高速更早开始减速是正常梯形轨迹特征；若实际位置已经落后，规划进入制动时用户会见到尚距目标较远就降速，随后位置环补误差。这是吻合现象的机制解释，尚无实机soft/real采样证明本次一定命中。示例a=35、J7实际峰速等于85/90/100上限25.5/27/30r/s时，正常单边制动距离约46.45/52.07/64.29mm；本轮实际加速度/峰速尚不明确，不能将示例当作实机减速点。
- **真实源码离线方向验证**：构建目录`build/analysis/protection_regression/rail_direction_analysis.cpp`直接包含当前motion_planner.h/cpp，并提取当前CompensateAdvancedAngle函数；MSVC /O2 /fp:fast执行144组（24/25.5/27/30r/s、4组加速度、3组行程、3组初速），逐控制周期校验镜像位置/速度相等，共14,309,281项断言通过。180mm、a35、静止起步示例24/25.5/27/30r/s规划结束2.18820/2.15130/2.11125/2.05960s，未出现高速计划本身更慢或方向差。扫描0～30r/s相位补偿，单单位跳变最大1、正反符号不对称最大1内部位置单位；未找到85/90附近突变导致数百ms的直接证据。测试不含电机、电源、编码器硬件和机械负载，不能排除真实闭环/传感器补偿不匹配。源码SHA和输出分别在rail_direction_manifest.json、rail_direction_result.txt，均为忽略的分析产物，未修改生产源代码。
- **所需最少实机证据**：电流2.8A已有回读；尚需#GETJACC 7以及85/90/100同点位双向完成后立即#MOTION_STATUS，保留完整34行，勿先发下一条覆盖快照。tx/plan/pos/settled是主控首次观察时间，具有查询/丢包误差，必须结合try/fail/age及其他轴看，不能冒充精确电机内部时刻。再在原已确认行程内保持关节目标固定，仅地轨±90运动，比较相近J7规划v/a下是否仍拖尾；若plan正常而pos/settled晚且反馈健康，优先跟随问题；若plan也晚，继续查实际规划输入、运动限制和反馈，不能仅凭日志总耗时判定硬件。
- **未完成/待确认**：J7本轮加速度、规划峰速、各轴计划/实际观察差及单轴/联动区别；尚未定位唯一根因。当前优先候选为高速转矩/电压余量、方向机械阻力或闭环/编码器标定匹配，100双向退化使“纯负向机械故障”也不能单独解释全部现象。本轮仅分析及按AGENTS 5.2更新本文，固件/配置/助手未改，未编译固件/烧录/再次提交。确定后再列修改文件/用途并等待授权，不自动提高电流或放宽完成判据。

## 已交付历史：七轴普通目标发送确认修复，用户已复测同步通过（2026-10-06）

以下保留20:40交付记录；其“待烧录/未提交/四文件删除未处理”为当时状态，最新实机及6a7cdc4提交见首节。

- **授权与修改前提交**：用户已同意主控普通目标发送/分轴诊断方案，随后要求先提交再实施。已用仓库本地身份 Liu3Beng-A / liu3beng@icloud.com 提交`76619f8`，说明上一轮完成流程成果、目测200～300ms抵达差和100ms补发缺口及下一步工作。本轮修复未再commit、未连接串口、未烧录。已有`handcode/`四文件删除未提交、未处理。
- **当前问题与已确认事实**：用户实机确认上一轮提前ok/OLED跳动未再出现，但部分轴晚到。普通MoveJ此前与现在均为逐节点单播；主控CAN关闭自动重传，旧100ms补发未关联ALST/TERR，因此丢首次目标可能放大起步差。汇总日志没有分轴时序，不能断言全部实机晚到均由丢帧引起。稳定同步公式、实际加速度/PID/有效电流及INT非零起速的边界仍按修改前分析。
- **最新改动，仅四个主控生产文件**：`Bsp/communication/interface_can.cpp/.hpp`给普通0x07在途邮箱关联节点/epoch/generation，实际TXOK/ALST/TERR/abort通知运动任务；`Robot/instances/dummy_robot.cpp/.h`区分入邮箱/发送成功/目标READY，轮转首发，短周期有界重试，过期/取消淘汰，发送失败转现有七轴软停保持锁定；新增各轴tx/ready/plan/pos/settled观察时间及v/a/d、try/fail诊断。沿用#MOTION_STATUS和old助手日志，无新增UI或线上的帧格式。
- **落实的常规决策**：ALST/TERR/abort后5/10/15ms错相重试，HAL拒绝入邮箱5ms重试；20ms在途请求非阻塞abort，唯实际结果释放令牌；发布起100ms七轴至少一次TXOK、300ms七轴目标READY、每节点16次成功入邮箱上限。失败明确TARGET_TX_TIMEOUT/TARGET_READY_TIMEOUT/TARGET_TX_RETRIES，FAILED终态清队列且软停保持，无自动断电/续旧目标；USB自动打印等待位图及各未确认节点。预算针对目标建立，不是几秒运动的实际到位超时。
- **保持与已排除方向**：保留上一轮编号/来源/同轮实际判稳/SEQ推进/OLED锁存。同步数学函数、普通50ms/0.1°/0.1mm、PID/电流/加速度、EEPROM、电机端、STOP/堵转/校准动作、串口助手、参考目录和esp32均未修改。未做统一GO或INT非零起速模型；不能把停稳观察时间当作电机内部精确时刻。
- **验证与边界**：真实源码主控38865、电机45338、电机CAN54657、主控CAN5814、配置132，共144806项C++断言通过；隐藏真实Tk普通编号/SEQ410及保护/校准53检查通过。新增1050组单轴0/1/2次发送失败模型、双CAN三邮箱所有权、立即回调、旧代次、重复回调、abort竞争、预算/回绕及诊断行长。两次失败模型首次TXOK≤30ms，不是实机保证；HAL/RTOS/总线/物理负载用替身或模型。源码SHA与产物在`build/analysis/motion_delivery_artifacts.json`。
- **最终构建与产物**：主控Release/Ninja成功，2026-10-06 20:40:23 BIN428236B，RAM56368B（增加400B），FLASH428236B；主控及电机ninja -n无待构建项，电机产物未变。BIN `E:/Dummy-code/firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`，SHA256 `19FF2DEE49DF98D9D65107C82B7CE0BAC1FF888CD18428A613AFD810C87BB695`，同目录HEX；本轮只需烧录主控，保留现有已验证八电机。既有RWX警告保留。
- **待用户实机测试**：DISABLE后SYNC_ACC确认finish，START完成后真实SEQ；档位35已确认±90点位往返5轮，每次自己的ok后、新目标前查询#MOTION_STATUS并保留完整34行。比较移动轴TXOK差及try/fail、规划用a和首次plan/pos/settled观察；J2/J3在该组往返不动。再HOME↔RESET5轮、现有SEQ、运动中STOP/START停点保持及DISABLE取消。若仍不同步提供完整快照及较晚轴，勿擅改电流/阈值。详见`docs/普通运动完成确认修复与测试.md`首节。

## 修改前分析历史：普通完成实机复测通过，七轴抵达时间差（2026-10-06）

以下为授权实施前的分析记录；其中“待确认/不修改”是当时状态，本轮授权及实施结果见上节。

- **最新反馈与授权**：用户19:50～19:53日志及目测确认上一轮提前完成/OLED跳动问题未再出现，但J1～J7部分轴晚约200～300ms或更久。当前只要求分析原因和方案；不修改固件、助手或配置，不编译、不提交。按AGENTS.md 5.2仅更新本会话记录。
- **日志事实**：19:51:04 SYNC_ACC finish；19:51:48真实模式已切到SEQ。SEQ任务13～18整条执行耗时3959～4064ms，现有记录没有分轴起步及实际抵达时刻；不能用汇总耗时证明同步或确认具体节点丢帧。日志只单独打印J4加速度30r/s²，其余本次实际参数未打印。
- **已确认代码缺口**：主控`Robot/instances/dummy_robot.cpp:382`逐轴下发，:384～391将HAL入邮箱成功作为motionSentMask，并在未READY时100ms重发；`Bsp/communication/interface_can.cpp:345～365`返回的是HAL入邮箱结果。`Core/Src/can.c:50`关闭自动重传，CAN错误回调只计数和释放资源，未将普通目标ALST/TERR反馈给运动任务。电机`UserApp/protocols/interface_can.cpp:135～153`收到0x07即更新速度/目标，没有七轴统一起步门禁。因此首次目标仲裁丢失可以造成100ms整数倍的起步及抵达偏差。上一轮100ms策略有此设计缺口，但尚不能断言本次每条实机偏差都来自它。
- **规划/反馈边界**：稳定同步小根及固定预算未在上一轮改变；主控`dummy_robot.cpp:41～113`按静止起止建模，:439～463只有位置及加速度，没有初始速度；电机同位置模式换目标保留PositionTracker速度（`motor.cpp:221～268,691～709`），故INT途中反向另有非零起速模型缺口，不能以此解释所有SEQ现象。数值内存模拟采用J1/J6=30、地轨30或35r/s²的示例参数，同起步全幅±90/轨±90时整数规划终点差仅数ms；不是实机测量或当前全轴参数确认。
- **完成确认与物理运动分开**：50ms判稳、35ms每轴轮询及回复重试影响回执/OLED观察时间，不会统一电机实际抵达时刻。负载、PID及有效电流限制的跟随滞后尚无本轮数据，不能直接归因为电流不足。首次READY/PLANNER_READY/SETTLED收包时间必须标记为主控观察时刻，不能冒充电机内部精确时间。
- **推荐第一阶段**：仅改主控普通目标发送及分轴诊断：区分入邮箱/TXOK/失败，关联节点及epoch/generation；ALST/TERR/abort后短周期、有界错相重试，TXOK后停止高频重发并取新nonce确认；取消/STOP/DISABLE淘汰旧目标与在途重试，超时明确失败及软停锁定。保留已实机验证的任务编号、SEQ完成门禁/OLED锁存、同步公式及保护参数。扩展MOTION_STATUS记录各轴发送成功及首次目标/规划/位置/停稳观察时刻、速度和规划使用的加速度，完成后保留一条快照。
- **后续按证据处理**：若TXOK相近而编码器晚到，进一步核对跟随误差和控制器输出电流命令；若要求严格统一起步，另行设计普通运动PREPARE/统一GO，必须包含任务代次、超时、迟到GO拒绝及漏收后的整体保护，不能把保护回退GO直接复用为普通运动入口。INT非零起速需独立补齐模型，保留覆盖语义。上述固件改动均待用户确认；没有擅自实施。

## 第二轮历史：普通运动完成流程，用户已复测通过完成问题（2026-10-06）

以下保留当时交付记录。用户之后复测未出现此前提前ok/OLED跳动，成果已提交到76619f8；本轮烧录范围及发送策略见首节。

- **授权与基线**：用户已同意拟实施方案并要求修改前提交。已用仓库本地身份 Liu3Beng-A / liu3beng@icloud.com 提交 `1c64174`（普通完成修复前基线），后续不自行提交，不烧录。
- **已确认事实**：上一版实际判稳窗口只有8内部单位，远小于关节/地轨的0.1到位容差，且完成结果会被后续反馈撤销；主控完成位与实时状态混用，旧扩展回复即使被拒绝仍可更新位置缓存，普通任务没有独立编号及终态。最新SEQ日志的同秒完成不能归因于INT；目前不声称已定位每一条实机回执的唯一原因。
- **已完成实现**：普通运动任务编号/代次/不可变七轴下发快照，目标READY与同轮实际停稳门禁，编号完成回执、按来源回复、完整串口行。电机FINISH历史与实时SETTLED分离；OLED每任务进度锁存；仍50ms与0.1°/0.1mm实际容差，峰峰范围0.02°/0.02mm，速度有符号均值绝对值0.01电机r/s；J8原细窗口不扩大。正常任务5ms查询轮转，单节点nonce重试；STOP/堵转/校准判据及START耗时不扩大修改。
- **文件与关键路径**：`dummy_robot.cpp/.h`任务/队列/唯一成功门禁/诊断；`ctrl_step.cpp/.hpp`及主控`interface_can.cpp/.hpp`本条快照及每次HAL等待后检查代次；主控`can_protocol.cpp`旧证明不能更新姿态、legacy反向证据撤销；`ascii_processor.hpp`完整行；`ascii_protocol.cpp`编号入队、真实模式、MOTION_STATUS；`UserApp/main.cpp`调度；`protection_coordinator.cpp`仅取消桥接；电机`motor.cpp/.h`历史/实时分离及阈值；共享协议flags；`串口助手_old.py`匹配id的SEQ、模式确认、毫秒日志。未改参考目录、串口助手.py、EEPROM/PID/速度公式或RGB。
- **验证**：真实源码主控12286、电机45338、电机CAN54657、主控CAN234、配置132，共112647项C++断言；隐藏真实Tk编号/SEQ/模式/分段410项，保护/校准回归53项。覆盖旧证据后RUNNING、stale姿态拒绝、发送中替换/STOP、慢回复同nonce、已完成扰动、新180°目标、队列取消/来源、错误模式与串口长度。35ms完成查询/保护持续仲裁模型共存通过，不是实机验收。
- **最终编译**：主控与35/42/57/gripper五目标Release/Ninja通过；主控17:58:29 BIN422884B、RAM55968B，SHA256 `DA82C2FD0940D0B6B35F08754147DB854316321BB68EEF01F14B29ECCC36B397`。电机35/42 97060B、57 97052B、gripper97012B，RAM10368B，95KiB应用区最小余量220B。完整SHA/文件用途/流程在`docs/普通运动完成确认修复与测试.md`最新版首节及`build/analysis/ordinary_completion_round2_artifacts.json`。保留既有RWX/编码器缺返回警告。
- **待完成**：仅用户烧录及实机反馈；需主控和J1～J7配套更新、运行新版old助手，J8可保留此前已验证固件。先HOME/RESET10轮，再SEQ±90档位35往返20轮和存储点序列，检查实际到位才匹配id完成、OLED不重开、STOP/DISABLE取消不续发。异常用MOTION_STATUS与毫秒日志。无代理烧录；修复未再次commit。
- **补充常规决策**：普通目标HAL入邮箱后若无READY，100ms重试；未入邮箱仍下一控制周期重试。避免20ms目标重发的旧RUNNING回复反复撤销待确认查询，兼顾电机可靠回复100ms上限；更晚的矛盾实时反馈撤销确认及该节点待回复nonce，必须重取新证明。`#GET_CMDMODE [req]`回显查询编号并返回已分配id水位，GUI拒绝迟到旧查询与旧queued回执。IO不在临界区等待，queued公告完成前命令线程不执行该条任务；取消通知同样等待公告。
- **模式边界及未扩大问题**：TUN不走普通点位调度，点位/HOME/RESET在TUN下明确MODE_UNSUPPORTED，避免无限等待；切回1～3执行。32条小取消记录队列满时计数诊断而不伪造成功；不新增串口结果重放服务。MoveL的SolveIK有效性检查、START耗时和真实堵转/回退/校准硬件验收仍独立保留，不扩大本轮。

## 第一轮历史：普通位置运动完成确认（被第二轮替代，2026-10-06）

- **授权与最新实机反馈**：用户15:22～15:27静止、HOME/RESET与多轮STOP没有再次出现PROTECTION_STATUS_TIMEOUT，记录为该轮通信复测改善，不等同于真实机械堵转/回退验收。START仍约1～2秒；HOME后RESET偶尔同秒返回。用户明确“可以，先修复普通运动的完成确认”，本轮不优化START、不commit、不烧录。
- **已确认原因**：主控`Robot/instances/dummy_robot.cpp`普通目标只清汇总FINISH、未清各节点缓存；任意普通回包会重建旧FINISH。`IsMoving`只看七轴FINISH；电机`Ctrl/Motor/motor.cpp`普通位置FINISH只看规划位置/速度。`0x07`即时回复可先于20kHz状态刷新。相关代码允许误报完成，不能仅凭日志判定每次竞争发生点。
- **本轮改动与常规决策**：新增`firmware_new/motion_completion_protocol.h`，复用0x23扩展查询：目标电机圈数+标记+24bit非零nonce，回复实际圈数+目标就绪/停稳flags+回显nonce；七轴新目标清完成缓存，每轴只接受本次目标代次中发出的新查询，300ms反馈有效期，不以CAN入邮箱视作完成。电机每次新目标撤销旧FINISH，同目标周期重发不清判稳；普通位置要求实际误差关节约0.1°/地轨0.1mm和独立50ms窗口（峰峰值≤8内部单位，估速有符号均值绝对值≤0.01电机r/s）。不复用STOP/内部保护窗口，不修改其判据或EEPROM。
- **发送兼容与边界**：主控完成查询沿用原普通查询有界1ms等待，避免紧跟0x07时零等待导致查询饥饿；电机`Core/Src/can.c`带标记0x23进入既有TXOK/有界重试队列，按nonce区分，不被普通位置样本覆盖。HOME/RESET规划失败不报成功；SEQ/TRJ普通位置等待新确认，SEQ/TRJ MoveL补齐相同等待；INT的即时ok仍是接受新目标。J8不参与整机完成。无新增上位机指令，助手接口不变。
- **验证与交付**：真实源码主控11339、电机38332、电机CAN51009、主控CAN201、配置132，共101013项断言通过；覆盖旧FINISH/nonce/代次/超期、未接收目标、实际未到位/运动/噪声、计时复位和真实ParseCommand完成/失败/取消回执。七节点同时完成回包与保护查询的持续仲裁模型通过；不是实机验收。16:02最终五目标Release/Ninja通过，主控418192B/RAM55712B；电机35/42 96756B、57 96748B、gripper96708B，RAM10360B，95KiB应用区最小余量524B。SHA与判据/测试见`docs/普通运动完成确认修复与测试.md`及build/analysis/ordinary_completion_artifacts.json。
- **待完成与边界**：用户需更新主控及J1～J7；J8可保留上一轮已验证版，本轮gripper也已编译。复测HOME→RESET10轮、地轨到位、STOP/DISABLE取消和SEQ下一条不提前。未烧录、未commit、未改GUI。START串行耗时、RGB不扩大；另记录MoveL未检查SolveIK返回/解标志、INT原有规划失败即时回执语义，以及电机FLASH余量，均未擅自改动。

## 当前工作：保护回包持续仲裁修复完成，待用户更新八电机（2026-10-06）

- **最新授权与实机反馈**：用户14:40～14:46 HOME/RESET反复出现J2/J3 `PROTECTION_STATUS_TIMEOUT`，节点通信age仅4/10ms、probe=1005ms，ALST持续增长；参数GET已证实300/200ms、0.5°误差及各节点90%电流阈值生效。没有机械reason=STALL，因此没有回退符合通信保护分支。用户明确“那你先修复吧”，允许继续修复和编译，不再提交、不烧录。
- **已确认发送缺口**：电机`Core/Src/can.c`保护回包在HAL入邮箱成功时出队，NART关闭且无发送结果重试，仲裁失败会丢失已入队回复。上轮测试覆盖有限丢帧，未覆盖持续运动仲裁。不能凭主控累计ALST直接断言每一次都丢在同一端。
- **已完成及边界**：仅本轮新增修改电机`Core/Src/can.c`。保护回复独立12槽（有效11）优先队列，TXOK才完成，ALST/TERR回调保留帧并1～4ms错相重试，16次尝试（含首次）/100ms双上限；0x23只合并未发送旧反馈。单一在途所有权避免硬件空闲/旧RQCP尚未处理时误用回调，过期在途abort。主控时隙/1秒看门狗和本地500ms租约不变；普通阈值、机械回退、校准动作、GUI不扩大修改。
- **验证及已排除**：新增真实电机CAN源码27308断言通过，含TXOK前不出队、三邮箱错误/abort/忙/回绕/独立事务/合并/超时边界。每5ms重复低ID运动仲裁模型，旧版八node均3秒0条保护回复，新版各19条、最大探针年龄160ms；这不是实机验收，不保证每次丢包均在回复端。前四组重跑全通过，总45678（11160/6877/132/201/27308）；阈值已生效及节点仍在线排除本次“纯机械阈值低/节点离线”解释。
- **编译交付**：主控与35/42/57/gripper全部Release/Ninja通过。主控415388B/SHA与上一轮相同；电机35/42 95572B、57 95564B、gripper95524B，RAM均10336B，95KiB应用区最小余量1708B。当前SHA在build/analysis/stall_fix_round2_artifacts.json；测试流程第9节与方案第16节记录发送决策和复测。
- **下一步**：用户只需更新八电机，主控保持上一轮。静止2分钟、HOME/RESET往返10轮，通过后测STOP和机械堵转回退；异常保留完整分项日志。本轮无新commit、未烧录；USB/其它校准问题保留。

## 上一轮：实机堵转误报修复完成并已提供烧录（2026-10-06）

- **最新授权**：用户要求先提交当前基线并说明问题，然后修正堵转、编译，由用户烧录测试。已使用本地身份 Liu3Beng-A / liu3beng@icloud.com 提交 `5e5eddc`；后续修复不自行再提交，不烧录。
- **实机证据**：附件有7次 `NODE_OFFLINE_OR_RESTART`，没有 `reason=STALL` 的机械检测事件；另有校准J8 `LEASE_TIMEOUT`、226条重复阶段行及两次USB ClearCommError。警告 STOP_LOCKED 是用户点击其他运动命令后的正常门禁，不归为故障。
- **已确认原因/局限**：`protection_coordinator.cpp` 健康判断将探针1s超时与EVENT缓存PROFILE无效合并并错误命名STALL；START范围下发不刷新探针，0x23位置回包不计在线。两端CAN NART使入邮箱成功不能证明总线成功，8节点320ms轮转丢一帧可达640ms超过500ms租约。实机每次到底命中哪项尚缺诊断，不能认定真实断线或硬件重启。
- **已完成修改**：0x36原子查询快照含坐标版本/flags/probe/实际位置；START新增下发后再确认；独立在线时间与保护快照；同编号探针有界重试；RENEW nonce确认/20ms重试、80ms正常续约及STOP优先；仅当前内部动作中的校准节点续约。主控故障打印PROTECT与分项诊断，阶段变化才打印；old助手适配，参考助手不动。
- **常规决策**：普通默认电流命令阈值90%有效上限、误差关节0.5°/地轨0.5mm、持续300ms、豁免200ms，速度阈值0.02r/s不变；EEPROM版本保持1且不覆盖用户已有设置。无有效堵转方向的通信保护仍只软停保持锁定；不为它执行猜测回退。不扩大校准动作或USB故障修复。
- **最终验证**：五类Release/Ninja构建通过；真实源码离线18370断言通过（主控11160、电机6877、配置132、CAN201），隐藏Tk界面/串口替身53项及分段回包检查通过。新增覆盖慢START、旧EVENT、入邮箱后丢探针/续约、七轴同时触发后的串行回退、真实离线及保护快照失联、NaN反馈拒绝、重复CAL阶段。未宣称实机通过。
- **产物及文档**：主控FLASH415388B/RAM55544B；电机35/42 FLASH93068B、57 93060B、gripper93020B/RAM9792B。最终SHA在build/analysis/stall_fix_artifacts.json；[堵转误报修复与实机测试流程](E:/Dummy-code/docs/堵转误报修复与实机测试流程.md)记录具体文件、协议/阈值决策、产物和测试命令；原方案第15节更新本轮决策。
- **待完成/边界**：只待用户烧录主控及全部八节点并实机验收；旧固件不能满足新START证明。EEPROM不自动迁移新默认值，必须GET核对再显式SET；USB断连及其他校准问题只记录。基线已提交5e5eddc，本轮修复未再提交、未烧录；既有Encoder缺return/RWX警告保留。

## 历史：初版堵转回退与 CALIBRATION 实施

## 当前工作：堵转回退与 CALIBRATION 实施完成，待用户烧录（2026-10-06）

- **授权**：用户已明确“开始执行改动工作”，允许按已确认方案修改、编译和多轮检验；禁止自行烧录和 commit。
- **当前改动**：电机端独立保护/校准上下文、版本化设置与 Flash 实际回读；主控新增 `Robot/instances/protection_coordinator.{h,cpp}` 统一协调内部动作、CAN 确认重试与保存；串口助手仅修改 `串口助手_old.py`。
- **实施细节**：主控每次 START 前重新核验八节点有效反馈与坐标版本，事务下发 J1–J7 的机械范围；DISABLE 确认与编码器有效性分开，避免实际失能但编码器无效导致 pending 永不结束；START 与 STOP 保持仍要求有效实际位置。协议 0x0A action=5 仅供堵转联动软停保留本地保护事件，用户 STOP=1 取消内部动作；上下文租约、查询探针和配置操作回显拒绝陈旧确认。
- **验证**：主控和MOTOR_TYPE=35/42/57/8四种电机固件Ninja/Release构建通过；真实主控/电机/规划/EEPROM/CAN共11203断言通过（4042/6849/111/201），真实Tk+串口模块替身47项通过。测试脚本、源码SHA和日志在主控`build/analysis/protection_regression/`；最终资源以方案第14节为准。已严格测试晚到轴先PREPARE再GO、校准取消后J8 START恢复且普通STOP仍排除J8。
- **补充决策**：校准J7异常转普通5mm回退后仍保持校准失败锁、清提前START；取消校准特别停止并确认J8，后续新START解除该内部保持；普通STOP仍不参与J8。每次START同步坐标版本与机械范围；CAN发送时隙/错误邮箱释放和实际Flash结果墓碑防丢包/重放；六轴真实50ms静止后失能标记REST。电机CMake仅对新增保护源禁异常/RTTI、FLASH限定95KiB；所有具体文件和作用/接口/范围/取舍已写方案第13节。
- **未完成/下一步**：仅待用户烧录主控及全部八节点做实际负载、阈值、噪声、承重保持及CAN硬件测试；离线机械模型不是实机验收。首次假定坐标下接触无法绝对排除中途障碍，单圈编码器也不保证掉电多圈恢复。未自动烧录、未commit；参考固件、`串口助手.py`、`esp32/`未改。


## 历史节点：堵转/自动回退与CALIBRATION只读分析（已被上述实施授权取代）

- **授权边界**：用户本轮明确要求把当前决策与方案写入文档，然后继续分析 CALIBRATION。仅授权方案和本状态记录写入；固件、配置、上位机实现仍须列文件/作用并等确认，不构建、不烧录、不提交 commit。
- **方案入口**：`docs/堵转保护与自动回退及校准方案.md` 保存职责、状态机、参数、协议、异常、拟修改模块和验证计划；尚未实现。后续从该文档继续，不重复推测用户已经决定的需求。
- **最新用户决策**：J8完全排除普通堵转检测；J1～J6回退5°、J7回退5mm；关节统一减速箱前电机轴3r/s（接受J6速度不同），地轨电机轴0.5r/s；回退超时5秒；其他参数用此前建议值；提前START在回退中挂起，全部成功后自动保持解锁，失败不自动解锁。
- **校准要求**：J1～J6只把当前位置修正为REST_POSE={0,-75,180,0,0,0}；J8向开度0寻物理限位、记0、回100，默认上电位置100；J7向逻辑正端寻限位、记+250mm、回0；全部完成才回`!CALIBRATION ok`。用户确认沿用现有行程：地轨默认0、可向±250mm；夹爪0～100是HAND开度映射。
- **2026-10-06最新确认**：CALIBRATION必须失能入口，未失能时主控提示先DISABLE，串口助手old参考顺序发送提示使用独立弹窗；不自动入口失能。全部返回原位并判稳后自动J1～J8失能，确认后保存EEPROM，全部保存回读成功才校准ok，最终失能。J8上电当前位置作为开度100，保持默认失能，上电不自动运动；只有CALIBRATION才寻0并返回100。此前上电/保存两项问题已答复，不再重复询问。
- **本轮确认的校准事实**：主控`dummy_robot.cpp:602`仍会改变J2/J3电流、广播两次置零、运动到REST并重启；电机`motor.cpp:822`原点单圈取模且未同步目标/规划/STOP保持点，`interface_can.cpp:188`再取模并立即请求Flash保存；不能直接复用。HAND实际是开度映射，`dummy_robot.h:75`与`ctrl_step.cpp:152`给出0～100对应约4.444电机圈，不按“100圈”旧注释实现。J8在`interface_can.cpp:36`拒绝普通STOP，需要校准专用取消/保持动作。
- **新增建议**：独立非阻塞校准，六轴停稳固定捕获并定义REST；J8闭合寻端/复触/记0/回100，随后J7正端寻限位/复触/记250/回0；实际到位并判稳后统一确认。本期采用既有行程和开度映射，不新增普通J8堵转检测。地轨0.5r/s下移动250mm约100秒，校准采用独立长超时；STOP必须取消正在校准的J8。无机械限位输入时，接触判定不能绝对排除中途障碍，方案明确窗口/复触/行程/操作前提。
- **本轮补充核查**：主控`dummy_robot.cpp:728`会在DISABLING时挂起START，`886`会自动转STARTING，必须在校准/保存上下文中禁止该路径；此前给校准设计的提前START建议已被最终失能要求取代，普通堵转回退的提前START保留。UI`串口助手_old.py:141`初始False及`1298`提前显示失能都不是节点确认，主控须新查询核验八轴。现有EEPROM.put无真实保存结果，get走RAM缓存，保存确认必须读取实际Flash。
- **2026-10-06本条已获同意**：用户明确同意校准及保存期间START返回busy、不挂起；堵转回退提前START仍按原决策；失败或取消时不保存不完整校准、不返回成功。已将方案第2节和12.6节由推荐更新为已确认，不再当待定项。
- **推荐细节/未实现**：六轴仅修改坐标、全程失能，只有J7/J8按阶段内部使能；部分保存列已保存掩码；EEPROM有界保存、CRC和真实回读。实施前仍列准确文件和作用并等确认，不构建、不烧录、不commit。
- **最新改动**：只更新方案文档与本段状态，替换“等用户另发DISABLE保存”“校准ok仅RAM完成”“校准期间挂起START”“完成保持使能”等旧建议；补齐入口弹窗/真实失能检查/自动保存/最终失能及验证计划。历史STOP及加速度同步结果保留，不把先前独立异常当成已经解决。

---

## 当前工作：失能状态同步实机成功，串口助手增加提示（2026-10-05）

- **最新实机结果**：用户提供21:38:51节点1读取30.00 r/s²、21:38:53同步pending→finish日志，并明确确认“失能就可以查询”。记录为失能状态下本次八节点同步实机成功；此前使能状态下两次全部missing仍保留，未据此确认具体CAN竞争或回包丢失机制。
- **用户授权与改动**：用户要求串口助手增加提示，仅在`串口助手_old.py`同步按钮下增加常驻提示：“建议在失能状态下同步加速度；使能时若超时，请支撑好机械臂后失能重试。”该提示专指同步，不把已正常工作的单节点GETJACC规定为必须失能；不自动发送失能指令。
- **边界与验证**：本轮不修改或重新构建固件，不修改参考串口助手，不提交commit；读回实际布局、Python语法、隐藏真实Tkinter界面与`git diff --check`均通过。确认提示紧随同步按钮、宽度不超过参数面板，按钮默认仍可用、发送队列为空；串口依赖用替身隔离，未连接硬件。
- **未完成**：使能状态下的同步发送调度原因未确认，本轮按用户要求提供操作提示，不扩大为固件调度修复。失能状态下pending→finish已由实机日志确认。

---

## 当前问题：SYNC_ACC实机两次全部节点超时（2026-10-05，原因待定位）

- **实机反馈**：21:28:51与21:29:06发送`#SYNC_ACC`，均先收到`ok SYNC_ACC pending`，再收到`error SYNC_ACC timeout missing=1,2,3,4,5,6,7,8 invalid=-`；21:29:00单节点`#GETJACC 1`返回30.00 r/s²，21:29:04节点7返回35.00 r/s²。用户询问为什么ok和失败同时出现。
- **已确认语义**：`Robot/instances/dummy_robot.cpp:214`的pending仅确认接受请求；最终结果由第296～299行返回finish或error。当前日志没有finish，因此本次同步失败，不能把pending视为缓存同步成功。代码仅在FINISHED分支统一发布本次样本。
- **已确认诊断局限**：第262行missing由本次已接收掩码计算；未发送、回复编号不匹配、零编号普通回复、超过500ms的回复都可能表现为missing。现有回执没有sent或原始收到/编号匹配信息，不能根据全部missing认定八节点均未回复。
- **复查路径**：普通`CtrlStepMotor::QueryAcceleration():322`发送零载荷并使用等待信号量的原CAN路径；新`TryQueryAcceleration():330`携带非零编号，并在`Bsp/communication/interface_can.cpp:250`使用零等待发送。OLED固定20ms尝试与控制任务的固定轮询可能遇到发送资源竞争，需要实机发送计数才能确认。当前firmware_new电机`UserApp/protocols/interface_can.cpp:340`及`Core/Src/can.c:181`确实将查询后四字节原样送出；参考firmware/motor_fw_f103_35的对应回复会清零后四字节。未取得当前设备实际0x2C帧，未认定固件版本不配套。
- **未完成/待确认**：同步实机验收未通过；需区分查询是否成功入邮箱、实际收到哪些0x2C回复、后四字节编号是否匹配，再决定发送调度或协议兼容修复。离线测试无法覆盖实际调度相位与设备所烧固件。pending前的ok容易被理解为同步成功，可在后续回执调整时明确接受/完成两阶段。
- **本轮改动范围**：仅记录实机失败与代码复查结论，未继续修改生产代码、重新构建、烧录或commit；此前授权的同步实现仍保留在未提交工作区。

---

## 当前工作：独立异步加速度同步与完成回执已实现（2026-10-05，离线验证通过）

- **授权与范围**：用户批准上轮方案并确认已提交此前修改；开工时 tracked 工作区干净，仅已有未跟踪 `%SystemDrive%/` 目录，未触碰该目录。修改 firmware_new 主控、目标串口助手及本记录，不修改电机固件、参考固件或 `串口助手.py`，不执行 commit。
- **已确认事实**：现有电机 `UserApp/protocols/interface_can.cpp:340` 的0x2C回复只改写前四字节，后四字节原样回传，且DLC为8。主控新查询用后四字节携带非零请求编号，可复用现有电机协议区分旧回包；普通零编号查询保持原流程。
- **最新改动**：主控增加独立 USB `#SYNC_ACC` 入口，pending先于后台执行；OLED任务每20ms尝试发送一个节点、CAN发送不等待信号量或邮箱。收集八节点编号匹配且有限正值的回复，齐全后统一更新主控缓存并返回finish；500ms超时列出missing/invalid，重复请求busy，STOP或运动世代变化cancelled。为此补充主控CtrlStep本地查询帧和Bsp非阻塞CAN发送辅助函数，原发送路径保持原实现。
- **串口助手**：同步按钮改发 `#SYNC_ACC`，等待期间禁用重复点击；SYNC_ACC回执支持跨串口读取拼接，pending/finish/error通过root.after处理，2秒无完成回执恢复按钮并提示；断连或STOP取消本地等待。前轮已删除的全部读取按钮保持删除。
- **验证结果**：主控Ninja 12/12成功，最终`ninja -n`无待构建项；`git diff --check`通过。提取当前主控同步/ASCII/查询/CAN函数及现有电机0x2C回复生成离线测试，33,291项检查通过，覆盖旧编号/零编号/重复回包、缺失或非法值、CAN忙重试、计时及编号回绕、STOP/世代变化取消、pending先于执行和超时后迟到回复拒收；6,561种missing/invalid组合的最长回执为56字节，低于64字节缓冲与USB单包上限。测试在`firmware_new/dummy-ref-core-fw/build/analysis/acc_sync/generate_test.py`与生成的`acc_sync_test.cpp`，使用替身HAL/RTOS，不代表实机总线已验证。
- **串口助手验证**：隐藏真实Tkinter界面565项检查通过，覆盖按钮发送独立指令、重复点击禁用、完成/失败/无回执/断连/STOP恢复、回执全部分段方式、普通参数查询保留以及SYNC_ACC不推进SEQ点位。原STOP界面/状态/取消/发送线程回归389项通过；仅在忽略的旧测试脚本中适配UTF-8 BOM读取和当前已提交的`[ !STOP ]`按钮文字。串口依赖用内存替身隔离，未打开任何设备；新测试在上述目录`ui_test.py`。
- **编译产物**：主控`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`，392,936B，2026-10-05 20:55:02生成；SHA256 `3DC55D64FCF4EEE0AB75934C3EE5310DA22683B2AFFD2AD0E4C6CB2D8491847F`。RAM55,504B、FLASH392,936B，保留既有链接RWX警告。电机代码与产物未改；基于当前firmware_new电机0x2C原样回传后四字节，无需本次配套重编电机。
- **未完成/实机验收**：软件修改和离线验证已完成，未烧录或commit。用户需烧录本次主控并重启目标串口助手，点击后应先见`ok SYNC_ACC pending`，八节点本次有效回复齐全且缓存发布后才见`ok SYNC_ACC finish`；500ms不足八节点则报timeout/missing，非法值报invalid，失败不发布本次部分样本。实机回执、所烧电机固件的后四字节回传及高负载CAN时序待设备验证。
- **已排除与边界**：不直接复制参考版仅dispatched的SYNC_ACC；不把发完CAN查询视为同步完成，也不把旧缓存当作新回复。未扩大修改原ALL查询结果打印、通用Respond/USB长文本发送、ACC_J自动回读或规划兜底值。

---

## 当前工作：加速度同步与 ALL 查询审计，删除串口助手全部读取按钮（2026-10-05）

- **当前问题**：用户询问为何同步按钮使用 `#GETJACC 0`、发送提示被拒绝的修复方案，以及 `#GET*** 0` 批量查询是否必要；明确要求删除 `串口助手_old.py` 的“全部读取(8 节点)”按钮。固件仅分析和提出方案，尚未授权修改相关协议或发送逻辑。
- **已确认事实**：`docs/查询端补齐方案.md:18` 记录以 ALL 查询替代独立 `#SYNC_*` 命令的设计；`Robot/instances/dummy_robot.cpp:178` 逐个查询 8 个节点，`UserApp/protocols/can_protocol.cpp:69` 将加速度回包写入主控缓存，MoveJ 第307行使用 J1～J7 缓存。ALL 分支没有八轴完成确认或缺失节点汇总，`PollPidPrint()` 只打印 PID，未实现 ACC/I_LIMIT 的 ALL 结果打印。
- **已确认发送缺陷**：主控 `UserApp/protocols/ascii_protocol.cpp:369` 的 GETJACC ALL 提示为75字节；`Bsp/communication/ascii_processor.hpp:29` 的64字节缓冲使用 snprintf 所需长度发送，`Bsp/communication/interface_usb.cpp:75` 又把总长度而非 chunk 传给64字节上限的发送器，故正文被拒绝，仅发送换行。当前 BIN 含该提示，ELF 反汇编确认发送缺陷仍存在；未据此认定设备已烧录的内容。
- **已排除方向**：当前 firmware_new 已没有旧版 accBase 百分比设置逻辑；主控仍有与电机默认30 r/s²不一致的加速度规划兜底。删除界面全部读取按钮不会修复 ALL 回包或发送缺陷；外部 `#GET*** 0` 是批量调试接口，主控内部启动回读并不依赖这组 ASCII 命令。
- **最新改动与验证**：仅删除 `串口助手_old.py` 的全部读取按钮、专属空容器和 `_qry_all` 回调；保留单参数读取、节点0选择及蓝色同步按钮。读回实际源文件、Python语法检查和隐藏Tkinter界面创建通过，确认删除项不存在、其余读取按钮存在、蓝色按钮仍只发送 `#GETJACC 0`。验证时以串口模块替身隔离默认Python缺少pyserial的依赖，未打开设备连接。未改固件或参考 `串口助手.py`，未重新编译、烧录或commit。
- **未完成/待确认**：用户决定是否保留外部 ALL 查询、是否引入明确的加速度同步入口；发送可采用短提示或完整长文本分包，但共同需要修正 Respond 长度处理，支持长文本时还需修正 USB chunk 传参。同步完成/缺失节点反馈及 ACC_J 设置后的缓存自动回读仍未修复。

---

## 当前工作：STOP 判稳窗口优化实机未见显著问题，主控等待诊断保留（2026-10-05）

- **最新实机反馈**：用户反馈“这次没有什么显著的问题”，未提供逐项验收日志；记录为本轮实机复测未见显著问题，不推定此前 HOME/USB 独立异常已解决。
- **诊断保留建议**：用户询问是否撤回上轮主控关节等待打印。核对 `Robot/instances/dummy_robot.cpp:831` 及 `UserApp/main.cpp`：详细快照只在 STOPPING/STARTING/DISABLING 等待至少 2 秒后输出一次，重复 START 可重新触发；USB 打印在 OLED 任务的 PollStopPrint 中执行，不在运动控制任务里打印。只记录阶段/回包/轮询数据，不参与判稳条件或决定解锁。建议保留便于定位偶发等待；等待节点不能直接认定为“卡死”。本轮没有修改诊断代码或重新构建固件。

- **已确认实机事实**：重新烧录 J8 后，19:01 的 START/DISABLE 均成功完成 pending→ok，之前仅缺 J8 的握手问题已解决。19:08～19:10 多次 STOP 可完成；慢的几次均收到 J1～J7 的 0x0A 回包，token 正确，但 J4 或 J4/J6 仍为 phase=1（DECELERATING）。例如 `ack=0x0EE wait=0x010 rx=0x0FE` 仅等待 J4；`ack=0x0AE wait=0x050 rx=0x0FE` 等待 J4/J6。期间 START 保留 pending，停稳后按既定方案自动完成。此次日志没有证明 ENABLE_HOLD 丢包。
- **用户补充**：等待 STOP 完成时，肉眼观察所有关节已经停止；手碰机械臂后才收到完成回执。与微小编码器反馈反复清零判稳计时的方向一致，但未取得实机速度/位置窗口采样，不能将具体噪声幅度或 PID 状态视为已测得事实。
- **代码证据**：电机 `firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp` 的既有 20kHz 估速器中，从零开始一次 1 内部单位的位置跳变产生 625 单位/s，即 0.012207 电机 r/s。旧 STOP 每个周期要求瞬时估速≤0.01 r/s，且位置相对窗口起点±8，连续 1000 周期（50ms）；一次速度超限即清零计时。真实控制函数注入有界反馈跳变，复现了位置变化仅 3 单位仍 20 秒不确认。
- **本轮改动**：仅修改电机模块 `Ctrl/Motor/motor.cpp/.h` 的 STOP 判稳；本地加速度减速、实际停点捕获、保持、保护和协议不变。参考速度归零后采集 1000 周期窗口，要求所有位置样本峰峰值≤8，窗口估计速度的有符号均值绝对值≤0.01 电机 r/s。位置范围超限立即重启窗口；窗口末平均速度超限也重启；不增加超时成功。新增最大位置及 int64 速度累计字段，20kHz 内使用整数运算。峰峰值≤8 比原起点±8 的允许空间范围更严格。
- **验证**：28 组真实完整 20kHz 控制函数反馈测试，新版静止及 1/3/4/6/8 单位有界跳变、6 单位三角波均在参考零速后 50ms 确认；9/16 单位超范围波动、±0.004 r/s 持续漂移和 0.02/1 r/s 运动在 20 秒预算内不确认。新版噪声回归 2,422,094 项断言通过；既有真实电机 35/夹爪回归分别 10,629,070 / 10,629,068 项通过，含 78 组速度/加速度、保持隔离和 DISABLE。测试替换硬件接口，不代表已测得实机噪声或闭环效果。记录在主控 `build/analysis/stop_regression/settle_result_old.txt`、`settle_result_new.txt`、`motor_result_35.txt`、`motor_result_8.txt`。
- **编译产物**：四种电机 Ninja 36/36 成功，19:21:27 生成；35/42 BIN72816B、57 BIN72808B、gripper BIN72776B，RAM8176B；最终电机/主控 `ninja -n` 无待构建项。35/42 SHA256 `6676AED6D07DAAFF1763C8018259126D4493F708CD92FB30C99EE58A4AEB8D90`；57 `9F249A76789A3BDBFF4C5F057D4E246614208A52216E8F9F3440680FE4ED74A8`；gripper `216231A136A1847CA9914D1D4D1BADBDBC07FAF55185165DE4DE37A99C67350D`。主控 BIN389604B、SHA256 `8BF59F47D9CFA1274A47A938A78BD8E27DB3D688293BFE9C27A8BA904004490E` 未变。
- **交付与待完成**：本轮用户只需更新 J1～J7：J1～J6 按实际电机类型选择 `motor-35.bin`/`motor-42.bin`，J7 选择 `motor-57.bin`；主控保持当前诊断版，J8 保持已验证工作的版本，协议未变化。反复测运动中 STOP、无需触碰即可确认、减速中 START 等待完成、停点保持及 DISABLE。若仍慢，记录等待节点和 phase；软件复现只证明已修正一条噪声敏感路径，实机改善尚待验证。
- **其他问题仅记录**：19:01 HOME 后 DISABLE 快照 `polls=0 rx=0`，与本轮 `polls>0 rx=0x0FE phase=1` 属于不同证据，不能用判稳优化解释或宣称修复；该次实时任务未继续轮询的原因未确认。19:10:50 的 USB ClearCommError 同样未定位，未改 USB。CAN 发送的无限信号量等待及正常目标共用发送缓冲需要另行验证，不能据此认定实机根因。本轮不改 HOME、普通 FINISH、主控、PID、串口助手或其他模块，无烧录、无 commit。

---

## 前一阶段：实机 START/DISABLE 持续 pending，主控诊断版已编译（2026-10-05 18:36，历史记录）

- **实机事实**：用户确认已更新主控和全部八个电机；串口助手连接 COM20 主控 USB 虚拟串口，UART4 连接 ESP32 暂不用。J6 重新烧录后恢复 OLED 角度刷新，目前可见的 J1～J7 均刷新，J8 不在 OLED 上，未取得其 0x0A 握手日志。
- **18:49～18:50 诊断结果**：START 停在 STARTING/QUERY，DISABLE 停在 DISABLING/DISABLE；两者均 `ack=0x0FE wait=0x100 rx=0x0FE badpos=0x000`，`polls=403 txbusy=0`，`J8 no 0x0A reply`。已确认 J1～J7 的对应阶段回包通过，仅缺 J8 的 0x0A 回包；不是收到 J8 FAULT/旧 token 后被拒绝。START 尚未进入 ENABLE_HOLD，因此主控没有完成本次八轴使能；DISABLE 已确认七轴失能，但八轴完成条件未满足。
- **夹爪产物核对**：用户确认实际文件是 `firmware_new/dummy-motor-fw/build/motor-gripper.bin`，计划重新烧录 J8。当前 Ninja 夹爪目标无待构建项；构建命令为 MOTOR_TYPE=8，ARM ELF Main 把运行时 ID 设为 8，OnCanCmd 对 QUERY/ENABLE_HOLD/DISABLE 均调用 RequestSoftStop 后发送 0x0A 状态回包。QUERY 不需要等待控制中断或编码器校准完成。BIN72640B，SHA256 仍为 `AEFAEAC29BD8CA2D4654F6F6213FD811C11BB533BA83C7FB35583305EAFA408C`。代码/产物核对不能证明设备 Flash 内容或 CAN 链路状态。
- **现象与结论边界**：多次 START 均只返回 `Start pending`，DISABLE 只返回 `Disable pending`，未见完成回执。位置刷新走 0x23，不能证明 0x0A 确认齐全。目前尚未确定哪个节点、哪个阶段或发送等待导致无法完成；不把 J8 缺失、CAN 配置或 UART4 未使用当作已确认根因。
- **纠正前述分析**：用户说明 `[WARN] STOP 锁定中，仅接受 !START 或 !DISABLE` 是点击 MoveJ 等其他指令后的正常拦截。已撤回把该提示认定为实现错误的结论，本轮不改串口助手锁定/拦截规则。
- **最新改动**：仅在主控 `Robot/instances/dummy_robot.cpp/.h` 的 STOP 流程补充等待诊断。每阶段等待超过 2 秒后，通过 USB 输出一次 state/action、ack/wait/rx/badpos 位图、控制轮询次数、CAN 发送调用是否尚未返回及当前节点；对未确认节点打印无回包或实际 phase/token 与期望 token。等待期间重复 START 可重新触发快照。日志只观察状态，不放宽确认规则、不自动解锁；电机固件和串口助手本轮未改。
- **本轮验证**：主控真实状态机/队列/ASCII/CAN 发送函数及诊断共 391 项断言通过。新增覆盖缺失 J8、状态未稳、旧 token、非有限位置、阶段切换不沿用旧诊断、计时回绕与发送阻塞快照；核对每条输出小于实际 Respond 的 64 字节缓冲。Ninja 主控 11/11 成功，最终无待编译项，FLASH389604B/RAM55400B；既有 RWX 警告仍在。四种电机 BIN SHA256 与上轮一致。
- **当前主控产物**：`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`，SHA256 `8BF59F47D9CFA1274A47A938A78BD8E27DB3D688293BFE9C27A8BA904004490E`。本次仅需用户更新主控，电机保持上一轮配套固件。
- **待完成**：等待用户重新烧录 J8 后的结果。主控保持当前诊断版，当前 DISABLING 会持续重试；J8 回包通过后应收到 Disabled ok，再测试 START。若仍缺 J8 回包，需进一步验证 J8 基础 CAN 回复/设备 Flash，而非假定其停稳、PID、故障状态或改为忽略 J8。另已询问“无法失能”是实际仍有保持力还是只有回执未完成，尚待回复。实机 pending 问题未宣称修复。此次未再改固件/串口助手，无代理烧录、无 commit。

---

## 上一阶段：STOP 初版已完成编译与软件回归（2026-10-05）

- **授权与范围**：用户已批准实施上轮 STOP 方案；仅修改 STOP/START/DISABLE、相关锁态/取消/反馈和目标串口助手，其他问题记录后提出，不擅自修复。无 commit、无代理烧录。
- **已确认决策**：J1~J7 按各自电机本地生效加速度减速，实际停稳后捕获停点并保持；STOP 期间仅 START/DISABLE 被执行；其他指令丢弃，包括新夹爪指令，夹爪原动作继续；提前 START 等七轴停稳后自动解锁；DISABLE 让 J1~J8 全部失能并取消待执行 START。
- **最新实现**：新增 `firmware_new/stop_protocol.h`（CAN 0x0A 请求/回包及 token）；主控 `dummy_robot.cpp/h` 增加状态机、队列 epoch 和旧任务取消；周期任务处理软停协议并停止旧目标发布；两套 ASCII 严格匹配 START/DISABLE，原生协议锁态拦截；电机 `motor.cpp/h` 复用速度跟踪器降速、实测稳定 50ms 后捕获编码器停点，切换位置保持；电机 CAN/调试串口/按键锁态拦截；`串口助手_old.py` 增加软停回执状态与待发点位取消。
- **收尾边界**：HOME/RESET/CALIBRATION 转后台并增加 epoch 取消检查，原动作内容保留；CAN 等待后拒绝已取消任务。首次 STOP 逐轴同步 token，缺失节点不妨碍其他轴软停；START 同步 token 防主控单独重启后的重复值。原生 UART 清旧半包，USB 记录收包锁态/epoch；Fibre 仅增加半包丢弃方法；串口助手只为 STOP 回执增加跨读取拼接。
- **最终验证**：主控真实函数 336 项、真实电机 35/夹爪分别 10,629,070/10,629,068 项、原生分包 1475 项、真实 Tkinter 389 项断言全部通过。包含完整 20kHz 控制函数、78 组速度/加速度、提前 START、八轴 DISABLE、缺失/旧回包、冷启动缺失节点、token 回绕及每种回执切分。测试硬件接口被替换，不能代替实机闭环验证。
- **最终构建**：Ninja 主控及 35/42/57/gripper 均成功，最终 dry-run 无待编译项；主控 FLASH387380B/RAM55360B/既有 CCMRAM65536B；电机 RAM8160B，FLASH 分别72672/72672/72664/72640B。主控 BIN SHA256 `A946452E8B4E0CAF724A11D6C1F49ED488EBCDECE8BFFCB1BB85B9E3E55BC8F7`。无新编译错误，既有非 void 空函数/RWX 警告未改。
- **交付与待测试**：详见 [STOP软停实现与测试报告.md](STOP软停实现与测试报告.md)，含产物、SHA256、协议、范围、验证与实机步骤。必须主控与全部八个电机配套更新（夹爪虽排除 STOP，也需更新 DISABLE/START 握手）。用户烧录验证实际减速、重力负载保持及 J8 行为；本代理未烧录、未连接设备、无 commit。
- **其他问题只记录**：普通 FINISH 只检查规划值；UART5 原生接收误用 uart4_stream_input；多通道 ASCII 共用静态缓冲；普通 CAN 准备缓冲仍共享；非 STOP 串口回包不跨读取拼接。证据与既有编译警告已列入报告，没有扩大修复范围。`esp32/`、`firmware/`、`firmware_old/`、`串口助手.py` 无本轮改动。

---

## 当前状态：串口助手已增加主动同步加速度按钮（2026-10-04）

- **授权与目标**：用户要求参考 `串口助手.py`，在 `串口助手_old.py` 增加同样作用的按钮，用于主动同步主控 RAM 中记录的加速度。
- **最新改动**：`串口助手_old.py:349` 在电机参数面板的加速度设置行下方增加蓝色“同步加速度至主控”按钮，沿用参考按钮文字与颜色；发送 `#GETJACC 0`，固定查询全部 8 个电机，不依赖当前节点选择。
- **已确认事实**：主控 `Robot/instances/dummy_robot.cpp:142` 上电调用 `SyncAllMotorAcceleration()`；CAN `0x2C` 回包在 `UserApp/protocols/can_protocol.cpp:66` 更新 `jointAccRuntime`；MoveJ 第286行读取 J1～J7 缓存用于同步规划。当前固件手动入口为 `#GETJACC 0`，没有旧参考的 `#SYNC_ACC` 入口，因此按钮使用已有有效指令。
- **验证**：Python语法检查及隐藏Tkinter窗口实际创建通过；在节点0/1/7/8下按钮均可点击，调用按钮回调均只发送 `#GETJACC 0`，不受写入按钮禁用逻辑影响；读回源文件及 `git diff --check` 通过。参考 `串口助手.py` SHA256保持不变。本轮未连接设备验证CAN回包，也未修改或编译固件，无commit。
- **待确认**：用户重启目标串口助手后，点击新增按钮验证实机缓存刷新。当前单节点加速度“应用/保存”仍不自动回读，设置后可点击新增同步按钮。
- **用户反馈与工作范围**：用户表示目前同步抵达已经很完美；TRJ高频发送偶发无响应难以复现，用户明确要求放下该问题，停止继续排查或修改。RGB仍刷新、电机停止的现象仅作记录，没有确定根因。

---

## 上一阶段：偶发整体降速数值修复已编译（2026-10-04）

- **授权与目标**：用户已明确要求按稳定公式修改固件并编译，由用户烧录测试。复现方式为INT模式、实际停止后手动在±90点位间以相同档位往返；所有运动轴偶发同时降速但仍同步。
- **最新改动**：仅本轮修改主控 `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`。同步速度反解改为 `v=2d/(T+sqrt(max(0,T²-4d/a)))`；预算固定为各轴受用户/物理限速约束的最短时间最大值，删除用反解浮点误差重新放大T的迭代；梯形时间等价改为d/v+v/a。保留1..100→0.3..30r/s、加速度回读/兜底、轴索引及已有微小距离阈值。
- **回归验证**：从修改后的源码提取真实C++函数与头文件常量，使用本机MSVC /O2 /fp:fast编译并运行。10404组不动J2/J3微小残差及加速度组合中，地轨限速保持24r/s；单细分残差例预算保持2.185714s；另2000组随机轨迹与零位移、三角曲线、加速度兜底、方向对称、重复确定性测试通过，共180089项断言。测试在主机执行规划函数，未模拟电机闭环硬件。
- **验证产物**：可复查 `firmware_new/dummy-ref-core-fw/build/analysis/sync_regression/sync_regression.cpp` 及 `result.txt`（均为构建目录内测试产物，非新增生产代码）。最终ARM ELF的MoveJ反汇编已确认vsqrt后使用vadd/vdiv稳定反解。
- **编译结果**：主控Ninja 2/2完成，退出码0；FLASH379272B、RAM55240B、CCMRAM65536B。既有LOAD segment RWX警告仍存在。`git diff --check`通过。
- **烧录产物**：`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`（379272B），同目录hex/elf。BIN SHA256：`e64d1d68f43cea245b82e79a62439004e8687351d2ac72d02939e78f7d516202`。本轮无电机端源代码修改，只需烧录主控。
- **待测试**：固定INT、点位±90、相同档位80，保持电流与加速度参数不变，等实际停止后多次往返，确认整体偶发降速是否消失；同时检查低速档位/短行程的同步性。尚未获得这版实机结果，不能宣称实机问题已经解决。
- **范围与遗留**：本轮未加入到位死区或以旧目标替代编码器起点，也未修改FINISH判定、参数缓存刷新、规划结果原子发布或CAN共享缓冲。本轮修复的是浮点残差引起的明显降速，不保证任意不同实际起点/初速度下完全相同的物理速度波形。没有commit或代理烧录。

---


## 修复前分析：相同点位与档位往返偶发整体降速（2026-10-04）

- **用户已确认**：仍为之前±90点位；INT模式；实际停止后手动发送；偶发所有运动轴一起变慢，仍能同步到达。本轮请求深入分析，尚未授权修改该问题的固件实现。
- **已定位的数值缺陷**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:105` 用 `(T-sqrt(T*T-4*d/a))*a/2` 求小根；不动的J2/J3也因编码器与浮点坐标转换残差形成小距离。相近数相减损失有效位，使其反解速度严重偏差。第110..117行用偏差速度重算各轴时间取最大，再迭代，微小残差可放大全局T，导致所有真正运动轴同步降低限速。
- **负数方向已缩小**：disc<0在第104行已有钳制；扫描中未发现负速度或非有限速度。关键在相减消去和迭代放大，不是浮点随机变化。相同指令并不保证相同规划输入，因为MoveJ第282..298行每次取异步实际位置与加速度缓存。
- **32位浮点复现（示例，不是实机CAN采样）**：按上一目标已实际到达，距离向量为地轨36圈、J1/J4/J5各25圈、J6为15圈、J2/J3不动；档位80，地轨a=35，其余a=30。J2残差0时T=2.185714s、地轨24r/s；仅加入1/51200电机圈残差（约0.000140625°关节角）后，T=2.730667s、地轨15.793456r/s。所有运动轴按同一个被放大的T降速，符合用户“仍同步”的特征。
- **转换链验证**：模拟实际CAN位置float→除50减速比→乘360→inverse→加initPose(-75)链路，J2目标0°在电机目标整数化后也存在小残差；例如编码器目标附近+1内部细分单位，读回currentJ2=-0.000190735°，a2=30示例T=2.4691s、地轨18.5706r/s。读数微变可触发明显降速。
- **参数扫描**：J2/J3残差0..16个内部细分单位，加速度15/30/35/100/150/200r/s²的10404组组合；旧公式T范围2.185714..4.915198s、地轨速度7.665813..24r/s；无负数/NaN。离线改为等价稳定小根 `v=2*d/(T+sqrt(max(0,T*T-4*d/a)))` 后，同一扫描地轨均为24r/s，T保持初始预算误差小于1ms。
- **ARM编译证据**：把仓库真实timeTrapezoid/ComputeSyncSpeeds函数和常量提取到标准输入，以当前ARM GCC15.2.1、C++17、-O3 -Ofast、硬浮点Cortex-M4编译为标准输出汇编；结果保留 `vsqrt.f32` 后的 `vsub.f32`，并未自动稳定化。未写入源文件或重新生成固件。
- **其他确定但未验证为本次原因的缺陷**：规划起点与加速度非一致快照；速度数组逐项写入后再写目标，Realtime发送任务可观察中间组合；CtrlStepMotor共享canBuf/txHeader在准备消息阶段无互斥，发送函数取信号量之前共享内容可能被另一调用改写。INT常规MoveJ只由实时任务周期发送，故共享缓冲竞争不是本次最优先解释。
- **建议修复**：先用稳定小根替换相减；固定理论最大时间预算，去除由微小残差反算误差驱动的全局放大；在实际到位容差内区分不动轴与待修正误差。若要求同一A/B路径下发速度严格一致，需明确“已实际到位”的起点规则，不能在INT中途覆盖时一律使用上一目标。增加规划输入/预算/输出日志验证实机触发，再考虑原子发布规划结果。
- **本轮改动与验证边界**：仅更新本文；Python/NumPy float32离线模拟与ARM汇编核对，不是执行整份固件或读取实机CAN。尚缺实机异常次的起点、七轴加速度缓存、T与outSpeeds记录；目前确认代码缺陷和可重复数值触发，尚未宣称实机唯一根因。无commit。

---


## 最新状态：地轨高速拖尾由限流不足触发，用户已复测解决（2026-10-04）

- **复现**：从 `>90,0,90,90,90,90,90,30` 转到 `>-90,0,90,-90,-90,-90,-90,80`，地轨实际到约-40mm后慢速走到-90mm，OLED七轴均显示*，rail数值持续改变、运动方向始终朝目标。
- **用户实测参数**：J7 PID/DCE为 kp=180、kv=600、ki=30、kd=300；加速度35 r/s²；电流上限1.50A。用户发送 `#I_LIMIT_J 7 2.8 &` 后，以80档双向运动复测，反馈拖尾消失。此次现象的直接触发因素已收敛为1.5A限流不足，不再优先修改同步反解算法。
- **默认电流核查**：`firmware_new/dummy-motor-fw/UserApp/main.cpp:56` 的57地轨默认已为2500mA；从地轨集成提交 e3bc506 起就是2.5A。启动先读取EEPROM，仅配置无效时使用默认，随后把boardConfig.currentLimit赋给ratedCurrent（main.cpp:33、34、88）。源代码默认值不意味着保留已有EEPROM的重新烧录会恢复默认。
- **1.5A可能来源**：目标串口助手的电流输入原先预填1.5A，手动点击应用/保存会下发；预填本身不会自动写入电机。历史保存或运行时应用均可覆盖电流，#GETI查询boardConfig运行值，不能独立证明1.5A已保存或追溯具体设置来源。
- **本轮授权与改动**：用户要求改为2.5A并核查默认。已把`串口助手_old.py`电流预填值改为2.5A，与固件默认一致；固件源代码无需修改，EEPROM优先级保持原设计。更新本文及速度/加速度报告的实机补充。
- **当前设备与待验证**：用户刚保存的是2.8A。要设置设备为2.5A，应发送 `#I_LIMIT_J 7 2.5 &`，再用 `#GETI 7` 回读并重启复查。本代理未连接串口或下发设备命令。用户已验证2.8A消除拖尾，2.5A下相同行程/80档仍需复测。
- **独立缺陷仍存在**：FINISH只检查内部规划完成，未检查实际到位；此缺陷解释了拖尾时显示*，提高电流仅消除了本次跟随滞后。#ACC_J不刷新主控加速度缓存的问题也未修复，本轮不扩大固件改动。
- **验证与提交**：串口助手源文件读回及Python语法检查通过。未修改固件，未新增烧录或commit。

---

## 历史分析：高速地轨拖尾及同步抵达（2026-10-04）

- **用户补充确认**：地轨慢速移动期间 OLED 七个状态字符均为 `*`。已结合代码确认“轨迹完成被当成实际到位”的缺陷在本次现象中显现；是否属于地轨实际位置滞后仍需确认位置数值，因为用户的“显示已到目标”可能仅指状态字符。
- **待区分**：已询问慢速期间 `rail:xx.xxmm` 是否等于目标、是否继续变化、运动方向是否一直朝目标。该读数通过 GetPosition(false) 来自编码器累计 realPosition，而非 softPosition/goalPosition。若数字也一直等于目标但仍有明显行程，应转查位置反馈、采样冻结或机械传动，不能继续把大行程归因于补位置。
- **修复边界**：修正 FINISH 应检查规划完成、实际位置误差、实际速度及持续稳定时间，影响全部位置模式电机及 SEQ 队列放行；只更正 FINISH 能解决误报/提前发下一条，不能保证消除高速拖尾。下一步诊断应同时记录 J7 的目标/规划/实际位置、规划/实际速度和电流，确认闭环跟随误差来源，再决定同步算法或控制参数改动。

- **用户现象**：档位约 20 时各轴同步到达；40 以上地轨在 J1～J6 完成时转入慢速，怀疑丢步/同步规划。尚缺起止位置、完整指令、实际加速度与拖尾阶段状态；已向用户询问。
- **确定缺陷 1**：电机位置模式的 FINISH 只检查 `softPosition==goalPosition && softVelocity==0`，不检查编码器位置/实际速度（`firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp:313`）。主控 IsMoving 直接依赖这些状态（`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:533`）。规划结束可早于物理到位。
- **优先假设（未实机确认）**：地轨高速跟随滞后，七轴参考轨迹到点后，J1～J6 已实际到位，而地轨闭环仍在补位置。PositionTracker 使用内部 trackPosition 而非持续用实测位置修正（`motion_planner.cpp:192`），DCE 仍按目标与估计位置误差输出（`motor.cpp:382`）；此机制与观察吻合，不等于已经证明丢步。
- **确定缺陷 2**：串口助手 #ACC_J 应用/保存只下发电机，不更新主控 jointAccRuntime 或触发查询（`UserApp/protocols/ascii_protocol.cpp:390`）。全局 SetJointAcceleration 已更新缓存，但串口助手不走该入口。单节点设置后规划可能继续用旧加速度；启动查询也没有缓存就绪屏障。
- **参考功能澄清**：当前与 firmware 均有 SyncAllMotorAcceleration，含 J7，作用为查询回填；在已核查的 MoveJ 路径中未发现运动时逐轴分配加速度梯度。参考还将 ALL/J1..J7 索引数组直接传给“地轨/J1..J6”规划数组，不能整体照抄（`firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:361`）。
- **离线验证**：对主控梯形反解及电机整数跟踪分支建立无物理负载的模型，示例地轨100mm/J1 90°/J2 36°、各轴30r/s²，档位20/40/60/100的轨迹结束时间差约0.0008/0.0045/0.0052/0.0019秒，未复现明显拖尾。属于示例模型验证，不是执行目标固件或真实电机验证。
- **不能排除加速度**：梯形时间有 v/a 项，减速距离为 v²/(2a)；低速同时到达不证明高速时缓存/真实加速度一致。示例a=30时，速度6→12r/s，地轨单边减速距离3→12mm。
- **额外运动限制**：主控按零初速规划，但 INT/TRJ 可能在运动中更新目标；电机保持既有 tracker 的速度（仅模式切换等情形 NewTask），因此此类命令不满足时间公式前提。
- **下一步实机区分**：先逐节点 #GETJACC 1..7 刷新并记录参数；保持同一路径重测20/40。慢速期间观察 OLED 第7个状态字符是否已为 *，并用 #GETRAILPOS 判断实际位置是否继续变化。若已 * 且位置变化，确认提前到位判定；若仍 _，检查实际速度、缓存值与规划轨迹。再对比地轨单独运动。
- **本轮改动**：仅同步本文分析状态，固件与串口助手没有新增修改；未编译、烧录、提交。上一阶段产物和报告仍保留。

---


## 上一阶段：速度与加速度单位统一（2026-10-04）

- **当前问题**：将 firmware 的单位和速度档位行为对齐到 firmware_new。用户已授权修改、编译及中文报告。
- **已确认事实**：电机 CAN 已使用电机轴 r/s、r/s²（`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp`）；主控默认速度链路断开、全局加速度仍为百分比、模式切换会覆盖加速度（`Robot/instances/dummy_robot.cpp`）。
- **最新改动**：主控 cpp/h 连通当前档位到 MoveJ/MoveL，全局加速度直接作用于 J1～J7、更新缓存并回读；模式切换保留设置；修正七参数地轨目标。`串口助手_old.py` 统一档位标签和输入校验。
- **已排除方向**：无需修改电机物理单位换算、EEPROM 格式或整体复制参考同步规划；本轮保留原同步算法。
- **验证**：主控 Ninja 11/11 通过；35/42/57/夹爪电机四目标构建通过；串口助手语法与 44 项模拟发送行为断言通过；diff 检查通过。
- **未完成/待确认**：本轮尚未烧录，待验证不同档位、MoveL、模式切换、七参数地轨目标及加速度应用/重启行为。现有编译警告见报告；没有 commit。
- **报告**：[速度与加速度单位统一及速度档位对齐报告](速度与加速度单位统一及速度档位对齐报告.md)。主控产物 `firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`。

---


## 上轮状态：命令任务栈修复已由用户确认解决卡死问题

完整证据与回归流程见 [COMMAND_STACK_ANALYSIS.md](COMMAND_STACK_ANALYSIS.md)。

- 未修改 HEAD 的 USB 浮点命令解析调用链下界 **2048 字节**，原任务只分配 **2000 字节**。数值来自当前 ARM 工具链的 `.su` 与已链接 libc 反汇编，不含更深层调用和上下文保存。
- 基线 c69430c 的命令处理源文件在当前头文件/工具链重编，对应下界 **2040 字节**。因此基线也缺乏可靠余量，尚不能证明 `&` 是唯一触发因素。
- 已将 USB/UART 命令任务改为 **4096 字节静态 SRAM 栈**，启用栈检测，加入 USB/UART4 的 **`#STACK`** 最小余量诊断。CAN/EEPROM/USB 发送/RGB 逻辑未改，便于隔离验证。
- 主控构建 **28/28 通过**；FLASH 379360 B、SRAM 55272 B。产物在 **`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`**，仓库根 `build/` 不是当前主控构建目录。
- 用户已反馈“解决了这个 bug”；原卡死问题的修复效果已获得用户实机确认，不据此声称排除了所有其他缺陷。
- 更正旧记录：当前电机端应用参数时就更新 `boardConfig`，`&` 只控制写入 Flash；“应用后查询必定读到 EEPROM 旧值”不符合当前代码。

以下保留历史分析与用户测试，不把历史中的“根治/真正元凶/已解决”措辞视为本轮已验证结论。

---
