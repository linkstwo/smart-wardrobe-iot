# SmartWardrobe 串口命令协议（中文）

## 通信参数
- 串口：`USART3`
- 波特率：`115200`，`8N1`
- 协议：ASCII 文本，行尾 `\r\n`
- 命令大小写不敏感（如 `m1 run` 等价 `M1 RUN`）

## 语法
`<TARGET> <ACTION> [K=V ...]`

- `TARGET`：`M1`~`M5`、`ALL`，或系统命令（`HELP`/`VER`/`PING`）。其中 `M5` 是磁铁组件上的手动旋转步进电机；自动取放衣默认使用 `PD8/ELM_CT` 电磁铁。
- `ACTION`：动作关键字
- `K=V`：参数键值对，使用空格分隔

## 默认参数
- `PPR=3200`
- `ACC=120 RPM/s`
- `DEC=120 RPM/s`
- `STOP=2500 ms`
- 限位保护为手动开关：首次上电默认 `PROTECT=OFF`，之后按 Flash 持久化状态恢复
- 曲线采样默认 `RATE=20 Hz`
- 步进频率限幅：`20~8000 Hz`
- RPM 换算：`Hz = RPM * PPR / 60`

## 电机控制命令
| 命令 | 说明 |
|---|---|
| `M1 RUN DIR=CW RPM=120` | 单电机连续运行 |
| `M1 STOP` | 按默认时长正弦缓停 |
| `M1 STOP T=3000` | 3000ms 正弦缓停 |
| `M1 ESTOP` | 立即急停 |
| `M1 SPEED RPM=150` | 设置目标巡航速度 |
| `M1 ACCEL TO=220 T=1500` | 1500ms 内加速到 220RPM |
| `M1 DECEL TO=80 T=1800` | 1800ms 内减速到 80RPM |
| `M1 MOVE DIR=CCW RPM=120 STEP=2400` | 定向走固定步数后缓停 |
| `M2 ABS_STATUS` | 通过 UART2/RS485 读取 M2 绝对值编码器角度 |
| `M2 ABS_MOVE DEG=90 RPM=20 [DIR=CW|CCW] [TOL=2]` | M2 按 RS485 绝对值编码器转到指定角度；不写 DIR 时自动选择较短方向 |
| `M1 ENC_MOVE DIR=CW RPM=20 CNT=1440 [TOL=5]` | M1 使用 EC1 编码器闭环相对运动，运行中按剩余误差 PID 调整 RPM，到达指定编码器计数后自动急停 |
| `M3 ENC_MOVE DIR=CW RPM=20 CNT=1440 [TOL=5]` | M3 使用 EC3 编码器闭环相对运动，运行中按剩余误差 PID 调整 RPM，到达指定编码器计数后自动急停 |
| `M4 ENC_MOVE DIR=CW RPM=20 CNT=1440 [TOL=5]` | M4 向下运动，EC4 计数增加；运行中按剩余误差 PID 调整 RPM，到达指定编码器计数后自动急停 |
| `M1 RECIP DIR=CW RPM=100 STEP=1000 CYCLE=20` | 有限往返（`CYCLE=1` 表示去+回一次） |
| `M1 RECIP_INF DIR=CW RPM=90 STEP=800` | 无限往返，直到 `STOP/ESTOP` |
| `M5 MOVE DIR=CW RPM=30 STEP=10800` | 磁铁组件顺时针旋转 |
| `M5 MOVE DIR=CCW RPM=30 STEP=10800` | 磁铁组件逆时针旋转 |
| `M1 HOME_EX1 RPM=120` | M1 以 `CCW` 方向回到 EX_KEY1 行程开关位置，EX_KEY1 低电平后急停 M1 |
| `M2 HOME_EX2 RPM=120` | 当前硬件 M2 没有行程开关，固件已禁用该命令；M2 只使用 `ABS_STATUS` / `ABS_ZERO` / `ABS_MOVE` 进行角度控制 |
| `M3 HOME_EX3 RPM=120` | M3 以 `CCW` 方向回到 EX_KEY3 行程开关位置，EX_KEY3 低电平后急停 M3 |
| `M4 HOME_EX4 RPM=120` | M4 以 `CCW` 向上回到 EX_KEY4；若启动时 EX_KEY4 已为低，先以 `CW` 向下释放，再反向重寻 |
| `ALL HOME RPM=120` | M1/M3以CCW回零；M4按“CW释放、CCW重寻”两段式回零；三轴均到位后清零EC1/EC3/EC4与XYZ；不包含M2-EX2 |
| `M1 CFG PPR=3200 ACC=120 DEC=120 STOP=2500` | 修改电机参数 |
| `M1 STATUS` | 查询单电机状态 |
| `ALL STOP [T=ms]` | 全部电机缓停 |
| `ALL ESTOP` | 全部电机急停，并强制释放电磁铁 |
| `ALL STATUS` | 全部电机状态 |

兼容别名：`Mx REV_EXn` 等价 `Mx HOME_EXn`，`ALL REV_EXKEY` / `ALL HOME_EXKEY` 等价 `ALL HOME`。

M4 回零要求 EX_KEY4 未触发为高、触发为低。释放阶段连续高 20ms 后切换到 CCW，1 秒仍未释放会报 `M4_HOME_RELEASE_TIMEOUT`；搜索阶段连续低 20ms 才完成，60 秒未触发会报 `M4_HOME_SEEK_TIMEOUT`。失败时不会清零 EC4/Z，也不会继续 M2 上电回零。

## 编码器命令
| 命令 | 说明 |
|---|---|
| `EC1 STATUS` ~ `EC4 STATUS` | 查询对应电机编码器状态：EC1=M1、EC2=M2、EC3=M3、EC4=M4 |
| `EC1 CLR` ~ `EC4 CLR` | 清零对应编码器累计计数与原始 TIM 计数 |
| `M1 ENC_STATUS` | 查询 M1 编码器：累计计数、方向、实测RPM |
| `M1 ENC_CLR` | 清零 M1 编码器累计计数 |
| `ALL ENC_STATUS` | 查询全部编码器状态 |
| `ALL ENC_CLR` | 清零全部编码器累计计数 |

EC 与电机映射：
- EC1 对应 M1，EC2 对应 M2，EC3 对应 M3，EC4 对应 M4。
- EC5 编码器功能已停用；CN5 的 EC5_A/EC5_B 已复用为 UART4 的第二测距模块 TX/RX。
- M1/M3/M4 编码器闭环分别使用 EC1/EC3/EC4，用于 PID 调速、到位判断、方向校验和卡滞检测。
- EC3_A：`PA6`，对应 `TIM3_CH1`
- EC3_B：`PA7`，对应 `TIM3_CH2`
- EC4_A：`PD12`，对应 `TIM4_CH1`
- EC4_B：`PD13`，对应 `TIM4_CH2`
- 固件已将 PA6/PA7 配置为 `TIM3` 编码器模式、PD12/PD13 配置为 `TIM4` 编码器模式，并启用内部上拉，适配 NPN 开路集电极编码器。
- E38S6G5-360B-G24N 为 360PPR，TIM 使用 AB 双相四倍频计数，`ECx STATUS` 按 `CPR=1440` 换算角度，1 个计数约 0.25 度。
- `M1/M3/M4 ENC_MOVE` 的 `CNT` 是对应 EC1/EC3/EC4 编码器计数，不是步进电机脉冲；默认到位容差为4个计数。M1/M3的CW使计数增加，M4的CW向下使EC4增加；运动中`RPM`作为PID调速上限。
- 如果 A/B 电平始终不变化，优先检查红线供电、黑线 GND、对应 ECx_A/ECx_B 接线，以及 A/B 是否有上拉。

## M5 磁铁组件旋转步进电机命令
M5 使用普通步进脉冲/方向/使能控制磁铁组件角度，不使用EC5编码器；UART4当前由CN5第二测距模块使用。

硬件参数：
- STEP：`PB14/TIM12_CH1`
- DIR：`PD0`
- EN：`PB15`
- EN 为低电平有效，默认禁用。

| 命令 | 说明 |
|---|---|
| `M5 MOVE DIR=CW RPM=30 STEP=10800` | 磁铁组件顺时针旋转 |
| `M5 MOVE DIR=CCW RPM=30 STEP=10800` | 磁铁组件逆时针旋转 |
| `M5 STOP` | 停止M5旋转 |
| `M5 ESTOP` | 立即急停M5 |
| `M5 STATUS` | 查询M5电机状态 |

M5 命令仅用于手动调整磁铁组件角度。自动放衣/取衣默认不驱动 M5，而使用 `PD8/ELM_CT`：高电平吸合、低电平释放，吸合/释放后各等待 2 秒。若自动流程因激光或轴故障停止，已吸合的电磁铁会保持 `ON`，串口会输出 `MAGNET=ON HOLD_FOR_MANUAL`，必须手动执行 `MAGNET OFF`。

## 电磁铁命令
| 命令 | 说明 |
|---|---|
| `MAGNET ON` | PD8 输出高电平，吸合继电器；无触点/电流反馈 |
| `MAGNET OFF` | PD8 输出低电平，释放继电器 |
| `MAGNET STATUS` | 查询命令状态和 PD8 实际电平 |

电磁铁为高电平有效，状态中的 `FEEDBACK=NONE` 只表示没有继电器触点或线圈电流反馈。电磁铁吸合时，`ALL HOME`、`AXIS MOVE`、`AXIS ENC_MOVE` 和新的取放衣任务会被拒绝，先执行 `MAGNET OFF`。`ALL ESTOP`、`AXIS ESTOP`、HardFault 和复位会强制释放；普通 `ALL STOP/AXIS STOP` 保留当前吸合状态。

## M2 RS485 绝对值编码器命令
M2 当前硬件没有 EX_KEY2 行程开关，不使用 `M2 HOME_EX2`；M2 回到指定位置请使用 `M2 ABS_MOVE DEG=目标角度 RPM=速度 [TOL=容差]`。
M2 上电自动回零使用用户实测固定绝对值位置：`RAW=1008, DEG=354.38, CPR=1024`。上电后固件会自动尝试回到 `RAW=1008`，不再依赖 `M2 ABS_ZERO` 保存的 `DEG=0`。

M2 额外接入 BRT38-R0M1024-RT1 绝对值编码器，通过 UART2 连接 RS485-TTL 模块读取。

硬件参数：
- UART2：`PA2=TXD`，`PA3=RXD`
- 串口参数：`9600, 8N1`
- 编码器默认 Modbus 地址：`1`
- 单圈分辨率：`CPR=1024`，1 个原始计数约 `0.3516°`
- 485 接线：编码器白线接模块 `A`，绿线接模块 `B`，红线接编码器供电正，黑线接 GND；GND 必须与主控共地

| 命令 | 说明 |
|---|---|
| `M2 ABS_STATUS` | 读取 M2 绝对值编码器，返回 `RAW` 和 `DEG` |
| `M2 ABS_ZERO` | 将 M2 当前机械位置设为编码器零点；当前上电自动归零已改为固定回 `RAW=1008`，不依赖该命令 |
| `M2 ABS_MOVE DEG=90 RPM=20` | M2 自动选择较短方向转到 90 度 |
| `M2 ABS_MOVE DEG=90 RPM=20 DIR=CW` | 强制 M2 正转到 90 度 |
| `M2 ABS_MOVE DEG=90 RPM=20 DIR=CCW TOL=3` | 强制 M2 反转到 90 度，到位容差 3 度 |

`ABS_MOVE` 带方向校验和卡滞检查：
- 连续 3 次通信失败会急停 M2，并返回 `ERR 208 M2_ABS_COMM_*`
- 运动中编码器方向与电机命令方向相反会急停 M2，并返回 `ERR 208 M2_ABS_WRONG_DIR`
- 运动中编码器长时间无变化会急停 M2，并返回 `ERR 208 M2_ABS_STALL`
- 当前 M2 实测方向映射为：`DIR=CW` 时编码器 `RAW` 减少，`DIR=CCW` 时编码器 `RAW` 增加。
- 上电后固件会延时约 2 秒自动执行 M2 归零：读取绝对值编码器并以 `RPM=25` 回到用户实测的固定位置 `RAW=1008`（约 `DEG=354.38`）；若编码器通信失败或 M2 忙，则不会强行动作。

指定 M2 零点的方法：
1. 手动或用低速命令把 M2 移动到你希望作为零点的位置。
2. 发送 `M2 ABS_ZERO`，编码器会把当前位置写成 `DEG=0`。
3. 发送 `M2 ABS_STATUS`，确认返回角度接近 `DEG=0`。
4. 以后上电自动归零会回到这个位置。

## M2 旋转角命令（开环脉冲换算角度）
角度定义：基于 M2 开环脉冲累计换算，`CW` 角度增加，`CCW` 角度减少；M2 还支持 RS485 绝对值编码器命令。

| 命令 | 说明 |
|---|---|
| `M2 ROT_STATUS` | 查询 M2 当前开环脉冲步数（STEP） |
| `M2 ROT_ZERO` | 将当前 M2 旋转轴步数置零（仅运行时基线） |
| `M2 ROT_SET STEP=12345` | 直接设置 M2 旋转轴步数并立即保存到 Flash |
| `M2 ROT_SAVE` | 将当前 M2 旋转轴步数保存到 Flash |
| `M2 ROT_GET` | 读取 M2 当前开环脉冲步数（STEP） |
| `M2 ROT_LIMIT_SET STEP=50000` | 设置并保存 M2 旋转步数上限（0~MAX） |
| `M2 ROT_LIMIT_GET` | 查询 M2 旋转限位状态（VALID/MAX_STEP/FAULT） |

## 速度曲线命令
| 命令 | 说明 |
|---|---|
| `M1 CURVE_ON RATE=20` | 开启 M1 实时速度曲线输出 |
| `M1 CURVE_OFF` | 关闭 M1 曲线输出 |
| `M1 CURVE_STATUS` | 查询 M1 曲线输出状态 |
| `ALL CURVE_ON RATE=20` | 开启全部电机曲线输出 |
| `ALL CURVE_OFF` | 关闭全部电机曲线输出 |
| `ALL CURVE_STATUS` | 查询全部曲线输出状态 |

## 三轴坐标与限位命令
三轴映射：`X=M1`、`Y=M3`、`Z=M4`（三轴命令只作用于这三轴）。
坐标方向约定：X/M1、Y/M3以`CW`为正；Z/M4以上端EX_KEY4为`Z=0`，`CW`向下为正、`CCW`向上为负方向。
限位保护开启后，同时对 `X/Y/Z` 和 `M2 ROT` 生效；越界仅急停对应电机，不联停其他电机。

| 命令 | 说明 |
|---|---|
| `AXIS MOVE X=12000 Y=8000 Z=6000 [RPM=120]` | 三轴联动到目标坐标（电机步数坐标，保留旧开环规划） |
| `AXIS ENC_MOVE X=720 Y=720 Z=720 RPM=30 [TOL=36]` | 三轴按 EC1/EC3/EC4 绝对编码器计数闭环移动，运行中按剩余误差 PID 调整各轴 RPM；按现场实测 720 count 约 10cm，默认容差 36 count（约 5mm） |
| `AXIS PLACE_CLOTH [RPM=150] [TOL=6] [SLOT=1]` | 按新机械顺序放衣；EC5第二测距先换算动态EC1取物位置，扫描退出后执行固定相对动作，最终`ALL HOME`并让M2以25RPM回`RAW=1008` |
| `AXIS TAKE_CLOTH [RPM=150] [TOL=6] [SLOT=1]` | 按新机械顺序取第`SLOT`块定位板上的衣服；释放前X闭环到`EC1=3200`，最终`ALL HOME`并让M2以25RPM回`RAW=1008` |
| `AXIS STOP` | 规划器软停当前三轴动作 |
| `AXIS ESTOP` | 规划器急停当前三轴动作 |
| `AXIS STATUS` | 查询三轴实时坐标、`PROTECT` 开关、限位上限、是否就绪、故障状态 |
| `AXIS LIMIT_SET X=120000 Y=80000 Z=95000` | 设置并保存三轴上限到 Flash |
| `AXIS LIMIT_GET` | 读取当前生效三轴上限与有效标志 |
| `AXIS PROTECT ON` | 开启三轴限位保护并保存到 Flash |
| `AXIS PROTECT OFF` | 关闭三轴限位保护并保存到 Flash |
| `AXIS PROTECT STATUS` | 查询限位保护开关、有效性、上限、故障状态 |

取衣和放衣共用第一测距定位板扫描器，且不复用任务开始前的在线缓存：两种流程均先闭环到`EC1=8860`，随后必须连续取得2帧新的合法Modbus响应；3秒未完成握手时不启动M1。扫描直接以75RPM开始，连续3帧满足距离200~250mm、`CONF>=70`和峰峰值不超过8mm时记录板面进入并降至50RPM；完整退出且宽度在1000~2200计数内才计为一个衣位。扫描期间连续500ms没有新增合法响应会报告离线，目标板完整退出后不回板中心。

- `LASER STATUS`：日常短状态，返回 `DIST/CONF/COMM/VALID/PLATE/STATE`。`PLATE=1`表示当前原始样本同时满足通信在线、距离有效、200~250mm和`CONF>=70`，不表示完整板面的进入、退出和宽度校验已经完成。
- `LASER DIAG`：完整通信诊断，返回请求、响应、超时、CRC、协议、拒绝、UART错误和最新原始值等累计字段。
- `LASER2 STATUS`：查询CN5/UART4第二测距模块的距离、置信度、通信状态和数据有效性。
- `LASER2 DIAG`：查询第二测距模块的完整Modbus请求、响应、超时及错误计数；该模块只参与放衣前动态EC1换算。
- `LASER2 STREAM ON`：在`DEBUG ON`文本调试模式下每200ms连续输出一次第二测距距离；二进制ESP32模式下不会输出。
- `LASER2 STREAM OFF`：停止第二测距连续输出。
- `LASER2 STREAM STATUS`：查询连续输出开关和当前文本调试状态。
| `AXIS ZERO` | 将当前编码器位置记为三轴零点（X=0,Y=0,Z=0） |
| `AXIS CFG RPM=120 ERR=300 TOL=20` | 设置并保存规划器默认速度/步编差阈值/到位容差 |

`SLOT=N`表示第N块完整定位板。取衣和放衣都在目标板完整退出后的停止点执行各自的固定相对位移，不再使用动态板中心作为X目标。自动保持器使用电磁铁，M5仅保留手动旋转调试。
Z轴以顶部EX_KEY4为零点，CW向下时EC4增加、CCW向上时EC4减少。非回零过程中M4以CCW向上触发EX_KEY4会立即急停并报告`ERR 212 M4_Z_UPPER_LIMIT`，不会自动清零。
PID 调速说明：
- 当前 PID 只作用于 M1/M3/M4 的编码器目标运动链路，包括 `M1/M3/M4 ENC_MOVE`、`AXIS ENC_MOVE`、`AXIS PLACE_CLOTH` 和 `AXIS TAKE_CLOTH` 中的 X/Y/Z 单轴移动。
- `RPM` 是最高速度上限；距离目标远时速度会被该上限夹住，接近目标时按剩余编码器误差自动降速。
- 默认参数为现场调试值：`Kp=0.04, Ki=0.12, Kd=0.0002, I_LIMIT=120, RPM_MIN=20`。若编码器连续 `250ms` 无计数变化，固件会先进入脱困提速，保持 `BOOST_RPM=45`；若到 `1200ms` 仍无计数变化才报 `ERR 206 Mx_ENC_STALL`。失速日志会输出 `LAST/RPM/STILL/BOOST/MODE/DIR/STEP`，用于判断最近计数、闭环输出速度、无计数变化时长、是否已进入脱困提速、电机模式、方向和累计 STEP 脉冲数。
- M1/M3/M4 的 `ENC_WRONG_DIR` 方向保护带小幅反向容忍：单次反向 `<=20 count` 不急停；单次反向 `>=80 count` 立即急停；反向超过 `20 count` 后连续 `3` 次或累计 `60 count` 才报错。方向错误日志会输出 `LAST/DELTA/WCNT/WACC`。
- 旧 `AXIS MOVE` 仍是电机步数坐标开环规划，M2 绝对值编码器角度运动也暂未并入本 PID 链路。

`AXIS PLACE_CLOTH SLOT=N` 顺序：
1. 先触发 `ALL HOME`，此步骤允许 M1/M3/M4 同时回零并清零 EC1/EC3/EC4 与 XYZ。
2. M2以10RPM顺时针650步，随后闭环到`EC4=10000`。
3. 第二测距取得3帧稳定新样本，通过两点标定公式计算动态EC1，X闭环到该目标；随后到`EC3=1381`和`EC4=12552`。
4. 执行`MAGNET ON`并等待2秒，M4以命令RPM逆时针上抬650电机步。
5. X到`EC1=1000`，Y回`EX_KEY3`，Z到`EC4=3263`，M2以10RPM顺时针4000步。
6. X闭环到`EC1=8860`，等待2帧新激光响应后以75RPM扫描，进入板面后降到50RPM。
7. 目标板完整退出后急停X，从停止点依次执行X左移1125、Z下降1500和Y前移1250个编码器计数。
8. M2以10RPM逆时针4000步，执行`MAGNET OFF`并等待2秒。
9. Z向上相对移动150个EC4计数，Y回`EX_KEY3`，随后M2以10RPM顺时针1000步。
10. 执行`ALL HOME`，最后让M2以25RPM回`RAW=1008`并确认到位。

`AXIS TAKE_CLOTH SLOT=N` 顺序：
1. 先触发 `ALL HOME`，此步骤允许 M1/M3/M4 同时回零并清零 EC1/EC3/EC4 与 XYZ。
2. M2以10RPM顺时针650步，Z到`EC4=3263`，M2再顺时针1500步，X到`EC1=8860`。
3. 清空旧样本并等待2帧新响应，以75RPM扫描；确认板面后降至50RPM，第N块有效板完整退出后急停X。
4. 从停止点依次相对执行X左1760、Z下1200、Y前1350，随后M2以10RPM逆时针1500步，Z再下降670计数。
5. 执行`MAGNET ON`并等待2秒，再相对执行Z上520、Y前150，M2以10RPM顺时针1500步。
6. Y通过EX_KEY3归零后，M2以10RPM逆时针5500步，X闭环到`EC1=1000`，M2再顺时针4000步。
7. Z闭环到`EC4=12552`，Y闭环到`EC3=1470`，X闭环到`EC1=3200`。
8. 执行`MAGNET OFF`并等待2秒，Z闭环到`EC4=10000`。
9. 执行`ALL HOME`，最后让M2以25RPM回`RAW=1008`并确认到位。

第二测距动态EC1标定参数位于`cmd_dispatch.c`的`PLACE_LASER2_CAL_*`宏。当前`PLACE_LASER2_CALIBRATION_VALID=0`，未标定时放衣会在`EC4=10000`后的动态X步骤报告`PLACE_CLOTH_LASER2_CAL_REQUIRED`并停止，不会吸合电磁铁。

流程运行中会拒绝新的单电机/三轴运动命令；可用 `AXIS STOP`、`AXIS ESTOP`、`ALL STOP` 或 `ALL ESTOP` 中断。

## 系统命令
| 命令 | 说明 |
|---|---|
| `HELP` | 查看命令摘要 |
| `VER` | 查看固件版本 |
| `PING` | 链路测试 |
| `MAGNET ON/OFF/STATUS` | 控制或查询 PD8 电磁铁继电器 |

## 曲线输出格式
实时输出示例：

`CURVE M=1 TS=123456 TARGET=120.00 CTRL=118.40 ENC=117.95 MODE=RUN`

字段说明：
- `TARGET`：目标 RPM
- `CTRL`：控制器当前 RPM（内部轨迹）
- `ENC`：编码器实测 RPM
- `TS`：系统毫秒时间戳

## 默认心跳输出
- 固件默认每 `1000ms` 输出一次心跳，用于确认程序仍在运行。
- 心跳示例：

`HB TS=123456 SEQ=42`

## 参数范围
- `RPM` / `TO`：需映射到有效步进频率 `20~8000Hz`
- `T`：`100~30000 ms`
- `STEP`：`>0`
- `CYCLE`：`>0`
- `PPR`：`100~50000`
- `ACC` / `DEC`：`1~5000 RPM/s`
- `STOP`：`100~30000 ms`
- `RATE`：`1~50 Hz`
- `ERR`（编码器-命令步数差阈值）：`10~20000`
- `TOL`（到位容差）：`1~1000`

## 返回格式
- 成功：`OK ...`
- `HOME_EX*` 到达行程开关：`OK Mx HOME_EXn_DONE EX_KEYn_LOW`
- 错误：`ERR <code> <msg>`
- 状态：`STAT ...` / `ENC ...` / `CURVE_STAT ...` / `AXIS ...`
- EC3 测试状态：`EC3 CNT=... RAW=... A=... B=... DIR=... DEG=... CPR=1440`

## 主要错误码
- `ERR 101~104`：解析错误（空命令/目标错误/动作错误/参数错误）
- `ERR 200`：目标不支持该动作
- `ERR 201`：缺少必填参数
- `ERR 202`：参数非法或多余参数
- `ERR 203`：电机控制失败
- `ERR 204`：状态读取失败
- `ERR 205`：响应过长
- `ERR 206`：编码器操作失败
- `ERR 207`：曲线输出操作失败
- `ERR 211`：三轴限位参数未就绪（未加载到有效 MAX）
- `ERR 212`：三轴限位越界（触发该轴急停）
- `ERR 213`：三轴限位保存失败（Flash 写入失败）
- `ERR 214`：三轴限位读取失败（Flash 参数无效）
