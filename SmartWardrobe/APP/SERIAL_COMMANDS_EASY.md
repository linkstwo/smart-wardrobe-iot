# SmartWardrobe 串口命令速查（仅本机）

## 1. 通信参数
- 串口：`USART3`
- 波特率：`115200`
- 格式：`8N1`
- 协议：ASCII 文本，命令行结尾 `\r\n`
- 命令大小写不敏感
- 当前固件默认关闭编码器限位保护（调试态）

## 2. 系统命令
- `PING`：链路测试
- `VER`：版本信息
- `HELP`：命令摘要

## 3. 电机命令（`M1~M5`）
- `M1 RUN DIR=CW RPM=120`
- `M1 STOP`
- `M1 STOP T=3000`
- `M1 ESTOP`
- `M1 SPEED RPM=150`
- `M1 ACCEL TO=220 T=1500`
- `M1 DECEL TO=80 T=1800`
- `M1 MOVE DIR=CCW RPM=120 STEP=2400`
- `M1 RECIP DIR=CW RPM=100 STEP=1000 CYCLE=20`
- `M1 RECIP_INF DIR=CW RPM=90 STEP=800`
- `M1 CFG PPR=3200 ACC=120 DEC=120 STOP=2500`
- `M1 STATUS`

Z/M4方向：`CCW`向上，`CW`向下；顶部EX_KEY4为`Z=0`，向下时Z和EC4计数增加。闭环向下测试使用`M4 ENC_MOVE DIR=CW RPM=10 CNT=100`。

## 3.1 M5 磁铁组件手动旋转命令
M5使用普通步进脉冲控制磁铁组件角度，不使用EC5编码器；CN5的EC5_A/EC5_B已复用为UART4第二测距接口。自动取放衣默认使用PD8电磁铁，M5不参与自动动作。

- `M5 MOVE DIR=CW RPM=30 STEP=10800`：顺时针旋转
- `M5 MOVE DIR=CCW RPM=30 STEP=10800`：逆时针旋转
- `M5 STOP`
- `M5 ESTOP`
- `M5 STATUS`

## 3.2 电磁铁命令
- `MAGNET ON`：PD8 输出高电平，吸合继电器。
- `MAGNET OFF`：PD8 输出低电平，释放继电器。
- `MAGNET STATUS`：查询命令状态、PD8 实际电平和 `FEEDBACK=NONE`。

自动流程故障时如果电磁铁已经吸合，会保持 `MAGNET=ON HOLD_FOR_MANUAL`；执行 `MAGNET OFF` 后才能再次执行 `ALL HOME`、`AXIS MOVE/ENC_MOVE` 或新的取放衣任务。普通 STOP 不改变电磁铁状态，ESTOP、HardFault 和复位会强制释放。

## 4. 全电机命令（`ALL`）
- `ALL STOP`
- `ALL STOP T=3000`
- `ALL ESTOP`
- `ALL STATUS`
- `ALL HOME RPM=120`

`ALL HOME`只回M1、M3、M4：M1/M3以CCW回零；M4若EX_KEY4为高则直接CCW向上搜索，若初始为低则先CW向下释放，连续高20ms后再CCW重寻。三轴到位后清零EC1、EC3、EC4和XYZ，不清M2绝对值编码器。M4释放超时1秒、搜索超时60秒，失败时不清零也不继续M2上电回零。

## 5. 编码器命令
- `M1 ENC_STATUS`
- `M1 ENC_CLR`
- `ALL ENC_STATUS`
- `ALL ENC_CLR`

## 6. 速度曲线命令
- `M1 CURVE_ON RATE=20`
- `M1 CURVE_OFF`
- `M1 CURVE_STATUS`
- `ALL CURVE_ON RATE=20`
- `ALL CURVE_OFF`
- `ALL CURVE_STATUS`

## 7. M2 绝对值编码器命令
- `M2 ABS_STATUS`
- `M2 ABS_ZERO`
- `M2 ABS_MOVE DEG=90 RPM=20`
- `M2 ABS_MOVE DEG=90 RPM=20 DIR=CW TOL=2`

`M2 ABS_*` 使用 M2 的 RS485 绝对值编码器；`ABS_ZERO` 会写入编码器内部零点，不会被 `ALL HOME` 或 `AXIS ZERO` 清掉。

## 8. M2 旋转轴步数命令
- `M2 ROT_STATUS`
- `M2 ROT_ZERO`
- `M2 ROT_SET STEP=12345`
- `M2 ROT_SAVE`
- `M2 ROT_GET`
- `M2 ROT_LIMIT_SET STEP=50000`
- `M2 ROT_LIMIT_GET`

`M2 ROT_*` 是 M2 旋转轴的开环步数基线和旋转限位；它不属于 XYZ 三轴坐标。

## 9. 三轴坐标与防撞命令
三轴映射：`X=M1`、`Y=M3`、`Z=M4`
M2 不参与 `AXIS MOVE` / `AXIS ZERO` / `AXIS LIMIT_SET` 的 XYZ 三轴坐标。

- `AXIS STATUS`
- `AXIS LIMIT_GET`
- `AXIS LIMIT_SET X=120000 Y=80000 Z=95000`
- `AXIS ZERO`
- `AXIS CFG RPM=120 ERR=300 TOL=20`
- `AXIS MOVE X=12000 Y=8000 Z=6000 RPM=120`
- `AXIS ENC_MOVE X=720 Y=720 Z=720 RPM=30 TOL=36`
- `AXIS PLACE_CLOTH RPM=10 TOL=6 SLOT=1`
- `AXIS TAKE_CLOTH RPM=10 TOL=6 SLOT=1`
- `AXIS STOP`
- `AXIS ESTOP`

`AXIS ENC_MOVE` 使用绝对编码器计数闭环到位：`X=EC1`、`Y=EC3`、`Z=EC4`。按现场实测，`720 count` 约等于 `10cm`，`36 count` 约等于 `5mm`。
Z轴当前以上端开关为零点，正Z表示CW向下，EC4计数增加；CCW向上时EC4计数减少。非回零CCW向上触发EX_KEY4会急停并报`M4_Z_UPPER_LIMIT`。
编码器闭环调速当前为 `Kp=0.04, Ki=0.12, Kd=0.0002, I_LIMIT=120, RPM_MIN=20`。若编码器连续 `250ms` 无计数变化，固件会先进入脱困提速，保持 `BOOST_RPM=45`；若到 `1200ms` 仍无计数变化才报 `ERR 206 Mx_ENC_STALL`。失速日志会带 `LAST/RPM/STILL/BOOST/MODE/DIR/STEP`，用于判断最近计数、闭环输出速度、无计数变化时长、是否已进入脱困提速、电机模式、方向和累计 STEP 脉冲数。M1/M3/M4 的方向保护允许 `20 count` 内的小反向抖动，超过后连续 `3` 次或累计 `60 count` 才报 `ENC_WRONG_DIR`；单次 `>=80 count` 仍立即急停。

取衣和放衣的`SLOT=N`都表示识别到的第N块完整定位板。两种流程都先到`EC1=8860`再直接扫描，目标板完整退出后不回中心，而是从停止点执行各自的固定相对位移。

`AXIS PLACE_CLOTH SLOT=N`执行：`ALL HOME`、M2 CW650、`EC4=10000`，随后用EC5第二测距的3帧稳定中值和两点标定公式计算动态EC1。X到动态目标后执行`EC3=1381`、`EC4=12552`、电磁铁ON等待2秒、M4 CCW上抬650步、`EC1=1000`、Y归零、`EC4=3263`、M2 CW4000和`EC1=8860`。扫描到目标板完整退出后执行X左1125、Z下1500、Y前1250、M2 CCW4000、电磁铁OFF等待2秒、Z上150、Y归零、M2 CW1000、`ALL HOME`，最后M2以25RPM回`RAW=1008`。

`AXIS TAKE_CLOTH SLOT=N`执行：`ALL HOME`、M2 CW650、`EC4=3263`、M2 CW1500和`EC1=8860`并扫描。目标板完整退出后执行X左1760、Z下1200、Y前1350、M2 CCW1500、Z下670、电磁铁ON等待2秒、Z上520、Y前150、M2 CW1500、Y归零、M2 CCW5500、`EC1=1000`、M2 CW4000、`EC4=12552`、`EC3=1470`、`EC1=3200`、电磁铁OFF等待2秒及`EC4=10000`，最后执行`ALL HOME`并让M2以25RPM回`RAW=1008`。

取衣和放衣共用第一测距定位规则：到`EC1=8860`后连续取得2帧新合法响应才启动M1；两种流程均以75RPM搜索，连续3帧满足200~250mm、`CONF>=70`、峰峰值不超过8mm后降到50RPM，完整板宽1000~2200计数才计为有效衣位。

- `LASER STATUS`：短输出，例如`OK LASER DIST=225 CONF=100 COMM=1 VALID=1 PLATE=1 STATE=IDLE`，用于日常对光和板面调试。`PLATE=1`要求当前距离在200~250mm且置信度不低于70，但不代表完整板面进入、退出和宽度校验完成。
- `LASER DIAG`：输出完整请求、响应、超时和错误计数，仅在通信排障时使用。
- `LASER2 STATUS`：查询CN5/UART4第二测距模块，例如`OK LASER2 DIST=300 CONF=100 COMM=1 VALID=1 ONLINE=1 STATE=IDLE`。
- `LASER2 DIAG`：输出第二测距模块的完整通信计数；该模块只参与放衣前动态EC1换算。
- `LASER2 STREAM ON`：先发送`DEBUG ON`后使用，每200ms输出`LASER2 DATA DIST=... CONF=...`，用于边移动X轴边记录距离。
- `LASER2 STREAM OFF`：停止连续数据；`DEBUG OFF`也会停止文本输出并恢复ESP32二进制模式。

第二测距标定参数在`APP/Src/cmd_dispatch.c`的`PLACE_LASER2_CAL_*`宏中。当前标定有效开关为0，执行放衣会在动态X步骤安全停止；实测两组`距离/EC1`后填写参数并把开关改为1。

## 10. 典型返回
- 成功：`OK ...`
- 失败：`ERR <code> <msg>`
- 状态：`STAT ...` / `ENC ...` / `CURVE ...` / `AXIS ...`

## 11. 你当前调试建议（电脑端只能发、不能收）
- 看 `RX` 指示灯（收到命令会翻转）
- 看 `TX` 指示灯（发回包会翻转）
- 先发 `AXIS LIMIT_SET X=... Y=... Z=...`，再发 `AXIS MOVE ...`，否则三轴运动命令会被限位未就绪拦截
- 测试旋转轴时用 `M2 ROT_STATUS`，不要再用 `M3 ROT_STATUS` 或 `M5 ROT_STATUS`。

