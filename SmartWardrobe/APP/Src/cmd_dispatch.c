#include "cmd_dispatch.h"

/*
 * 中文总说明：
 * 本文件是 STM32 端命令分发和衣柜业务流程的核心文件。
 * 主要负责解析后的命令执行、M1/M3/M4 编码器目标运动、M2 绝对值编码器运动、
 * ALL HOME 回零、AXIS PLACE_CLOTH 放衣服、AXIS TAKE_CLOTH 取衣服，以及把执行结果返回给串口/ESP32。
 * PLACE_CLOTH/TAKE_CLOTH 共用激光板面扫描器，SLOT 表示越过门槛后的第 N 块有效定位板。
 */

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "axis_coord.h"
#include "axis_limit.h"
#include "axis_planner.h"
#include "electromagnet.h"
#include "encoder_ctrl.h"
#include "encoder_hw.h"
#include "laser_range.h"
#include "laser_range2.h"
#include "main.h"
#include "motor_ctrl.h"
#include "rs485_abs_encoder.h"
#include "telemetry_curve.h"
#include "uart_console.h"

/*
 * 命令分发执行层：
 * 1. 接收 cmd_parser.c 解析出来的 ParsedCmd_t；
 * 2. 根据目标类型分发到电机、全部电机、三轴、编码器、系统命令；
 * 3. 管理取衣/放衣流程状态机；
 * 4. 把执行结果格式化为串口文本响应，并通过事件接口通知 main.c 生成云端二进制响应帧。
 */
#define CMD_VERSION_TEXT "SmartWardrobe-STM32F407 v1.5.13-place-sequence-v2"
#define ENCODER_PPR 360.0f
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR * 4.0f)
#define M4_ENCODER_ID MOTOR_ID_4
#define ENC_MOVE_STALL_TIMEOUT_MS 1200U
#define ENC_MOVE_STALL_BOOST_START_MS 250U
#define ENC_MOVE_DEFAULT_TOL 4U
#define AXIS_ENC_MOVE_DEFAULT_TOL 36U
/*
 * M1/M3/M4 编码器目标运动的 PID 调速参数。
 * 当前只在 ENC_MOVE / AXIS ENC_MOVE / 取放衣状态机使用的编码器目标运动链路中启用，
 * 不改动旧的 AXIS MOVE 开环三轴规划，也不影响 M2 绝对值编码器角度运动。
 *
 * 说明：
 * - Kp 根据剩余编码器计数生成目标 RPM，距离远时会被用户传入的 RPM 上限夹住；
 * - Ki 默认 0，避免衣柜机械在未充分调参前出现积分饱和和过冲；
 * - Kd 默认 0，先采用稳定保守的一阶位置调速，后续实机确认后再打开微分项。
 */
#define ENC_MOVE_PID_ENABLE 1U
#define ENC_MOVE_PID_SAMPLE_MS 20U
#define ENC_MOVE_PID_KP_DEFAULT 0.04f
#define ENC_MOVE_PID_KI_DEFAULT 0.20f
#define ENC_MOVE_PID_KD_DEFAULT 0.0002f
#define ENC_MOVE_PID_INTEGRAL_LIMIT 120.0f
#define ENC_MOVE_PID_RPM_MIN_DEFAULT 20.0f
#define ENC_MOVE_PID_STALL_BOOST_RPM 45.0f
#define ENC_MOVE_PID_RPM_CHANGE_EPS 0.25f
#define Y_ENC_MOVE_DECEL_COUNTS 1200.0f
#define ENC_MOVE_WRONG_DIR_TOL 20L
#define ENC_MOVE_WRONG_DIR_COUNT_LIMIT 3U
#define ENC_MOVE_WRONG_DIR_ACCUM_LIMIT 60UL
#define ENC_MOVE_WRONG_DIR_IMMEDIATE_LIMIT 80L
#define M2_ABS_POLL_PERIOD_MS 60U
#define M2_ABS_STALL_TIMEOUT_MS 900U
#define M2_ABS_MAX_COMM_FAIL 3U
#define M2_ABS_DEFAULT_TOL_DEG 2.0f
#define M2_ABS_CW_RAW_INCREASES 1U
/*
 * M2 绝对值编码器上电回零会在 1ms 调度路径中执行 RS485 读取。
 * 暂时关闭该路径，用于隔离启动阶段的阻塞通信与 USART3 状态帧突发问题。
 * M1/M3/M4 的 ALL HOME 不受此开关影响。
 */
#define M2_ABS_BOOT_HOME_ENABLE 1U               /* 1=启用M2绝对编码器回基准，0=关闭。 */
#define M2_ABS_BOOT_HOME_RETRY_PERIOD_MS 500U    /* M2绝对编码器通信失败后的重试周期，单位ms。 */
#define M2_ABS_BOOT_HOME_TIMEOUT_MS 15000U       /* M2回绝对基准的最长允许时间，单位ms。 */
#define M2_ABS_BOOT_HOME_RPM 25.0f               /* M2回RAW基准时的速度，单位RPM；不是流程内10RPM。 */
/* M2 上电固定回到用户实测的绝对值编码器零点：RAW=1008, DEG=354.38, CPR=1024。 */
#define M2_ABS_BOOT_HOME_USE_FIXED_RAW 1U        /* 1=按RAW目标回位；0=按角度目标回位。 */
#define M2_ABS_BOOT_HOME_TARGET_RAW 1008U        /* 取衣/放衣结束时M2绝对编码器最终RAW目标。 */
#define M2_ABS_BOOT_HOME_TARGET_DEG 354.38f      /* 不使用固定RAW模式时的M2角度目标，单位度。 */
#define M2_ABS_BOOT_HOME_TOL_DEG 2.0f            /* M2绝对回位允许角度误差，单位度。 */
/*
 * 取放衣流程默认参数。
 * EC1/EC3/EC4表示绝对编码器目标，COUNTS表示从动作起点计算的相对编码器计数，
 * STEPS表示步进电机脉冲步数。修改前必须先确认单位和方向。
 */
#define PLACE_CLOTH_DEFAULT_RPM 150.0f            /* 未写RPM时XYZ轴及M4抬升的默认速度，单位RPM。 */
#define PLACE_CLOTH_DEFAULT_TOL 6U               /* 未写TOL时XYZ编码器目标允许误差，单位计数，即±6。 */
#define PLACE_CLOTH_GRIP_DELAY_MS 2000U          /* 电磁铁ON/OFF后的完整等待时间，单位ms。 */
#ifndef CLOTH_HOLDER_USE_ELECTROMAGNET
#define CLOTH_HOLDER_USE_ELECTROMAGNET 1U        /* 1=PD8电磁铁；0=旧M5夹爪，当前自动流程应保持1。 */
#endif
#define PLACE_CLOTH_M2_INITIAL_CW_STEPS 650U       /* 放衣开始：M2顺时针CW步数，速度固定CLOTH_M2_RPM。 */
#define PLACE_CLOTH_TRANSFER_Z 10000L              /* 放衣取物前：Z轴向下到绝对EC4=10000。 */
#define PLACE_LASER2_X_ZERO_DISTANCE_MM 241.72f    /* 四组实测数据线性拟合得到的X轴有效光学零点距离，单位mm。 */
#define PLACE_LASER2_CALIBRATION_VALID 1U          /* 1=启用当前四点拟合标定，放衣动态X允许按第二测距结果运动。 */
#define PLACE_LASER2_CAL_D_A_MM PLACE_LASER2_X_ZERO_DISTANCE_MM /* 标定点A固定为X归零位置的原始激光距离，单位mm。 */
#define PLACE_LASER2_CAL_EC_A 0L                   /* 标定点A固定对应X轴归零后的绝对EC1=0。 */
#define PLACE_LASER2_CAL_D_B_MM 558.0f             /* 四点拟合等效标定上限：原始激光距离558mm。 */
#define PLACE_LASER2_CAL_EC_B 5919L                /* 四点拟合等效B点：558mm对应绝对EC1=5919。 */
#define PLACE_LASER2_SAMPLE_TIMEOUT_MS 3000U       /* 等待第二测距取得稳定新样本的最长时间，单位ms。 */
#define PLACE_LASER2_FILTER_SAMPLES 3U             /* 动态X换算使用的连续第二测距样本数量，固定为3帧。 */
#define PLACE_LASER2_STABLE_SPREAD_MM 8U           /* 三帧第二测距峰峰值最大允许差，单位mm。 */
#define PLACE_LASER2_CONFIDENCE_MIN 70U            /* 动态X换算允许的第二测距置信度下限。 */
#define PLACE_CLOTH_PICK_Y_EC3 1381L               /* 放衣取物点：Y轴向前到绝对EC3=1381。 */
#define PLACE_CLOTH_PICK_Z 12844L                  /* 放衣取物点：Z轴向下到绝对EC4=12552。 */
#define PLACE_CLOTH_LIFT_STEPS 650U                /* 吸合后M4沿CCW向上抬升的电机步数。 */
#define PLACE_CLOTH_TRANSFER_X 1000L               /* 放衣吸合抬升后的中转点：X轴到绝对EC1=1000。 */
#define PLACE_CLOTH_LASER_Z 3263L                  /* 放衣扫描高度：Z轴到绝对EC4=3263。 */
#define PLACE_CLOTH_M2_SCAN_CW_STEPS 4000U         /* 放衣扫描前：M2顺时针CW步数，速度固定10RPM。 */
#define PLACE_CLOTH_LASER_GATE 8860L               /* 放衣激光起点：X轴向右到绝对EC1=8860。 */
#define PLACE_CLOTH_X_BACK_COUNTS 1760U            /* 板完整退出后：X从实际停止点向左相对1125个EC1计数。 */
#define PLACE_CLOTH_Z_DOWN_COUNTS 1500U            /* 放衣挂衣前：Z从当前位置向下相对1500个EC4计数。 */
#define PLACE_CLOTH_Y_FORWARD_COUNTS 1250U         /* 放衣挂衣前：Y从当前位置向前相对1250个EC3计数。 */
#define PLACE_CLOTH_M2_RELEASE_CCW_STEPS 4000U     /* 放衣释放前：M2逆时针CCW步数，速度固定10RPM。 */
#define PLACE_CLOTH_Z_RELEASE_UP_COUNTS 150U       /* 释放后：Z从当前位置向上相对150个EC4计数。 */
#define PLACE_CLOTH_M2_AFTER_Y_HOME_CW_STEPS 1000U /* 最终Y归零后：M2顺时针CW步数，速度固定10RPM。 */
#define CLOTH_GRIP_RPM 30.0f                       /* 旧M5夹爪速度；使用PD8电磁铁时自动流程不使用。 */
#define CLOTH_GRIP_CLOSE_STEPS 10800U              /* 旧M5夹爪闭合步数；电磁铁模式不使用。 */
#define CLOTH_GRIP_OPEN_STEPS 10800U               /* 旧M5夹爪张开步数；电磁铁模式不使用。 */
#define TAKE_CLOTH_DEFAULT_RPM PLACE_CLOTH_DEFAULT_RPM /* 取衣XYZ默认RPM，目前与放衣默认值共用。 */
#define TAKE_CLOTH_DEFAULT_TOL PLACE_CLOTH_DEFAULT_TOL /* 取衣XYZ默认编码器容差，目前与放衣默认值共用。 */
#define TAKE_CLOTH_GRIP_DELAY_MS PLACE_CLOTH_GRIP_DELAY_MS /* 取衣电磁铁等待时间，目前与放衣共用。 */
#define CLOTH_M2_RPM 10.0f                         /* 取衣和放衣流程内所有M2步进动作的固定速度。 */
#define TAKE_CLOTH_M2_INITIAL_CW_STEPS 650U        /* 取衣开始：M2顺时针CW步数。 */
#define TAKE_CLOTH_LASER_Z 3263L                   /* 取衣扫描高度：Z轴到绝对EC4=3263。 */
#define TAKE_CLOTH_M2_SCAN_CW_STEPS 1500U          /* 取衣扫描前：M2顺时针CW步数。 */
#define TAKE_CLOTH_LASER_GATE 8860L                /* 取衣激光起点：X轴向右到绝对EC1=8860。 */
#define TAKE_CLOTH_X_LEFT_COUNTS 1760U             /* 板完整退出后：X从实际停止点向左相对1760个EC1计数。 */
#define TAKE_CLOTH_Z_TRANSFER_DOWN_COUNTS 1200U    /* 取衣接近衣物：Z从当前位置向下相对1200个EC4计数。 */
#define TAKE_CLOTH_Y_FORWARD_COUNTS 1350U          /* 取衣接近衣物：Y从当前位置向前相对1350个EC3计数。 */
#define TAKE_CLOTH_M2_PICK_CCW_STEPS 1500U         /* 吸取前：M2逆时针CCW步数。 */
#define TAKE_CLOTH_Z_PICK_DOWN_COUNTS 670U         /* 吸取前：Z从当前位置向下相对670个EC4计数。 */
#define TAKE_CLOTH_Z_LIFT_UP_COUNTS 520U           /* 电磁铁吸合后：Z从当前位置向上相对520个EC4计数。 */
#define TAKE_CLOTH_Y_EXTRA_FORWARD_COUNTS 150U     /* 吸合后：Y从当前位置向前相对150个EC3计数。 */
#define TAKE_CLOTH_M2_RETURN_CW_STEPS 1500U        /* 取衣回转：M2顺时针CW步数。 */
#define TAKE_CLOTH_M2_TRANSFER_CCW_STEPS 5500U     /* Y归零后：M2逆时针CCW步数。 */
#define TAKE_CLOTH_TRANSFER_X 1000L                /* 取衣中转点：X轴到绝对EC1=1000。 */
#define TAKE_CLOTH_M2_DROP_CW_STEPS 4000U          /* 放到取衣出口前：M2顺时针CW步数。 */
#define TAKE_CLOTH_DROP_Z 12552L                   /* 取衣放下点：Z轴向下到绝对EC4=12552。 */
#define TAKE_CLOTH_DROP_Y 1470L                    /* 取衣放下点：Y轴向前到绝对EC3=1470。 */
#define TAKE_CLOTH_DROP_X 3200L                    /* 取衣释放前：X轴闭环移动到绝对EC1=3200。 */
#define TAKE_CLOTH_POST_DROP_Z 10000L              /* 释放衣服后：Z轴回到绝对EC4=10000。 */
#define TAKE_LASER_APPROACH_RPM 75.0f              /* 非直扫模式到识别门槛前的X速度；当前自动流程未使用。 */
#define TAKE_LASER_SCAN_RPM 75.0f                  /* 取衣/放衣开始识别定位板时的X扫描速度。 */
#define TAKE_LASER_PRECISION_RPM 50.0f             /* 识别进入有效板面后降低到的X扫描速度。 */
#define TAKE_LASER_READY_TIMEOUT_MS 3000U          /* 等待激光通信恢复及新响应的最长时间，单位ms。 */
#define TAKE_LASER_READY_FRESH_RESPONSES 2U        /* 扫描前必须收到的全新激光响应帧数。 */
#define TAKE_LASER_COMM_SILENCE_TIMEOUT_MS 500U    /* 扫描中连续无激光响应的离线判定时间，单位ms。 */
#define TAKE_LASER_GATE_TIMEOUT_MS 60000U          /* 非直扫模式等待X到识别门槛的超时；当前自动流程未使用。 */
#define TAKE_LASER_SCAN_TIMEOUT_MS 70000U          /* 一直找不到目标有效板时停止X的超时，单位ms，即70秒。 */
#define TAKE_LASER_SCAN_STALL_TIMEOUT_MS 1200U     /* 扫描中EC1不变化的失速判定时间，单位ms。 */
#define TAKE_LASER_GATE_LOG_PERIOD_MS 5000U        /* 非直扫门槛等待日志输出周期，单位ms。 */
#define TAKE_LASER_FILTER_SAMPLES 3U               /* 激光稳定判断使用的连续样本数量。 */
#define TAKE_LASER_STABLE_SPREAD_MM 8U             /* 连续样本峰峰值最大允许差，单位mm。 */
#define TAKE_LASER_PLATE_CONFIDENCE_MIN 70U        /* 有效定位板激光置信度下限，CONF必须不小于70。 */
#define TAKE_LASER_ENABLE_ENCODER_COUNT 12200L     /* 旧扫描识别启用点保留值；当前自动流程未使用。 */
#define TAKE_LASER_BOARD_DISTANCE_MIN_MM 200U      /* 取衣/放衣有效定位板距离下限，包含200mm。 */
#define TAKE_LASER_BOARD_DISTANCE_MAX_MM 250U      /* 取衣/放衣有效定位板距离上限，包含250mm。 */
#define TAKE_LASER_BACKGROUND_CONFIRM_FRAMES 3U    /* 离开板面时连续背景确认帧数。 */
#define TAKE_LASER_BOARD_MIN_WIDTH_COUNTS 1000L    /* 有效定位板完整宽度下限，单位EC1计数。 */
#define TAKE_LASER_BOARD_MAX_WIDTH_COUNTS 2200L    /* 有效定位板完整宽度上限，单位EC1计数。 */
#define TAKE_LASER_X_OFFSET_COUNTS 0L              /* 取衣板中心日志偏移；当前流程不回中心，通常保持0。 */
#define PLACE_LASER_X_OFFSET_COUNTS 0L             /* 放衣板中心日志偏移；当前流程不回中心，通常保持0。 */
#define TAKE_LASER_DEBUG_LOG_PERIOD_MS 50U         /* DEBUG ON时激光扫描明细日志周期，单位ms。 */
#define TAKE_CLOTH_FAULT_SENSOR 0x04U
#define TAKE_CLOTH_FAULT_SLOT_EMPTY 0x08U
/* 未指定 SLOT 时默认选择第一块有效定位板，显式 SLOT 支持 1..255。 */
#define CLOTH_SLOT_DEFAULT 1U                     /* 命令未写SLOT时默认取/放第1块有效定位板。 */
#define HOME_EXKEY_STABLE_MS 20U
#define M4_HOME_RELEASE_TIMEOUT_MS 1000U
#define M4_HOME_SEEK_TIMEOUT_MS 60000U

/* 取放衣都从目标板完整退出位置执行各自的固定相对动作。 */

/*
 * 反向回零限位保护：
 * 每个元素绑定一个电机和一个 EX_KEY 输入。
 * active 表示该电机正在做“向限位开关方向回零”；
 * include_all_home 表示 ALL_HOME 时是否包含该电机。
 */
typedef struct {
    MotorId_t motor_id;
    GPIO_TypeDef *key_port;
    uint16_t key_pin;
    uint8_t key_index;
    MotorDir_t home_dir;
    uint8_t active;
    uint8_t include_all_home;
    uint16_t blocked_stable_ms;
} RevExKeyGuard_t;

typedef enum {
    M4_HOME_PHASE_IDLE = 0,
    M4_HOME_PHASE_RELEASE,
    M4_HOME_PHASE_SEEK
} M4HomePhase_t;

typedef struct {
    M4HomePhase_t phase;
    uint32_t elapsed_ms;
    uint16_t stable_ms;
    float rpm;
} M4HomeRuntime_t;

/* 增量编码器目标运动状态，用于 M1/M3/M4 按编码器计数运动到目标。 */
typedef struct {
    uint8_t active;
    uint8_t done;
    uint8_t fault;
    MotorDir_t dir;
    uint8_t count_increases;
    int32_t start_count;
    int32_t target_count;
    int32_t last_count;
    uint32_t tol;
    uint32_t still_ms;
    uint8_t stall_boost_active;
    uint8_t wrong_dir_count;
    uint32_t wrong_dir_accum;
    uint16_t pid_elapsed_ms;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_integral;
    float pid_prev_error;
    float pid_rpm_min;
    float pid_rpm_max;
    float pid_last_rpm;
} EncMove_t;

/* M2 绝对值编码器目标运动状态，用于按绝对角度移动 M2。 */
typedef struct {
    uint8_t active;
    MotorDir_t dir;
    uint16_t target_raw;
    uint16_t last_raw;
    uint16_t last_remain;
    uint16_t tol_counts;
    uint16_t poll_ms;
    uint16_t still_ms;
    uint8_t comm_fail_count;
    uint8_t boot_home;
    float target_deg;
    float tol_deg;
} M2AbsMove_t;

typedef enum {
    M2_ABS_READ_OWNER_NONE = 0,
    M2_ABS_READ_OWNER_BOOT_HOME,
    M2_ABS_READ_OWNER_MOVE
} M2AbsReadOwner_t;

/*
 * 放衣服流程状态机：
 * 按新机械顺序完成取衣、扫描、相对定位、释放及最终 M2 绝对回零。
 */
typedef enum {
    PLACE_CLOTH_IDLE = 0,
    PLACE_CLOTH_START_HOME,
    PLACE_CLOTH_WAIT_START_HOME,
    PLACE_CLOTH_M2_INITIAL,
    PLACE_CLOTH_WAIT_M2_INITIAL,
    PLACE_CLOTH_MOVE_TRANSFER_Z,
    PLACE_CLOTH_START_LASER2_SAMPLE,
    PLACE_CLOTH_WAIT_LASER2_SAMPLE,
    PLACE_CLOTH_MOVE_DYNAMIC_X,
    PLACE_CLOTH_MOVE_PICK_Y,
    PLACE_CLOTH_MOVE_PICK_Z,
    PLACE_CLOTH_MAGNET_ON,
    PLACE_CLOTH_WAIT_MAGNET_ON,
    PLACE_CLOTH_LIFT_Z,
    PLACE_CLOTH_WAIT_LIFT_Z,
    PLACE_CLOTH_MOVE_TRANSFER_X,
    PLACE_CLOTH_HOME_Y,
    PLACE_CLOTH_WAIT_HOME_Y,
    PLACE_CLOTH_MOVE_LASER_Z,
    PLACE_CLOTH_MOVE_M2_SCAN,
    PLACE_CLOTH_WAIT_M2_SCAN,
    PLACE_CLOTH_MOVE_LASER_GATE,
    PLACE_CLOTH_WAIT_LASER_READY,
    PLACE_CLOTH_START_LASER_SCAN,
    PLACE_CLOTH_LASER_SCAN,
    PLACE_CLOTH_MOVE_X_BACK,
    PLACE_CLOTH_MOVE_Z_DOWN,
    PLACE_CLOTH_MOVE_Y_FORWARD,
    PLACE_CLOTH_MOVE_M2_RELEASE,
    PLACE_CLOTH_WAIT_M2_RELEASE,
    PLACE_CLOTH_MAGNET_OFF,
    PLACE_CLOTH_WAIT_MAGNET_OFF,
    PLACE_CLOTH_MOVE_Z_RELEASE_UP,
    PLACE_CLOTH_HOME_Y_FINAL,
    PLACE_CLOTH_WAIT_HOME_Y_FINAL,
    PLACE_CLOTH_MOVE_M2_AFTER_Y_HOME,
    PLACE_CLOTH_WAIT_M2_AFTER_Y_HOME,
    PLACE_CLOTH_ALL_HOME,
    PLACE_CLOTH_M2_HOME,
    PLACE_CLOTH_DONE
} PlaceClothStep_t;

/* 放衣服流程运行状态，保存当前步骤、等待电机、目标衣格坐标和运行参数。 */
typedef struct {
    uint8_t active;
    PlaceClothStep_t step;
    MotorId_t wait_motor;
    uint32_t wait_ms;
    uint8_t y_home_done;
    uint8_t m2_home_started;
    float rpm;
    uint32_t tol;
    uint32_t slot;
    int32_t relative_target;
    int32_t laser2_target;
    uint32_t laser2_started_ms;
    uint16_t laser2_samples[PLACE_LASER2_FILTER_SAMPLES];
    uint8_t laser2_sample_count;
} PlaceCloth_t;

/*
 * 取衣服流程状态机：
 * 先回零，再移动到目标衣格夹住衣服，抬升/转移到取衣点，松开夹爪后回零。
 */
typedef enum {
    TAKE_CLOTH_IDLE = 0,
    TAKE_CLOTH_START_HOME,
    TAKE_CLOTH_WAIT_START_HOME,
    TAKE_CLOTH_M2_INITIAL,
    TAKE_CLOTH_WAIT_M2_INITIAL,
    TAKE_CLOTH_MOVE_LASER_Z,
    TAKE_CLOTH_M2_SCAN,
    TAKE_CLOTH_WAIT_M2_SCAN,
    TAKE_CLOTH_MOVE_LASER_GATE,
    TAKE_CLOTH_WAIT_LASER_READY,
    TAKE_CLOTH_START_LASER_SCAN,
    TAKE_CLOTH_LASER_SCAN,
    TAKE_CLOTH_MOVE_X_LEFT,
    TAKE_CLOTH_MOVE_Z_TRANSFER_DOWN,
    TAKE_CLOTH_MOVE_Y_FORWARD,
    TAKE_CLOTH_M2_PICK,
    TAKE_CLOTH_WAIT_M2_PICK,
    TAKE_CLOTH_MOVE_Z_PICK_DOWN,
    TAKE_CLOTH_MAGNET_ON,
    TAKE_CLOTH_WAIT_MAGNET_ON,
    TAKE_CLOTH_MOVE_Z_LIFT_UP,
    TAKE_CLOTH_MOVE_Y_EXTRA_FORWARD,
    TAKE_CLOTH_M2_RETURN,
    TAKE_CLOTH_WAIT_M2_RETURN,
    TAKE_CLOTH_HOME_Y,
    TAKE_CLOTH_WAIT_HOME_Y,
    TAKE_CLOTH_M2_TRANSFER,
    TAKE_CLOTH_WAIT_M2_TRANSFER,
    TAKE_CLOTH_MOVE_TRANSFER_X,
    TAKE_CLOTH_M2_DROP,
    TAKE_CLOTH_WAIT_M2_DROP,
    TAKE_CLOTH_MOVE_DROP_Z,
    TAKE_CLOTH_MOVE_DROP_Y,
    TAKE_CLOTH_MOVE_DROP_X,
    TAKE_CLOTH_MAGNET_OFF,
    TAKE_CLOTH_WAIT_MAGNET_OFF,
    TAKE_CLOTH_MOVE_POST_DROP_Z,
    TAKE_CLOTH_START_ALL_HOME,
    TAKE_CLOTH_ALL_HOME,
    TAKE_CLOTH_M2_HOME,
    TAKE_CLOTH_DONE
} TakeClothStep_t;

/* 取衣服流程运行状态，保存当前步骤、等待计时、目标衣格坐标和运行参数。 */
typedef struct {
    uint8_t active;
    TakeClothStep_t step;
    MotorId_t wait_motor;
    uint32_t wait_ms;
    uint8_t y_home_done;
    uint8_t m2_home_started;
    float rpm;
    uint32_t tol;
    uint32_t slot;
    int32_t relative_target;
} TakeCloth_t;

typedef enum {
    TAKE_LASER_SAMPLE_WARMUP = 0,
    TAKE_LASER_SAMPLE_BACKGROUND,
    TAKE_LASER_SAMPLE_TRANSITION,
    TAKE_LASER_SAMPLE_PLATE
} TakeLaserSampleClass_t;

typedef enum {
    TAKE_LASER_WINDOW_BACKGROUND = -2,
    TAKE_LASER_WINDOW_TRANSITION = -1,
    TAKE_LASER_WINDOW_WARMUP = 0,
    TAKE_LASER_WINDOW_STABLE = 1
} TakeLaserWindowResult_t;

typedef enum {
    TAKE_LASER_BOARD_SEARCH = 0,
    TAKE_LASER_BOARD_TARGET
} TakeLaserBoardState_t;

typedef enum {
    CLOTH_LASER_OWNER_NONE = 0,
    CLOTH_LASER_OWNER_PLACE,
    CLOTH_LASER_OWNER_TAKE
} ClothLaserOwner_t;

typedef enum {
    CLOTH_LASER_PROGRESS = 0,
    CLOTH_LASER_READY = 1,
    CLOTH_LASER_FOUND = 2,
    CLOTH_LASER_ERR_READY_TIMEOUT = -1,
    CLOTH_LASER_ERR_START_LIMIT = -2,
    CLOTH_LASER_ERR_START = -3,
    CLOTH_LASER_ERR_STATUS = -4,
    CLOTH_LASER_ERR_OFFLINE = -5,
    CLOTH_LASER_ERR_LIMIT = -6,
    CLOTH_LASER_ERR_MOTOR = -7,
    CLOTH_LASER_ERR_STALL = -8,
    CLOTH_LASER_ERR_GATE_TIMEOUT = -9,
    CLOTH_LASER_ERR_SCAN_SPEED = -10,
    CLOTH_LASER_ERR_PRECISION_SPEED = -11,
    CLOTH_LASER_ERR_SEARCH_SPEED = -12,
    CLOTH_LASER_ERR_TARGET_TIMEOUT = -13,
    CLOTH_LASER_ERR_SEARCH_TIMEOUT = -14
} ClothLaserResult_t;

typedef struct {
    uint8_t filter_count;
    uint8_t filter_index;
    uint8_t exit_count;
    uint8_t found_slots;
    uint8_t recognition_enabled;
    ClothLaserOwner_t owner;
    TakeLaserBoardState_t board_state;
    uint16_t filter_mm[TAKE_LASER_FILTER_SAMPLES];
    int32_t filter_x[TAKE_LASER_FILTER_SAMPLES];
    uint16_t last_spread_mm;
    uint16_t board_min_mm;
    uint16_t board_max_mm;
    uint32_t wait_ms;
    uint32_t scan_ms;
    uint32_t still_ms;
    uint32_t debug_last_ms;
    uint32_t ready_response_base;
    uint32_t ready_fault_base;
    uint32_t comm_last_response_count;
    uint32_t comm_silence_ms;
    uint32_t target_slot;
    int32_t last_encoder_count;
    int32_t board_enter_x;
    int32_t board_exit_start_x;
    int32_t center_offset_counts;
    int32_t gate_count;
    uint8_t direct_search;
} TakeLaserScan_t;

/* M1/M2/M3/M4 对应的回零限位输入配置。 */
static RevExKeyGuard_t g_rev_exkey_guards[] = {
    { MOTOR_ID_1, EX_KEY1_GPIO_Port, EX_KEY1_Pin, 1U, MOTOR_DIR_CCW, 0U, 1U, 0U },
    { MOTOR_ID_2, EX_KEY2_GPIO_Port, EX_KEY2_Pin, 2U, MOTOR_DIR_CCW, 0U, 0U, 0U },
    { MOTOR_ID_3, EX_KEY3_GPIO_Port, EX_KEY3_Pin, 3U, MOTOR_DIR_CCW, 0U, 1U, 0U },
    { MOTOR_ID_4, EX_KEY4_GPIO_Port, EX_KEY4_Pin, 4U, MOTOR_DIR_CCW, 0U, 1U, 0U }
};

/* 以下全局变量保存命令分发层的异步任务状态。 */
static M2AbsMove_t g_m2_abs_move;
static EncMove_t g_enc_move[MOTOR_COUNT];
static uint8_t g_m2_abs_boot_home_pending;
static uint32_t g_m2_abs_boot_home_ms;
static uint32_t g_m2_abs_boot_home_last_try_ms;
static uint8_t g_m2_abs_boot_home_done;
static uint8_t g_m2_abs_boot_home_fault;
static M2AbsReadOwner_t g_m2_abs_read_owner;
static uint8_t g_all_home_active;
static uint32_t g_all_home_target_mask;
static uint32_t g_all_home_done_mask;
static uint8_t g_m4_upper_limit_latched;
static M4HomeRuntime_t g_m4_home;
static PlaceCloth_t g_place_cloth;
static TakeCloth_t g_take_cloth;
static TakeLaserScan_t g_take_laser_scan;
static LaserRangeStatus_t g_take_laser_fault_status;
static char g_take_laser_fault_msg[256];
/* 取/放衣完成或故障事件，由 main.c 轮询后转换成 ESP32/阿里云响应帧。 */
static CmdClothEvent_t g_cloth_event = CMD_CLOTH_EVENT_NONE;
static uint8_t g_cloth_fault_code = 0U;

static CmdResult_t resp_format(char *resp, uint16_t resp_len, const char *fmt, ...);
static CmdResult_t resp_err(char *resp, uint16_t resp_len, uint16_t code, const char *msg);
static void cloth_event_set(CmdClothEvent_t event, uint8_t fault_code);
static void enc_move_stop_xyz(uint8_t estop);
static void rev_exkey_cancel_if_motor(MotorId_t motor_id);
static RevExKeyGuard_t *rev_exkey_guard_by_motor(MotorId_t motor_id);
static void m4_home_runtime_reset(void);

#define REV_EXKEY_GUARD_COUNT (sizeof(g_rev_exkey_guards) / sizeof(g_rev_exkey_guards[0]))

static void m2_abs_cancel_pending_read(void)
{
    if (g_m2_abs_read_owner != M2_ABS_READ_OWNER_NONE) {
        Rs485AbsEncoder_CancelM2Read();
        g_m2_abs_read_owner = M2_ABS_READ_OWNER_NONE;
    }
}

enum {
    /* 文本命令错误码，响应格式一般为 ERR code message。 */
    CMD_ERR_BAD_TARGET_ACTION = 200,
    CMD_ERR_MISSING_ARG = 201,
    CMD_ERR_BAD_ARG = 202,
    CMD_ERR_MOTOR_OP = 203,
    CMD_ERR_STATUS = 204,
    CMD_ERR_RESP_OVERFLOW = 205,
    CMD_ERR_ENCODER_OP = 206,
    CMD_ERR_CURVE_OP = 207,
    CMD_ERR_RS485_ENCODER_OP = 208,
    CMD_ERR_LIMIT_NOT_READY = 211,
    CMD_ERR_LIMIT_VIOLATION = 212,
    CMD_ERR_LIMIT_SAVE = 213,
    CMD_ERR_LIMIT_LOAD = 214
};

enum {
    /* 参数位掩码，用于快速检查某个命令是否带齐必要参数、是否带了不允许的参数。 */
    ARG_DIR = (1U << 0),
    ARG_RPM = (1U << 1),
    ARG_TO = (1U << 2),
    ARG_T = (1U << 3),
    ARG_STEP = (1U << 4),
    ARG_CYCLE = (1U << 5),
    ARG_PPR = (1U << 6),
    ARG_ACC = (1U << 7),
    ARG_DEC = (1U << 8),
    ARG_STOP = (1U << 9),
    ARG_RATE = (1U << 10),
    ARG_X = (1U << 11),
    ARG_Y = (1U << 12),
    ARG_Z = (1U << 13),
    ARG_ERR = (1U << 14),
    ARG_TOL = (1UL << 15),
    ARG_DEG = (1UL << 16),
    ARG_REG = (1UL << 17),
    ARG_VAL = (1UL << 18),
    ARG_SLOT = (1UL << 19)
};

static uint32_t args_mask(const CmdArgs_t *args)
{
    uint32_t mask = 0U;

    /*
     * 把 ParsedCmd_t 中的 has_xxx 标志转换成统一位掩码。
     * 后续各类命令只需要用 args_check() 比较 required/allowed 掩码，
     * 就能判断参数是否缺失或是否传入了当前命令不允许的多余参数。
     */
    if (args == 0) {
        return 0U;
    }

    if (args->has_dir) mask |= ARG_DIR;
    if (args->has_rpm) mask |= ARG_RPM;
    if (args->has_to) mask |= ARG_TO;
    if (args->has_time_ms) mask |= ARG_T;
    if (args->has_step) mask |= ARG_STEP;
    if (args->has_cycle) mask |= ARG_CYCLE;
    if (args->has_ppr) mask |= ARG_PPR;
    if (args->has_acc) mask |= ARG_ACC;
    if (args->has_dec) mask |= ARG_DEC;
    if (args->has_stop_ms) mask |= ARG_STOP;
    if (args->has_rate) mask |= ARG_RATE;
    if (args->has_x) mask |= ARG_X;
    if (args->has_y) mask |= ARG_Y;
    if (args->has_z) mask |= ARG_Z;
    if (args->has_slot) mask |= ARG_SLOT;
    if (args->has_err) mask |= ARG_ERR;
    if (args->has_tol) mask |= ARG_TOL;
    if (args->has_deg) mask |= ARG_DEG;
    if (args->has_reg) mask |= ARG_REG;
    if (args->has_val) mask |= ARG_VAL;

    return mask;
}

    /* 根据衣格编号查找坐标配置。当前表里只有 1 号位。 */
static uint8_t rev_exkey_is_blocked_low(const RevExKeyGuard_t *guard)
{
    /* EX_KEY 低电平表示触发/被挡住，用于回零停止条件。 */
    return (HAL_GPIO_ReadPin(guard->key_port, guard->key_pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void cloth_event_set(CmdClothEvent_t event, uint8_t fault_code)
{
    /* 设置取放衣事件，主循环会通过 CmdDispatch_PollClothEvent() 取走。 */
    g_cloth_event = event;
    g_cloth_fault_code = fault_code;
}

static uint32_t motor_bit(MotorId_t motor_id)
{
    /* 把电机编号转换成位掩码，ALL HOME 用它记录哪些电机应该回零、哪些已经完成。 */
    return (motor_id < MOTOR_COUNT) ? (1UL << (uint32_t)motor_id) : 0U;
}

static uint32_t abs_i32(int32_t v)
{
    /* int32_t 取绝对值，主要用于编码器计数差值。 */
    return (v >= 0) ? (uint32_t)v : (uint32_t)(-v);
}

static uint32_t abs_diff_i32(int32_t a, int32_t b)
{
    /* 计算两个 int32_t 计数之间的绝对差，用 int64_t 中间值避免减法溢出。 */
    int64_t d = (int64_t)a - (int64_t)b;
    return (d >= 0) ? (uint32_t)d : (uint32_t)(-d);
}

static uint16_t m2_abs_deg_to_raw(float deg)
{
    uint32_t raw;

    /* 把 0~360 度角度换算成绝对值编码器一圈内的原始计数。 */
    raw = (uint32_t)((deg * (float)RS485_ABS_ENCODER_CPR / 360.0f) + 0.5f);
    return (uint16_t)(raw % RS485_ABS_ENCODER_CPR);
}

static uint16_t m2_abs_tol_deg_to_counts(float tol_deg)
{
    uint32_t counts;

    /* 把 M2 角度容差从“度”换算为绝对值编码器计数，并限制在有效范围内。 */
    counts = (uint32_t)((tol_deg * (float)RS485_ABS_ENCODER_CPR / 360.0f) + 0.5f);
    if (counts < 1U) {
        counts = 1U;
    }
    if (counts > (RS485_ABS_ENCODER_CPR / 2U)) {
        counts = RS485_ABS_ENCODER_CPR / 2U;
    }
    return (uint16_t)counts;
}

static int32_t m2_abs_signed_delta(uint16_t from_raw, uint16_t to_raw)
{
    /*
     * 计算从 from_raw 到 to_raw 的最短有符号差值。
     * 绝对值编码器是环形计数，需要处理跨 0 点的情况。
     */
    int32_t delta = (int32_t)to_raw - (int32_t)from_raw;

    if (delta > ((int32_t)RS485_ABS_ENCODER_CPR / 2)) {
        delta -= (int32_t)RS485_ABS_ENCODER_CPR;
    } else if (delta < -((int32_t)RS485_ABS_ENCODER_CPR / 2)) {
        delta += (int32_t)RS485_ABS_ENCODER_CPR;
    }

    return delta;
}

static uint16_t m2_abs_remain_for_dir(uint16_t raw, uint16_t target_raw, MotorDir_t dir)
{
    /* 计算按指定方向转到目标 raw 还剩多少计数。 */
    uint8_t raw_increases;

    raw_increases = (dir == MOTOR_DIR_CW) ? M2_ABS_CW_RAW_INCREASES : (uint8_t)(M2_ABS_CW_RAW_INCREASES == 0U);

    if (raw_increases != 0U) {
        return (uint16_t)(((uint32_t)target_raw + RS485_ABS_ENCODER_CPR - raw) % RS485_ABS_ENCODER_CPR);
    }

    return (uint16_t)(((uint32_t)raw + RS485_ABS_ENCODER_CPR - target_raw) % RS485_ABS_ENCODER_CPR);
}

static uint8_t m2_abs_delta_matches_dir(int32_t delta, MotorDir_t dir)
{
    uint8_t raw_increases;

    /*
     * 判断本次绝对值编码器变化方向是否和电机运行方向一致。
     * 如果方向不一致，说明可能接线、方向配置或机械动作异常。
     */
    if (delta == 0) {
        return 1U;
    }

    raw_increases = (dir == MOTOR_DIR_CW) ? M2_ABS_CW_RAW_INCREASES : (uint8_t)(M2_ABS_CW_RAW_INCREASES == 0U);
    if (raw_increases != 0U) {
        return (delta > 0) ? 1U : 0U;
    }

    return (delta < 0) ? 1U : 0U;
}

static uint16_t m2_abs_short_distance(uint16_t raw, uint16_t target_raw)
{
    int32_t delta = m2_abs_signed_delta(raw, target_raw);

    /* 返回当前位置 raw 到目标 target_raw 的环形最短距离。 */
    return (uint16_t)((delta >= 0) ? delta : -delta);
}

static uint8_t m2_abs_target_reached(uint16_t raw)
{
    /* 判断 M2 绝对值编码器当前位置是否已经到达目标 raw 附近。 */
    return (m2_abs_short_distance(raw, g_m2_abs_move.target_raw) <= g_m2_abs_move.tol_counts) ? 1U : 0U;
}

static uint8_t enc_move_motor_supported(MotorId_t motor_id)
{
    /* 当前编码器目标运动只支持 XYZ 对应的 M1/M3/M4。 */
    return ((motor_id == MOTOR_ID_1) ||
            (motor_id == MOTOR_ID_3) ||
            (motor_id == MOTOR_ID_4)) ? 1U : 0U;
}

static uint8_t enc_move_target_reached(const EncMove_t *move, int32_t current)
{
    /* 根据预期计数增减、目标计数和容差判断是否到位。 */
    if (move == 0) {
        return 0U;
    }

    if (move->count_increases != 0U) {
        if (current >= move->target_count) {
            return 1U;
        }
    } else {
        if (current <= move->target_count) {
            return 1U;
        }
    }

    return (abs_diff_i32(current, move->target_count) <= move->tol) ? 1U : 0U;
}

static float clamp_f32(float value, float min_value, float max_value)
{
    /* 浮点限幅工具，PID 输出和积分项都通过这里约束，避免调速指令越界。 */
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float abs_f32(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float enc_move_y_profile_rpm(float remaining_counts,
                                    float rpm_min,
                                    float rpm_max)
{
    float ratio;

    /* Y 轴进入最后 1200 个编码器计数后，按剩余距离线性降低转速。 */
    if (remaining_counts >= Y_ENC_MOVE_DECEL_COUNTS) {
        return rpm_max;
    }

    ratio = clamp_f32(remaining_counts / Y_ENC_MOVE_DECEL_COUNTS, 0.0f, 1.0f);
    return rpm_min + ((rpm_max - rpm_min) * ratio);
}

static float enc_move_start_rpm(MotorId_t motor_id,
                                int32_t current_count,
                                int32_t target_count,
                                float rpm_max)
{
    MotorStatusSnapshot_t status;
    float rpm_min = ENC_MOVE_PID_RPM_MIN_DEFAULT;

    if (motor_id != MOTOR_ID_3) {
        return rpm_max;
    }

    if (MotorCtrl_GetStatus(motor_id, &status) == 0) {
        float hw_min_rpm = MotorCtrl_GetRpmMin(status.ppr);
        if (rpm_min < hw_min_rpm) {
            rpm_min = hw_min_rpm;
        }
    }
    if (rpm_min > rpm_max) {
        rpm_min = rpm_max;
    }

    return enc_move_y_profile_rpm((float)abs_diff_i32(target_count, current_count),
                                  rpm_min,
                                  rpm_max);
}

static void enc_move_pid_init(MotorId_t motor_id,
                              EncMove_t *move,
                              float rpm_max,
                              int32_t current_count)
{
    MotorStatusSnapshot_t status;
    float hw_min_rpm = 0.0f;

    /*
     * 编码器目标运动 PID 初始化。
     * rpm_max 使用命令传入的 RPM，保证 PID 不会突破上位机/取放衣流程指定的最高速度。
     */
    if ((move == 0) || (rpm_max <= 0.0f)) {
        return;
    }

    if (MotorCtrl_GetStatus(motor_id, &status) == 0) {
        hw_min_rpm = MotorCtrl_GetRpmMin(status.ppr);
    }

    move->pid_kp = ENC_MOVE_PID_KP_DEFAULT;
    move->pid_ki = ENC_MOVE_PID_KI_DEFAULT;
    move->pid_kd = ENC_MOVE_PID_KD_DEFAULT;
    move->pid_integral = 0.0f;
    move->pid_prev_error = (float)abs_diff_i32(move->target_count, current_count);
    move->pid_rpm_max = rpm_max;
    move->pid_rpm_min = ENC_MOVE_PID_RPM_MIN_DEFAULT;
    if (move->pid_rpm_min < hw_min_rpm) {
        move->pid_rpm_min = hw_min_rpm;
    }
    if (move->pid_rpm_min > move->pid_rpm_max) {
        move->pid_rpm_min = move->pid_rpm_max;
    }
    if (motor_id == MOTOR_ID_3) {
        move->pid_last_rpm = enc_move_y_profile_rpm(move->pid_prev_error,
                                                    move->pid_rpm_min,
                                                    move->pid_rpm_max);
    } else {
        move->pid_last_rpm = rpm_max;
    }
    move->pid_elapsed_ms = ENC_MOVE_PID_SAMPLE_MS;
    move->stall_boost_active = 0U;
}

static int enc_move_pid_update(MotorId_t motor_id, EncMove_t *move, int32_t current_count)
{
#if ENC_MOVE_PID_ENABLE
    float error;
    float derivative;
    float rpm_cmd;
    const float dt_s = ((float)ENC_MOVE_PID_SAMPLE_MS) / 1000.0f;

    /*
     * 1ms SafetyTick 中的低频 PID 调速。
     * 为降低频繁改 ARR/CCR 的抖动，实际每 ENC_MOVE_PID_SAMPLE_MS 更新一次速度。
     */
    if ((move == 0) || (move->active == 0U)) {
        return 0;
    }

    if (move->pid_elapsed_ms < ENC_MOVE_PID_SAMPLE_MS) {
        move->pid_elapsed_ms++;
        return 0;
    }
    move->pid_elapsed_ms = 0U;

    error = (float)abs_diff_i32(move->target_count, current_count);
    if (error <= (float)move->tol) {
        return 0;
    }

    if (move->stall_boost_active != 0U) {
        rpm_cmd = clamp_f32(ENC_MOVE_PID_STALL_BOOST_RPM,
                            move->pid_rpm_min,
                            move->pid_rpm_max);
    } else if (motor_id == MOTOR_ID_3) {
        rpm_cmd = enc_move_y_profile_rpm(error,
                                         move->pid_rpm_min,
                                         move->pid_rpm_max);
    } else {
        move->pid_integral += error * dt_s;
        move->pid_integral = clamp_f32(move->pid_integral,
                                       -ENC_MOVE_PID_INTEGRAL_LIMIT,
                                       ENC_MOVE_PID_INTEGRAL_LIMIT);

        derivative = (error - move->pid_prev_error) / dt_s;
        move->pid_prev_error = error;

        rpm_cmd = (move->pid_kp * error) +
                  (move->pid_ki * move->pid_integral) +
                  (move->pid_kd * derivative);
    }
    rpm_cmd = clamp_f32(rpm_cmd, move->pid_rpm_min, move->pid_rpm_max);

    if (abs_f32(rpm_cmd - move->pid_last_rpm) < ENC_MOVE_PID_RPM_CHANGE_EPS) {
        return 0;
    }

    if (MotorCtrl_SetSpeed(motor_id, rpm_cmd) != 0) {
        return -1;
    }
    move->pid_last_rpm = rpm_cmd;
#else
    (void)motor_id;
    (void)move;
    (void)current_count;
#endif
    return 0;
}

static int enc_move_apply_stall_boost(MotorId_t motor_id, EncMove_t *move)
{
    float rpm_cmd;

    if ((move == 0) || (move->active == 0U) || (move->stall_boost_active != 0U)) {
        return 0;
    }

    rpm_cmd = clamp_f32(ENC_MOVE_PID_STALL_BOOST_RPM,
                        move->pid_rpm_min,
                        move->pid_rpm_max);
    if (MotorCtrl_SetSpeed(motor_id, rpm_cmd) != 0) {
        return -1;
    }

    move->pid_last_rpm = rpm_cmd;
    move->stall_boost_active = 1U;
    return 0;
}

static void place_cloth_cancel(void)
{
    /* 清空放衣任务状态，不主动停电机；停运动由 place_cloth_stop_motion() 负责。 */
    g_place_cloth.active = 0U;
    g_place_cloth.step = PLACE_CLOTH_IDLE;
    g_place_cloth.wait_motor = MOTOR_ID_INVALID;
    g_place_cloth.wait_ms = 0U;
    g_place_cloth.y_home_done = 0U;
    g_place_cloth.m2_home_started = 0U;
    g_place_cloth.slot = 0U;
    g_place_cloth.relative_target = 0L;
    g_place_cloth.laser2_target = 0L;
    g_place_cloth.laser2_started_ms = 0U;
    g_place_cloth.laser2_sample_count = 0U;
    memset(g_place_cloth.laser2_samples, 0, sizeof(g_place_cloth.laser2_samples));
}

static uint8_t cloth_motor_mode_busy(MotorMode_t mode)
{
    /* 判断电机模式是否属于“仍在运动或正在软停止”的忙碌状态。 */
    return ((mode == MOTOR_MODE_RUN) ||
            (mode == MOTOR_MODE_RAMP_UP) ||
            (mode == MOTOR_MODE_RAMP_DOWN) ||
            (mode == MOTOR_MODE_MOVE_STEPS) ||
            (mode == MOTOR_MODE_RECIP_FINITE) ||
            (mode == MOTOR_MODE_RECIP_INFINITE) ||
            (mode == MOTOR_MODE_SOFT_STOP)) ? 1U : 0U;
}

static void place_cloth_stop_motion(uint8_t estop)
{
    const MotorId_t motors[] = { MOTOR_ID_1, MOTOR_ID_2, MOTOR_ID_3, MOTOR_ID_4, MOTOR_ID_5 };
    uint32_t i;

    /*
     * 停止取/放衣流程涉及的电机：
     * estop=1 时急停；
     * estop=0 时普通软停止。
     */
    for (i = 0U; i < (sizeof(motors) / sizeof(motors[0])); i++) {
        if (estop != 0U) {
            (void)MotorCtrl_EStop(motors[i]);
        } else {
            (void)MotorCtrl_Stop(motors[i], 0U);
        }
        rev_exkey_cancel_if_motor(motors[i]);
    }

    g_all_home_active = 0U;
    g_all_home_target_mask = 0U;
    g_all_home_done_mask = 0U;
}

static uint8_t place_cloth_motor_busy(MotorId_t motor_id)
{
    MotorStatusSnapshot_t status;

    /* 某个电机只要编码器目标运动未完成，或电机控制状态仍忙，就认为该电机忙。 */
    if (motor_id >= MOTOR_COUNT) {
        return 1U;
    }
    if (g_enc_move[motor_id].active != 0U) {
        return 1U;
    }
    if (MotorCtrl_GetStatus(motor_id, &status) != 0) {
        return 1U;
    }
    return cloth_motor_mode_busy(status.mode);
}

static uint8_t cloth_xyz_motion_busy(void)
{
    /* 取/放衣新任务启动前，检查 M2、XYZ 和保持器是否还存在未完成动作。 */
    if (g_all_home_active != 0U) {
        return 1U;
    }
    if ((g_m2_abs_boot_home_pending != 0U) ||
        (g_m2_abs_move.active != 0U) ||
        (place_cloth_motor_busy(MOTOR_ID_2) != 0U)) {
        return 1U;
    }
    if (place_cloth_motor_busy(MOTOR_ID_1) != 0U) {
        return 1U;
    }
    if (place_cloth_motor_busy(MOTOR_ID_3) != 0U) {
        return 1U;
    }
    if (place_cloth_motor_busy(MOTOR_ID_4) != 0U) {
        return 1U;
    }
    if (place_cloth_motor_busy(MOTOR_ID_5) != 0U) {
        return 1U;
    }
    return 0U;
}

#if !CLOTH_HOLDER_USE_ELECTROMAGNET
static int cloth_grip_start(MotorDir_t dir, uint32_t steps)
{
    /* 启动 M5 夹爪步进运动；M5 忙或步数为 0 时拒绝启动。 */
    if ((steps == 0U) || (place_cloth_motor_busy(MOTOR_ID_5) != 0U)) {
        return -1;
    }
    return MotorCtrl_MoveSteps(MOTOR_ID_5, dir, CLOTH_GRIP_RPM, steps);
}

static int cloth_grip_wait_done(uint32_t *wait_ms)
{
    MotorStatusSnapshot_t status;

    /*
     * 等待夹爪运动完成后，再额外等待 PLACE_CLOTH_GRIP_DELAY_MS，
     * 给机械夹紧/松开动作留出稳定时间。
     */
    if (wait_ms == 0) {
        return -1;
    }
    if (MotorCtrl_GetStatus(MOTOR_ID_5, &status) != 0) {
        return -1;
    }
    if (status.mode == MOTOR_MODE_ESTOP) {
        return -1;
    }
    if (cloth_motor_mode_busy(status.mode) != 0U) {
        return 0;
    }
    if (*wait_ms == 0U) {
        *wait_ms = PLACE_CLOTH_GRIP_DELAY_MS;
        if (*wait_ms == 0U) {
            return 1;
        }
        return 0;
    }

    (*wait_ms)--;
    return (*wait_ms == 0U) ? 1 : 0;
}
#endif

static int cloth_holder_engage(uint32_t *wait_ms)
{
#if CLOTH_HOLDER_USE_ELECTROMAGNET
    if (wait_ms == 0) {
        return -1;
    }
    if (Electromagnet_Set(1U) != 0) {
        return -1;
    }
    *wait_ms = PLACE_CLOTH_GRIP_DELAY_MS;
    return 0;
#else
    (void)wait_ms;
    return cloth_grip_start(MOTOR_DIR_CW, CLOTH_GRIP_CLOSE_STEPS);
#endif
}

static int cloth_holder_release(uint32_t *wait_ms)
{
#if CLOTH_HOLDER_USE_ELECTROMAGNET
    if (wait_ms == 0) {
        return -1;
    }
    if (Electromagnet_Set(0U) != 0) {
        return -1;
    }
    *wait_ms = PLACE_CLOTH_GRIP_DELAY_MS;
    return 0;
#else
    (void)wait_ms;
    return cloth_grip_start(MOTOR_DIR_CCW, CLOTH_GRIP_OPEN_STEPS);
#endif
}

static int cloth_holder_wait_done(uint32_t *wait_ms)
{
#if CLOTH_HOLDER_USE_ELECTROMAGNET
    if (wait_ms == 0) {
        return -1;
    }
    if (*wait_ms > 0U) {
        (*wait_ms)--;
    }
    return (*wait_ms == 0U) ? 1 : 0;
#else
    return cloth_grip_wait_done(wait_ms);
#endif
}

static void cloth_fault_report_magnet_hold(void)
{
#if CLOTH_HOLDER_USE_ELECTROMAGNET
    if (Electromagnet_IsOn() != 0U) {
        (void)UartConsole_QueueLineDropOldest("MAGNET=ON HOLD_FOR_MANUAL", 0);
    }
#endif
}

static int cloth_x_axis_wait_done(int32_t target, uint32_t tol)
{
    EncoderStatus_t enc;
    EncMove_t *move = &g_enc_move[MOTOR_ID_1];

    /*
     * X 轴到位判断单独处理：
     * 到达目标附近时直接急停 M1，并把编码器目标运动标记为完成。
     */
    if (EncoderCtrl_GetStatus(MOTOR_ID_1, &enc) != 0) {
        return -1;
    }
    if (abs_diff_i32(target, enc.count_total) <= tol) {
        (void)MotorCtrl_EStop(MOTOR_ID_1);
        move->active = 0U;
        move->done = 1U;
        move->fault = 0U;
        move->target_count = target;
        move->last_count = enc.count_total;
        move->tol = tol;
        move->still_ms = 0U;
        move->stall_boost_active = 0U;
        move->wrong_dir_count = 0U;
        move->wrong_dir_accum = 0U;
        return 1;
    }
    if (move->target_count != target) {
        return -1;
    }
    if ((move->count_increases != 0U) && (enc.count_total >= target)) {
        (void)MotorCtrl_EStop(MOTOR_ID_1);
        move->active = 0U;
        move->done = 1U;
        move->fault = 0U;
        move->last_count = enc.count_total;
        move->stall_boost_active = 0U;
        move->wrong_dir_count = 0U;
        move->wrong_dir_accum = 0U;
        return 1;
    }
    if ((move->count_increases == 0U) && (enc.count_total <= target)) {
        (void)MotorCtrl_EStop(MOTOR_ID_1);
        move->active = 0U;
        move->done = 1U;
        move->fault = 0U;
        move->last_count = enc.count_total;
        move->stall_boost_active = 0U;
        move->wrong_dir_count = 0U;
        move->wrong_dir_accum = 0U;
        return 1;
    }

    if (move->target_count == target) {
        if (move->fault != 0U) {
            return -1;
        }
        if ((move->active == 0U) && (move->done != 0U)) {
            return 1;
        }
    }

    return 0;
}

static int cloth_axis_wait_done(MotorId_t motor_id, int32_t target, uint32_t tol)
{
    EncoderStatus_t enc;
    EncMove_t *move;

    /* 通用轴到位判断。M1 使用更严格的 X 轴专用处理，其余轴走这里。 */
    if (motor_id >= MOTOR_COUNT) {
        return -1;
    }

    if (motor_id == MOTOR_ID_1) {
        return cloth_x_axis_wait_done(target, tol);
    }

    move = &g_enc_move[motor_id];
    if (move->target_count == target) {
        if (move->fault != 0U) {
            return -1;
        }
        if ((move->active == 0U) && (move->done != 0U)) {
            return 1;
        }
    }

    if (place_cloth_motor_busy(motor_id) != 0U) {
        return 0;
    }
    if (EncoderCtrl_GetStatus(motor_id, &enc) != 0) {
        return -1;
    }
    if (abs_diff_i32(target, enc.count_total) <= tol) {
        return 1;
    }
    if (move->target_count != target) {
        return -1;
    }
    if ((move->count_increases != 0U) && (enc.count_total >= target)) {
        move->done = 1U;
        move->stall_boost_active = 0U;
        move->wrong_dir_count = 0U;
        move->wrong_dir_accum = 0U;
        return 1;
    }
    if ((move->count_increases == 0U) && (enc.count_total <= target)) {
        move->done = 1U;
        move->stall_boost_active = 0U;
        move->wrong_dir_count = 0U;
        move->wrong_dir_accum = 0U;
        return 1;
    }

    return -1;
}

static int cloth_y_home_wait_done(void)
{
    RevExKeyGuard_t *guard;

    /* 等待 Y 轴回零完成。Y 轴使用 M3 对应的 EX_KEY3 作为低电平停止条件。 */
    if (g_place_cloth.y_home_done != 0U) {
        g_place_cloth.y_home_done = 0U;
        return 1;
    }
    if (g_take_cloth.y_home_done != 0U) {
        g_take_cloth.y_home_done = 0U;
        return 1;
    }
    if (place_cloth_motor_busy(MOTOR_ID_3) != 0U) {
        return 0;
    }

    guard = rev_exkey_guard_by_motor(MOTOR_ID_3);
    if (guard == 0) {
        return -1;
    }
    return (rev_exkey_is_blocked_low(guard) != 0U) ? 1 : -1;
}

static int take_cloth_y_home_wait_done(void)
{
    RevExKeyGuard_t *guard;

    /* 取衣流程专用的 Y 轴回零等待，避免和放衣流程状态混用。 */
    if (g_take_cloth.y_home_done != 0U) {
        g_take_cloth.y_home_done = 0U;
        return 1;
    }
    if (place_cloth_motor_busy(MOTOR_ID_3) != 0U) {
        return 0;
    }

    guard = rev_exkey_guard_by_motor(MOTOR_ID_3);
    if (guard == 0) {
        return -1;
    }
    return (rev_exkey_is_blocked_low(guard) != 0U) ? 1 : -1;
}

static void place_cloth_fail(const char *msg)
{
    /* 放衣流程失败：急停相关运动、清任务、设置故障事件并输出错误文本。 */
    place_cloth_stop_motion(1U);
    place_cloth_cancel();
    cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, 0xFFU);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
    cloth_fault_report_magnet_hold();
}

static void place_cloth_start_next(PlaceClothStep_t step)
{
    g_place_cloth.step = step;
    g_place_cloth.wait_motor = MOTOR_ID_INVALID;
    g_place_cloth.wait_ms = 0U;
}

static uint32_t take_laser_communication_fault_count(const LaserRangeStatus_t *status)
{
    if (status == 0) {
        return 0U;
    }
    return status->timeout_count +
           status->crc_error_count +
           status->protocol_error_count +
           status->exception_count +
           status->uart_error_count +
           status->tx_error_count;
}

static void take_laser_scan_reset(void)
{
    LaserRangeStatus_t status;

    memset(&g_take_laser_scan, 0, sizeof(g_take_laser_scan));
    if (LaserRange_GetStatus(&status)) {
        g_take_laser_scan.ready_response_base = status.response_count;
        g_take_laser_scan.ready_fault_base =
            take_laser_communication_fault_count(&status);
        g_take_laser_scan.comm_last_response_count = status.response_count;
    }
    LaserRange_FlushSamples();
}

static const char *cloth_laser_owner_text(void)
{
    return (g_take_laser_scan.owner == CLOTH_LASER_OWNER_PLACE) ?
           "PLACE_CLOTH" : "TAKE_CLOTH";
}

static void cloth_laser_begin(ClothLaserOwner_t owner,
                              uint32_t target_slot,
                              int32_t center_offset_counts,
                              int32_t gate_count,
                              uint8_t direct_search)
{
    take_laser_scan_reset();
    g_take_laser_scan.owner = owner;
    g_take_laser_scan.target_slot = target_slot;
    g_take_laser_scan.center_offset_counts = center_offset_counts;
    g_take_laser_scan.gate_count = gate_count;
    g_take_laser_scan.direct_search = direct_search;
}

static void take_laser_recognition_arm(int32_t encoder_count)
{
    char msg[160];

    g_take_laser_scan.filter_count = 0U;
    g_take_laser_scan.filter_index = 0U;
    g_take_laser_scan.exit_count = 0U;
    g_take_laser_scan.recognition_enabled = 1U;
    g_take_laser_scan.board_state = TAKE_LASER_BOARD_SEARCH;
    g_take_laser_scan.last_spread_mm = 0U;
    g_take_laser_scan.board_min_mm = 0U;
    g_take_laser_scan.board_max_mm = 0U;
    g_take_laser_scan.scan_ms = 0U;
    g_take_laser_scan.debug_last_ms = 0U;
    g_take_laser_scan.board_enter_x = 0L;
    g_take_laser_scan.board_exit_start_x = 0L;
    LaserRange_FlushSamples();

    (void)snprintf(msg,
                   sizeof(msg),
                   "OK %s LASER_ARM EC1=%ld GATE=%ld APPROACH=%.0f SEARCH=%.0f PRECISION=%.0f BOARD=%u-%uMM WIDTH=%ld-%ld",
                   cloth_laser_owner_text(),
                   (long)encoder_count,
                   (long)g_take_laser_scan.gate_count,
                   (double)TAKE_LASER_APPROACH_RPM,
                   (double)TAKE_LASER_SCAN_RPM,
                   (double)TAKE_LASER_PRECISION_RPM,
                   (unsigned int)TAKE_LASER_BOARD_DISTANCE_MIN_MM,
                   (unsigned int)TAKE_LASER_BOARD_DISTANCE_MAX_MM,
                   (long)TAKE_LASER_BOARD_MIN_WIDTH_COUNTS,
                   (long)TAKE_LASER_BOARD_MAX_WIDTH_COUNTS);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
}

static uint8_t take_laser_board_distance_valid(uint16_t distance_mm)
{
    return ((distance_mm >= TAKE_LASER_BOARD_DISTANCE_MIN_MM) &&
            (distance_mm <= TAKE_LASER_BOARD_DISTANCE_MAX_MM)) ? 1U : 0U;
}

static uint8_t take_laser_board_sample_valid(const LaserRangeSample_t *sample)
{
    if (sample == 0) {
        return 0U;
    }
    return (sample->distance_valid &&
            (sample->confidence >= TAKE_LASER_PLATE_CONFIDENCE_MIN) &&
            (take_laser_board_distance_valid(sample->distance_mm) != 0U)) ? 1U : 0U;
}

static uint16_t take_laser_median3(const uint16_t *values)
{
    uint16_t a = values[0];
    uint16_t b = values[1];
    uint16_t c = values[2];
    uint16_t temp;

    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }
    if (b > c) {
        temp = b;
        b = c;
        c = temp;
    }
    if (a > b) {
        b = a;
    }
    return b;
}

static uint8_t place_laser2_calibration_parameters_valid(void)
{
    if (PLACE_LASER2_CAL_EC_A != 0L) {
        return 0U;
    }
    if (abs_f32(PLACE_LASER2_CAL_D_A_MM -
                PLACE_LASER2_X_ZERO_DISTANCE_MM) >= 0.0001f) {
        return 0U;
    }
    if ((PLACE_LASER2_CAL_D_B_MM <= PLACE_LASER2_CAL_D_A_MM) ||
        (PLACE_LASER2_CAL_EC_B <= PLACE_LASER2_CAL_EC_A)) {
        return 0U;
    }
    return 1U;
}

static int place_laser2_calculate_target(uint16_t distance_mm,
                                         int32_t *out_target,
                                         float *out_k,
                                         float *out_move_mm)
{
    int64_t ec_span;
    int64_t target_i64;
    int32_t ec_min;
    int32_t ec_max;
    float distance_span;
    float distance_min;
    float distance_max;
    float k;
    float move_mm;
    float target_f;

    /*
     * 激光读数是激光到衣架的总距离，必须先减去激光到X零点工作中心的固定距离。
     * 标定点A固定为D_ZERO/EC1=0；K=(D_B-D_A)/(EC_B-EC_A)，
     * X移动距离=D_RAW-D_ZERO，EC目标=X移动距离/K。
     */
    if ((out_target == 0) || (out_k == 0) || (out_move_mm == 0)) {
        return -1;
    }
    if (PLACE_LASER2_CALIBRATION_VALID == 0U) {
        return -2;
    }
    if (place_laser2_calibration_parameters_valid() == 0U) {
        return -3;
    }

    ec_span = (int64_t)PLACE_LASER2_CAL_EC_B -
              (int64_t)PLACE_LASER2_CAL_EC_A;
    distance_span = PLACE_LASER2_CAL_D_B_MM -
                    PLACE_LASER2_CAL_D_A_MM;
    if ((ec_span == 0) || (abs_f32(distance_span) < 0.0001f)) {
        return -3;
    }

    distance_min = (PLACE_LASER2_CAL_D_A_MM < PLACE_LASER2_CAL_D_B_MM) ?
                   PLACE_LASER2_CAL_D_A_MM : PLACE_LASER2_CAL_D_B_MM;
    distance_max = (PLACE_LASER2_CAL_D_A_MM > PLACE_LASER2_CAL_D_B_MM) ?
                   PLACE_LASER2_CAL_D_A_MM : PLACE_LASER2_CAL_D_B_MM;
    move_mm = (float)distance_mm - PLACE_LASER2_X_ZERO_DISTANCE_MM;
    if (move_mm < 0.0f) {
        return -6;
    }
    if (((float)distance_mm < distance_min) ||
        ((float)distance_mm > distance_max)) {
        return -4;
    }

    k = distance_span / (float)ec_span;
    if (k < 0.000001f) {
        return -3;
    }
    target_f = move_mm / k;
    target_i64 = (target_f >= 0.0f) ?
                 (int64_t)(target_f + 0.5f) :
                 (int64_t)(target_f - 0.5f);
    if ((target_i64 < 0) || (target_i64 > (int64_t)INT32_MAX)) {
        return -5;
    }

    ec_min = (PLACE_LASER2_CAL_EC_A < PLACE_LASER2_CAL_EC_B) ?
             PLACE_LASER2_CAL_EC_A : PLACE_LASER2_CAL_EC_B;
    ec_max = (PLACE_LASER2_CAL_EC_A > PLACE_LASER2_CAL_EC_B) ?
             PLACE_LASER2_CAL_EC_A : PLACE_LASER2_CAL_EC_B;
    if ((target_i64 < (int64_t)ec_min) ||
        (target_i64 > (int64_t)ec_max)) {
        return -5;
    }

    *out_target = (int32_t)target_i64;
    *out_k = k;
    *out_move_mm = move_mm;
    return 0;
}

static int place_laser2_poll_stable_distance(uint16_t *out_distance,
                                              uint16_t *out_spread)
{
    LaserRangeSample_t sample;
    uint16_t min_mm;
    uint16_t max_mm;
    uint8_t i;

    if ((out_distance == 0) || (out_spread == 0)) {
        return -1;
    }

    while (LaserRange2_PollSample(&sample)) {
        if ((!sample.distance_valid) ||
            (sample.confidence < PLACE_LASER2_CONFIDENCE_MIN)) {
            g_place_cloth.laser2_sample_count = 0U;
            continue;
        }

        if (g_place_cloth.laser2_sample_count < PLACE_LASER2_FILTER_SAMPLES) {
            g_place_cloth.laser2_samples[g_place_cloth.laser2_sample_count] =
                sample.distance_mm;
            g_place_cloth.laser2_sample_count++;
        }
        if (g_place_cloth.laser2_sample_count < PLACE_LASER2_FILTER_SAMPLES) {
            continue;
        }

        min_mm = g_place_cloth.laser2_samples[0];
        max_mm = min_mm;
        for (i = 1U; i < PLACE_LASER2_FILTER_SAMPLES; i++) {
            if (g_place_cloth.laser2_samples[i] < min_mm) {
                min_mm = g_place_cloth.laser2_samples[i];
            }
            if (g_place_cloth.laser2_samples[i] > max_mm) {
                max_mm = g_place_cloth.laser2_samples[i];
            }
        }
        *out_spread = (uint16_t)(max_mm - min_mm);
        if (*out_spread <= PLACE_LASER2_STABLE_SPREAD_MM) {
            *out_distance = take_laser_median3(g_place_cloth.laser2_samples);
            return 1;
        }

        g_place_cloth.laser2_samples[0] =
            g_place_cloth.laser2_samples[1];
        g_place_cloth.laser2_samples[1] =
            g_place_cloth.laser2_samples[2];
        g_place_cloth.laser2_sample_count =
            PLACE_LASER2_FILTER_SAMPLES - 1U;
    }

    if ((uint32_t)(HAL_GetTick() - g_place_cloth.laser2_started_ms) >=
        PLACE_LASER2_SAMPLE_TIMEOUT_MS) {
        return -2;
    }
    return 0;
}

static int take_laser_window_push(const LaserRangeSample_t *sample,
                                  int32_t encoder_count,
                                  uint16_t *filtered_mm,
                                  uint16_t *spread_mm,
                                  int32_t *window_start_x)
{
    uint16_t min_mm;
    uint16_t max_mm;
    uint8_t i;

    if ((sample == 0) || (filtered_mm == 0) ||
        (spread_mm == 0) || (window_start_x == 0)) {
        return 0;
    }
    if (take_laser_board_sample_valid(sample) == 0U) {
        g_take_laser_scan.filter_count = 0U;
        g_take_laser_scan.filter_index = 0U;
        g_take_laser_scan.last_spread_mm = 0U;
        return TAKE_LASER_WINDOW_BACKGROUND;
    }

    g_take_laser_scan.filter_mm[g_take_laser_scan.filter_index] = sample->distance_mm;
    g_take_laser_scan.filter_x[g_take_laser_scan.filter_index] = encoder_count;
    g_take_laser_scan.filter_index =
        (uint8_t)((g_take_laser_scan.filter_index + 1U) % TAKE_LASER_FILTER_SAMPLES);
    if (g_take_laser_scan.filter_count < TAKE_LASER_FILTER_SAMPLES) {
        g_take_laser_scan.filter_count++;
    }
    if (g_take_laser_scan.filter_count < TAKE_LASER_FILTER_SAMPLES) {
        return TAKE_LASER_WINDOW_WARMUP;
    }

    min_mm = g_take_laser_scan.filter_mm[0];
    max_mm = min_mm;
    for (i = 1U; i < TAKE_LASER_FILTER_SAMPLES; i++) {
        if (g_take_laser_scan.filter_mm[i] < min_mm) {
            min_mm = g_take_laser_scan.filter_mm[i];
        }
        if (g_take_laser_scan.filter_mm[i] > max_mm) {
            max_mm = g_take_laser_scan.filter_mm[i];
        }
    }

    *spread_mm = (uint16_t)(max_mm - min_mm);
    g_take_laser_scan.last_spread_mm = *spread_mm;
    *filtered_mm = take_laser_median3(g_take_laser_scan.filter_mm);
    *window_start_x = g_take_laser_scan.filter_x[g_take_laser_scan.filter_index];
    return (*spread_mm <= TAKE_LASER_STABLE_SPREAD_MM) ?
           TAKE_LASER_WINDOW_STABLE : TAKE_LASER_WINDOW_TRANSITION;
}

static const char *take_laser_scan_state_text(void)
{
    return (g_take_laser_scan.board_state == TAKE_LASER_BOARD_TARGET) ?
           "TARGET" : "SEARCH";
}

static const char *take_laser_sample_class_text(TakeLaserSampleClass_t sample_class)
{
    switch (sample_class) {
        case TAKE_LASER_SAMPLE_BACKGROUND: return "BACKGROUND";
        case TAKE_LASER_SAMPLE_TRANSITION: return "TRANSITION";
        case TAKE_LASER_SAMPLE_PLATE: return "PLATE";
        default: return "WARMUP";
    }
}

static void take_laser_debug_log(const LaserRangeSample_t *sample,
                                 int window_result,
                                 uint16_t filtered_mm,
                                 uint16_t spread_mm,
                                 int32_t encoder_count,
                                 TakeLaserSampleClass_t sample_class)
{
    char msg[256];
    const char *window_text;
    uint32_t now_ms;
    uint8_t plate_valid;

    if ((sample == 0) || !UartConsole_GetTextOutputEnabled()) {
        return;
    }
    now_ms = HAL_GetTick();
    if ((g_take_laser_scan.debug_last_ms != 0U) &&
        ((uint32_t)(now_ms - g_take_laser_scan.debug_last_ms) < TAKE_LASER_DEBUG_LOG_PERIOD_MS)) {
        return;
    }
    g_take_laser_scan.debug_last_ms = now_ms;
    switch ((TakeLaserWindowResult_t)window_result) {
        case TAKE_LASER_WINDOW_STABLE: window_text = "STABLE"; break;
        case TAKE_LASER_WINDOW_TRANSITION: window_text = "TRANSITION"; break;
        case TAKE_LASER_WINDOW_BACKGROUND: window_text = "BACKGROUND"; break;
        default: window_text = "WARMUP"; break;
    }
    plate_valid = take_laser_board_sample_valid(sample);
    (void)snprintf(msg,
                   sizeof(msg),
                   "LASER RAW=%u CONF=%u DIST_VALID=%u PLATE_VALID=%u WIN=%s CLASS=%s FILT=%u SPREAD=%u STATE=%s TARGET=%u SLOT=%u ENTER=%ld EXIT_CNT=%u MIN=%u MAX=%u X=%ld",
                   (unsigned int)sample->distance_mm,
                   (unsigned int)sample->confidence,
                   sample->distance_valid ? 1U : 0U,
                   plate_valid,
                   window_text,
                   take_laser_sample_class_text(sample_class),
                   (unsigned int)filtered_mm,
                   (unsigned int)spread_mm,
                   take_laser_scan_state_text(),
                   (g_take_laser_scan.board_state == TAKE_LASER_BOARD_TARGET) ? 1U : 0U,
                   (unsigned int)g_take_laser_scan.found_slots,
                   (long)g_take_laser_scan.board_enter_x,
                   (unsigned int)g_take_laser_scan.exit_count,
                   (unsigned int)g_take_laser_scan.board_min_mm,
                   (unsigned int)g_take_laser_scan.board_max_mm,
                   (long)encoder_count);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
}

static void take_laser_board_search_reset(void)
{
    g_take_laser_scan.filter_count = 0U;
    g_take_laser_scan.filter_index = 0U;
    g_take_laser_scan.exit_count = 0U;
    g_take_laser_scan.board_state = TAKE_LASER_BOARD_SEARCH;
    g_take_laser_scan.last_spread_mm = 0U;
    g_take_laser_scan.board_min_mm = 0U;
    g_take_laser_scan.board_max_mm = 0U;
    g_take_laser_scan.board_enter_x = 0L;
    g_take_laser_scan.board_exit_start_x = 0L;
    g_take_laser_scan.scan_ms = 0U;
    LaserRange_FlushSamples();
}

static void take_laser_board_track_distance(uint16_t distance_mm)
{
    g_take_laser_scan.exit_count = 0U;
    g_take_laser_scan.board_exit_start_x = 0L;
    if ((g_take_laser_scan.board_min_mm == 0U) ||
        (distance_mm < g_take_laser_scan.board_min_mm)) {
        g_take_laser_scan.board_min_mm = distance_mm;
    }
    if (distance_mm > g_take_laser_scan.board_max_mm) {
        g_take_laser_scan.board_max_mm = distance_mm;
    }
}

static int take_laser_board_enter(int32_t window_start_x,
                                  uint16_t distance_mm,
                                  uint16_t spread_mm)
{
    char msg[144];
    uint8_t i;

    if (MotorCtrl_SetSpeed(MOTOR_ID_1, TAKE_LASER_PRECISION_RPM) != 0) {
        return -1;
    }
    g_take_laser_scan.board_state = TAKE_LASER_BOARD_TARGET;
    g_take_laser_scan.board_enter_x = window_start_x;
    g_take_laser_scan.board_min_mm = g_take_laser_scan.filter_mm[0];
    g_take_laser_scan.board_max_mm = g_take_laser_scan.filter_mm[0];
    for (i = 1U; i < TAKE_LASER_FILTER_SAMPLES; i++) {
        if (g_take_laser_scan.filter_mm[i] < g_take_laser_scan.board_min_mm) {
            g_take_laser_scan.board_min_mm = g_take_laser_scan.filter_mm[i];
        }
        if (g_take_laser_scan.filter_mm[i] > g_take_laser_scan.board_max_mm) {
            g_take_laser_scan.board_max_mm = g_take_laser_scan.filter_mm[i];
        }
    }
    g_take_laser_scan.exit_count = 0U;
    g_take_laser_scan.board_exit_start_x = 0L;
    g_take_laser_scan.scan_ms = 0U;
    (void)snprintf(msg,
                   sizeof(msg),
                   "OK %s LASER_TARGET_ENTER X=%ld DIST=%u SPREAD=%u RPM=%.0f",
                   cloth_laser_owner_text(),
                   (long)window_start_x,
                   (unsigned int)distance_mm,
                   (unsigned int)spread_mm,
                   (double)TAKE_LASER_PRECISION_RPM);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
    return 0;
}

static int take_laser_board_complete(int32_t exit_x, int32_t *target_x)
{
    char msg[192];
    int32_t enter_x = g_take_laser_scan.board_enter_x;
    int32_t width = exit_x - enter_x;
    int64_t center = ((int64_t)enter_x + (int64_t)exit_x) / 2LL;
    uint8_t valid = ((width >= TAKE_LASER_BOARD_MIN_WIDTH_COUNTS) &&
                     (width <= TAKE_LASER_BOARD_MAX_WIDTH_COUNTS)) ? 1U : 0U;

    center += g_take_laser_scan.center_offset_counts;
    if (center < 0LL) {
        center = 0LL;
    }
    if (center > (int64_t)INT32_MAX) {
        center = (int64_t)INT32_MAX;
    }
    if ((valid != 0U) && (g_take_laser_scan.found_slots < 0xFFU)) {
        g_take_laser_scan.found_slots++;
    }
    (void)snprintf(msg,
                   sizeof(msg),
                   "OK %s LASER_TARGET_EXIT ENTER=%ld EXIT=%ld WIDTH=%ld MIN=%u MAX=%u VALID=%u FOUND=%u WANT=%lu CENTER=%ld",
                   cloth_laser_owner_text(),
                   (long)enter_x,
                   (long)exit_x,
                   (long)width,
                   (unsigned int)g_take_laser_scan.board_min_mm,
                   (unsigned int)g_take_laser_scan.board_max_mm,
                   (unsigned int)valid,
                   (unsigned int)g_take_laser_scan.found_slots,
                   (unsigned long)g_take_laser_scan.target_slot,
                   (long)center);
    (void)UartConsole_QueueLineDropOldest(msg, 0);

    if ((valid != 0U) &&
        (g_take_laser_scan.found_slots >= g_take_laser_scan.target_slot)) {
        if (target_x != 0) {
            *target_x = (int32_t)center;
        }
        return 1;
    }
    if (valid == 0U) {
        if (MotorCtrl_SetSpeed(MOTOR_ID_1, TAKE_LASER_SCAN_RPM) != 0) {
            return -1;
        }
    }
    take_laser_board_search_reset();
    (void)snprintf(msg,
                   sizeof(msg),
                   "OK %s LASER_SEARCH RESUME RPM=%.0f FOUND=%u WANT=%lu",
                   cloth_laser_owner_text(),
                   (double)((valid != 0U) ? TAKE_LASER_PRECISION_RPM : TAKE_LASER_SCAN_RPM),
                   (unsigned int)g_take_laser_scan.found_slots,
                   (unsigned long)g_take_laser_scan.target_slot);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
    return 0;
}

static int take_laser_board_process_background(int32_t encoder_count,
                                               int32_t *target_x)
{
    if (g_take_laser_scan.board_state != TAKE_LASER_BOARD_TARGET) {
        return 0;
    }
    if (g_take_laser_scan.exit_count == 0U) {
        g_take_laser_scan.board_exit_start_x = encoder_count;
    }
    if (g_take_laser_scan.exit_count < TAKE_LASER_BACKGROUND_CONFIRM_FRAMES) {
        g_take_laser_scan.exit_count++;
    }
    if (g_take_laser_scan.exit_count < TAKE_LASER_BACKGROUND_CONFIRM_FRAMES) {
        return 0;
    }
    return take_laser_board_complete(g_take_laser_scan.board_exit_start_x,
                                     target_x);
}

static void take_cloth_cancel(void)
{
    /* 清空取衣任务状态，不主动停电机；停运动由 place_cloth_stop_motion() 负责。 */
    g_take_cloth.active = 0U;
    g_take_cloth.step = TAKE_CLOTH_IDLE;
    g_take_cloth.wait_motor = MOTOR_ID_INVALID;
    g_take_cloth.wait_ms = 0U;
    g_take_cloth.y_home_done = 0U;
    g_take_cloth.m2_home_started = 0U;
    g_take_cloth.slot = 0U;
    g_take_cloth.relative_target = 0L;
    take_laser_scan_reset();
}

static void take_cloth_fail_code(const char *msg, uint8_t fault_code)
{
    /* 取衣流程失败：急停相关运动、清任务、设置故障事件并输出错误文本。 */
    place_cloth_stop_motion(1U);
    take_cloth_cancel();
    cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, fault_code);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
    cloth_fault_report_magnet_hold();
}

static void take_cloth_fail(const char *msg)
{
    take_cloth_fail_code(msg, 0xFFU);
}

static void cloth_laser_fail_diagnostic(const char *reason, uint32_t silence_ms)
{
    ClothLaserOwner_t owner = g_take_laser_scan.owner;
    const char *owner_text = cloth_laser_owner_text();

    if ((reason != 0) && LaserRange_GetStatus(&g_take_laser_fault_status)) {
        (void)snprintf(g_take_laser_fault_msg,
                       sizeof(g_take_laser_fault_msg),
                       "ERR 204 %s_%s SILENCE=%lu REQ=%lu RESP=%lu TIMEOUT=%lu PROTO_ERR=%lu CRC_ERR=%lu UART_ERR=%lu",
                       owner_text,
                       reason,
                       (unsigned long)silence_ms,
                       (unsigned long)g_take_laser_fault_status.request_count,
                       (unsigned long)g_take_laser_fault_status.response_count,
                       (unsigned long)g_take_laser_fault_status.timeout_count,
                       (unsigned long)g_take_laser_fault_status.protocol_error_count,
                       (unsigned long)g_take_laser_fault_status.crc_error_count,
                       (unsigned long)g_take_laser_fault_status.uart_error_count);
    } else {
        (void)snprintf(g_take_laser_fault_msg,
                       sizeof(g_take_laser_fault_msg),
                       "ERR 204 %s_%s SILENCE=%lu",
                       owner_text,
                       (reason != 0) ? reason : "LASER_STATUS_FAIL",
                       (unsigned long)silence_ms);
    }
    place_cloth_stop_motion(1U);
    if (owner == CLOTH_LASER_OWNER_PLACE) {
        place_cloth_cancel();
        take_laser_scan_reset();
        cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, TAKE_CLOTH_FAULT_SENSOR);
    } else {
        take_cloth_cancel();
        cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, TAKE_CLOTH_FAULT_SENSOR);
    }
    (void)UartConsole_QueueLineDropOldest(g_take_laser_fault_msg, 0);
    cloth_fault_report_magnet_hold();
}

static void cloth_laser_fail_regular(uint16_t code,
                                     const char *reason,
                                     uint8_t fault_code)
{
    ClothLaserOwner_t owner = g_take_laser_scan.owner;

    (void)snprintf(g_take_laser_fault_msg,
                   sizeof(g_take_laser_fault_msg),
                   "ERR %u %s_%s",
                   (unsigned int)code,
                   cloth_laser_owner_text(),
                   (reason != 0) ? reason : "LASER_FAIL");
    place_cloth_stop_motion(1U);
    if (owner == CLOTH_LASER_OWNER_PLACE) {
        place_cloth_cancel();
        take_laser_scan_reset();
        cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, fault_code);
    } else {
        take_cloth_cancel();
        cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, fault_code);
    }
    (void)UartConsole_QueueLineDropOldest(g_take_laser_fault_msg, 0);
    cloth_fault_report_magnet_hold();
}

static void cloth_laser_handle_error(ClothLaserResult_t result)
{
    switch (result) {
        case CLOTH_LASER_ERR_READY_TIMEOUT:
            cloth_laser_fail_diagnostic("LASER_READY_TIMEOUT",
                                        g_take_laser_scan.wait_ms);
            break;
        case CLOTH_LASER_ERR_STATUS:
            cloth_laser_fail_diagnostic("LASER_STATUS_FAIL",
                                        g_take_laser_scan.comm_silence_ms);
            break;
        case CLOTH_LASER_ERR_OFFLINE:
            cloth_laser_fail_diagnostic("LASER_OFFLINE",
                                        g_take_laser_scan.comm_silence_ms);
            break;
        case CLOTH_LASER_ERR_START_LIMIT:
            cloth_laser_fail_regular(212U, "LASER_SCAN_LIMIT", TAKE_CLOTH_FAULT_SLOT_EMPTY);
            break;
        case CLOTH_LASER_ERR_START:
            cloth_laser_fail_regular(203U, "LASER_SCAN_START_FAIL", 0xFFU);
            break;
        case CLOTH_LASER_ERR_LIMIT:
            cloth_laser_fail_regular(212U, "SLOT_NOT_FOUND_LIMIT", TAKE_CLOTH_FAULT_SLOT_EMPTY);
            break;
        case CLOTH_LASER_ERR_MOTOR:
            cloth_laser_fail_regular(203U, "LASER_SCAN_MOTOR", 0xFFU);
            break;
        case CLOTH_LASER_ERR_STALL:
            cloth_laser_fail_regular(206U, "LASER_SCAN_STALL", 0xFFU);
            break;
        case CLOTH_LASER_ERR_GATE_TIMEOUT:
            cloth_laser_fail_regular(208U, "LASER_GATE_TIMEOUT", TAKE_CLOTH_FAULT_SLOT_EMPTY);
            break;
        case CLOTH_LASER_ERR_SCAN_SPEED:
            cloth_laser_fail_regular(203U, "LASER_SCAN_SPEED_FAIL", 0xFFU);
            break;
        case CLOTH_LASER_ERR_PRECISION_SPEED:
            cloth_laser_fail_regular(203U, "LASER_PRECISION_SPEED_FAIL", 0xFFU);
            break;
        case CLOTH_LASER_ERR_SEARCH_SPEED:
            cloth_laser_fail_regular(203U, "LASER_SEARCH_SPEED_FAIL", 0xFFU);
            break;
        case CLOTH_LASER_ERR_TARGET_TIMEOUT:
            cloth_laser_fail_regular(208U, "LASER_TARGET_TIMEOUT", TAKE_CLOTH_FAULT_SLOT_EMPTY);
            break;
        case CLOTH_LASER_ERR_SEARCH_TIMEOUT:
            cloth_laser_fail_regular(208U, "SLOT_NOT_FOUND_TIMEOUT", TAKE_CLOTH_FAULT_SLOT_EMPTY);
            break;
        default:
            cloth_laser_fail_regular(203U, "LASER_UNKNOWN_FAIL", 0xFFU);
            break;
    }
}

static ClothLaserResult_t cloth_laser_ready_tick(void)
{
    LaserRangeStatus_t laser_status;

    if (g_take_laser_scan.wait_ms < 0xFFFFFFFFUL) {
        g_take_laser_scan.wait_ms++;
    }
    if (LaserRange_GetStatus(&laser_status)) {
        uint32_t fault_count =
            take_laser_communication_fault_count(&laser_status);

        if (fault_count != g_take_laser_scan.ready_fault_base) {
            g_take_laser_scan.ready_fault_base = fault_count;
            g_take_laser_scan.ready_response_base = laser_status.response_count;
        } else if (laser_status.communication_online &&
                   ((uint32_t)(laser_status.response_count -
                               g_take_laser_scan.ready_response_base) >=
                    TAKE_LASER_READY_FRESH_RESPONSES)) {
            g_take_laser_scan.comm_last_response_count = laser_status.response_count;
            g_take_laser_scan.comm_silence_ms = 0U;
            return CLOTH_LASER_READY;
        }
    }
    return (g_take_laser_scan.wait_ms >= TAKE_LASER_READY_TIMEOUT_MS) ?
           CLOTH_LASER_ERR_READY_TIMEOUT : CLOTH_LASER_PROGRESS;
}

static ClothLaserResult_t cloth_laser_start_scan(void)
{
    MotorStatusSnapshot_t motor_status;
    EncoderStatus_t encoder_status;
    float start_rpm = (g_take_laser_scan.direct_search != 0U) ?
                      TAKE_LASER_SCAN_RPM : TAKE_LASER_APPROACH_RPM;
    char msg[112];

    if ((AxisLimit_CanMoveMotor(MOTOR_ID_1) != 0) ||
        (MotorCtrl_GetStatus(MOTOR_ID_1, &motor_status) != 0) ||
        (EncoderCtrl_GetStatus(MOTOR_ID_1, &encoder_status) != 0)) {
        return CLOTH_LASER_ERR_START_LIMIT;
    }
    if (start_rpm < MotorCtrl_GetRpmMin(motor_status.ppr)) {
        start_rpm = MotorCtrl_GetRpmMin(motor_status.ppr);
    }
    if ((start_rpm <= 0.0f) ||
        (start_rpm > MotorCtrl_GetRpmMax(motor_status.ppr)) ||
        (MotorCtrl_Run(MOTOR_ID_1, MOTOR_DIR_CW, start_rpm) != 0)) {
        return CLOTH_LASER_ERR_START;
    }

    rev_exkey_cancel_if_motor(MOTOR_ID_1);
    g_take_laser_scan.scan_ms = 0U;
    g_take_laser_scan.still_ms = 0U;
    g_take_laser_scan.last_encoder_count = encoder_status.count_total;
    LaserRange_FlushSamples();
    if (g_take_laser_scan.direct_search != 0U) {
        take_laser_recognition_arm(encoder_status.count_total);
    }
    (void)snprintf(msg,
                   sizeof(msg),
                   "OK %s LASER_SCAN START RPM=%.0f GATE=%ld DIRECT=%u",
                   cloth_laser_owner_text(),
                   (double)start_rpm,
                   (long)g_take_laser_scan.gate_count,
                   (unsigned int)g_take_laser_scan.direct_search);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
    return CLOTH_LASER_READY;
}

static ClothLaserResult_t cloth_laser_scan_tick(int32_t *target_x)
{
    MotorStatusSnapshot_t motor_status;
    EncoderStatus_t encoder_status;
    LaserRangeStatus_t laser_status;
    LaserRangeSample_t sample;
    uint16_t filtered_mm;
    uint16_t spread_mm;
    uint8_t processed = 0U;

    if (g_take_laser_scan.scan_ms < 0xFFFFFFFFUL) {
        g_take_laser_scan.scan_ms++;
    }
    if (!LaserRange_GetStatus(&laser_status)) {
        return CLOTH_LASER_ERR_STATUS;
    }
    if (laser_status.response_count != g_take_laser_scan.comm_last_response_count) {
        g_take_laser_scan.comm_last_response_count = laser_status.response_count;
        g_take_laser_scan.comm_silence_ms = 0U;
    } else if (g_take_laser_scan.comm_silence_ms < 0xFFFFFFFFUL) {
        g_take_laser_scan.comm_silence_ms++;
    }
    if (g_take_laser_scan.comm_silence_ms >= TAKE_LASER_COMM_SILENCE_TIMEOUT_MS) {
        return CLOTH_LASER_ERR_OFFLINE;
    }
    if (AxisLimit_CanMoveMotor(MOTOR_ID_1) != 0) {
        return CLOTH_LASER_ERR_LIMIT;
    }
    if ((MotorCtrl_GetStatus(MOTOR_ID_1, &motor_status) != 0) ||
        ((motor_status.mode != MOTOR_MODE_RUN) &&
         (motor_status.mode != MOTOR_MODE_RAMP_UP) &&
         (motor_status.mode != MOTOR_MODE_RAMP_DOWN)) ||
        (EncoderCtrl_GetStatus(MOTOR_ID_1, &encoder_status) != 0)) {
        return CLOTH_LASER_ERR_MOTOR;
    }

    if (encoder_status.count_total == g_take_laser_scan.last_encoder_count) {
        if (g_take_laser_scan.still_ms < 0xFFFFFFFFUL) {
            g_take_laser_scan.still_ms++;
        }
    } else {
        g_take_laser_scan.last_encoder_count = encoder_status.count_total;
        g_take_laser_scan.still_ms = 0U;
    }
    if (g_take_laser_scan.still_ms >= TAKE_LASER_SCAN_STALL_TIMEOUT_MS) {
        return CLOTH_LASER_ERR_STALL;
    }

    if (g_take_laser_scan.recognition_enabled == 0U) {
        LaserRange_FlushSamples();
        if (encoder_status.count_total < g_take_laser_scan.gate_count) {
            if (UartConsole_GetTextOutputEnabled() &&
                ((g_take_laser_scan.scan_ms % TAKE_LASER_GATE_LOG_PERIOD_MS) == 0U)) {
                char msg[112];

                (void)snprintf(msg,
                               sizeof(msg),
                               "%s LASER GATE WAIT EC1=%ld TARGET=%ld T=%lu/%lu",
                               cloth_laser_owner_text(),
                               (long)encoder_status.count_total,
                               (long)g_take_laser_scan.gate_count,
                               (unsigned long)g_take_laser_scan.scan_ms,
                               (unsigned long)TAKE_LASER_GATE_TIMEOUT_MS);
                (void)UartConsole_QueueLineDropOldest(msg, 0);
            }
            return (g_take_laser_scan.scan_ms >= TAKE_LASER_GATE_TIMEOUT_MS) ?
                   CLOTH_LASER_ERR_GATE_TIMEOUT : CLOTH_LASER_PROGRESS;
        }
        if (MotorCtrl_SetSpeed(MOTOR_ID_1, TAKE_LASER_SCAN_RPM) != 0) {
            return CLOTH_LASER_ERR_SCAN_SPEED;
        }
        take_laser_recognition_arm(encoder_status.count_total);
        return CLOTH_LASER_PROGRESS;
    }

    while ((processed < 4U) && LaserRange_PollSample(&sample)) {
        int32_t window_start_x = encoder_status.count_total;
        int window_result;
        int found = 0;
        TakeLaserSampleClass_t sample_class = TAKE_LASER_SAMPLE_WARMUP;

        processed++;
        filtered_mm = 0U;
        spread_mm = 0U;
        window_result = take_laser_window_push(&sample,
                                               encoder_status.count_total,
                                               &filtered_mm,
                                               &spread_mm,
                                               &window_start_x);
        switch ((TakeLaserWindowResult_t)window_result) {
            case TAKE_LASER_WINDOW_STABLE:
                sample_class = TAKE_LASER_SAMPLE_PLATE;
                if (g_take_laser_scan.board_state == TAKE_LASER_BOARD_SEARCH) {
                    if (take_laser_board_enter(window_start_x,
                                               filtered_mm,
                                               spread_mm) != 0) {
                        return CLOTH_LASER_ERR_PRECISION_SPEED;
                    }
                } else {
                    take_laser_board_track_distance(filtered_mm);
                }
                break;
            case TAKE_LASER_WINDOW_TRANSITION:
                sample_class = TAKE_LASER_SAMPLE_TRANSITION;
                if (g_take_laser_scan.board_state == TAKE_LASER_BOARD_TARGET) {
                    take_laser_board_track_distance(sample.distance_mm);
                }
                break;
            case TAKE_LASER_WINDOW_BACKGROUND:
                sample_class = TAKE_LASER_SAMPLE_BACKGROUND;
                found = take_laser_board_process_background(
                    encoder_status.count_total,
                    target_x);
                if (found < 0) {
                    return CLOTH_LASER_ERR_SEARCH_SPEED;
                }
                break;
            default:
                if ((g_take_laser_scan.board_state == TAKE_LASER_BOARD_TARGET) &&
                    (take_laser_board_sample_valid(&sample) != 0U)) {
                    take_laser_board_track_distance(sample.distance_mm);
                }
                break;
        }
        take_laser_debug_log(&sample,
                             window_result,
                             filtered_mm,
                             spread_mm,
                             encoder_status.count_total,
                             sample_class);
        if (found == 1) {
            return CLOTH_LASER_FOUND;
        }
    }

    if (g_take_laser_scan.scan_ms >= TAKE_LASER_SCAN_TIMEOUT_MS) {
        return (g_take_laser_scan.board_state == TAKE_LASER_BOARD_TARGET) ?
               CLOTH_LASER_ERR_TARGET_TIMEOUT : CLOTH_LASER_ERR_SEARCH_TIMEOUT;
    }
    return CLOTH_LASER_PROGRESS;
}

static void take_cloth_start_next(TakeClothStep_t step)
{
    g_take_cloth.step = step;
    g_take_cloth.wait_motor = MOTOR_ID_INVALID;
    g_take_cloth.wait_ms = 0U;
}

static void cloth_continue_after_start_home(void)
{
    /*
     * 取放衣流程的前置 ALL_HOME 完成后，必须由同一条云端指令继续后续动作。
     * 同时兼容 START_HOME 和 WAIT_START_HOME，避免回零完成与状态切换边界导致只回零不继续。
     */
    if ((g_place_cloth.active != 0U) &&
        ((g_place_cloth.step == PLACE_CLOTH_START_HOME) ||
         (g_place_cloth.step == PLACE_CLOTH_WAIT_START_HOME))) {
        place_cloth_start_next(PLACE_CLOTH_M2_INITIAL);
    }

    if ((g_take_cloth.active != 0U) &&
        ((g_take_cloth.step == TAKE_CLOTH_START_HOME) ||
         (g_take_cloth.step == TAKE_CLOTH_WAIT_START_HOME))) {
        take_laser_scan_reset();
        take_cloth_start_next(TAKE_CLOTH_M2_INITIAL);
    }
}

#if !CLOTH_HOLDER_USE_ELECTROMAGNET
static void take_cloth_start_wait(TakeClothStep_t step, uint32_t wait_ms)
{
    g_take_cloth.step = step;
    g_take_cloth.wait_motor = MOTOR_ID_INVALID;
    g_take_cloth.wait_ms = wait_ms;
}
#endif

static RevExKeyGuard_t *rev_exkey_guard_by_motor(MotorId_t motor_id)
{
    uint32_t i;

    for (i = 0U; i < REV_EXKEY_GUARD_COUNT; i++) {
        if (g_rev_exkey_guards[i].motor_id == motor_id) {
            return &g_rev_exkey_guards[i];
        }
    }

    return 0;
}

static RevExKeyGuard_t *rev_exkey_guard_by_action(CmdAction_t action)
{
    switch (action) {
        case CMD_ACTION_REV_EX1: return rev_exkey_guard_by_motor(MOTOR_ID_1);
        case CMD_ACTION_REV_EX2: return rev_exkey_guard_by_motor(MOTOR_ID_2);
        case CMD_ACTION_REV_EX3: return rev_exkey_guard_by_motor(MOTOR_ID_3);
        case CMD_ACTION_REV_EX4: return rev_exkey_guard_by_motor(MOTOR_ID_4);
        default: return 0;
    }
}

static void m4_home_runtime_reset(void)
{
    memset(&g_m4_home, 0, sizeof(g_m4_home));
    g_m4_home.phase = M4_HOME_PHASE_IDLE;
}

static void rev_exkey_cancel_if_motor(MotorId_t motor_id)
{
    RevExKeyGuard_t *guard = rev_exkey_guard_by_motor(motor_id);

    if (guard != 0) {
        guard->active = 0U;
        guard->blocked_stable_ms = 0U;
    }
    if (motor_id == MOTOR_ID_4) {
        m4_home_runtime_reset();
    }
    if (motor_id == MOTOR_ID_2) {
        m2_abs_cancel_pending_read();
        g_m2_abs_move.active = 0U;
        g_m2_abs_boot_home_pending = 0U;
        g_m2_abs_boot_home_done = 0U;
        g_m2_abs_boot_home_fault = 0U;
    }
    if (motor_id < MOTOR_COUNT) {
        g_enc_move[motor_id].active = 0U;
        g_enc_move[motor_id].done = 0U;
        g_enc_move[motor_id].fault = 0U;
    }
}

static void rev_exkey_cancel_all(void)
{
    uint32_t i;

    /*
     * 取消所有回零/编码器目标运动/取放衣状态。
     * ALL STOP、ALL ESTOP 或错误恢复时会调用这里，确保不会留下异步运动状态。
     */
    for (i = 0U; i < REV_EXKEY_GUARD_COUNT; i++) {
        g_rev_exkey_guards[i].active = 0U;
        g_rev_exkey_guards[i].blocked_stable_ms = 0U;
    }
    m4_home_runtime_reset();
    m2_abs_cancel_pending_read();
    g_m2_abs_move.active = 0U;
    g_m2_abs_boot_home_pending = 0U;
    g_m2_abs_boot_home_done = 0U;
    g_m2_abs_boot_home_fault = 0U;
    memset(g_enc_move, 0, sizeof(g_enc_move));
    g_all_home_active = 0U;
    g_all_home_target_mask = 0U;
    g_all_home_done_mask = 0U;
    place_cloth_cancel();
    take_cloth_cancel();
}


static int cloth_step_motor_wait_done(MotorId_t motor_id)
{
    MotorStatusSnapshot_t status;

    /* 等待普通 MOVE 定步电机运动完成；用于取衣流程中插入的 M2 开环步数动作。 */
    if (motor_id >= MOTOR_COUNT) {
        return -1;
    }
    if (MotorCtrl_GetStatus(motor_id, &status) != 0) {
        return -1;
    }
    if (status.mode == MOTOR_MODE_ESTOP) {
        return -1;
    }
    if (cloth_motor_mode_busy(status.mode) != 0U) {
        return 0;
    }
    return 1;
}

static int cloth_start_m2_move(MotorDir_t dir, uint32_t steps)
{
    if ((g_m2_abs_move.active != 0U) || (place_cloth_motor_busy(MOTOR_ID_2) != 0U)) {
        return -1;
    }
    if (MotorCtrl_MoveSteps(MOTOR_ID_2, dir, CLOTH_M2_RPM, steps) != 0) {
        return -1;
    }
    rev_exkey_cancel_if_motor(MOTOR_ID_2);
    return 0;
}
static int enc_move_start_relative(MotorId_t motor_id,
                                   MotorDir_t dir,
                                   float rpm,
                                   uint32_t count,
                                   uint32_t tol,
                                   int32_t *out_start,
                                   int32_t *out_target)
{
    EncoderStatus_t enc;
    int64_t target;
    uint8_t count_increases;
    float start_rpm;
    int rc;

    /*
     * 启动“相对编码器计数”运动。
     * 例如 M1 ENC_MOVE DIR=CW CNT=1440，会以当前编码器计数为起点，
     * 按方向计算目标计数，然后让电机连续运行，由 SafetyTick 周期判断到位。
     */
    if ((enc_move_motor_supported(motor_id) == 0U) || (count == 0U)) {
        return -1;
    }
    if (EncoderCtrl_GetStatus(motor_id, &enc) != 0) {
        return -2;
    }

    count_increases = MotorHw_DirectionIncreasesPosition(motor_id, dir) ? 1U : 0U;
    target = (int64_t)enc.count_total;
    if (count_increases != 0U) {
        target += (int64_t)count;
    } else {
        target -= (int64_t)count;
    }
    if ((target > (int64_t)INT32_MAX) || (target < (int64_t)INT32_MIN)) {
        return -3;
    }

    start_rpm = enc_move_start_rpm(motor_id,
                                   enc.count_total,
                                   (int32_t)target,
                                   rpm);
    rc = MotorCtrl_Run(motor_id, dir, start_rpm);
    if (rc != 0) {
        return -4;
    }

    rev_exkey_cancel_if_motor(motor_id);
    g_enc_move[motor_id].active = 1U;
    g_enc_move[motor_id].done = 0U;
    g_enc_move[motor_id].fault = 0U;
    g_enc_move[motor_id].dir = dir;
    g_enc_move[motor_id].count_increases = count_increases;
    g_enc_move[motor_id].start_count = enc.count_total;
    g_enc_move[motor_id].target_count = (int32_t)target;
    g_enc_move[motor_id].last_count = enc.count_total;
    g_enc_move[motor_id].tol = tol;
    g_enc_move[motor_id].still_ms = 0U;
    g_enc_move[motor_id].wrong_dir_count = 0U;
    g_enc_move[motor_id].wrong_dir_accum = 0U;
    enc_move_pid_init(motor_id, &g_enc_move[motor_id], rpm, enc.count_total);

    if (out_start != 0) {
        *out_start = enc.count_total;
    }
    if (out_target != 0) {
        *out_target = (int32_t)target;
    }
    return 0;
}

static int enc_move_start_absolute(MotorId_t motor_id,
                                   int32_t target,
                                   float rpm,
                                   uint32_t tol,
                                   uint8_t *out_started)
{
    EncoderStatus_t enc;
    MotorDir_t dir;
    uint8_t count_increases;
    float start_rpm;
    int rc;

    /*
     * 启动“绝对编码器计数”运动。
     * 取/放衣状态机移动 X/Y/Z 时主要走这里：给出目标坐标，
     * 本函数根据当前位置自动决定方向，后续由 SafetyTick 监控是否到位。
     */
    if (out_started != 0) {
        *out_started = 0U;
    }
    if (enc_move_motor_supported(motor_id) == 0U) {
        return -1;
    }
    if (EncoderCtrl_GetStatus(motor_id, &enc) != 0) {
        return -2;
    }
    if (abs_diff_i32(target, enc.count_total) <= tol) {
        g_enc_move[motor_id].active = 0U;
        g_enc_move[motor_id].done = 1U;
        g_enc_move[motor_id].fault = 0U;
        g_enc_move[motor_id].count_increases =
            (target >= enc.count_total) ? 1U : 0U;
        g_enc_move[motor_id].start_count = enc.count_total;
        g_enc_move[motor_id].target_count = target;
        g_enc_move[motor_id].last_count = enc.count_total;
        g_enc_move[motor_id].tol = tol;
        g_enc_move[motor_id].still_ms = 0U;
        g_enc_move[motor_id].stall_boost_active = 0U;
        g_enc_move[motor_id].wrong_dir_count = 0U;
        g_enc_move[motor_id].wrong_dir_accum = 0U;
        return 0;
    }

    count_increases = (target >= enc.count_total) ? 1U : 0U;
    dir = MotorHw_GetDirectionForPositionChange(motor_id,
                                                count_increases != 0U);
    start_rpm = enc_move_start_rpm(motor_id, enc.count_total, target, rpm);
    rc = MotorCtrl_Run(motor_id, dir, start_rpm);
    if (rc != 0) {
        return -4;
    }

    rev_exkey_cancel_if_motor(motor_id);
    g_enc_move[motor_id].active = 1U;
    g_enc_move[motor_id].done = 0U;
    g_enc_move[motor_id].fault = 0U;
    g_enc_move[motor_id].dir = dir;
    g_enc_move[motor_id].count_increases = count_increases;
    g_enc_move[motor_id].start_count = enc.count_total;
    g_enc_move[motor_id].target_count = target;
    g_enc_move[motor_id].last_count = enc.count_total;
    g_enc_move[motor_id].tol = tol;
    g_enc_move[motor_id].still_ms = 0U;
    g_enc_move[motor_id].wrong_dir_count = 0U;
    g_enc_move[motor_id].wrong_dir_accum = 0U;
    enc_move_pid_init(motor_id, &g_enc_move[motor_id], rpm, enc.count_total);

    if (out_started != 0) {
        *out_started = 1U;
    }
    return 0;
}

static void enc_move_stop_xyz(uint8_t estop)
{
    const MotorId_t motors[] = { MOTOR_ID_1, MOTOR_ID_3, MOTOR_ID_4 };
    uint32_t i;

    /* 停止 XYZ 对应的 M1/M3/M4 编码器目标运动，estop=1 表示急停。 */
    for (i = 0U; i < (sizeof(motors) / sizeof(motors[0])); i++) {
        MotorId_t motor = motors[i];
        if (g_enc_move[motor].active != 0U) {
            if (estop != 0U) {
                (void)MotorCtrl_EStop(motor);
            } else {
                (void)MotorCtrl_Stop(motor, 0U);
            }
        }
        g_enc_move[motor].active = 0U;
        g_enc_move[motor].done = 0U;
        g_enc_move[motor].fault = 0U;
        g_enc_move[motor].stall_boost_active = 0U;
        g_enc_move[motor].wrong_dir_count = 0U;
        g_enc_move[motor].wrong_dir_accum = 0U;
    }
}

static int all_home_clear_encoder_and_axis_zero(void)
{
    /*
     * ALL HOME 完成后的统一清零动作。
     * M1/M3/M4 对应 EC1/EC3/EC4，清零后再把 AxisCoord 的 XYZ 坐标归零。
     */
    if (EncoderCtrl_Clear(MOTOR_ID_1) != 0) {
        return -1;
    }
    if (EncoderCtrl_Clear(MOTOR_ID_3) != 0) {
        return -2;
    }
    if (EncoderCtrl_Clear(MOTOR_ID_4) != 0) {
        return -3;
    }
    if (AxisCoord_ZeroNow() != 0) {
        return -4;
    }
    AxisLimit_ClearFault();
    return 0;
}

static void all_home_mark_done(MotorId_t motor_id)
{
    char msg[96];
    int rc;

    /*
     * 标记某个回零电机完成。
     * 当 ALL HOME 目标掩码里的电机全部完成后，统一清编码器和 XYZ 坐标，
     * 同时根据当前是否处于取/放衣流程，继续推进状态机或产生完成事件。
     */
    if (g_all_home_active == 0U) {
        return;
    }

    g_all_home_done_mask |= motor_bit(motor_id);
    if ((g_all_home_done_mask & g_all_home_target_mask) != g_all_home_target_mask) {
        return;
    }

    g_all_home_active = 0U;
    rc = all_home_clear_encoder_and_axis_zero();
    if (rc == 0) {
        (void)UartConsole_QueueLineDropOldest("OK ALL HOME_ZERO_DONE EC=1,3,4 AXIS=XYZ", 0);
        cloth_continue_after_start_home();
        if ((g_place_cloth.active != 0U) && (g_place_cloth.step == PLACE_CLOTH_DONE)) {
            cloth_event_set(CMD_CLOTH_EVENT_PLACE_DONE, 0U);
            place_cloth_cancel();
            (void)UartConsole_QueueLineDropOldest("OK PLACE_CLOTH DONE", 0);
        }
        if ((g_take_cloth.active != 0U) && (g_take_cloth.step == TAKE_CLOTH_DONE)) {
            cloth_event_set(CMD_CLOTH_EVENT_TAKE_DONE, 0U);
            take_cloth_cancel();
            (void)UartConsole_QueueLineDropOldest("OK TAKE_CLOTH DONE", 0);
        }
    } else {
        (void)snprintf(msg, sizeof(msg), "ERR 206 ALL_HOME_ZERO_FAIL RC=%d", rc);
        (void)UartConsole_QueueLineDropOldest(msg, 0);
        if ((g_place_cloth.active != 0U) &&
            ((g_place_cloth.step == PLACE_CLOTH_WAIT_START_HOME) ||
             (g_place_cloth.step == PLACE_CLOTH_ALL_HOME))) {
            cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, 0xFFU);
            place_cloth_cancel();
            cloth_fault_report_magnet_hold();
        }
        if ((g_take_cloth.active != 0U) && (g_take_cloth.step == TAKE_CLOTH_WAIT_START_HOME)) {
            cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, 0xFFU);
            take_cloth_cancel();
        }
        if ((g_place_cloth.active != 0U) && (g_place_cloth.step == PLACE_CLOTH_DONE)) {
            cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, 0xFFU);
            place_cloth_cancel();
        }
        if ((g_take_cloth.active != 0U) && (g_take_cloth.step == TAKE_CLOTH_DONE)) {
            cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, 0xFFU);
            take_cloth_cancel();
        }
    }
}

static const char *m4_home_phase_to_string(M4HomePhase_t phase)
{
    switch (phase) {
        case M4_HOME_PHASE_RELEASE: return "RELEASE";
        case M4_HOME_PHASE_SEEK: return "SEEK";
        default: return "IDLE";
    }
}

static void home_log_action(const RevExKeyGuard_t *guard,
                            const char *action,
                            MotorDir_t dir,
                            float rpm)
{
    char msg[112];

    (void)snprintf(msg,
                   sizeof(msg),
                   "OK HOME M%u EX%u ACTION=%s DIR=%s RPM=%.2f EX=%s",
                   (unsigned int)(guard->motor_id + 1U),
                   (unsigned int)guard->key_index,
                   action,
                   MotorCtrl_DirToString(dir),
                   rpm,
                   rev_exkey_is_blocked_low(guard) ? "LOW" : "HIGH");
    (void)UartConsole_QueueLineDropOldest(msg, 0);
}

static void m4_home_abort(const char *reason)
{
    uint32_t i;
    uint8_t place_active = g_place_cloth.active;
    uint8_t take_active = g_take_cloth.active;

    for (i = 0U; i < REV_EXKEY_GUARD_COUNT; i++) {
        if (g_rev_exkey_guards[i].active != 0U) {
            (void)MotorCtrl_EStop(g_rev_exkey_guards[i].motor_id);
        }
    }
    if (place_active != 0U) {
        cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, 0xFFU);
    }
    if (take_active != 0U) {
        cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, 0xFFU);
    }
    rev_exkey_cancel_all();
    (void)UartConsole_QueueLineDropOldest(reason, 0);
}

static int m4_home_start(RevExKeyGuard_t *guard, float rpm)
{
    MotorDir_t dir;

    if ((guard == 0) || (guard->motor_id != MOTOR_ID_4)) {
        return -1;
    }

    m4_home_runtime_reset();
    g_m4_home.rpm = rpm;
    if (rev_exkey_is_blocked_low(guard) != 0U) {
        g_m4_home.phase = M4_HOME_PHASE_RELEASE;
        dir = MOTOR_DIR_CW;
    } else {
        g_m4_home.phase = M4_HOME_PHASE_SEEK;
        dir = guard->home_dir;
    }

    if (MotorCtrl_Run(MOTOR_ID_4, dir, rpm) != 0) {
        m4_home_runtime_reset();
        guard->active = 0U;
        return -2;
    }
    guard->active = 1U;
    home_log_action(guard, m4_home_phase_to_string(g_m4_home.phase), dir, rpm);
    return 0;
}

static void m4_home_tick(RevExKeyGuard_t *guard)
{
    uint8_t blocked_low;
    char msg[112];

    if ((guard == 0) || (guard->active == 0U) ||
        (g_m4_home.phase == M4_HOME_PHASE_IDLE)) {
        return;
    }

    if (g_m4_home.elapsed_ms < 0xFFFFFFFFUL) {
        g_m4_home.elapsed_ms++;
    }
    blocked_low = rev_exkey_is_blocked_low(guard);

    if (g_m4_home.phase == M4_HOME_PHASE_RELEASE) {
        if (blocked_low == 0U) {
            if (g_m4_home.stable_ms < HOME_EXKEY_STABLE_MS) {
                g_m4_home.stable_ms++;
            }
        } else {
            g_m4_home.stable_ms = 0U;
        }

        if (g_m4_home.stable_ms >= HOME_EXKEY_STABLE_MS) {
            (void)MotorCtrl_EStop(MOTOR_ID_4);
            if (MotorCtrl_Run(MOTOR_ID_4, guard->home_dir, g_m4_home.rpm) != 0) {
                m4_home_abort("ERR 203 M4_HOME_SEEK_START_FAIL");
                return;
            }
            g_m4_home.phase = M4_HOME_PHASE_SEEK;
            g_m4_home.elapsed_ms = 0U;
            g_m4_home.stable_ms = 0U;
            home_log_action(guard, "SEEK", guard->home_dir, g_m4_home.rpm);
            return;
        }

        if (g_m4_home.elapsed_ms >= M4_HOME_RELEASE_TIMEOUT_MS) {
            (void)snprintf(msg,
                           sizeof(msg),
                           "ERR 208 M4_HOME_RELEASE_TIMEOUT T=%lu EX4=%s",
                           (unsigned long)g_m4_home.elapsed_ms,
                           blocked_low ? "LOW" : "HIGH");
            m4_home_abort(msg);
        }
        return;
    }

    if (blocked_low != 0U) {
        if (g_m4_home.stable_ms < HOME_EXKEY_STABLE_MS) {
            g_m4_home.stable_ms++;
        }
    } else {
        g_m4_home.stable_ms = 0U;
    }

    if (g_m4_home.stable_ms >= HOME_EXKEY_STABLE_MS) {
        (void)MotorCtrl_EStop(MOTOR_ID_4);
        guard->active = 0U;
        (void)UartConsole_QueueLineDropOldest(
            "OK M4 HOME_EX4_DONE ACTION=SEEK EX_KEY4_LOW_STABLE=20MS", 0);
        m4_home_runtime_reset();
        all_home_mark_done(MOTOR_ID_4);
        return;
    }

    if (g_m4_home.elapsed_ms >= M4_HOME_SEEK_TIMEOUT_MS) {
        (void)snprintf(msg,
                       sizeof(msg),
                       "ERR 208 M4_HOME_SEEK_TIMEOUT T=%lu EX4=%s",
                       (unsigned long)g_m4_home.elapsed_ms,
                       blocked_low ? "LOW" : "HIGH");
        m4_home_abort(msg);
    }
}

static int home_guard_start(RevExKeyGuard_t *guard,
                            float rpm,
                            uint8_t *started,
                            uint8_t *skipped)
{
    if ((guard == 0) || (started == 0) || (skipped == 0)) {
        return -1;
    }
    *started = 0U;
    *skipped = 0U;
    rev_exkey_cancel_if_motor(guard->motor_id);

    if (guard->motor_id == MOTOR_ID_4) {
        if (m4_home_start(guard, rpm) != 0) {
            return -2;
        }
        *started = 1U;
        return 0;
    }

    if (rev_exkey_is_blocked_low(guard) != 0U) {
        guard->active = 0U;
        guard->blocked_stable_ms = 0U;
        *skipped = 1U;
        home_log_action(guard, "SKIP", guard->home_dir, rpm);
        return 0;
    }

    if (MotorCtrl_Run(guard->motor_id, guard->home_dir, rpm) != 0) {
        return -3;
    }
    guard->active = 1U;
    guard->blocked_stable_ms = 0U;
    *started = 1U;
    home_log_action(guard, "RUN", guard->home_dir, rpm);
    return 0;
}

static int all_home_start(float rpm, uint32_t *out_run_count, uint32_t *out_skip_count)
{
    uint32_t i;
    uint32_t run_count = 0U;
    uint32_t skip_count = 0U;
    uint32_t target_mask = 0U;
    uint32_t done_mask = 0U;

    g_all_home_active = 0U;
    g_all_home_target_mask = 0U;
    g_all_home_done_mask = 0U;

    for (i = 0U; i < REV_EXKEY_GUARD_COUNT; i++) {
        uint8_t started;
        uint8_t skipped;
        RevExKeyGuard_t *guard = &g_rev_exkey_guards[i];

        if (guard->include_all_home == 0U) {
            continue;
        }
        target_mask |= motor_bit(guard->motor_id);
        if (home_guard_start(guard, rpm, &started, &skipped) != 0) {
            uint32_t j;

            for (j = 0U; j < REV_EXKEY_GUARD_COUNT; j++) {
                if (g_rev_exkey_guards[j].active != 0U) {
                    (void)MotorCtrl_EStop(g_rev_exkey_guards[j].motor_id);
                }
            }
            rev_exkey_cancel_all();
            return -1;
        }
        if (started != 0U) {
            run_count++;
        }
        if (skipped != 0U) {
            skip_count++;
            done_mask |= motor_bit(guard->motor_id);
        }
    }

    g_all_home_target_mask = target_mask;
    g_all_home_done_mask = done_mask;
    if (out_run_count != 0) {
        *out_run_count = run_count;
    }
    if (out_skip_count != 0) {
        *out_skip_count = skip_count;
    }
    if (run_count == 0U) {
        return (all_home_clear_encoder_and_axis_zero() == 0) ? 0 : -2;
    }
    g_all_home_active = 1U;
    return 1;
}

static void take_cloth_log_axis_move(const char *step,
                                     MotorId_t motor_id,
                                     int32_t target,
                                     uint8_t started)
{
    EncoderStatus_t enc;
    char msg[128];

    /* 取衣流程调试日志：记录当前步骤、目标轴、当前计数、目标计数和是否真正启动电机。 */
    if (EncoderCtrl_GetStatus(motor_id, &enc) == 0) {
        (void)snprintf(msg,
                       sizeof(msg),
                       "OK TAKE_CLOTH %s M%u CNT=%ld TARGET=%ld %s",
                       step,
                       (unsigned int)(motor_id + 1U),
                       (long)enc.count_total,
                       (long)target,
                       (started != 0U) ? "START" : "ALREADY_DONE");
    } else {
        (void)snprintf(msg,
                       sizeof(msg),
                       "OK TAKE_CLOTH %s M%u TARGET=%ld %s",
                       step,
                       (unsigned int)(motor_id + 1U),
                       (long)target,
                       (started != 0U) ? "START" : "ALREADY_DONE");
    }
    (void)UartConsole_QueueLineDropOldest(msg, 0);
}

static int take_cloth_start_axis_move(MotorId_t motor_id, int32_t target, uint8_t *out_started)
{
    uint8_t started;
    int rc;

    /* 取衣流程启动某个轴的绝对坐标运动，并记录正在等待的电机。 */
    if (out_started != 0) {
        *out_started = 0U;
    }
    rc = enc_move_start_absolute(motor_id, target, g_take_cloth.rpm, g_take_cloth.tol, &started);
    if (rc != 0) {
        return rc;
    }
    if (out_started != 0) {
        *out_started = started;
    }
    g_take_cloth.wait_motor = motor_id;
    return 0;
}

static int take_cloth_start_relative_axis_move(MotorId_t motor_id,
                                               MotorDir_t dir,
                                               uint32_t count,
                                               const char *step)
{
    int32_t start_count;
    int32_t target_count;
    char msg[160];
    int rc;

    rc = enc_move_start_relative(motor_id,
                                 dir,
                                 g_take_cloth.rpm,
                                 count,
                                 g_take_cloth.tol,
                                 &start_count,
                                 &target_count);
    if (rc != 0) {
        return rc;
    }

    g_take_cloth.wait_motor = motor_id;
    g_take_cloth.relative_target = target_count;
    (void)snprintf(msg,
                   sizeof(msg),
                   "OK TAKE_CLOTH %s M%u DIR=%s START=%ld TARGET=%ld DELTA=%lu",
                   (step != 0) ? step : "RELATIVE_MOVE",
                   (unsigned int)(motor_id + 1U),
                   MotorCtrl_DirToString(dir),
                   (long)start_count,
                   (long)target_count,
                   (unsigned long)count);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
    return 0;
}

static int take_cloth_start_y_home(void)
{
    RevExKeyGuard_t *guard = rev_exkey_guard_by_motor(MOTOR_ID_3);
    int rc;

    /*
     * 取衣流程中让 Y 轴回零。
     * Y 轴实际对应 M3，回零方向为 CCW，EX_KEY3 低电平表示已经到回零位置。
     */
    if (guard == 0) {
        return -1;
    }
    if (rev_exkey_is_blocked_low(guard)) {
        guard->active = 0U;
        g_take_cloth.wait_motor = MOTOR_ID_INVALID;
        g_take_cloth.y_home_done = 1U;
        return 0;
    }
    g_take_cloth.y_home_done = 0U;
    rc = MotorCtrl_Run(MOTOR_ID_3, MOTOR_DIR_CCW, g_take_cloth.rpm);
    if (rc != 0) {
        return -2;
    }
    rev_exkey_cancel_if_motor(MOTOR_ID_3);
    guard->active = 1U;
    g_take_cloth.wait_motor = MOTOR_ID_3;
    return 0;
}

static int take_cloth_start_all_home(void)
{
    return all_home_start(g_take_cloth.rpm, 0, 0);
}

static int place_cloth_start_axis_move(MotorId_t motor_id, int32_t target)
{
    uint8_t started;
    int rc;

    /* 放衣流程启动某个轴的绝对坐标运动，并记录正在等待的电机。 */
    rc = enc_move_start_absolute(motor_id, target, g_place_cloth.rpm, g_place_cloth.tol, &started);
    if (rc != 0) {
        return rc;
    }
    g_place_cloth.wait_motor = motor_id;
    return 0;
}

static int place_cloth_start_y_home(void)
{
    RevExKeyGuard_t *guard = rev_exkey_guard_by_motor(MOTOR_ID_3);
    int rc;

    /*
     * 放衣流程中让 Y 轴回零，避免移动到挂衣位置时发生碰撞。
     * Y 轴实际对应 M3，EX_KEY3 低电平表示回零完成。
     */
    if (guard == 0) {
        return -1;
    }
    if (rev_exkey_is_blocked_low(guard)) {
        guard->active = 0U;
        g_place_cloth.wait_motor = MOTOR_ID_INVALID;
        g_place_cloth.y_home_done = 1U;
        return 0;
    }
    g_place_cloth.y_home_done = 0U;
    rc = MotorCtrl_Run(MOTOR_ID_3, MOTOR_DIR_CCW, g_place_cloth.rpm);
    if (rc != 0) {
        return -2;
    }
    rev_exkey_cancel_if_motor(MOTOR_ID_3);
    guard->active = 1U;
    g_place_cloth.wait_motor = MOTOR_ID_3;
    return 0;
}

static int place_cloth_start_all_home(void)
{
    return all_home_start(g_place_cloth.rpm, 0, 0);
}

static int place_cloth_start_relative_axis_move(MotorId_t motor_id,
                                                MotorDir_t dir,
                                                uint32_t count,
                                                const char *step)
{
    int32_t start_count;
    int32_t target_count;
    char msg[160];
    int rc;

    rc = enc_move_start_relative(motor_id,
                                 dir,
                                 g_place_cloth.rpm,
                                 count,
                                 g_place_cloth.tol,
                                 &start_count,
                                 &target_count);
    if (rc != 0) {
        return rc;
    }

    g_place_cloth.wait_motor = motor_id;
    g_place_cloth.relative_target = target_count;
    (void)snprintf(msg,
                   sizeof(msg),
                   "OK PLACE_CLOTH %s M%u DIR=%s START=%ld TARGET=%ld DELTA=%lu",
                   (step != 0) ? step : "RELATIVE_MOVE",
                   (unsigned int)(motor_id + 1U),
                   MotorCtrl_DirToString(dir),
                   (long)start_count,
                   (long)target_count,
                   (unsigned long)count);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
    return 0;
}

static void place_cloth_tick(void)
{
    int rc;
    uint16_t laser2_distance;
    uint16_t laser2_spread;
    float laser2_k;
    float laser2_move_mm;
    char msg[192];

    /*
     * 放衣流程主状态机。
     * 主循环周期性调用 CmdDispatch_SafetyTick1ms()，再由它推进到这里。
     * 每个 case 只做当前步骤需要做的事，没到位就 return，下次 tick 继续检查。
     */
    if (g_place_cloth.active == 0U) {
        return;
    }
	//	printf("step:%d\n",g_place_cloth.step);
    switch (g_place_cloth.step) {
        case PLACE_CLOTH_START_HOME:
            /* 第一步：执行 ALL_HOME，建立 XYZ 基准。 */
            rc = place_cloth_start_all_home();
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_START_HOME_FAIL");
                return;
            }
            if (rc == 0) {
                place_cloth_start_next(PLACE_CLOTH_M2_INITIAL);
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_WAIT_START_HOME);
            return;

        case PLACE_CLOTH_WAIT_START_HOME:
            /* 等待 ALL_HOME 完成。 */
            if (g_all_home_active != 0U) {
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_M2_INITIAL);
            return;

        case PLACE_CLOTH_M2_INITIAL:
            rc = cloth_start_m2_move(MOTOR_DIR_CW, PLACE_CLOTH_M2_INITIAL_CW_STEPS);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_INITIAL_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH M2_INITIAL M2 CW RPM=10 STEP=650", 0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_M2_INITIAL);
            return;

        case PLACE_CLOTH_WAIT_M2_INITIAL:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_INITIAL_NOT_DONE");
                return;
            }
            rc = place_cloth_start_axis_move(MOTOR_ID_4, PLACE_CLOTH_TRANSFER_Z);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_TRANSFER_Z_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH TRANSFER_Z M4 TARGET_EC4=10000", 0);
            place_cloth_start_next(PLACE_CLOTH_MOVE_TRANSFER_Z);
            return;

        case PLACE_CLOTH_MOVE_TRANSFER_Z:
            rc = cloth_axis_wait_done(MOTOR_ID_4, PLACE_CLOTH_TRANSFER_Z, g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_TRANSFER_Z_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_START_LASER2_SAMPLE);
            return;

        case PLACE_CLOTH_START_LASER2_SAMPLE:
            if (PLACE_LASER2_CALIBRATION_VALID == 0U) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_CAL_REQUIRED");
                return;
            }
            if (place_laser2_calibration_parameters_valid() == 0U) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_CAL_INVALID");
                return;
            }
            LaserRange2_FlushSamples();
            memset(g_place_cloth.laser2_samples,
                   0,
                   sizeof(g_place_cloth.laser2_samples));
            g_place_cloth.laser2_sample_count = 0U;
            g_place_cloth.laser2_started_ms = HAL_GetTick();
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH LASER2_WAIT FRESH=3 CONF_MIN=70 SPREAD_MAX=8 TIMEOUT=3000",
                0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_LASER2_SAMPLE);
            return;

        case PLACE_CLOTH_WAIT_LASER2_SAMPLE:
            rc = place_laser2_poll_stable_distance(&laser2_distance,
                                                    &laser2_spread);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_SAMPLE_TIMEOUT");
                return;
            }

            rc = place_laser2_calculate_target(laser2_distance,
                                               &g_place_cloth.laser2_target,
                                               &laser2_k,
                                               &laser2_move_mm);
            if (rc == -2) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_CAL_REQUIRED");
                return;
            }
            if (rc == -3) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_CAL_INVALID");
                return;
            }
            if (rc == -4) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_DIST_OUT_OF_RANGE");
                return;
            }
            if (rc == -6) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_BEFORE_X_ZERO");
                return;
            }
            if (rc != 0) {
                place_cloth_fail("ERR 204 PLACE_CLOTH_LASER2_TARGET_RANGE");
                return;
            }

            (void)snprintf(msg,
                           sizeof(msg),
                           "OK PLACE_CLOTH LASER2_MAP D_RAW=%u D_ZERO=%.1f D_MOVE=%.1f SPREAD=%u K=%.6f EC_TARGET=%ld",
                           (unsigned int)laser2_distance,
                           (double)PLACE_LASER2_X_ZERO_DISTANCE_MM,
                           (double)laser2_move_mm,
                           (unsigned int)laser2_spread,
                           (double)laser2_k,
                           (long)g_place_cloth.laser2_target);
            (void)UartConsole_QueueLineDropOldest(msg, 0);
            if (AxisLimit_CanMoveMotor(MOTOR_ID_1) != 0) {
                place_cloth_fail("ERR 212 PLACE_CLOTH_DYNAMIC_X_LIMIT");
                return;
            }
            rc = place_cloth_start_axis_move(MOTOR_ID_1,
                                             g_place_cloth.laser2_target);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_DYNAMIC_X_FAIL");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MOVE_DYNAMIC_X);
            return;

        case PLACE_CLOTH_MOVE_DYNAMIC_X:
            rc = cloth_axis_wait_done(MOTOR_ID_1,
                                      g_place_cloth.laser2_target,
                                      g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_DYNAMIC_X_NOT_REACHED");
                return;
            }
            rc = place_cloth_start_axis_move(MOTOR_ID_3, PLACE_CLOTH_PICK_Y_EC3);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_PICK_Y_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH PICK_Y M3 TARGET_EC3=1381", 0);
            place_cloth_start_next(PLACE_CLOTH_MOVE_PICK_Y);
            return;

        case PLACE_CLOTH_MOVE_PICK_Y:
            rc = cloth_axis_wait_done(MOTOR_ID_3, PLACE_CLOTH_PICK_Y_EC3, g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_PICK_Y_NOT_REACHED");
                return;
            }
            rc = place_cloth_start_axis_move(MOTOR_ID_4, PLACE_CLOTH_PICK_Z);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_PICK_Z_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH PICK_Z M4 TARGET_EC4=12552", 0);
            place_cloth_start_next(PLACE_CLOTH_MOVE_PICK_Z);
            return;

        case PLACE_CLOTH_MOVE_PICK_Z:
            rc = cloth_axis_wait_done(MOTOR_ID_4, PLACE_CLOTH_PICK_Z, g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_PICK_Z_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MAGNET_ON);
            return;

        case PLACE_CLOTH_MAGNET_ON:
            rc = cloth_holder_engage(&g_place_cloth.wait_ms);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_MAGNET_ON_FAIL");
                return;
            }
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            (void)UartConsole_QueueLineDropOldest("OK PLACE_CLOTH MAGNET_ON WAIT=2000", 0);
#else
            (void)UartConsole_QueueLineDropOldest("OK PLACE_CLOTH GRIP_CLOSE M5 CW STEP=10800", 0);
#endif
            place_cloth_start_next(PLACE_CLOTH_WAIT_MAGNET_ON);
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            g_place_cloth.wait_ms = PLACE_CLOTH_GRIP_DELAY_MS;
#endif
            return;

        case PLACE_CLOTH_WAIT_MAGNET_ON:
            rc = cloth_holder_wait_done(&g_place_cloth.wait_ms);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_MAGNET_ON_NOT_DONE");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_LIFT_Z);
            return;

        case PLACE_CLOTH_LIFT_Z:
            rc = MotorCtrl_MoveSteps(MOTOR_ID_4,
                                     MOTOR_DIR_CCW,
                                     g_place_cloth.rpm,
                                     PLACE_CLOTH_LIFT_STEPS);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_LIFT_Z_FAIL");
                return;
            }
            rev_exkey_cancel_if_motor(MOTOR_ID_4);
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH LIFT_Z M4 CCW STEP=650", 0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_LIFT_Z);
            return;

        case PLACE_CLOTH_WAIT_LIFT_Z:
            rc = cloth_step_motor_wait_done(MOTOR_ID_4);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_LIFT_Z_NOT_DONE");
                return;
            }
            rc = place_cloth_start_axis_move(MOTOR_ID_1, PLACE_CLOTH_TRANSFER_X);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_TRANSFER_X_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH TRANSFER_X M1 TARGET_EC1=1000", 0);
            place_cloth_start_next(PLACE_CLOTH_MOVE_TRANSFER_X);
            return;

        case PLACE_CLOTH_MOVE_TRANSFER_X:
            rc = cloth_axis_wait_done(MOTOR_ID_1, PLACE_CLOTH_TRANSFER_X, g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_TRANSFER_X_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_HOME_Y);
            return;

        case PLACE_CLOTH_HOME_Y:
            rc = place_cloth_start_y_home();
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_HOME_Y_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest("OK PLACE_CLOTH HOME_Y EX_KEY3", 0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_HOME_Y);
            return;

        case PLACE_CLOTH_WAIT_HOME_Y:
            rc = cloth_y_home_wait_done();
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_HOME_Y_NOT_REACHED");
                return;
            }
            rc = place_cloth_start_axis_move(MOTOR_ID_4, PLACE_CLOTH_LASER_Z);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_LASER_Z_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH LASER_Z M4 TARGET_EC4=3263", 0);
            place_cloth_start_next(PLACE_CLOTH_MOVE_LASER_Z);
            return;

        case PLACE_CLOTH_MOVE_LASER_Z:
            rc = cloth_axis_wait_done(MOTOR_ID_4, PLACE_CLOTH_LASER_Z, g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_LASER_Z_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MOVE_M2_SCAN);
            return;

        case PLACE_CLOTH_MOVE_M2_SCAN:
            rc = cloth_start_m2_move(MOTOR_DIR_CW, PLACE_CLOTH_M2_SCAN_CW_STEPS);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_SCAN_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH M2_SCAN M2 CW RPM=10 STEP=4000", 0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_M2_SCAN);
            return;

        case PLACE_CLOTH_WAIT_M2_SCAN:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_SCAN_NOT_DONE");
                return;
            }
            rc = place_cloth_start_axis_move(MOTOR_ID_1, PLACE_CLOTH_LASER_GATE);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_LASER_GATE_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH LASER_GATE M1 TARGET_EC1=8860", 0);
            place_cloth_start_next(PLACE_CLOTH_MOVE_LASER_GATE);
            return;

        case PLACE_CLOTH_MOVE_LASER_GATE:
            rc = cloth_axis_wait_done(MOTOR_ID_1, PLACE_CLOTH_LASER_GATE, g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_LASER_GATE_NOT_REACHED");
                return;
            }
            cloth_laser_begin(CLOTH_LASER_OWNER_PLACE,
                              g_place_cloth.slot,
                              PLACE_LASER_X_OFFSET_COUNTS,
                              PLACE_CLOTH_LASER_GATE,
                              1U);
            place_cloth_start_next(PLACE_CLOTH_WAIT_LASER_READY);
            return;

        case PLACE_CLOTH_WAIT_LASER_READY:
            rc = cloth_laser_ready_tick();
            if (rc == CLOTH_LASER_READY) {
                place_cloth_start_next(PLACE_CLOTH_START_LASER_SCAN);
            } else if (rc < 0) {
                cloth_laser_handle_error((ClothLaserResult_t)rc);
            }
            return;

        case PLACE_CLOTH_START_LASER_SCAN:
            rc = cloth_laser_start_scan();
            if (rc == CLOTH_LASER_READY) {
                place_cloth_start_next(PLACE_CLOTH_LASER_SCAN);
            } else if (rc < 0) {
                cloth_laser_handle_error((ClothLaserResult_t)rc);
            }
            return;

        case PLACE_CLOTH_LASER_SCAN:
        {
            int32_t diagnostic_center_x = 0L;

            rc = cloth_laser_scan_tick(&diagnostic_center_x);
            if (rc == CLOTH_LASER_FOUND) {
                char msg[144];

                (void)MotorCtrl_EStop(MOTOR_ID_1);
                rev_exkey_cancel_if_motor(MOTOR_ID_1);
                (void)snprintf(msg,
                               sizeof(msg),
                               "OK PLACE_CLOTH LASER_SLOT SLOT=%lu FOUND=%u CENTER_DIAG=%ld NEXT=X_LEFT_1125",
                               (unsigned long)g_place_cloth.slot,
                               (unsigned int)g_take_laser_scan.found_slots,
                               (long)diagnostic_center_x);
                (void)UartConsole_QueueLineDropOldest(msg, 0);
                take_laser_scan_reset();
                place_cloth_start_next(PLACE_CLOTH_MOVE_X_BACK);
            } else if (rc < 0) {
                cloth_laser_handle_error((ClothLaserResult_t)rc);
            }
            return;
        }

        case PLACE_CLOTH_MOVE_X_BACK:
            if (g_place_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = place_cloth_start_relative_axis_move(MOTOR_ID_1,
                                                          MOTOR_DIR_CCW,
                                                          PLACE_CLOTH_X_BACK_COUNTS,
                                                          "X_LEFT");
                if (rc != 0) {
                    place_cloth_fail("ERR 203 PLACE_CLOTH_X_LEFT_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_1,
                                      g_place_cloth.relative_target,
                                      g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_X_LEFT_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MOVE_Z_DOWN);
            return;

        case PLACE_CLOTH_MOVE_Z_DOWN:
            if (g_place_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = place_cloth_start_relative_axis_move(MOTOR_ID_4,
                                                          MOTOR_DIR_CW,
                                                          PLACE_CLOTH_Z_DOWN_COUNTS,
                                                          "Z_DOWN");
                if (rc != 0) {
                    place_cloth_fail("ERR 203 PLACE_CLOTH_Z_DOWN_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      g_place_cloth.relative_target,
                                      g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_Z_DOWN_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MOVE_Y_FORWARD);
            return;

        case PLACE_CLOTH_MOVE_Y_FORWARD:
            if (g_place_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = place_cloth_start_relative_axis_move(MOTOR_ID_3,
                                                          MOTOR_DIR_CW,
                                                          PLACE_CLOTH_Y_FORWARD_COUNTS,
                                                          "Y_FORWARD");
                if (rc != 0) {
                    place_cloth_fail("ERR 203 PLACE_CLOTH_Y_FORWARD_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_3,
                                      g_place_cloth.relative_target,
                                      g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_Y_FORWARD_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MOVE_M2_RELEASE);
            return;

        case PLACE_CLOTH_MOVE_M2_RELEASE:
            rc = cloth_start_m2_move(MOTOR_DIR_CCW, PLACE_CLOTH_M2_RELEASE_CCW_STEPS);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_RELEASE_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH M2_RELEASE M2 CCW RPM=10 STEP=4000", 0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_M2_RELEASE);
            return;

        case PLACE_CLOTH_WAIT_M2_RELEASE:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_RELEASE_NOT_DONE");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MAGNET_OFF);
            return;

        case PLACE_CLOTH_MAGNET_OFF:
            rc = cloth_holder_release(&g_place_cloth.wait_ms);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_MAGNET_OFF_FAIL");
                return;
            }
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            (void)UartConsole_QueueLineDropOldest("OK PLACE_CLOTH MAGNET_OFF WAIT=2000", 0);
#else
            (void)UartConsole_QueueLineDropOldest("OK PLACE_CLOTH GRIP_OPEN M5 CCW STEP=10800", 0);
#endif
            place_cloth_start_next(PLACE_CLOTH_WAIT_MAGNET_OFF);
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            g_place_cloth.wait_ms = PLACE_CLOTH_GRIP_DELAY_MS;
#endif
            return;

        case PLACE_CLOTH_WAIT_MAGNET_OFF:
            rc = cloth_holder_wait_done(&g_place_cloth.wait_ms);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_MAGNET_OFF_NOT_DONE");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MOVE_Z_RELEASE_UP);
            return;

        case PLACE_CLOTH_MOVE_Z_RELEASE_UP:
            if (g_place_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = place_cloth_start_relative_axis_move(MOTOR_ID_4,
                                                          MOTOR_DIR_CCW,
                                                          PLACE_CLOTH_Z_RELEASE_UP_COUNTS,
                                                          "Z_RELEASE_UP");
                if (rc != 0) {
                    place_cloth_fail("ERR 203 PLACE_CLOTH_Z_RELEASE_UP_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      g_place_cloth.relative_target,
                                      g_place_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_Z_RELEASE_UP_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_HOME_Y_FINAL);
            return;

        case PLACE_CLOTH_HOME_Y_FINAL:
            rc = place_cloth_start_y_home();
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_HOME_Y_FINAL_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH HOME_Y_FINAL EX_KEY3", 0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_HOME_Y_FINAL);
            return;

        case PLACE_CLOTH_WAIT_HOME_Y_FINAL:
            rc = cloth_y_home_wait_done();
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_HOME_Y_FINAL_NOT_REACHED");
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_MOVE_M2_AFTER_Y_HOME);
            return;

        case PLACE_CLOTH_MOVE_M2_AFTER_Y_HOME:
            rc = cloth_start_m2_move(MOTOR_DIR_CW,
                                     PLACE_CLOTH_M2_AFTER_Y_HOME_CW_STEPS);
            if (rc != 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_AFTER_Y_HOME_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH M2_AFTER_Y_HOME M2 CW RPM=10 STEP=1000", 0);
            place_cloth_start_next(PLACE_CLOTH_WAIT_M2_AFTER_Y_HOME);
            return;

        case PLACE_CLOTH_WAIT_M2_AFTER_Y_HOME:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_M2_AFTER_Y_HOME_NOT_DONE");
                return;
            }
            rc = place_cloth_start_all_home();
            if (rc < 0) {
                place_cloth_fail("ERR 203 PLACE_CLOTH_ALL_HOME_FAIL");
                return;
            }
            if (rc == 0) {
                place_cloth_start_next(PLACE_CLOTH_M2_HOME);
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_ALL_HOME);
            return;

        case PLACE_CLOTH_ALL_HOME:
            if (g_all_home_active != 0U) {
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_M2_HOME);
            return;

        case PLACE_CLOTH_M2_HOME:
            if (g_place_cloth.m2_home_started == 0U) {
                CmdDispatch_RequestM2BootHome();
                g_place_cloth.m2_home_started = 1U;
                (void)UartConsole_QueueLineDropOldest(
                    "OK PLACE_CLOTH M2_HOME REQUEST TARGET_RAW=1008 RPM=25", 0);
                return;
            }
            if (g_m2_abs_boot_home_fault != 0U) {
                place_cloth_fail("ERR 208 PLACE_CLOTH_M2_HOME_FAIL");
                return;
            }
            if (g_m2_abs_boot_home_done == 0U) {
                return;
            }
            place_cloth_start_next(PLACE_CLOTH_DONE);
            return;

        case PLACE_CLOTH_DONE:
            cloth_event_set(CMD_CLOTH_EVENT_PLACE_DONE, 0U);
            (void)UartConsole_QueueLineDropOldest(
                "OK PLACE_CLOTH DONE ZERO=EC1,EC3,EC4,XYZ M2_RAW=1008", 0);
            place_cloth_cancel();
            return;

        default:
            place_cloth_fail("ERR 203 PLACE_CLOTH_BAD_STATE");
            return;
    }
}

static void take_cloth_tick(void)
{
    uint8_t started;
    int rc;

    if (g_take_cloth.active == 0U) {
        return;
    }

    switch (g_take_cloth.step) {
        case TAKE_CLOTH_START_HOME:
            rc = take_cloth_start_all_home();
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_START_HOME_FAIL");
                return;
            }
            if (rc == 0) {
                take_cloth_start_next(TAKE_CLOTH_M2_INITIAL);
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_WAIT_START_HOME);
            return;

        case TAKE_CLOTH_WAIT_START_HOME:
            if (g_all_home_active != 0U) {
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_M2_INITIAL);
            return;

        case TAKE_CLOTH_M2_INITIAL:
            rc = cloth_start_m2_move(MOTOR_DIR_CW,
                                     TAKE_CLOTH_M2_INITIAL_CW_STEPS);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_INITIAL_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH M2_INITIAL M2 CW RPM=10 STEP=650", 0);
            take_cloth_start_next(TAKE_CLOTH_WAIT_M2_INITIAL);
            return;

        case TAKE_CLOTH_WAIT_M2_INITIAL:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_INITIAL_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_LASER_Z);
            return;

        case TAKE_CLOTH_MOVE_LASER_Z:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_axis_move(MOTOR_ID_4,
                                                TAKE_CLOTH_LASER_Z,
                                                &started);
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_LASER_Z_FAIL");
                    return;
                }
                take_cloth_log_axis_move("LASER_Z",
                                         MOTOR_ID_4,
                                         TAKE_CLOTH_LASER_Z,
                                         started);
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      TAKE_CLOTH_LASER_Z,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_LASER_Z_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_M2_SCAN);
            return;

        case TAKE_CLOTH_M2_SCAN:
            rc = cloth_start_m2_move(MOTOR_DIR_CW,
                                     TAKE_CLOTH_M2_SCAN_CW_STEPS);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_SCAN_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH M2_SCAN M2 CW RPM=10 STEP=1500", 0);
            take_cloth_start_next(TAKE_CLOTH_WAIT_M2_SCAN);
            return;

        case TAKE_CLOTH_WAIT_M2_SCAN:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_SCAN_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_LASER_GATE);
            return;

        case TAKE_CLOTH_MOVE_LASER_GATE:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_axis_move(MOTOR_ID_1,
                                                TAKE_CLOTH_LASER_GATE,
                                                &started);
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_LASER_GATE_FAIL");
                    return;
                }
                take_cloth_log_axis_move("LASER_GATE",
                                         MOTOR_ID_1,
                                         TAKE_CLOTH_LASER_GATE,
                                         started);
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_1,
                                      TAKE_CLOTH_LASER_GATE,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_LASER_GATE_NOT_REACHED");
                return;
            }
            cloth_laser_begin(CLOTH_LASER_OWNER_TAKE,
                              g_take_cloth.slot,
                              TAKE_LASER_X_OFFSET_COUNTS,
                              TAKE_CLOTH_LASER_GATE,
                              1U);
            take_cloth_start_next(TAKE_CLOTH_WAIT_LASER_READY);
            return;

        case TAKE_CLOTH_WAIT_LASER_READY:
            rc = cloth_laser_ready_tick();
            if (rc == CLOTH_LASER_READY) {
                take_cloth_start_next(TAKE_CLOTH_START_LASER_SCAN);
            } else if (rc < 0) {
                cloth_laser_handle_error((ClothLaserResult_t)rc);
            }
            return;

        case TAKE_CLOTH_START_LASER_SCAN:
            rc = cloth_laser_start_scan();
            if (rc == CLOTH_LASER_READY) {
                take_cloth_start_next(TAKE_CLOTH_LASER_SCAN);
            } else if (rc < 0) {
                cloth_laser_handle_error((ClothLaserResult_t)rc);
            }
            return;

        case TAKE_CLOTH_LASER_SCAN:
        {
            int32_t diagnostic_center_x = 0L;
            char msg[144];

            rc = cloth_laser_scan_tick(&diagnostic_center_x);
            if (rc == CLOTH_LASER_FOUND) {
                (void)MotorCtrl_EStop(MOTOR_ID_1);
                rev_exkey_cancel_if_motor(MOTOR_ID_1);
                (void)snprintf(msg,
                               sizeof(msg),
                               "OK TAKE_CLOTH LASER_SLOT SLOT=%lu FOUND=%u CENTER_DIAG=%ld NEXT=X_LEFT_1760",
                               (unsigned long)g_take_cloth.slot,
                               (unsigned int)g_take_laser_scan.found_slots,
                               (long)diagnostic_center_x);
                (void)UartConsole_QueueLineDropOldest(msg, 0);
                take_laser_scan_reset();
                take_cloth_start_next(TAKE_CLOTH_MOVE_X_LEFT);
            } else if (rc < 0) {
                cloth_laser_handle_error((ClothLaserResult_t)rc);
            }
            return;
        }

        case TAKE_CLOTH_MOVE_X_LEFT:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_relative_axis_move(
                    MOTOR_ID_1,
                    MOTOR_DIR_CCW,
                    TAKE_CLOTH_X_LEFT_COUNTS,
                    "X_LEFT");
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_X_LEFT_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_1,
                                      g_take_cloth.relative_target,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_X_LEFT_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_Z_TRANSFER_DOWN);
            return;

        case TAKE_CLOTH_MOVE_Z_TRANSFER_DOWN:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_relative_axis_move(
                    MOTOR_ID_4,
                    MOTOR_DIR_CW,
                    TAKE_CLOTH_Z_TRANSFER_DOWN_COUNTS,
                    "Z_TRANSFER_DOWN");
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_Z_TRANSFER_DOWN_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      g_take_cloth.relative_target,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_Z_TRANSFER_DOWN_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_Y_FORWARD);
            return;

        case TAKE_CLOTH_MOVE_Y_FORWARD:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_relative_axis_move(
                    MOTOR_ID_3,
                    MOTOR_DIR_CW,
                    TAKE_CLOTH_Y_FORWARD_COUNTS,
                    "Y_FORWARD");
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_Y_FORWARD_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_3,
                                      g_take_cloth.relative_target,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_Y_FORWARD_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_M2_PICK);
            return;

        case TAKE_CLOTH_M2_PICK:
            rc = cloth_start_m2_move(MOTOR_DIR_CCW,
                                     TAKE_CLOTH_M2_PICK_CCW_STEPS);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_PICK_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH M2_PICK M2 CCW RPM=10 STEP=1500", 0);
            take_cloth_start_next(TAKE_CLOTH_WAIT_M2_PICK);
            return;

        case TAKE_CLOTH_WAIT_M2_PICK:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_PICK_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_Z_PICK_DOWN);
            return;

        case TAKE_CLOTH_MOVE_Z_PICK_DOWN:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_relative_axis_move(
                    MOTOR_ID_4,
                    MOTOR_DIR_CW,
                    TAKE_CLOTH_Z_PICK_DOWN_COUNTS,
                    "Z_PICK_DOWN");
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_Z_PICK_DOWN_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      g_take_cloth.relative_target,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_Z_PICK_DOWN_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MAGNET_ON);
            return;

        case TAKE_CLOTH_MAGNET_ON:
            rc = cloth_holder_engage(&g_take_cloth.wait_ms);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_MAGNET_ON_FAIL");
                return;
            }
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH MAGNET_ON WAIT=2000", 0);
#else
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH GRIP_CLOSE M5 CW STEP=10800", 0);
#endif
            take_cloth_start_next(TAKE_CLOTH_WAIT_MAGNET_ON);
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            g_take_cloth.wait_ms = TAKE_CLOTH_GRIP_DELAY_MS;
#endif
            return;

        case TAKE_CLOTH_WAIT_MAGNET_ON:
            rc = cloth_holder_wait_done(&g_take_cloth.wait_ms);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_MAGNET_ON_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_Z_LIFT_UP);
            return;

        case TAKE_CLOTH_MOVE_Z_LIFT_UP:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_relative_axis_move(
                    MOTOR_ID_4,
                    MOTOR_DIR_CCW,
                    TAKE_CLOTH_Z_LIFT_UP_COUNTS,
                    "Z_LIFT_UP");
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_Z_LIFT_UP_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      g_take_cloth.relative_target,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_Z_LIFT_UP_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_Y_EXTRA_FORWARD);
            return;

        case TAKE_CLOTH_MOVE_Y_EXTRA_FORWARD:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_relative_axis_move(
                    MOTOR_ID_3,
                    MOTOR_DIR_CW,
                    TAKE_CLOTH_Y_EXTRA_FORWARD_COUNTS,
                    "Y_EXTRA_FORWARD");
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_Y_EXTRA_FORWARD_FAIL");
                }
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_3,
                                      g_take_cloth.relative_target,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_Y_EXTRA_FORWARD_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_M2_RETURN);
            return;

        case TAKE_CLOTH_M2_RETURN:
            rc = cloth_start_m2_move(MOTOR_DIR_CW,
                                     TAKE_CLOTH_M2_RETURN_CW_STEPS);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_RETURN_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH M2_RETURN M2 CW RPM=10 STEP=1500", 0);
            take_cloth_start_next(TAKE_CLOTH_WAIT_M2_RETURN);
            return;

        case TAKE_CLOTH_WAIT_M2_RETURN:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_RETURN_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_HOME_Y);
            return;

        case TAKE_CLOTH_HOME_Y:
            rc = take_cloth_start_y_home();
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_HOME_Y_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH HOME_Y EX_KEY3", 0);
            take_cloth_start_next(TAKE_CLOTH_WAIT_HOME_Y);
            return;

        case TAKE_CLOTH_WAIT_HOME_Y:
            rc = take_cloth_y_home_wait_done();
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_HOME_Y_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_M2_TRANSFER);
            return;

        case TAKE_CLOTH_M2_TRANSFER:
            rc = cloth_start_m2_move(MOTOR_DIR_CCW,
                                     TAKE_CLOTH_M2_TRANSFER_CCW_STEPS);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_TRANSFER_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH M2_TRANSFER M2 CCW RPM=10 STEP=5500", 0);
            take_cloth_start_next(TAKE_CLOTH_WAIT_M2_TRANSFER);
            return;

        case TAKE_CLOTH_WAIT_M2_TRANSFER:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_TRANSFER_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_TRANSFER_X);
            return;

        case TAKE_CLOTH_MOVE_TRANSFER_X:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_axis_move(MOTOR_ID_1,
                                                TAKE_CLOTH_TRANSFER_X,
                                                &started);
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_TRANSFER_X_FAIL");
                    return;
                }
                take_cloth_log_axis_move("TRANSFER_X",
                                         MOTOR_ID_1,
                                         TAKE_CLOTH_TRANSFER_X,
                                         started);
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_1,
                                      TAKE_CLOTH_TRANSFER_X,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_TRANSFER_X_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_M2_DROP);
            return;

        case TAKE_CLOTH_M2_DROP:
            rc = cloth_start_m2_move(MOTOR_DIR_CW,
                                     TAKE_CLOTH_M2_DROP_CW_STEPS);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_DROP_FAIL");
                return;
            }
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH M2_DROP M2 CW RPM=10 STEP=4000", 0);
            take_cloth_start_next(TAKE_CLOTH_WAIT_M2_DROP);
            return;

        case TAKE_CLOTH_WAIT_M2_DROP:
            rc = cloth_step_motor_wait_done(MOTOR_ID_2);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_M2_DROP_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_DROP_Z);
            return;

        case TAKE_CLOTH_MOVE_DROP_Z:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_axis_move(MOTOR_ID_4,
                                                TAKE_CLOTH_DROP_Z,
                                                &started);
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_DROP_Z_FAIL");
                    return;
                }
                take_cloth_log_axis_move("DROP_Z",
                                         MOTOR_ID_4,
                                         TAKE_CLOTH_DROP_Z,
                                         started);
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      TAKE_CLOTH_DROP_Z,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_DROP_Z_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_DROP_Y);
            return;

        case TAKE_CLOTH_MOVE_DROP_Y:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_axis_move(MOTOR_ID_3,
                                                TAKE_CLOTH_DROP_Y,
                                                &started);
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_DROP_Y_FAIL");
                    return;
                }
                take_cloth_log_axis_move("DROP_Y",
                                         MOTOR_ID_3,
                                         TAKE_CLOTH_DROP_Y,
                                         started);
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_3,
                                      TAKE_CLOTH_DROP_Y,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_DROP_Y_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_DROP_X);
            return;

        case TAKE_CLOTH_MOVE_DROP_X:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_axis_move(MOTOR_ID_1,
                                                TAKE_CLOTH_DROP_X,
                                                &started);
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_DROP_X_FAIL");
                    return;
                }
                take_cloth_log_axis_move("DROP_X",
                                         MOTOR_ID_1,
                                         TAKE_CLOTH_DROP_X,
                                         started);
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_1,
                                      TAKE_CLOTH_DROP_X,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_DROP_X_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MAGNET_OFF);
            return;

        case TAKE_CLOTH_MAGNET_OFF:
            rc = cloth_holder_release(&g_take_cloth.wait_ms);
            if (rc != 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_MAGNET_OFF_FAIL");
                return;
            }
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH MAGNET_OFF WAIT=2000", 0);
#else
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH GRIP_OPEN M5 CCW STEP=10800", 0);
#endif
            take_cloth_start_next(TAKE_CLOTH_WAIT_MAGNET_OFF);
#if CLOTH_HOLDER_USE_ELECTROMAGNET
            g_take_cloth.wait_ms = TAKE_CLOTH_GRIP_DELAY_MS;
#endif
            return;

        case TAKE_CLOTH_WAIT_MAGNET_OFF:
            rc = cloth_holder_wait_done(&g_take_cloth.wait_ms);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_MAGNET_OFF_NOT_DONE");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_MOVE_POST_DROP_Z);
            return;

        case TAKE_CLOTH_MOVE_POST_DROP_Z:
            if (g_take_cloth.wait_motor == MOTOR_ID_INVALID) {
                rc = take_cloth_start_axis_move(MOTOR_ID_4,
                                                TAKE_CLOTH_POST_DROP_Z,
                                                &started);
                if (rc != 0) {
                    take_cloth_fail("ERR 203 TAKE_CLOTH_POST_DROP_Z_FAIL");
                    return;
                }
                take_cloth_log_axis_move("POST_DROP_Z",
                                         MOTOR_ID_4,
                                         TAKE_CLOTH_POST_DROP_Z,
                                         started);
                return;
            }
            rc = cloth_axis_wait_done(MOTOR_ID_4,
                                      TAKE_CLOTH_POST_DROP_Z,
                                      g_take_cloth.tol);
            if (rc == 0) {
                return;
            }
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_POST_DROP_Z_NOT_REACHED");
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_START_ALL_HOME);
            return;

        case TAKE_CLOTH_START_ALL_HOME:
            rc = take_cloth_start_all_home();
            if (rc < 0) {
                take_cloth_fail("ERR 203 TAKE_CLOTH_ALL_HOME_FAIL");
                return;
            }
            if (rc == 0) {
                take_cloth_start_next(TAKE_CLOTH_M2_HOME);
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_ALL_HOME);
            return;

        case TAKE_CLOTH_ALL_HOME:
            if (g_all_home_active != 0U) {
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_M2_HOME);
            return;

        case TAKE_CLOTH_M2_HOME:
            if (g_take_cloth.m2_home_started == 0U) {
                CmdDispatch_RequestM2BootHome();
                g_take_cloth.m2_home_started = 1U;
                (void)UartConsole_QueueLineDropOldest(
                    "OK TAKE_CLOTH M2_HOME REQUEST TARGET_RAW=1008 RPM=25", 0);
                return;
            }
            if (g_m2_abs_boot_home_fault != 0U) {
                take_cloth_fail("ERR 208 TAKE_CLOTH_M2_HOME_FAIL");
                return;
            }
            if (g_m2_abs_boot_home_done == 0U) {
                return;
            }
            take_cloth_start_next(TAKE_CLOTH_DONE);
            return;

        case TAKE_CLOTH_DONE:
            cloth_event_set(CMD_CLOTH_EVENT_TAKE_DONE, 0U);
            (void)UartConsole_QueueLineDropOldest(
                "OK TAKE_CLOTH DONE ZERO=EC1,EC3,EC4,XYZ M2_RAW=1008", 0);
            take_cloth_cancel();
            return;

        default:
            take_cloth_fail("ERR 203 TAKE_CLOTH_BAD_STATE");
            return;
    }
}

static void m2_abs_start_move_state(MotorDir_t move_dir,
                                    uint16_t start_raw,
                                    uint16_t target_raw,
                                    float target_deg,
                                    float tol_deg,
                                    uint8_t boot_home)
{
    /* 记录 M2 绝对值编码器目标运动状态，后续由 CmdDispatch_SafetyTick1ms() 周期监控。 */
    g_m2_abs_move.active = 1U;
    g_m2_abs_move.dir = move_dir;
    g_m2_abs_move.target_raw = target_raw;
    g_m2_abs_move.last_raw = start_raw;
    g_m2_abs_move.last_remain = m2_abs_remain_for_dir(start_raw, target_raw, move_dir);
    g_m2_abs_move.tol_counts = m2_abs_tol_deg_to_counts(tol_deg);
    g_m2_abs_move.poll_ms = 0U;
    g_m2_abs_move.still_ms = 0U;
    g_m2_abs_move.comm_fail_count = 0U;
    g_m2_abs_move.boot_home = boot_home;
    g_m2_abs_move.target_deg = target_deg;
    g_m2_abs_move.tol_deg = tol_deg;
}

void CmdDispatch_RequestM2BootHome(void)
{
#if M2_ABS_BOOT_HOME_ENABLE
    m2_abs_cancel_pending_read();
    g_m2_abs_boot_home_done = 0U;
    g_m2_abs_boot_home_fault = 0U;
    g_m2_abs_boot_home_pending = 1U;
    g_m2_abs_boot_home_ms = 0U;
    g_m2_abs_boot_home_last_try_ms = 0U;
#else
    g_m2_abs_boot_home_pending = 0U;
    g_m2_abs_boot_home_done = 0U;
    g_m2_abs_boot_home_fault = 1U;
#endif
}

static void m2_abs_boot_home_tick(void)
{
    Rs485AbsEncoderStatus_t abs_st;
    MotorStatusSnapshot_t motor_st;
    uint16_t target_raw;
    uint16_t cw_remain;
    uint16_t ccw_remain;
    uint16_t tol_counts;
    MotorDir_t move_dir;
    char msg[112];
    int rc;

    /*
     * M2 上电自动回绝对零点：
     * 等待 M1/M3/M4 回零完成，然后读取绝对值编码器，
     * 如果不在目标角度容差内，则选择最近方向启动 M2 转动。
     */
    if (g_m2_abs_boot_home_pending == 0U) {
        return;
    }

    /* M2 只在 M1/M3/M4 的 ALL HOME 完成后启动。 */
    if (g_all_home_active != 0U) {
        return;
    }

    if (g_m2_abs_boot_home_ms < 0xFFFFFFFFUL) {
        g_m2_abs_boot_home_ms++;
    }

    if (g_m2_abs_read_owner == M2_ABS_READ_OWNER_BOOT_HOME) {
        rc = Rs485AbsEncoder_PollM2Read(&abs_st);
        if (rc == 0) {
            return;
        }
        g_m2_abs_read_owner = M2_ABS_READ_OWNER_NONE;
        if (rc < 0) {
            if (g_m2_abs_boot_home_ms >= M2_ABS_BOOT_HOME_TIMEOUT_MS) {
                g_m2_abs_boot_home_pending = 0U;
                g_m2_abs_boot_home_fault = 1U;
                (void)snprintf(msg,
                               sizeof(msg),
                               "ERR 208 M2_BOOT_HOME_COMM_%s",
                               Rs485AbsEncoder_ErrorText(rc));
                (void)UartConsole_QueueLineDropOldest(msg, 0);
            }
            return;
        }
        rc = 0;
    } else if (g_m2_abs_read_owner != M2_ABS_READ_OWNER_NONE) {
        return;
    } else {
        if ((g_m2_abs_boot_home_last_try_ms != 0U) &&
            ((g_m2_abs_boot_home_ms - g_m2_abs_boot_home_last_try_ms) < M2_ABS_BOOT_HOME_RETRY_PERIOD_MS)) {
            return;
        }
        g_m2_abs_boot_home_last_try_ms = g_m2_abs_boot_home_ms;

        /* 只有 M2 空闲时才允许启动上电回零。 */
        if (MotorCtrl_GetStatus(MOTOR_ID_2, &motor_st) != 0) {
            g_m2_abs_boot_home_pending = 0U;
            g_m2_abs_boot_home_fault = 1U;
            (void)UartConsole_QueueLineDropOldest("ERR 208 M2_BOOT_HOME_STATUS_FAIL", 0);
            return;
        }
        if (motor_st.mode != MOTOR_MODE_IDLE) {
            g_m2_abs_boot_home_pending = 0U;
            g_m2_abs_boot_home_fault = 1U;
            (void)UartConsole_QueueLineDropOldest("ERR 208 M2_BOOT_HOME_BUSY", 0);
            return;
        }

        rc = Rs485AbsEncoder_RequestM2Read();
        if (rc == 0) {
            g_m2_abs_read_owner = M2_ABS_READ_OWNER_BOOT_HOME;
        } else if (g_m2_abs_boot_home_ms >= M2_ABS_BOOT_HOME_TIMEOUT_MS) {
            g_m2_abs_boot_home_pending = 0U;
            g_m2_abs_boot_home_fault = 1U;
            (void)snprintf(msg,
                           sizeof(msg),
                           "ERR 208 M2_BOOT_HOME_COMM_%s",
                           Rs485AbsEncoder_ErrorText(rc));
            (void)UartConsole_QueueLineDropOldest(msg, 0);
        }
        return;
    }

    g_m2_abs_boot_home_pending = 0U;

    /* 如果当前角度已经在目标容差内，直接认为上电回零完成。 */
#if M2_ABS_BOOT_HOME_USE_FIXED_RAW
    target_raw = (uint16_t)(M2_ABS_BOOT_HOME_TARGET_RAW % RS485_ABS_ENCODER_CPR);
#else
    target_raw = m2_abs_deg_to_raw(M2_ABS_BOOT_HOME_TARGET_DEG);
#endif
    tol_counts = m2_abs_tol_deg_to_counts(M2_ABS_BOOT_HOME_TOL_DEG);
    if (m2_abs_short_distance(abs_st.raw, target_raw) <= tol_counts) {
        g_m2_abs_boot_home_done = 1U;
        (void)snprintf(msg,
                       sizeof(msg),
                       "OK M2 BOOT_HOME_DONE RAW=%u DEG=%.2f TARGET=%.2f TOL=%.2f",
                       (unsigned int)abs_st.raw,
                       abs_st.deg,
                       M2_ABS_BOOT_HOME_TARGET_DEG,
                       M2_ABS_BOOT_HOME_TOL_DEG);
        (void)UartConsole_QueueLineDropOldest(msg, 0);
        return;
    }

    /* 比较顺时针/逆时针剩余计数，选择更短的一侧。 */
    cw_remain = m2_abs_remain_for_dir(abs_st.raw, target_raw, MOTOR_DIR_CW);
    ccw_remain = m2_abs_remain_for_dir(abs_st.raw, target_raw, MOTOR_DIR_CCW);
    move_dir = (cw_remain <= ccw_remain) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;

    rc = MotorCtrl_Run(MOTOR_ID_2, move_dir, M2_ABS_BOOT_HOME_RPM);
    if (rc != 0) {
        g_m2_abs_boot_home_fault = 1U;
        (void)UartConsole_QueueLineDropOldest("ERR 208 M2_BOOT_HOME_RUN_FAIL", 0);
        return;
    }

    m2_abs_start_move_state(move_dir,
                            abs_st.raw,
                            target_raw,
                            M2_ABS_BOOT_HOME_TARGET_DEG,
                            M2_ABS_BOOT_HOME_TOL_DEG,
                            1U);

    (void)snprintf(msg,
                   sizeof(msg),
                   "OK M2 BOOT_HOME DIR=%s RPM=%.2f START_RAW=%u START_DEG=%.2f TARGET_RAW=%u TARGET_DEG=%.2f TOL=%.2f",
                   MotorCtrl_DirToString(move_dir),
                   M2_ABS_BOOT_HOME_RPM,
                   (unsigned int)abs_st.raw,
                   abs_st.deg,
                   (unsigned int)target_raw,
                   M2_ABS_BOOT_HOME_TARGET_DEG,
                   M2_ABS_BOOT_HOME_TOL_DEG);
    (void)UartConsole_QueueLineDropOldest(msg, 0);
}

static void m4_upper_limit_safety_tick(void)
{
    RevExKeyGuard_t *guard = rev_exkey_guard_by_motor(MOTOR_ID_4);
    MotorStatusSnapshot_t status;
    uint8_t place_active;
    uint8_t take_active;

    if (guard == 0) {
        return;
    }
    if (guard->active != 0U) {
        /* 正常回零由 EX_KEY 回零状态机处理，不走异常限位路径。 */
        g_m4_upper_limit_latched = 0U;
        return;
    }
    if (rev_exkey_is_blocked_low(guard) == 0U) {
        g_m4_upper_limit_latched = 0U;
        return;
    }
    if (MotorCtrl_GetStatus(MOTOR_ID_4, &status) != 0) {
        return;
    }
    if ((status.dir != MOTOR_DIR_CCW) ||
        (cloth_motor_mode_busy(status.mode) == 0U)) {
        return;
    }

    (void)MotorCtrl_EStop(MOTOR_ID_4);
    (void)AxisPlanner_EStop();
    rev_exkey_cancel_if_motor(MOTOR_ID_4);

    place_active = g_place_cloth.active;
    take_active = g_take_cloth.active;
    if (place_active != 0U) {
        place_cloth_stop_motion(1U);
        place_cloth_cancel();
        cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, 0xFFU);
    } else if (take_active != 0U) {
        place_cloth_stop_motion(1U);
        take_cloth_cancel();
        cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, 0xFFU);
    }

    if (g_m4_upper_limit_latched == 0U) {
        (void)UartConsole_QueueLineDropOldest(
            "ERR 212 M4_Z_UPPER_LIMIT EX_KEY4_LOW DIR=CCW", 0);
        cloth_fault_report_magnet_hold();
        g_m4_upper_limit_latched = 1U;
    }
}

void CmdDispatch_SafetyTick1ms(void)
{
    MotorStatusSnapshot_t status;
    uint32_t i;

    /*
     * 命令分发层的周期安全任务：
     * 1. 推进 M2 上电绝对值回零；
     * 2. 推进放衣/取衣状态机；
     * 3. 监控 M2 绝对值目标运动是否到位、失速、反向或通信失败；
     * 4. 监控 M1/M3/M4 编码器目标运动；
     * 5. 监控 EX_KEY 回零触发并停止对应电机。
     */
    m2_abs_boot_home_tick();
    m4_upper_limit_safety_tick();
    place_cloth_tick();
    take_cloth_tick();

    /* M2 绝对值目标运动监控。 */
    if (g_m2_abs_move.active != 0U) {
        if (MotorCtrl_GetStatus(MOTOR_ID_2, &status) != 0) {
            if (g_m2_abs_read_owner == M2_ABS_READ_OWNER_MOVE) {
                m2_abs_cancel_pending_read();
            }
            g_m2_abs_move.active = 0U;
            if (g_m2_abs_move.boot_home != 0U) {
                g_m2_abs_boot_home_fault = 1U;
                (void)UartConsole_QueueLineDropOldest("ERR 208 M2_BOOT_HOME_STATUS_FAIL", 0);
            }
        } else if ((status.mode != MOTOR_MODE_RUN) &&
                   (status.mode != MOTOR_MODE_RAMP_UP) &&
                   (status.mode != MOTOR_MODE_RAMP_DOWN)) {
            if (g_m2_abs_read_owner == M2_ABS_READ_OWNER_MOVE) {
                m2_abs_cancel_pending_read();
            }
            g_m2_abs_move.active = 0U;
            if (g_m2_abs_move.boot_home != 0U) {
                g_m2_abs_boot_home_fault = 1U;
                (void)UartConsole_QueueLineDropOldest("ERR 208 M2_BOOT_HOME_STOPPED", 0);
            }
        } else {
            Rs485AbsEncoderStatus_t abs_st;
            char msg[112];
            int enc_rc = 0;
            int poll_rc;
            uint8_t have_result = 0U;

            if (g_m2_abs_read_owner == M2_ABS_READ_OWNER_MOVE) {
                poll_rc = Rs485AbsEncoder_PollM2Read(&abs_st);
                if (poll_rc != 0) {
                    g_m2_abs_read_owner = M2_ABS_READ_OWNER_NONE;
                    g_m2_abs_move.poll_ms = M2_ABS_POLL_PERIOD_MS;
                    enc_rc = (poll_rc == 1) ? 0 : poll_rc;
                    have_result = 1U;
                }
            } else if (g_m2_abs_read_owner == M2_ABS_READ_OWNER_NONE) {
                if (g_m2_abs_move.poll_ms > 0U) {
                    g_m2_abs_move.poll_ms--;
                } else {
                    enc_rc = Rs485AbsEncoder_RequestM2Read();
                    if (enc_rc == 0) {
                        g_m2_abs_read_owner = M2_ABS_READ_OWNER_MOVE;
                    } else {
                        g_m2_abs_move.poll_ms = M2_ABS_POLL_PERIOD_MS;
                        have_result = 1U;
                    }
                }
            }

            if ((have_result != 0U) && (enc_rc != 0)) {
                if (g_m2_abs_move.comm_fail_count < M2_ABS_MAX_COMM_FAIL) {
                    g_m2_abs_move.comm_fail_count++;
                }
                if (g_m2_abs_move.comm_fail_count >= M2_ABS_MAX_COMM_FAIL) {
                    (void)MotorCtrl_EStop(MOTOR_ID_2);
                    g_m2_abs_move.active = 0U;
                    if (g_m2_abs_move.boot_home != 0U) {
                        g_m2_abs_boot_home_fault = 1U;
                    }
                    (void)snprintf(msg,
                                   sizeof(msg),
                                   "ERR 208 M2_ABS_COMM_%s",
                                   Rs485AbsEncoder_ErrorText(enc_rc));
                    (void)UartConsole_QueueLineDropOldest(msg, 0);
                }
            } else if ((have_result != 0U) && m2_abs_target_reached(abs_st.raw)) {
                /* 到达目标角度容差内，立即急停 M2 并输出完成消息。 */
                (void)MotorCtrl_EStop(MOTOR_ID_2);
                g_m2_abs_move.active = 0U;
                if (g_m2_abs_move.boot_home != 0U) {
                    g_m2_abs_boot_home_done = 1U;
                    (void)snprintf(msg,
                                   sizeof(msg),
                                   "OK M2 BOOT_HOME_DONE RAW=%u DEG=%.2f TARGET=%.2f TOL=%.2f",
                                   (unsigned int)abs_st.raw,
                                   abs_st.deg,
                                   g_m2_abs_move.target_deg,
                                   g_m2_abs_move.tol_deg);
                } else {
                    (void)snprintf(msg,
                                   sizeof(msg),
                                   "OK M2 ABS_MOVE_DONE RAW=%u DEG=%.2f TARGET=%.2f TOL=%.2f",
                                   (unsigned int)abs_st.raw,
                                   abs_st.deg,
                                   g_m2_abs_move.target_deg,
                                   g_m2_abs_move.tol_deg);
                }
                (void)UartConsole_QueueLineDropOldest(msg, 0);
            } else if (have_result != 0U) {
                int32_t delta = m2_abs_signed_delta(g_m2_abs_move.last_raw, abs_st.raw);
                uint16_t remain = m2_abs_remain_for_dir(abs_st.raw,
                                                        g_m2_abs_move.target_raw,
                                                        g_m2_abs_move.dir);
                uint8_t wrong_dir = 0U;

                g_m2_abs_move.comm_fail_count = 0U;
                /* 没有角度变化持续太久，认为 M2 失速。 */
                if (delta == 0) {
                    if (g_m2_abs_move.still_ms < M2_ABS_STALL_TIMEOUT_MS) {
                        g_m2_abs_move.still_ms = (uint16_t)(g_m2_abs_move.still_ms + M2_ABS_POLL_PERIOD_MS);
                    }
                } else {
                    if (m2_abs_delta_matches_dir(delta, g_m2_abs_move.dir) == 0U) {
                        wrong_dir = 1U;
                    }
                    g_m2_abs_move.last_raw = abs_st.raw;
                    g_m2_abs_move.still_ms = 0U;
                }

                /* 角度变化方向和目标方向相反，立即急停。 */
                if (wrong_dir != 0U) {
                    (void)MotorCtrl_EStop(MOTOR_ID_2);
                    g_m2_abs_move.active = 0U;
                    if (g_m2_abs_move.boot_home != 0U) {
                        g_m2_abs_boot_home_fault = 1U;
                    }
                    (void)snprintf(msg,
                                   sizeof(msg),
                                   "ERR 208 M2_ABS_WRONG_DIR RAW=%u TARGET_RAW=%u",
                                   (unsigned int)abs_st.raw,
                                   (unsigned int)g_m2_abs_move.target_raw);
                    (void)UartConsole_QueueLineDropOldest(msg, 0);
                } else if (g_m2_abs_move.still_ms >= M2_ABS_STALL_TIMEOUT_MS) {
                    (void)MotorCtrl_EStop(MOTOR_ID_2);
                    g_m2_abs_move.active = 0U;
                    if (g_m2_abs_move.boot_home != 0U) {
                        g_m2_abs_boot_home_fault = 1U;
                    }
                    (void)snprintf(msg,
                                   sizeof(msg),
                                   "ERR 208 M2_ABS_STALL RAW=%u TARGET_RAW=%u",
                                   (unsigned int)abs_st.raw,
                                   (unsigned int)g_m2_abs_move.target_raw);
                    (void)UartConsole_QueueLineDropOldest(msg, 0);
                } else if ((delta != 0) &&
                           (remain > (uint16_t)(g_m2_abs_move.last_remain + g_m2_abs_move.tol_counts))) {
                    /* 剩余距离反而变大，说明越过目标点，按完成处理并停止。 */
                    (void)MotorCtrl_EStop(MOTOR_ID_2);
                    g_m2_abs_move.active = 0U;
                    if (g_m2_abs_move.boot_home != 0U) {
                        g_m2_abs_boot_home_fault = 1U;
                        (void)snprintf(msg,
                                       sizeof(msg),
                                       "ERR 208 M2_BOOT_HOME_NOT_REACHED RAW=%u TARGET_RAW=%u OVERSHOOT=1",
                                       (unsigned int)abs_st.raw,
                                       (unsigned int)g_m2_abs_move.target_raw);
                    } else {
                        (void)snprintf(msg,
                                       sizeof(msg),
                                       "OK M2 ABS_MOVE_DONE RAW=%u DEG=%.2f TARGET=%.2f OVERSHOOT=1",
                                       (unsigned int)abs_st.raw,
                                       abs_st.deg,
                                       g_m2_abs_move.target_deg);
                    }
                    (void)UartConsole_QueueLineDropOldest(msg, 0);
                } else {
                    g_m2_abs_move.last_remain = remain;
                }
            }
        }
    }

    /* M1~M4 编码器目标运动监控，M5 夹爪不走增量编码器目标运动。 */
    for (i = 0U; i < (uint32_t)MOTOR_ID_5; i++) {
        EncMove_t *move = &g_enc_move[i];
        MotorId_t motor = (MotorId_t)i;
        EncoderStatus_t enc;
        char msg[160];

        if (move->active == 0U) {
            continue;
        }

        /* 电机停止、编码器读取失败、反向或长时间无位移都会终止该编码器目标运动。 */
        if (MotorCtrl_GetStatus(motor, &status) != 0) {
            move->active = 0U;
            move->done = 0U;
            move->fault = 1U;
        } else if ((status.mode != MOTOR_MODE_RUN) &&
                   (status.mode != MOTOR_MODE_RAMP_UP) &&
                   (status.mode != MOTOR_MODE_RAMP_DOWN)) {
            move->active = 0U;
            move->done = 0U;
            move->fault = 1U;
        } else if (EncoderCtrl_GetStatus(motor, &enc) != 0) {
            (void)MotorCtrl_EStop(motor);
            move->active = 0U;
            move->done = 0U;
            move->fault = 1U;
            (void)snprintf(msg, sizeof(msg), "ERR 206 M%u_EC%u_STATUS_FAIL",
                           (unsigned int)(i + 1U),
                           (unsigned int)(i + 1U));
            (void)UartConsole_QueueLineDropOldest(msg, 0);
        } else if (enc_move_target_reached(move, enc.count_total)) {
            (void)MotorCtrl_EStop(motor);
            move->active = 0U;
            move->done = 1U;
            move->fault = 0U;
            move->stall_boost_active = 0U;
            move->wrong_dir_count = 0U;
            move->wrong_dir_accum = 0U;
            (void)snprintf(msg,
                           sizeof(msg),
                           "OK M%u ENC_MOVE_DONE CNT=%ld TARGET=%ld",
                           (unsigned int)(i + 1U),
                           (long)enc.count_total,
                           (long)move->target_count);
            (void)UartConsole_QueueLineDropOldest(msg, 0);
        } else {
            int32_t delta = enc.count_total - move->last_count;
            int32_t prev_count = move->last_count;
            uint8_t wrong_dir = 0U;
            int32_t wrong_delta = 0L;
            uint32_t wrong_abs = 0UL;

            if (delta == 0) {
                if (move->still_ms < ENC_MOVE_STALL_TIMEOUT_MS) {
                    move->still_ms++;
                }
            } else {
                if (((move->count_increases != 0U) && (delta < 0)) ||
                    ((move->count_increases == 0U) && (delta > 0))) {
                    wrong_delta = delta;
                    wrong_abs = (uint32_t)((delta < 0) ? -delta : delta);
                    if (wrong_abs >= (uint32_t)ENC_MOVE_WRONG_DIR_IMMEDIATE_LIMIT) {
                        move->wrong_dir_count++;
                        move->wrong_dir_accum += wrong_abs;
                        wrong_dir = 1U;
                    } else if (wrong_abs > (uint32_t)ENC_MOVE_WRONG_DIR_TOL) {
                        move->wrong_dir_count++;
                        move->wrong_dir_accum += wrong_abs;
                        if ((move->wrong_dir_count >= ENC_MOVE_WRONG_DIR_COUNT_LIMIT) ||
                            (move->wrong_dir_accum >= ENC_MOVE_WRONG_DIR_ACCUM_LIMIT)) {
                            wrong_dir = 1U;
                        }
                    }
                } else {
                    move->wrong_dir_count = 0U;
                    move->wrong_dir_accum = 0U;
                }
                move->last_count = enc.count_total;
                move->still_ms = 0U;
                move->stall_boost_active = 0U;
            }

            if (wrong_dir != 0U) {
                (void)MotorCtrl_EStop(motor);
                move->active = 0U;
                move->done = 0U;
                move->fault = 1U;
                (void)snprintf(msg,
                               sizeof(msg),
                               "ERR 206 M%u_ENC_WRONG_DIR CNT=%ld TARGET=%ld LAST=%ld DELTA=%ld WCNT=%u WACC=%lu",
                               (unsigned int)(i + 1U),
                               (long)enc.count_total,
                               (long)move->target_count,
                               (long)prev_count,
                               (long)wrong_delta,
                               (unsigned int)move->wrong_dir_count,
                               (unsigned long)move->wrong_dir_accum);
                (void)UartConsole_QueueLineDropOldest(msg, 0);
            } else if (move->still_ms >= ENC_MOVE_STALL_TIMEOUT_MS) {
                (void)MotorCtrl_EStop(motor);
                move->active = 0U;
                move->done = 0U;
                move->fault = 1U;
                (void)snprintf(msg,
                               sizeof(msg),
                               "ERR 206 M%u_ENC_STALL CNT=%ld TARGET=%ld LAST=%ld RPM=%.2f STILL=%lu BOOST=%u MODE=%s DIR=%s STEP=%lu",
                               (unsigned int)(i + 1U),
                               (long)enc.count_total,
                               (long)move->target_count,
                               (long)move->last_count,
                               (double)move->pid_last_rpm,
                               (unsigned long)move->still_ms,
                               (unsigned int)move->stall_boost_active,
                               MotorCtrl_ModeToString(status.mode),
                               MotorCtrl_DirToString(status.dir),
                               (unsigned long)status.steps_total);
                (void)UartConsole_QueueLineDropOldest(msg, 0);
            } else if ((move->still_ms >= ENC_MOVE_STALL_BOOST_START_MS) &&
                       (move->stall_boost_active == 0U) &&
                       (enc_move_apply_stall_boost(motor, move) != 0)) {
                (void)MotorCtrl_EStop(motor);
                move->active = 0U;
                move->done = 0U;
                move->fault = 1U;
                (void)snprintf(msg,
                               sizeof(msg),
                               "ERR 206 M%u_ENC_BOOST_SPEED_FAIL CNT=%ld TARGET=%ld",
                               (unsigned int)(i + 1U),
                               (long)enc.count_total,
                               (long)move->target_count);
                (void)UartConsole_QueueLineDropOldest(msg, 0);
            } else if (enc_move_pid_update(motor, move, enc.count_total) != 0) {
                (void)MotorCtrl_EStop(motor);
                move->active = 0U;
                move->done = 0U;
                move->fault = 1U;
                (void)snprintf(msg,
                               sizeof(msg),
                               "ERR 206 M%u_ENC_PID_SPEED_FAIL CNT=%ld TARGET=%ld",
                               (unsigned int)(i + 1U),
                               (long)enc.count_total,
                               (long)move->target_count);
                (void)UartConsole_QueueLineDropOldest(msg, 0);
            }
        }
    }

    /* 回零限位监控：EX_KEY 触发低电平后急停对应电机，并标记 ALL_HOME 单轴完成。 */
    for (i = 0U; i < REV_EXKEY_GUARD_COUNT; i++) {
        RevExKeyGuard_t *guard = &g_rev_exkey_guards[i];

        if (guard->active == 0U) {
            continue;
        }

        if (MotorCtrl_GetStatus(guard->motor_id, &status) != 0) {
            guard->active = 0U;
            guard->blocked_stable_ms = 0U;
            if (guard->motor_id == MOTOR_ID_4) {
                m4_home_runtime_reset();
            }
            if ((g_all_home_active != 0U) &&
                ((g_all_home_target_mask & motor_bit(guard->motor_id)) != 0U) &&
                ((g_all_home_done_mask & motor_bit(guard->motor_id)) == 0U)) {
                if (g_place_cloth.active != 0U) {
                    cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, 0xFFU);
                }
                if (g_take_cloth.active != 0U) {
                    cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, 0xFFU);
                }
                rev_exkey_cancel_all();
                (void)UartConsole_QueueLineDropOldest("ERR 203 ALL_HOME_ABORT_STATUS_FAIL", 0);
            }
            continue;
        }

        if ((status.mode != MOTOR_MODE_RUN) &&
            (status.mode != MOTOR_MODE_RAMP_UP) &&
            (status.mode != MOTOR_MODE_RAMP_DOWN)) {
            guard->active = 0U;
            guard->blocked_stable_ms = 0U;
            if (guard->motor_id == MOTOR_ID_4) {
                m4_home_runtime_reset();
            }
            if ((g_all_home_active != 0U) &&
                ((g_all_home_target_mask & motor_bit(guard->motor_id)) != 0U) &&
                ((g_all_home_done_mask & motor_bit(guard->motor_id)) == 0U)) {
                if (g_place_cloth.active != 0U) {
                    cloth_event_set(CMD_CLOTH_EVENT_PLACE_FAULT, 0xFFU);
                }
                if (g_take_cloth.active != 0U) {
                    cloth_event_set(CMD_CLOTH_EVENT_TAKE_FAULT, 0xFFU);
                }
                rev_exkey_cancel_all();
                (void)UartConsole_QueueLineDropOldest("ERR 203 ALL_HOME_ABORT_MOTOR_STOPPED", 0);
            }
            continue;
        }

        if (guard->motor_id == MOTOR_ID_4) {
            m4_home_tick(guard);
            continue;
        }

        if (rev_exkey_is_blocked_low(guard)) {
            if (guard->blocked_stable_ms < HOME_EXKEY_STABLE_MS) {
                guard->blocked_stable_ms++;
            }
        } else {
            guard->blocked_stable_ms = 0U;
        }

        if (guard->blocked_stable_ms >= HOME_EXKEY_STABLE_MS) {
            char msg[80];

            (void)MotorCtrl_EStop(guard->motor_id);
            guard->active = 0U;
            (void)snprintf(msg,
                           sizeof(msg),
                           "OK M%u HOME_EX%u_DONE EX_KEY%u_LOW_STABLE=%uMS",
                           (unsigned int)(guard->motor_id + 1U),
                           (unsigned int)guard->key_index,
                           (unsigned int)guard->key_index,
                           (unsigned int)HOME_EXKEY_STABLE_MS);
            guard->blocked_stable_ms = 0U;
            (void)UartConsole_QueueLineDropOldest(msg, 0);
            if (guard->motor_id == MOTOR_ID_3) {
                if ((g_place_cloth.active != 0U) &&
                    ((g_place_cloth.step == PLACE_CLOTH_WAIT_HOME_Y) ||
                     (g_place_cloth.step == PLACE_CLOTH_WAIT_HOME_Y_FINAL) ||
                     (g_place_cloth.step == PLACE_CLOTH_ALL_HOME))) {
                    g_place_cloth.y_home_done = 1U;
                }
                if ((g_take_cloth.active != 0U) &&
                    ((g_take_cloth.step == TAKE_CLOTH_WAIT_HOME_Y) ||
                     (g_take_cloth.step == TAKE_CLOTH_ALL_HOME))) {
                    g_take_cloth.y_home_done = 1U;
                }
            }
            all_home_mark_done(guard->motor_id);
        }
    }
}

bool CmdDispatch_PollClothEvent(CmdClothEvent_t *event, uint8_t *fault_code)
{
    /*
     * 主循环调用本函数轮询取/放衣事件。
     * 成功事件会被 main.c 转成 WARDROBE_RESP_SUCCESS；
     * 故障事件会被 main.c 转成 WARDROBE_RESP_FAULT。
     */
    if ((event == 0) || (fault_code == 0)) {
        return false;
    }

    if (g_cloth_event == CMD_CLOTH_EVENT_NONE) {
        return false;
    }

    *event = g_cloth_event;
    *fault_code = g_cloth_fault_code;
    g_cloth_event = CMD_CLOTH_EVENT_NONE;
    g_cloth_fault_code = 0U;
    return true;
}

static int args_check(uint32_t have_mask, uint32_t required_mask, uint32_t allowed_mask)
{
    /* required_mask 检查必填参数，allowed_mask 检查是否传入了当前命令不允许的多余参数。 */
    if ((have_mask & required_mask) != required_mask) {
        return CMD_ERR_MISSING_ARG;
    }
    if ((have_mask & (~allowed_mask)) != 0U) {
        return CMD_ERR_BAD_ARG;
    }
    return 0;
}

static CmdResult_t resp_vformat(char *resp, uint16_t resp_len, const char *fmt, va_list ap)
{
    int written;

    /* 安全格式化响应字符串，缓冲区不足时返回错误。 */
    if ((resp == 0) || (resp_len == 0U) || (fmt == 0)) {
        return CMD_RESULT_ERR;
    }

    written = vsnprintf(resp, resp_len, fmt, ap);
    if ((written < 0) || (written >= (int)resp_len)) {
        resp[resp_len - 1U] = '\0';
        return CMD_RESULT_ERR;
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_format(char *resp, uint16_t resp_len, const char *fmt, ...)
{
    CmdResult_t ret;
    va_list ap;

    /* resp_vformat() 的可变参数包装函数，所有 OK/STAT 响应基本都从这里生成。 */
    va_start(ap, fmt);
    ret = resp_vformat(resp, resp_len, fmt, ap);
    va_end(ap);

    return ret;
}

static size_t bounded_strlen(const char *text, size_t max_len)
{
    size_t len = 0U;

    /* 限长 strlen，防止响应缓冲区没有 NUL 结尾时越界扫描。 */
    if (text == 0) {
        return 0U;
    }

    while ((len < max_len) && (text[len] != '\0')) {
        len++;
    }

    return len;
}

static CmdResult_t resp_err(char *resp, uint16_t resp_len, uint16_t code, const char *msg)
{
    /* 统一错误响应格式：ERR 错误码 错误文本。 */
    if (msg == 0) {
        msg = "UNKNOWN";
    }

    if (resp_format(resp, resp_len, "ERR %u %s", (unsigned int)code, msg) != CMD_RESULT_OK) {
        return CMD_RESULT_ERR;
    }
    return CMD_RESULT_ERR;
}

static int resp_append(char *resp, uint16_t resp_len, const char *fmt, ...)
{
    va_list ap;
    size_t used;
    int written;

    /* 在已有响应尾部追加文本，用于拼接多电机状态。 */
    if ((resp == 0) || (resp_len == 0U) || (fmt == 0)) {
        return -1;
    }

    used = bounded_strlen(resp, resp_len);
    if (used >= resp_len) {
        return -1;
    }

    va_start(ap, fmt);
    written = vsnprintf(resp + used, resp_len - (uint16_t)used, fmt, ap);
    va_end(ap);

    if ((written < 0) || (written >= (int)(resp_len - used))) {
        return -1;
    }
    return 0;
}

static CmdResult_t resp_motor_status(MotorId_t id, char *resp, uint16_t resp_len)
{
    MotorStatusSnapshot_t st;

    /* 单电机状态响应，包含模式、方向、当前/目标转速、配置和累计步数。 */
    if (MotorCtrl_GetStatus(id, &st) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "STATUS_READ_FAIL");
    }

    if (resp_format(resp,
                    resp_len,
                    "STAT M%u MODE=%s DIR=%s RPM=%.2f TARGET=%.2f PPR=%lu ACC=%.2f DEC=%.2f STOP=%lu STEP_TOTAL=%lu STEP_REM=%lu CYCLE=%lu/%lu",
                    (unsigned int)(id + 1U),
                    MotorCtrl_ModeToString(st.mode),
                    MotorCtrl_DirToString(st.dir),
                    st.rpm_current,
                    st.rpm_target,
                    (unsigned long)st.ppr,
                    st.acc_rpm_s,
                    st.dec_rpm_s,
                    (unsigned long)st.soft_stop_ms,
                    (unsigned long)st.steps_total,
                    (unsigned long)st.steps_remaining,
                    (unsigned long)st.recip_cycles_done,
                    (unsigned long)st.recip_cycles) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_all_status(char *resp, uint16_t resp_len)
{
    uint32_t i;
    MotorStatusSnapshot_t st;

    /* 拼接 M1~M5 的简要状态。 */
    resp[0] = '\0';
    if (resp_append(resp, resp_len, "STAT ALL") != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (MotorCtrl_GetStatus((MotorId_t)i, &st) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_STATUS, "STATUS_READ_FAIL");
        }

        if (resp_append(resp,
                        resp_len,
                        " | M%u MODE=%s DIR=%s RPM=%.2f TARGET=%.2f STEP=%lu REM=%lu",
                        (unsigned int)(i + 1U),
                        MotorCtrl_ModeToString(st.mode),
                        MotorCtrl_DirToString(st.dir),
                        st.rpm_current,
                        st.rpm_target,
                        (unsigned long)st.steps_total,
                        (unsigned long)st.steps_remaining) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    if (resp_append(resp,
                    resp_len,
                    " | MAGNET CMD=%s PIN=%s FEEDBACK=NONE",
                    (Electromagnet_IsOn() != 0U) ? "ON" : "OFF",
                    (Electromagnet_PinHigh() != 0U) ? "HIGH" : "LOW") != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_encoder_status(MotorId_t id, char *resp, uint16_t resp_len)
{
    EncoderStatus_t st;

    /* 单个增量编码器状态响应。 */
    if (EncoderCtrl_GetStatus(id, &st) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "ENC_STATUS_FAIL");
    }

    if (resp_format(resp,
                    resp_len,
                    "ENC M%u CNT=%ld DIR=%s RPM=%.2f",
                    (unsigned int)(id + 1U),
                    (long)st.count_total,
                    MotorCtrl_DirToString(st.dir),
                    st.rpm) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_m2_abs_status(char *resp, uint16_t resp_len)
{
    Rs485AbsEncoderStatus_t st;
    int rc;

    /* M2 绝对值编码器状态响应，直接从 RS485 编码器读取当前 raw 和角度。 */
    rc = Rs485AbsEncoder_ReadM2(&st);
    if (rc != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RS485_ENCODER_OP, Rs485AbsEncoder_ErrorText(rc));
    }

    if (resp_format(resp,
                    resp_len,
                    "ABS M2 ADDR=%u RAW=%u DEG=%.2f CPR=%u",
                    (unsigned int)st.addr,
                    (unsigned int)st.raw,
                    st.deg,
                    (unsigned int)RS485_ABS_ENCODER_CPR) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_m2_abs_zero_set(char *resp, uint16_t resp_len)
{
    Rs485AbsEncoderStatus_t st;
    int rc;

    /* 把 M2 当前绝对值编码器位置设置为零点，然后读回确认。 */
    rc = Rs485AbsEncoder_SetM2CurrentZero();
    if (rc != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RS485_ENCODER_OP, Rs485AbsEncoder_ErrorText(rc));
    }

    rc = Rs485AbsEncoder_ReadM2(&st);
    if (rc != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RS485_ENCODER_OP, Rs485AbsEncoder_ErrorText(rc));
    }

    if (resp_format(resp,
                    resp_len,
                    "OK M2 ABS_ZERO RAW=%u DEG=%.2f",
                    (unsigned int)st.raw,
                    st.deg) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_all_encoder_status(char *resp, uint16_t resp_len)
{
    uint32_t i;
    EncoderStatus_t st;

    /* 拼接 EC1~EC4 的编码器状态；EC5 已停用，所以只遍历到 MOTOR_ID_5 之前。 */
    resp[0] = '\0';
    if (resp_append(resp, resp_len, "ENC ALL") != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    for (i = 0U; i < (uint32_t)MOTOR_ID_5; i++) {
        if (EncoderCtrl_GetStatus((MotorId_t)i, &st) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_STATUS, "ENC_STATUS_FAIL");
        }

        if (resp_append(resp,
                        resp_len,
                        " | M%u CNT=%ld DIR=%s RPM=%.2f",
                        (unsigned int)(i + 1U),
                        (long)st.count_total,
                        MotorCtrl_DirToString(st.dir),
                        st.rpm) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_encoder_target_status(MotorId_t encoder_id, char *resp, uint16_t resp_len)
{
    EncoderStatus_t st;
    uint32_t raw;
    bool a_high = false;
    bool b_high = false;
    float deg;
    unsigned int ec_index;

    /* EC1~EC4 详细状态，包含累计计数、硬件原始计数、AB 相电平、方向和换算角度。 */
    if (encoder_id >= MOTOR_ID_5) {
        return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "EC_ID_RANGE");
    }
    ec_index = (unsigned int)(encoder_id + 1U);

    if (EncoderCtrl_GetStatus(encoder_id, &st) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "EC_STATUS_FAIL");
    }
    if (EncoderHw_ReadAB(encoder_id, &a_high, &b_high) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "EC_AB_FAIL");
    }

    raw = EncoderHw_ReadCounter(encoder_id);
    deg = ((float)st.count_total * 360.0f) / ENCODER_COUNTS_PER_REV;

    if (resp_format(resp,
                    resp_len,
                    "EC%u M%u CNT=%ld RAW=%lu A=%u B=%u DIR=%s DEG=%.2f CPR=1440",
                    ec_index,
                    ec_index,
                    (long)st.count_total,
                    (unsigned long)raw,
                    a_high ? 1U : 0U,
                    b_high ? 1U : 0U,
                    MotorCtrl_DirToString(st.dir),
                    deg) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static const char *gpio_level_text(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? "H" : "L";
}

static const char *gpio_level_text_zh(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? "高" : "低";
}

static const char *ex_key1_state_text(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY1_GPIO_Port, EX_KEY1_Pin) == GPIO_PIN_RESET) ? "BLOCK" : "CLEAR";
}

static const char *ex_key2_state_text(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY2_GPIO_Port, EX_KEY2_Pin) == GPIO_PIN_RESET) ? "BLOCK" : "CLEAR";
}

static const char *ex_key3_state_text(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY3_GPIO_Port, EX_KEY3_Pin) == GPIO_PIN_RESET) ? "BLOCK" : "CLEAR";
}

static const char *ex_key4_state_text(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY4_GPIO_Port, EX_KEY4_Pin) == GPIO_PIN_RESET) ? "BLOCK" : "CLEAR";
}

static const char *ex_key1_state_text_zh(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY1_GPIO_Port, EX_KEY1_Pin) == GPIO_PIN_RESET) ? "触发" : "未触发";
}

static const char *ex_key2_state_text_zh(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY2_GPIO_Port, EX_KEY2_Pin) == GPIO_PIN_RESET) ? "触发" : "未触发";
}

static const char *ex_key3_state_text_zh(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY3_GPIO_Port, EX_KEY3_Pin) == GPIO_PIN_RESET) ? "触发" : "未触发";
}

static const char *ex_key4_state_text_zh(void)
{
    return (HAL_GPIO_ReadPin(EX_KEY4_GPIO_Port, EX_KEY4_Pin) == GPIO_PIN_RESET) ? "触发" : "未触发";
}

static const char *motor_mode_text_zh(MotorMode_t mode)
{
    switch (mode) {
        case MOTOR_MODE_IDLE: return "空闲";
        case MOTOR_MODE_RUN: return "运行";
        case MOTOR_MODE_RAMP_UP: return "加速";
        case MOTOR_MODE_RAMP_DOWN: return "减速";
        case MOTOR_MODE_MOVE_STEPS: return "定步";
        case MOTOR_MODE_RECIP_FINITE: return "往复";
        case MOTOR_MODE_RECIP_INFINITE: return "无限往复";
        case MOTOR_MODE_SOFT_STOP: return "软停";
        case MOTOR_MODE_ESTOP: return "急停";
        default: return "未知";
    }
}

static const char *motor_dir_text_zh(MotorDir_t dir)
{
    return (dir == MOTOR_DIR_CCW) ? "反转" : "正转";
}

static const char *bool_text_zh(bool value)
{
    return value ? "是" : "否";
}

static const char *switch_text_zh(bool enabled)
{
    return enabled ? "开" : "关";
}

CmdResult_t CmdDispatch_FormatSystemStatus(char *resp, uint16_t resp_len)
{
    uint32_t i;
    MotorStatusSnapshot_t mst;
    EncoderStatus_t est;
    EncoderStatus_t ec4;
    AxisPos_t pos;
    AxisLimitConfig_t cfg;
    AxisLimitFault_t fault;
    bool a_high = false;
    bool b_high = false;
    uint32_t raw;
    float deg;

    /*
     * 生成中文系统状态文本。
     * 包含电机、编码器、EC4、XYZ 行程保护、EX_KEY 和按键状态，主要供人工调试查看。
     */
    resp[0] = '\0';
    if (resp_append(resp, resp_len, "系统状态 电机") != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (MotorCtrl_GetStatus((MotorId_t)i, &mst) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_STATUS, "STATUS_READ_FAIL");
        }

        if (resp_append(resp,
                        resp_len,
                        " M%u=%s,%s,当前%.1f/目标%.1f,余%lu",
                        (unsigned int)(i + 1U),
                        motor_mode_text_zh(mst.mode),
                        motor_dir_text_zh(mst.dir),
                        mst.rpm_current,
                        mst.rpm_target,
                        (unsigned long)mst.steps_remaining) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    if (resp_append(resp, resp_len, " | 编码器") != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    for (i = 0U; i < (uint32_t)MOTOR_ID_5; i++) {
        if (EncoderCtrl_GetStatus((MotorId_t)i, &est) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_STATUS, "ENC_STATUS_FAIL");
        }

        if (resp_append(resp,
                        resp_len,
                        " E%u=计数%ld,转速%.1f",
                        (unsigned int)(i + 1U),
                        (long)est.count_total,
                        est.rpm) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    if (EncoderCtrl_GetStatus(M4_ENCODER_ID, &ec4) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "EC4_STATUS_FAIL");
    }
    if (EncoderHw_ReadAB(M4_ENCODER_ID, &a_high, &b_high) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "EC4_AB_FAIL");
    }
    raw = EncoderHw_ReadCounter(M4_ENCODER_ID);
    deg = ((float)ec4.count_total * 360.0f) / ENCODER_COUNTS_PER_REV;

    if (resp_append(resp,
                    resp_len,
                    " | EC4(M4) 计数=%ld 原始=%lu A=%u B=%u 角度=%.1f",
                    (long)ec4.count_total,
                    (unsigned long)raw,
                    a_high ? 1U : 0U,
                    b_high ? 1U : 0U,
                    deg) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    if (AxisCoord_GetAll(&pos) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "AXIS_POS_FAIL");
    }
    if (AxisLimit_Get(&cfg) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_LIMIT_LOAD, "LIMIT_LOAD_FAIL");
    }
    if (AxisLimit_GetFault(&fault) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "LIMIT_FAULT_FAIL");
    }

    if (resp_append(resp,
                    resp_len,
                    " | 行程 X=%ld Y=%ld Z=%ld 保护=%s 有效=%s 最大=%lu/%lu/%lu 故障=%s",
                    (long)pos.x,
                    (long)pos.y,
                    (long)pos.z,
                    switch_text_zh(AxisLimit_IsProtectEnabled()),
                    bool_text_zh(cfg.valid),
                    (unsigned long)cfg.x_max,
                    (unsigned long)cfg.y_max,
                    (unsigned long)cfg.z_max,
                    bool_text_zh(fault.active)) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    if (fault.active) {
        if (resp_append(resp,
                        resp_len,
                        ",轴=%s,边界=%s,当前=%ld",
                        AxisLimit_AxisToString(fault.axis),
                        AxisLimit_BoundToString(fault.bound),
                        (long)fault.value) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    if (resp_append(resp,
                    resp_len,
                    " | 开关 EX1=%s EX2=%s EX3=%s EX4=%s K1=%s K2=%s K3=%s",
                    ex_key1_state_text_zh(),
                    ex_key2_state_text_zh(),
                    ex_key3_state_text_zh(),
                    ex_key4_state_text_zh(),
                    gpio_level_text_zh(KEY1_GPIO_Port, KEY1_Pin),
                    gpio_level_text_zh(KEY2_GPIO_Port, KEY2_Pin),
                    gpio_level_text_zh(KEY3_GPIO_Port, KEY3_Pin)) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    if (resp_append(resp,
                    resp_len,
                    " | 电磁铁=%s PD8=%s 反馈=无",
                    (Electromagnet_IsOn() != 0U) ? "吸合" : "释放",
                    (Electromagnet_PinHigh() != 0U) ? "高" : "低") != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_curve_status(MotorId_t id, char *resp, uint16_t resp_len)
{
    CurveStatus_t st;

    /* 查询单个电机的曲线遥测开关、采样率和丢包计数。 */
    if (TelemetryCurve_GetStatus(id, &st) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "CURVE_STATUS_FAIL");
    }

    if (resp_format(resp,
                    resp_len,
                    "CURVE_STAT M%u EN=%u RATE=%u LOST=%lu",
                    (unsigned int)(id + 1U),
                    st.enabled ? 1U : 0U,
                    (unsigned int)st.rate_hz,
                    (unsigned long)st.lost) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_m3_rot_status(char *resp, uint16_t resp_len)
{
    int32_t steps = 0;
    float deg_dummy = 0.0f;

    /* 查询旋转轴当前位置。历史命名为 M3Rot，但当前业务注释中对应 M2 旋转轴。 */
    if (AxisCoord_M3RotGet(&steps, &deg_dummy) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "M2_ROT_STATUS_FAIL");
    }

    if (resp_format(resp,
                    resp_len,
                    "ROT M2 STEP=%ld",
                    (long)steps) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_m3_rot_limit_status(char *resp, uint16_t resp_len)
{
    AxisRotLimitStatus_t st;

    /* 查询旋转轴行程限制配置和当前是否触发行程故障。 */
    if (AxisLimit_GetM3RotStatus(&st) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_LIMIT_LOAD, "ROT_LIMIT_LOAD_FAIL");
    }

    if (resp_format(resp,
                    resp_len,
                    "ROT_LIMIT M2 VALID=%u MAX_STEP=%lu FAULT=%u",
                    st.valid ? 1U : 0U,
                    (unsigned long)st.max_step,
                    st.fault_active ? 1U : 0U) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    if (st.fault_active) {
        if (resp_append(resp,
                        resp_len,
                        " BOUND=%s CUR=%ld MAX=%lu",
                        AxisLimit_BoundToString(st.fault_bound),
                        (long)st.fault_value,
                        (unsigned long)st.max_step) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_all_curve_status(char *resp, uint16_t resp_len)
{
    uint32_t i;
    CurveStatus_t st;

    /* 拼接全部电机的曲线遥测状态。 */
    resp[0] = '\0';
    if (resp_append(resp, resp_len, "CURVE_STAT ALL") != 0) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (TelemetryCurve_GetStatus((MotorId_t)i, &st) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_STATUS, "CURVE_STATUS_FAIL");
        }

        if (resp_append(resp,
                        resp_len,
                        " | M%u EN=%u RATE=%u LOST=%lu",
                        (unsigned int)(i + 1U),
                        st.enabled ? 1U : 0U,
                        (unsigned int)st.rate_hz,
                        (unsigned long)st.lost) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_axis_limit_get(char *resp, uint16_t resp_len)
{
    AxisLimitConfig_t cfg;

    if (AxisLimit_Get(&cfg) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_LIMIT_LOAD, "LIMIT_LOAD_FAIL");
    }

    return resp_format(resp,
                       resp_len,
                       "AXIS LIMIT_GET VALID=%u MAX_X=%lu MAX_Y=%lu MAX_Z=%lu",
                       cfg.valid ? 1U : 0U,
                       (unsigned long)cfg.x_max,
                       (unsigned long)cfg.y_max,
                       (unsigned long)cfg.z_max);
}

static CmdResult_t resp_axis_status(char *resp, uint16_t resp_len)
{
    AxisPos_t pos;
    AxisLimitConfig_t cfg;
    AxisLimitFault_t fault;
    AxisPlanStatus_t plan;
    bool protect_enabled;

    if (AxisCoord_GetAll(&pos) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "AXIS_POS_FAIL");
    }
    if (AxisLimit_Get(&cfg) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_LIMIT_LOAD, "LIMIT_LOAD_FAIL");
    }
    if (AxisLimit_GetFault(&fault) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "LIMIT_FAULT_FAIL");
    }
    if (AxisPlanner_GetStatus(&plan) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "PLAN_STATUS_FAIL");
    }
    protect_enabled = AxisLimit_IsProtectEnabled();

    if (resp_format(resp,
                    resp_len,
                    "AXIS STAT X=%ld Y=%ld Z=%ld PROTECT=%u VALID=%u MAX_X=%lu MAX_Y=%lu MAX_Z=%lu FAULT=%u PLAN=%s TARGET_X=%ld TARGET_Y=%ld TARGET_Z=%ld",
                    (long)pos.x,
                    (long)pos.y,
                    (long)pos.z,
                    protect_enabled ? 1U : 0U,
                    cfg.valid ? 1U : 0U,
                    (unsigned long)cfg.x_max,
                    (unsigned long)cfg.y_max,
                    (unsigned long)cfg.z_max,
                    fault.active ? 1U : 0U,
                    AxisPlanner_StateToString(plan.state),
                    (long)plan.target.x,
                    (long)plan.target.y,
                    (long)plan.target.z) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    if (fault.active) {
        if (resp_append(resp,
                        resp_len,
                        " AX=%s BOUND=%s CUR=%ld MAX=%lu",
                        AxisLimit_AxisToString(fault.axis),
                        AxisLimit_BoundToString(fault.bound),
                        (long)fault.value,
                        (unsigned long)fault.max) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    return CMD_RESULT_OK;
}

static CmdResult_t resp_axis_protect_status(char *resp, uint16_t resp_len)
{
    AxisLimitConfig_t cfg;
    AxisLimitFault_t fault;
    bool protect_enabled;

    if (AxisLimit_Get(&cfg) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_LIMIT_LOAD, "LIMIT_LOAD_FAIL");
    }
    if (AxisLimit_GetFault(&fault) != 0) {
        return resp_err(resp, resp_len, CMD_ERR_STATUS, "LIMIT_FAULT_FAIL");
    }

    protect_enabled = AxisLimit_IsProtectEnabled();
    if (resp_format(resp,
                    resp_len,
                    "AXIS PROTECT STATUS PROTECT=%u VALID=%u MAX_X=%lu MAX_Y=%lu MAX_Z=%lu FAULT=%u",
                    protect_enabled ? 1U : 0U,
                    cfg.valid ? 1U : 0U,
                    (unsigned long)cfg.x_max,
                    (unsigned long)cfg.y_max,
                    (unsigned long)cfg.z_max,
                    fault.active ? 1U : 0U) != CMD_RESULT_OK) {
        return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
    }

    if (fault.active) {
        if (resp_append(resp,
                        resp_len,
                        " AX=%s BOUND=%s CUR=%ld MAX=%lu",
                        AxisLimit_AxisToString(fault.axis),
                        AxisLimit_BoundToString(fault.bound),
                        (long)fault.value,
                        (unsigned long)fault.max) != 0) {
            return resp_err(resp, resp_len, CMD_ERR_RESP_OVERFLOW, "RESP_TOO_LONG");
        }
    }

    return CMD_RESULT_OK;
}

static CmdResult_t handle_motor_cmd(const ParsedCmd_t *cmd, char *resp, uint16_t resp_len)
{
    uint32_t have_mask;
    uint8_t cloth_task_active;
    int rc;

    /*
     * 单电机命令处理：
     * 目标格式通常是 M1 RUN/STOP/MOVE/ENC_MOVE/ABS_MOVE 等。
     * 如果取/放衣流程正在运行，只允许 STOP/ESTOP 介入，避免人工命令破坏流程状态。
     */
    if ((cmd->motor_id >= MOTOR_COUNT) || (cmd->motor_id == MOTOR_ID_INVALID)) {
        return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "BAD_MOTOR_ID");
    }

    have_mask = args_mask(&cmd->args);
    cloth_task_active = ((g_place_cloth.active != 0U) || (g_take_cloth.active != 0U)) ? 1U : 0U;

    if ((cloth_task_active != 0U) &&
        (cmd->action != CMD_ACTION_STOP) &&
        (cmd->action != CMD_ACTION_ESTOP)) {
        return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "CLOTH_TASK_BUSY");
    }

    switch (cmd->action) {
        case CMD_ACTION_RUN:
            rc = args_check(have_mask, ARG_DIR | ARG_RPM, ARG_DIR | ARG_RPM);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "RUN_ARGS");
            rc = MotorCtrl_Run(cmd->motor_id, cmd->args.dir, cmd->args.rpm);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "RUN_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_format(resp,
                               resp_len,
                               "OK M%u RUN DIR=%s RPM=%.2f",
                               (unsigned int)(cmd->motor_id + 1U),
                               MotorCtrl_DirToString(cmd->args.dir),
                               cmd->args.rpm);

        case CMD_ACTION_STOP:
            rc = args_check(have_mask, 0U, ARG_T);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "STOP_ARGS");
            rc = MotorCtrl_Stop(cmd->motor_id, cmd->args.has_time_ms ? cmd->args.time_ms : 0U);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "STOP_FAIL");
            if (cloth_task_active != 0U) {
                place_cloth_stop_motion(0U);
            }
            place_cloth_cancel();
            take_cloth_cancel();
            if (cloth_task_active == 0U) {
                rev_exkey_cancel_if_motor(cmd->motor_id);
            }
            if (cmd->args.has_time_ms) {
                return resp_format(resp,
                                   resp_len,
                                   "OK M%u STOP T=%lu",
                                   (unsigned int)(cmd->motor_id + 1U),
                                   (unsigned long)cmd->args.time_ms);
            }
            return resp_format(resp, resp_len, "OK M%u STOP T=DEFAULT", (unsigned int)(cmd->motor_id + 1U));

        case CMD_ACTION_ESTOP:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ESTOP_ARGS");
            rc = MotorCtrl_EStop(cmd->motor_id);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "ESTOP_FAIL");
            if (cloth_task_active != 0U) {
                place_cloth_stop_motion(1U);
            }
            place_cloth_cancel();
            take_cloth_cancel();
            if (cloth_task_active == 0U) {
                rev_exkey_cancel_if_motor(cmd->motor_id);
            }
            return resp_format(resp, resp_len, "OK M%u ESTOP", (unsigned int)(cmd->motor_id + 1U));

        case CMD_ACTION_SPEED:
            rc = args_check(have_mask, ARG_RPM, ARG_RPM);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "SPEED_ARGS");
            rc = MotorCtrl_SetSpeed(cmd->motor_id, cmd->args.rpm);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "SPEED_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_format(resp,
                               resp_len,
                               "OK M%u SPEED RPM=%.2f",
                               (unsigned int)(cmd->motor_id + 1U),
                               cmd->args.rpm);

        case CMD_ACTION_ACCEL:
            rc = args_check(have_mask, ARG_TO | ARG_T, ARG_TO | ARG_T);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ACCEL_ARGS");
            rc = MotorCtrl_AccelTo(cmd->motor_id, cmd->args.to_rpm, cmd->args.time_ms);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "ACCEL_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_format(resp,
                               resp_len,
                               "OK M%u ACCEL TO=%.2f T=%lu",
                               (unsigned int)(cmd->motor_id + 1U),
                               cmd->args.to_rpm,
                               (unsigned long)cmd->args.time_ms);

        case CMD_ACTION_DECEL:
            rc = args_check(have_mask, ARG_TO | ARG_T, ARG_TO | ARG_T);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "DECEL_ARGS");
            rc = MotorCtrl_DecelTo(cmd->motor_id, cmd->args.to_rpm, cmd->args.time_ms);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "DECEL_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_format(resp,
                               resp_len,
                               "OK M%u DECEL TO=%.2f T=%lu",
                               (unsigned int)(cmd->motor_id + 1U),
                               cmd->args.to_rpm,
                               (unsigned long)cmd->args.time_ms);

        case CMD_ACTION_MOVE:
            rc = args_check(have_mask, ARG_DIR | ARG_RPM | ARG_STEP, ARG_DIR | ARG_RPM | ARG_STEP);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "MOVE_ARGS");
            rc = MotorCtrl_MoveSteps(cmd->motor_id, cmd->args.dir, cmd->args.rpm, cmd->args.step);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MOVE_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_format(resp,
                               resp_len,
                               "OK M%u MOVE DIR=%s RPM=%.2f STEP=%lu",
                               (unsigned int)(cmd->motor_id + 1U),
                               MotorCtrl_DirToString(cmd->args.dir),
                               cmd->args.rpm,
                               (unsigned long)cmd->args.step);

        case CMD_ACTION_ENC_MOVE:
        {
            int32_t start_count = 0;
            int32_t target_count = 0;
            uint32_t tol;

            rc = args_check(have_mask,
                            ARG_DIR | ARG_RPM | ARG_STEP,
                            ARG_DIR | ARG_RPM | ARG_STEP | ARG_TOL);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ENC_MOVE_ARGS");
            if (enc_move_motor_supported(cmd->motor_id) == 0U) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M1_M3_M4_ONLY");
            }
            if (cmd->args.step == 0U) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "CNT_ZERO");
            if (cmd->args.has_tol && ((cmd->args.tol == 0U) || (cmd->args.tol > 1000U))) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "TOL_RANGE");
            }
            tol = cmd->args.has_tol ? cmd->args.tol : ENC_MOVE_DEFAULT_TOL;
            rc = enc_move_start_relative(cmd->motor_id,
                                         cmd->args.dir,
                                         cmd->args.rpm,
                                         cmd->args.step,
                                         tol,
                                         &start_count,
                                         &target_count);
            if (rc == -2) return resp_err(resp, resp_len, CMD_ERR_STATUS, "EC_STATUS_FAIL");
            if (rc == -3) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "TARGET_RANGE");
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "ENC_MOVE_RUN_FAIL");

            return resp_format(resp,
                               resp_len,
                               "OK M%u ENC_MOVE DIR=%s RPM=%.2f CNT=%lu START=%ld TARGET=%ld TOL=%lu",
                               (unsigned int)(cmd->motor_id + 1U),
                               MotorCtrl_DirToString(cmd->args.dir),
                               cmd->args.rpm,
                               (unsigned long)cmd->args.step,
                               (long)start_count,
                               (long)target_count,
                               (unsigned long)tol);
        }

        case CMD_ACTION_ABS_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ABS_STATUS_ARGS");
            if (cmd->motor_id != MOTOR_ID_2) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            return resp_m2_abs_status(resp, resp_len);

        case CMD_ACTION_ABS_ZERO:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ABS_ZERO_ARGS");
            if (cmd->motor_id != MOTOR_ID_2) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            rev_exkey_cancel_if_motor(MOTOR_ID_2);
            return resp_m2_abs_zero_set(resp, resp_len);

        case CMD_ACTION_ABS_MOVE:
        {
            Rs485AbsEncoderStatus_t abs_st;
            uint16_t target_raw;
            uint16_t tol_counts;
            uint16_t cw_remain;
            uint16_t ccw_remain;
            MotorDir_t move_dir;
            float tol_deg;

            rc = args_check(have_mask,
                            ARG_RPM | ARG_DEG,
                            ARG_RPM | ARG_DEG | ARG_DIR | ARG_TOL);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ABS_MOVE_ARGS");
            if (cmd->motor_id != MOTOR_ID_2) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            if ((cmd->args.deg < 0.0f) || (cmd->args.deg >= 360.0f)) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "DEG_RANGE");
            }
            if (cmd->args.has_tol && ((cmd->args.tol == 0U) || (cmd->args.tol > 180U))) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "TOL_DEG_RANGE");
            }

            rc = Rs485AbsEncoder_ReadM2(&abs_st);
            if (rc != 0) {
                return resp_err(resp, resp_len, CMD_ERR_RS485_ENCODER_OP, Rs485AbsEncoder_ErrorText(rc));
            }

            target_raw = m2_abs_deg_to_raw(cmd->args.deg);
            tol_deg = cmd->args.has_tol ? (float)cmd->args.tol : M2_ABS_DEFAULT_TOL_DEG;
            tol_counts = m2_abs_tol_deg_to_counts(tol_deg);
            if (m2_abs_short_distance(abs_st.raw, target_raw) <= tol_counts) {
                return resp_format(resp,
                                   resp_len,
                                   "OK M2 ABS_MOVE_DONE RAW=%u DEG=%.2f TARGET=%.2f TOL=%.2f",
                                   (unsigned int)abs_st.raw,
                                   abs_st.deg,
                                   cmd->args.deg,
                                   tol_deg);
            }

            if (cmd->args.has_dir) {
                move_dir = cmd->args.dir;
            } else {
                cw_remain = m2_abs_remain_for_dir(abs_st.raw, target_raw, MOTOR_DIR_CW);
                ccw_remain = m2_abs_remain_for_dir(abs_st.raw, target_raw, MOTOR_DIR_CCW);
                move_dir = (cw_remain <= ccw_remain) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
            }

            rc = MotorCtrl_Run(MOTOR_ID_2, move_dir, cmd->args.rpm);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "ABS_MOVE_RUN_FAIL");

            rev_exkey_cancel_if_motor(MOTOR_ID_2);
            m2_abs_start_move_state(move_dir, abs_st.raw, target_raw, cmd->args.deg, tol_deg, 0U);

            return resp_format(resp,
                               resp_len,
                               "OK M2 ABS_MOVE DIR=%s RPM=%.2f START_RAW=%u START_DEG=%.2f TARGET_RAW=%u TARGET_DEG=%.2f TOL=%.2f",
                               MotorCtrl_DirToString(move_dir),
                               cmd->args.rpm,
                               (unsigned int)abs_st.raw,
                               abs_st.deg,
                               (unsigned int)target_raw,
                               cmd->args.deg,
                               tol_deg);
        }

        case CMD_ACTION_RECIP:
            rc = args_check(have_mask, ARG_DIR | ARG_RPM | ARG_STEP | ARG_CYCLE, ARG_DIR | ARG_RPM | ARG_STEP | ARG_CYCLE);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "RECIP_ARGS");
            rc = MotorCtrl_Recip(cmd->motor_id, cmd->args.dir, cmd->args.rpm, cmd->args.step, cmd->args.cycle);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "RECIP_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_format(resp,
                               resp_len,
                               "OK M%u RECIP DIR=%s RPM=%.2f STEP=%lu CYCLE=%lu",
                               (unsigned int)(cmd->motor_id + 1U),
                               MotorCtrl_DirToString(cmd->args.dir),
                               cmd->args.rpm,
                               (unsigned long)cmd->args.step,
                               (unsigned long)cmd->args.cycle);

        case CMD_ACTION_RECIP_INF:
            rc = args_check(have_mask, ARG_DIR | ARG_RPM | ARG_STEP, ARG_DIR | ARG_RPM | ARG_STEP);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "RECIP_INF_ARGS");
            rc = MotorCtrl_RecipInfinite(cmd->motor_id, cmd->args.dir, cmd->args.rpm, cmd->args.step);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "RECIP_INF_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_format(resp,
                               resp_len,
                               "OK M%u RECIP_INF DIR=%s RPM=%.2f STEP=%lu",
                               (unsigned int)(cmd->motor_id + 1U),
                               MotorCtrl_DirToString(cmd->args.dir),
                               cmd->args.rpm,
                               (unsigned long)cmd->args.step);

        case CMD_ACTION_REV_EX1:
        case CMD_ACTION_REV_EX2:
        case CMD_ACTION_REV_EX3:
        case CMD_ACTION_REV_EX4:
        {
            RevExKeyGuard_t *guard;
            MotorDir_t active_dir;
            uint8_t started;
            uint8_t skipped;

            rc = args_check(have_mask, ARG_RPM, ARG_RPM);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "HOME_EXKEY_ARGS");
            if (cmd->action == CMD_ACTION_REV_EX2) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_NO_EXKEY_USE_ABS_MOVE");
            }
            guard = rev_exkey_guard_by_action(cmd->action);
            if ((guard == 0) || (cmd->motor_id != guard->motor_id)) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "HOME_EXKEY_MOTOR_MISMATCH");
            }
            if (guard->motor_id == MOTOR_ID_4) {
                rc = home_guard_start(guard,
                                      cmd->args.rpm,
                                      &started,
                                      &skipped);
                if ((rc != 0) || (started == 0U) || (skipped != 0U)) {
                    return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "M4_HOME_START_FAIL");
                }
                active_dir = (g_m4_home.phase == M4_HOME_PHASE_RELEASE) ?
                             MOTOR_DIR_CW : guard->home_dir;
                return resp_format(resp,
                                   resp_len,
                                   "OK M4 HOME_EX4 ACTION=%s DIR=%s RPM=%.2f RELEASE_T=1000 SEEK_T=60000",
                                   m4_home_phase_to_string(g_m4_home.phase),
                                   MotorCtrl_DirToString(active_dir),
                                   cmd->args.rpm);
            }
            if (rev_exkey_is_blocked_low(guard)) {
                return resp_err(resp, resp_len, CMD_ERR_LIMIT_VIOLATION, "EX_KEY_LOW");
            }
            rev_exkey_cancel_if_motor(guard->motor_id);
            rc = MotorCtrl_Run(guard->motor_id, guard->home_dir, cmd->args.rpm);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "HOME_EXKEY_FAIL");
            guard->active = 1U;
            return resp_format(resp,
                               resp_len,
                               "OK M%u HOME_EX%u DIR=%s RPM=%.2f STOP=EX_KEY%u_LOW",
                               (unsigned int)(guard->motor_id + 1U),
                               (unsigned int)guard->key_index,
                               MotorCtrl_DirToString(guard->home_dir),
                               cmd->args.rpm,
                               (unsigned int)guard->key_index);
        }

        case CMD_ACTION_CFG:
            rc = args_check(have_mask, 0U, ARG_PPR | ARG_ACC | ARG_DEC | ARG_STOP);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "CFG_ARGS");
            if ((have_mask & (ARG_PPR | ARG_ACC | ARG_DEC | ARG_STOP)) == 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MISSING_ARG, "CFG_EMPTY");
            }
            rc = MotorCtrl_Configure(cmd->motor_id,
                                     cmd->args.has_ppr,
                                     cmd->args.ppr,
                                     cmd->args.has_acc,
                                     cmd->args.acc,
                                     cmd->args.has_dec,
                                     cmd->args.dec,
                                     cmd->args.has_stop_ms,
                                     cmd->args.stop_ms);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "CFG_FAIL");
            rev_exkey_cancel_if_motor(cmd->motor_id);
            return resp_motor_status(cmd->motor_id, resp, resp_len);

        case CMD_ACTION_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "STATUS_ARGS");
            return resp_motor_status(cmd->motor_id, resp, resp_len);

        case CMD_ACTION_ENC_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ENC_STATUS_ARGS");
            if (cmd->motor_id >= MOTOR_ID_5) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M1_M4_ONLY");
            return resp_encoder_status(cmd->motor_id, resp, resp_len);

        case CMD_ACTION_ENC_CLR:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ENC_CLR_ARGS");
            if (cmd->motor_id >= MOTOR_ID_5) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M1_M4_ONLY");
            rc = EncoderCtrl_Clear(cmd->motor_id);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_ENCODER_OP, "ENC_CLR_FAIL");
            return resp_format(resp, resp_len, "OK M%u ENC_CLR", (unsigned int)(cmd->motor_id + 1U));

        case CMD_ACTION_CURVE_ON:
            rc = args_check(have_mask, 0U, ARG_RATE);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "CURVE_ON_ARGS");
            rc = TelemetryCurve_Set(cmd->motor_id,
                                    true,
                                    (uint16_t)(cmd->args.has_rate ? cmd->args.rate : CURVE_RATE_DEFAULT_HZ));
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_CURVE_OP, "CURVE_ON_FAIL");
            return resp_curve_status(cmd->motor_id, resp, resp_len);

        case CMD_ACTION_CURVE_OFF:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "CURVE_OFF_ARGS");
            rc = TelemetryCurve_Set(cmd->motor_id, false, 0U);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_CURVE_OP, "CURVE_OFF_FAIL");
            return resp_curve_status(cmd->motor_id, resp, resp_len);

        case CMD_ACTION_CURVE_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "CURVE_STATUS_ARGS");
            return resp_curve_status(cmd->motor_id, resp, resp_len);

        case CMD_ACTION_ROT_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ROT_STATUS_ARGS");
            if (cmd->motor_id != AXIS_ROT_MOTOR_ID) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            return resp_m3_rot_status(resp, resp_len);

        case CMD_ACTION_ROT_ZERO:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ROT_ZERO_ARGS");
            if (cmd->motor_id != AXIS_ROT_MOTOR_ID) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            if (AxisCoord_M3RotZeroNow() != 0) return resp_err(resp, resp_len, CMD_ERR_STATUS, "ROT_ZERO_FAIL");
            return resp_format(resp, resp_len, "OK M2 ROT_ZERO");

        case CMD_ACTION_ROT_SET:
            rc = args_check(have_mask, ARG_STEP, ARG_STEP);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ROT_SET_ARGS");
            if (cmd->motor_id != AXIS_ROT_MOTOR_ID) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            if (cmd->args.step > (uint32_t)INT32_MAX) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "ROT_STEP_RANGE");
            if (AxisCoord_M3RotSetAndSaveSteps((int32_t)cmd->args.step) != 0) return resp_err(resp, resp_len, CMD_ERR_LIMIT_SAVE, "ROT_SET_FAIL");
            return resp_m3_rot_status(resp, resp_len);

        case CMD_ACTION_ROT_SAVE:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ROT_SAVE_ARGS");
            if (cmd->motor_id != AXIS_ROT_MOTOR_ID) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            if (AxisCoord_M3RotSaveCurrentToFlash() != 0) return resp_err(resp, resp_len, CMD_ERR_LIMIT_SAVE, "ROT_SAVE_FAIL");
            return resp_m3_rot_status(resp, resp_len);

        case CMD_ACTION_ROT_GET:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ROT_GET_ARGS");
            if (cmd->motor_id != AXIS_ROT_MOTOR_ID) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            return resp_m3_rot_status(resp, resp_len);

        case CMD_ACTION_ROT_LIMIT_SET:
            rc = args_check(have_mask, ARG_STEP, ARG_STEP);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ROT_LIMIT_SET_ARGS");
            if (cmd->motor_id != AXIS_ROT_MOTOR_ID) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            rc = AxisLimit_SetM3RotMaxAndSave(cmd->args.step);
            if (rc == -1) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "ROT_LIMIT_RANGE");
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_LIMIT_SAVE, "ROT_LIMIT_SAVE_FAIL");
            return resp_m3_rot_limit_status(resp, resp_len);

        case CMD_ACTION_ROT_LIMIT_GET:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ROT_LIMIT_GET_ARGS");
            if (cmd->motor_id != AXIS_ROT_MOTOR_ID) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "M2_ONLY");
            return resp_m3_rot_limit_status(resp, resp_len);

        default:
            return resp_err(resp, resp_len, CMD_ERR_BAD_TARGET_ACTION, "ACTION_NOT_ALLOWED_FOR_MOTOR");
    }
}

static CmdResult_t handle_all_cmd(const ParsedCmd_t *cmd, char *resp, uint16_t resp_len)
{
    uint32_t have_mask = args_mask(&cmd->args);
    int rc;

    /*
     * ALL 命令处理：
     * 面向全部电机或全部编码器，例如 ALL STOP、ALL ESTOP、ALL HOME、ALL STATUS。
     */
    switch (cmd->action) {
        case CMD_ACTION_STOP:
            rc = args_check(have_mask, 0U, ARG_T);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_STOP_ARGS");
            MotorCtrl_StopAll(cmd->args.has_time_ms ? cmd->args.time_ms : 0U);
            rev_exkey_cancel_all();
            if (cmd->args.has_time_ms) {
                return resp_format(resp, resp_len, "OK ALL STOP T=%lu", (unsigned long)cmd->args.time_ms);
            }
            return resp_format(resp, resp_len, "OK ALL STOP T=DEFAULT");

        case CMD_ACTION_ESTOP:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_ESTOP_ARGS");
            MotorCtrl_EStopAll();
            rev_exkey_cancel_all();
            Electromagnet_ForceOff();
            return resp_format(resp, resp_len, "OK ALL ESTOP MAGNET=OFF");

        case CMD_ACTION_REV_EXKEY:
        {
            uint32_t run_count = 0U;
            uint32_t skip_low_count = 0U;

            rc = args_check(have_mask, ARG_RPM, ARG_RPM);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_HOME_ARGS");
            if (Electromagnet_IsOn() != 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MAGNET_ON_RELEASE_REQUIRED");
            }
            if ((g_place_cloth.active != 0U) || (g_take_cloth.active != 0U)) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "CLOTH_TASK_BUSY");
            }
            rc = all_home_start(cmd->args.rpm, &run_count, &skip_low_count);
            if (rc == -1) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "ALL_HOME_FAIL");
            }
            if (rc == -2) {
                return resp_err(resp, resp_len, CMD_ERR_ENCODER_OP, "ALL_HOME_ZERO_FAIL");
            }
            if (rc == 0) {
                return resp_format(resp,
                                   resp_len,
                                   "OK ALL HOME ALREADY_DONE MAP=M1-EX1,M3-EX3,M4-EX4 SKIP=M2-EX2 SKIP_LOW=%lu ZERO=EC1,EC3,EC4,XYZ",
                                   (unsigned long)skip_low_count);
            }
            return resp_format(resp,
                               resp_len,
                               "OK ALL HOME DIRS=M1:CCW,M3:CCW,M4:CCW M4_PHASE=%s RPM=%.2f MAP=M1-EX1,M3-EX3,M4-EX4 SKIP=M2-EX2 RUN=%lu SKIP_LOW=%lu",
                               m4_home_phase_to_string(g_m4_home.phase),
                               cmd->args.rpm,
                               (unsigned long)run_count,
                               (unsigned long)skip_low_count);
        }

        case CMD_ACTION_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_STATUS_ARGS");
            return resp_all_status(resp, resp_len);

        case CMD_ACTION_ENC_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_ENC_STATUS_ARGS");
            return resp_all_encoder_status(resp, resp_len);

        case CMD_ACTION_ENC_CLR:
        {
            uint32_t i;
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_ENC_CLR_ARGS");
            for (i = 0U; i < (uint32_t)MOTOR_ID_5; i++) {
                if (EncoderCtrl_Clear((MotorId_t)i) != 0) {
                    return resp_err(resp, resp_len, CMD_ERR_ENCODER_OP, "ALL_ENC_CLR_FAIL");
                }
            }
            return resp_format(resp, resp_len, "OK ALL ENC_CLR");
        }

        case CMD_ACTION_CURVE_ON:
        {
            uint32_t rate = cmd->args.has_rate ? cmd->args.rate : CURVE_RATE_DEFAULT_HZ;
            rc = args_check(have_mask, 0U, ARG_RATE);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_CURVE_ON_ARGS");
            if ((rate < CURVE_RATE_MIN_HZ) || (rate > CURVE_RATE_MAX_HZ)) {
                return resp_err(resp, resp_len, CMD_ERR_CURVE_OP, "ALL_CURVE_ON_FAIL");
            }
            TelemetryCurve_SetAll(true, (uint16_t)rate);
            return resp_format(resp,
                               resp_len,
                               "OK ALL CURVE_ON RATE=%u",
                               (unsigned int)rate);
        }

        case CMD_ACTION_CURVE_OFF:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_CURVE_OFF_ARGS");
            TelemetryCurve_SetAll(false, 0U);
            return resp_format(resp, resp_len, "OK ALL CURVE_OFF");

        case CMD_ACTION_CURVE_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "ALL_CURVE_STATUS_ARGS");
            return resp_all_curve_status(resp, resp_len);

        default:
            return resp_err(resp, resp_len, CMD_ERR_BAD_TARGET_ACTION, "ACTION_NOT_ALLOWED_FOR_ALL");
    }
}

static CmdResult_t handle_axis_cmd(const ParsedCmd_t *cmd, char *resp, uint16_t resp_len)
{
    uint32_t have_mask = args_mask(&cmd->args);
    AxisPlanConfig_t plan_cfg;
    int rc;

    /*
     * 三轴/衣柜业务命令处理：
     * AXIS MOVE/ENC_MOVE 负责三轴移动；
     * AXIS PLACE_CLOTH 和 AXIS TAKE_CLOTH 负责完整放衣/取衣流程；
     * AXIS LIMIT/PROTECT/ZERO/CFG 负责限位、保护、原点和规划配置。
     */
    switch (cmd->action) {
        case CMD_ACTION_PLACE_CLOTH:
        {
            float rpm;
            uint32_t tol;
            uint32_t slot;

            /* 放衣 SLOT=N：选择 EC1 越过门槛后遇到的第 N 块有效定位板。 */
            rc = args_check(have_mask, 0U, ARG_RPM | ARG_TOL | ARG_SLOT);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "PLACE_CLOTH_ARGS");
            if (Electromagnet_IsOn() != 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MAGNET_ON_RELEASE_REQUIRED");
            }
            if ((g_place_cloth.active != 0U) || (g_take_cloth.active != 0U)) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "CLOTH_TASK_BUSY");
            }
            if (cloth_xyz_motion_busy() != 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "AXIS_BUSY");
            }
            rpm = cmd->args.has_rpm ? cmd->args.rpm : PLACE_CLOTH_DEFAULT_RPM;
            tol = cmd->args.has_tol ? cmd->args.tol : PLACE_CLOTH_DEFAULT_TOL;
            if ((rpm <= 0.0f) || (tol == 0U) || (tol > 10000U)) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "PLACE_CLOTH_RANGE");
            }
            /* 未带 SLOT 时使用默认 1 号位。 */
            slot = cmd->args.has_slot ? cmd->args.slot : CLOTH_SLOT_DEFAULT;
            if ((slot == 0U) || (slot > 255U)) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "PLACE_CLOTH_SLOT");
            }
            (void)AxisPlanner_Stop();
            enc_move_stop_xyz(1U);
            g_place_cloth.active = 1U;
            g_place_cloth.rpm = rpm;
            g_place_cloth.tol = tol;
            g_place_cloth.slot = slot;
            g_place_cloth.m2_home_started = 0U;
            g_place_cloth.relative_target = 0L;
            g_place_cloth.laser2_target = 0L;
            g_place_cloth.laser2_started_ms = 0U;
            g_place_cloth.laser2_sample_count = 0U;
            memset(g_place_cloth.laser2_samples,
                   0,
                   sizeof(g_place_cloth.laser2_samples));
            place_cloth_start_next(PLACE_CLOTH_START_HOME);
            return resp_format(resp,
                               resp_len,
                               "OK AXIS PLACE_CLOTH RPM=%.2f TOL=%lu SLOT=%lu HOLDER=MAGNET SEQ=HOME,M2_CW650,Z10000,LASER2_TO_X,Y1381,Z12552,MAG_ON,Z_UP650,X1000,Y_HOME,Z3263,M2_CW4000,X8860,SCAN75_50,X_LEFT1125,Z_DOWN1500,Y_FWD1250,M2_CCW4000,MAG_OFF,Z_UP150,Y_HOME,M2_CW1000,HOME,M2_RAW1008_RPM25",
                               rpm,
                               (unsigned long)tol,
                               (unsigned long)g_place_cloth.slot);
        }

        case CMD_ACTION_TAKE_CLOTH:
        {
            float rpm;
            uint32_t tol;
            uint32_t slot;

            /*
             * 取衣命令：
             * 和放衣一样先用 SLOT 查坐标表，再启动取衣状态机。
             */
            rc = args_check(have_mask, 0U, ARG_RPM | ARG_TOL | ARG_SLOT);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "TAKE_CLOTH_ARGS");
            if (Electromagnet_IsOn() != 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MAGNET_ON_RELEASE_REQUIRED");
            }
            if ((g_place_cloth.active != 0U) || (g_take_cloth.active != 0U)) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "CLOTH_TASK_BUSY");
            }
            if (cloth_xyz_motion_busy() != 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "AXIS_BUSY");
            }
            rpm = cmd->args.has_rpm ? cmd->args.rpm : TAKE_CLOTH_DEFAULT_RPM;
            tol = cmd->args.has_tol ? cmd->args.tol : TAKE_CLOTH_DEFAULT_TOL;
            if ((rpm <= 0.0f) || (tol == 0U) || (tol > 10000U)) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "TAKE_CLOTH_RANGE");
            }
            /* TAKE_CLOTH slot is the Nth laser marker from the X origin. */
            slot = cmd->args.has_slot ? cmd->args.slot : CLOTH_SLOT_DEFAULT;
            if ((slot == 0U) || (slot > 255U)) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "TAKE_CLOTH_SLOT");
            }
            (void)AxisPlanner_Stop();
            enc_move_stop_xyz(1U);
            g_take_cloth.active = 1U;
            g_take_cloth.rpm = rpm;
            g_take_cloth.tol = tol;
            g_take_cloth.slot = slot;
            g_take_cloth.y_home_done = 0U;
            g_take_cloth.m2_home_started = 0U;
            g_take_cloth.relative_target = 0L;
            take_laser_scan_reset();
            take_cloth_start_next(TAKE_CLOTH_START_HOME);
            return resp_format(resp,
                               resp_len,
                               "OK AXIS TAKE_CLOTH RPM=%.2f TOL=%lu SLOT=%lu HOLDER=MAGNET SEQ=HOME,M2_CW650,Z3263,M2_CW1500,X8860,SCAN75_50,X_LEFT1760,Z_DOWN1200,Y_FWD1350,M2_CCW1500,Z_DOWN670,MAG_ON,Z_UP520,Y_FWD150,M2_CW1500,Y_HOME,M2_CCW5500,X1000,M2_CW4000,Z12552,Y1470,X3200,MAG_OFF,Z10000,HOME,M2_RAW1008_RPM25",
                               rpm,
                               (unsigned long)tol,
                               (unsigned long)g_take_cloth.slot);
        }

        case CMD_ACTION_ENC_MOVE:
        {
            uint32_t tol;
            uint8_t started;
            uint32_t started_count = 0U;

            rc = args_check(have_mask, ARG_X | ARG_Y | ARG_Z | ARG_RPM, ARG_X | ARG_Y | ARG_Z | ARG_RPM | ARG_TOL);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_ENC_MOVE_ARGS");
            if (Electromagnet_IsOn() != 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MAGNET_ON_RELEASE_REQUIRED");
            }
            if ((g_place_cloth.active != 0U) || (g_take_cloth.active != 0U)) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "CLOTH_TASK_BUSY");
            }
            if ((cmd->args.x > (uint32_t)INT32_MAX) ||
                (cmd->args.y > (uint32_t)INT32_MAX) ||
                (cmd->args.z > (uint32_t)INT32_MAX)) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "TARGET_RANGE");
            }
            if (cmd->args.has_tol && ((cmd->args.tol == 0U) || (cmd->args.tol > 10000U))) {
                return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "TOL_RANGE");
            }

            tol = cmd->args.has_tol ? cmd->args.tol : AXIS_ENC_MOVE_DEFAULT_TOL;
            (void)AxisPlanner_Stop();
            enc_move_stop_xyz(1U);

            rc = enc_move_start_absolute(MOTOR_ID_1, (int32_t)cmd->args.x, cmd->args.rpm, tol, &started);
            if (rc != 0) {
                enc_move_stop_xyz(1U);
                return resp_err(resp, resp_len, (rc == -2) ? CMD_ERR_STATUS : CMD_ERR_MOTOR_OP, "AXIS_ENC_X_FAIL");
            }
            started_count += started;

            rc = enc_move_start_absolute(MOTOR_ID_3, (int32_t)cmd->args.y, cmd->args.rpm, tol, &started);
            if (rc != 0) {
                enc_move_stop_xyz(1U);
                return resp_err(resp, resp_len, (rc == -2) ? CMD_ERR_STATUS : CMD_ERR_MOTOR_OP, "AXIS_ENC_Y_FAIL");
            }
            started_count += started;

            rc = enc_move_start_absolute(MOTOR_ID_4, (int32_t)cmd->args.z, cmd->args.rpm, tol, &started);
            if (rc != 0) {
                enc_move_stop_xyz(1U);
                return resp_err(resp, resp_len, (rc == -2) ? CMD_ERR_STATUS : CMD_ERR_MOTOR_OP, "AXIS_ENC_Z_FAIL");
            }
            started_count += started;

            if (started_count == 0U) {
                return resp_format(resp,
                                   resp_len,
                                   "OK AXIS ENC_MOVE ALREADY_DONE X=%lu Y=%lu Z=%lu TOL=%lu",
                                   (unsigned long)cmd->args.x,
                                   (unsigned long)cmd->args.y,
                                   (unsigned long)cmd->args.z,
                                   (unsigned long)tol);
            }

            return resp_format(resp,
                               resp_len,
                               "OK AXIS ENC_MOVE X=%lu Y=%lu Z=%lu RPM=%.2f TOL=%lu RUN=%lu MAP=X:EC1,Y:EC3,Z:EC4",
                               (unsigned long)cmd->args.x,
                               (unsigned long)cmd->args.y,
                               (unsigned long)cmd->args.z,
                               cmd->args.rpm,
                               (unsigned long)tol,
                               (unsigned long)started_count);
        }

        case CMD_ACTION_MOVE:
            rc = args_check(have_mask, ARG_X | ARG_Y | ARG_Z, ARG_X | ARG_Y | ARG_Z | ARG_RPM);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_MOVE_ARGS");
            if (Electromagnet_IsOn() != 0U) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MAGNET_ON_RELEASE_REQUIRED");
            }
            if ((g_place_cloth.active != 0U) || (g_take_cloth.active != 0U)) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "CLOTH_TASK_BUSY");
            }
            rc = AxisPlanner_MoveTo((int32_t)cmd->args.x,
                                    (int32_t)cmd->args.y,
                                    (int32_t)cmd->args.z,
                                    cmd->args.has_rpm ? cmd->args.rpm : 0.0f);
            if (rc == -1) return resp_err(resp, resp_len, CMD_ERR_LIMIT_NOT_READY, "LIMIT_NOT_READY");
            if (rc == -3) return resp_err(resp, resp_len, CMD_ERR_LIMIT_VIOLATION, "TARGET_OUT_OF_RANGE");
            if (rc == -5) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "PLAN_BUSY");
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "AXIS_MOVE_FAIL");
            return resp_format(resp,
                               resp_len,
                               "OK AXIS MOVE X=%lu Y=%lu Z=%lu RPM=%s",
                               (unsigned long)cmd->args.x,
                               (unsigned long)cmd->args.y,
                               (unsigned long)cmd->args.z,
                               cmd->args.has_rpm ? "SET" : "DEFAULT");

        case CMD_ACTION_STOP:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_STOP_ARGS");
            (void)AxisPlanner_Stop();
            enc_move_stop_xyz(0U);
            place_cloth_stop_motion(0U);
            place_cloth_cancel();
            take_cloth_cancel();
            return resp_format(resp, resp_len, "OK AXIS STOP");

        case CMD_ACTION_ESTOP:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_ESTOP_ARGS");
            (void)AxisPlanner_EStop();
            enc_move_stop_xyz(1U);
            place_cloth_stop_motion(1U);
            place_cloth_cancel();
            take_cloth_cancel();
            Electromagnet_ForceOff();
            return resp_format(resp, resp_len, "OK AXIS ESTOP MAGNET=OFF");

        case CMD_ACTION_CFG:
            rc = args_check(have_mask, ARG_RPM | ARG_ERR | ARG_TOL, ARG_RPM | ARG_ERR | ARG_TOL);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_CFG_ARGS");
            rc = AxisPlanner_SetAndSaveConfig(cmd->args.rpm, cmd->args.err, cmd->args.tol);
            if (rc == -1) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "AXIS_CFG_RANGE");
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_LIMIT_SAVE, "AXIS_CFG_SAVE_FAIL");
            if (AxisPlanner_GetConfig(&plan_cfg) != 0) {
                return resp_err(resp, resp_len, CMD_ERR_STATUS, "AXIS_CFG_READ_FAIL");
            }
            return resp_format(resp,
                               resp_len,
                               "OK AXIS CFG RPM=%.2f ERR=%lu TOL=%lu",
                               plan_cfg.move_rpm_default,
                               (unsigned long)plan_cfg.enc_err_max,
                               (unsigned long)plan_cfg.pos_tol);

        case CMD_ACTION_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_STATUS_ARGS");
            return resp_axis_status(resp, resp_len);

        case CMD_ACTION_LIMIT_GET:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_LIMIT_GET_ARGS");
            return resp_axis_limit_get(resp, resp_len);

        case CMD_ACTION_LIMIT_SET:
            rc = args_check(have_mask, ARG_X | ARG_Y | ARG_Z, ARG_X | ARG_Y | ARG_Z);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_LIMIT_SET_ARGS");
            rc = AxisLimit_SetAndSave(cmd->args.x, cmd->args.y, cmd->args.z);
            if (rc == -1) return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "LIMIT_RANGE");
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_LIMIT_SAVE, "LIMIT_SAVE_FAIL");
            return resp_axis_limit_get(resp, resp_len);

        case CMD_ACTION_ZERO:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_ZERO_ARGS");
            if (AxisCoord_ZeroNow() != 0) {
                return resp_err(resp, resp_len, CMD_ERR_STATUS, "AXIS_ZERO_FAIL");
            }
            AxisLimit_ClearFault();
            return resp_format(resp, resp_len, "OK AXIS ZERO");

        case CMD_ACTION_PROTECT_ON:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_PROTECT_ON_ARGS");
            rc = AxisLimit_SetProtectAndSave(true);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_LIMIT_SAVE, "PROTECT_SAVE_FAIL");
            return resp_axis_protect_status(resp, resp_len);

        case CMD_ACTION_PROTECT_OFF:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_PROTECT_OFF_ARGS");
            rc = AxisLimit_SetProtectAndSave(false);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_LIMIT_SAVE, "PROTECT_SAVE_FAIL");
            return resp_axis_protect_status(resp, resp_len);

        case CMD_ACTION_PROTECT_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "AXIS_PROTECT_STATUS_ARGS");
            return resp_axis_protect_status(resp, resp_len);

        default:
            return resp_err(resp, resp_len, CMD_ERR_BAD_TARGET_ACTION, "ACTION_NOT_ALLOWED_FOR_AXIS");
    }
}

static CmdResult_t handle_encoder_cmd(const ParsedCmd_t *cmd, char *resp, uint16_t resp_len)
{
    uint32_t have_mask = args_mask(&cmd->args);
    int rc;
    unsigned int ec_index;

    /* EC1~EC4 编码器目标命令处理，只支持状态查询和清零。 */
    if (cmd->motor_id >= MOTOR_ID_5) {
        return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "EC_ID_RANGE");
    }
    ec_index = (unsigned int)(cmd->motor_id + 1U);

    switch (cmd->action) {
        case CMD_ACTION_STATUS:
        case CMD_ACTION_ENC_STATUS:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "EC_STATUS_ARGS");
            return resp_encoder_target_status(cmd->motor_id, resp, resp_len);

        case CMD_ACTION_CLR:
        case CMD_ACTION_ENC_CLR:
            rc = args_check(have_mask, 0U, 0U);
            if (rc != 0) return resp_err(resp, resp_len, (uint16_t)rc, "EC_CLR_ARGS");
            rc = EncoderCtrl_Clear(cmd->motor_id);
            if (rc != 0) return resp_err(resp, resp_len, CMD_ERR_ENCODER_OP, "EC_CLR_FAIL");
            return resp_format(resp, resp_len, "OK EC%u CLR", ec_index);

        default:
            return resp_err(resp, resp_len, CMD_ERR_BAD_TARGET_ACTION, "ACTION_NOT_ALLOWED_FOR_EC");
    }
}

static CmdResult_t handle_system_cmd(const ParsedCmd_t *cmd, char *resp, uint16_t resp_len)
{
    uint32_t have_mask = args_mask(&cmd->args);

    /* SYS/STATUS/HELP/VER/PING 等系统级命令处理。 */
    if (have_mask != 0U) {
        return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "SYS_NO_ARGS");
    }

    switch (cmd->action) {
        case CMD_ACTION_HELP:
            return resp_format(resp,
                               resp_len,
                               "OK HELP STATUS; M1~M5 RUN/STOP/ESTOP/SPEED/ACCEL/DECEL/MOVE/RECIP/RECIP_INF/CFG/STATUS/CURVE_ON/CURVE_OFF/CURVE_STATUS; M1~M4 ENC_STATUS/ENC_CLR; M1/M3/M4 ENC_MOVE DIR=CW|CCW RPM=n CNT=n [TOL=n]; M2 ABS_STATUS/ABS_ZERO; M2 ABS_MOVE DEG=n RPM=n [DIR=CW|CCW] [TOL=n]; M1 HOME_EX1 RPM=n; M3 HOME_EX3 RPM=n; M4 HOME_EX4 RPM=n; ALL HOME RPM=n/STOP/ESTOP/STATUS/ENC_STATUS/ENC_CLR/CURVE_ON/CURVE_OFF/CURVE_STATUS; EC1~EC4 STATUS/CLR; M2 ROT_STATUS/ROT_ZERO/ROT_SET STEP=n/ROT_SAVE/ROT_GET/ROT_LIMIT_SET STEP=n/ROT_LIMIT_GET; AXIS MOVE/ENC_MOVE/PLACE_CLOTH/TAKE_CLOTH/STOP/ESTOP/STATUS/LIMIT_GET/LIMIT_SET/ZERO/CFG/PROTECT ON|OFF|STATUS; SYS STATUS/HELP/VER/PING; DEBUG ON/OFF/STATUS; LASER STATUS/DIAG; LASER2 STATUS/DIAG/STREAM; MAGNET ON/OFF/STATUS");

        case CMD_ACTION_STATUS:
            return CmdDispatch_FormatSystemStatus(resp, resp_len);

        case CMD_ACTION_VER:
            return resp_format(resp, resp_len, "OK VER %s", CMD_VERSION_TEXT);

        case CMD_ACTION_PING:
            return resp_format(resp, resp_len, "OK PONG");

        case CMD_ACTION_MAGNET_ON:
            if (Electromagnet_Set(1U) != 0) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MAGNET_ON_FAIL");
            }
            return resp_format(resp,
                               resp_len,
                               "OK MAGNET CMD=ON PIN=%s FEEDBACK=NONE",
                               (Electromagnet_PinHigh() != 0U) ? "HIGH" : "LOW");

        case CMD_ACTION_MAGNET_OFF:
            if (Electromagnet_Set(0U) != 0) {
                return resp_err(resp, resp_len, CMD_ERR_MOTOR_OP, "MAGNET_OFF_FAIL");
            }
            return resp_format(resp,
                               resp_len,
                               "OK MAGNET CMD=OFF PIN=%s FEEDBACK=NONE",
                               (Electromagnet_PinHigh() != 0U) ? "HIGH" : "LOW");

        case CMD_ACTION_MAGNET_STATUS:
            return resp_format(resp,
                               resp_len,
                               "OK MAGNET CMD=%s PIN=%s FEEDBACK=NONE",
                               (Electromagnet_IsOn() != 0U) ? "ON" : "OFF",
                               (Electromagnet_PinHigh() != 0U) ? "HIGH" : "LOW");

        case CMD_ACTION_LASER_STATUS:
        {
            LaserRangeStatus_t status;
            uint8_t plate_valid;

            if (!LaserRange_GetStatus(&status)) {
                return resp_err(resp, resp_len, CMD_ERR_STATUS, "LASER_STATUS_FAIL");
            }
            plate_valid = (status.communication_online &&
                           status.data_valid &&
                           (status.raw_confidence >= TAKE_LASER_PLATE_CONFIDENCE_MIN) &&
                           (take_laser_board_distance_valid(status.raw_distance_mm) != 0U)) ? 1U : 0U;
            return resp_format(resp,
                               resp_len,
                               "OK LASER DIST=%u CONF=%u COMM=%u VALID=%u PLATE=%u STATE=%s",
                               (unsigned int)status.raw_distance_mm,
                               (unsigned int)status.raw_confidence,
                               status.communication_online ? 1U : 0U,
                               status.data_valid ? 1U : 0U,
                               (unsigned int)plate_valid,
                               LaserRange_StateText(status.state));
        }

        case CMD_ACTION_LASER_DIAG:
        {
            LaserRangeStatus_t status;

            if (!LaserRange_GetStatus(&status)) {
                return resp_err(resp, resp_len, CMD_ERR_STATUS, "LASER_DIAG_FAIL");
            }
            return resp_format(resp,
                               resp_len,
                               "OK LASER DIAG MODE=MODBUS ADDR=%u STATE=%s REQ=%lu RESP=%lu TIMEOUT=%lu CRC_ERR=%lu PROTO_ERR=%lu EXC=%lu REJECT=%lu DIST_REJECT=%lu CONF_REJECT=%lu UART_ERR=%lu TX_ERR=%lu COMM_ONLINE=%u DATA_VALID=%u ONLINE=%u RAW_DIST=%u RAW_CONF=%u DIST=%u CONF=%u",
                               (unsigned int)status.address,
                               LaserRange_StateText(status.state),
                               (unsigned long)status.request_count,
                               (unsigned long)status.response_count,
                               (unsigned long)status.timeout_count,
                               (unsigned long)status.crc_error_count,
                               (unsigned long)status.protocol_error_count,
                               (unsigned long)status.exception_count,
                               (unsigned long)status.rejected_count,
                               (unsigned long)status.distance_reject_count,
                               (unsigned long)status.confidence_reject_count,
                               (unsigned long)status.uart_error_count,
                               (unsigned long)status.tx_error_count,
                               status.communication_online ? 1U : 0U,
                               status.data_valid ? 1U : 0U,
                               status.online ? 1U : 0U,
                               (unsigned int)status.raw_distance_mm,
                               (unsigned int)status.raw_confidence,
                               (unsigned int)status.distance_mm,
                                (unsigned int)status.confidence);
        }

        case CMD_ACTION_LASER2_STATUS:
        {
            LaserRangeStatus_t status;

            if (!LaserRange2_GetStatus(&status)) {
                return resp_err(resp, resp_len, CMD_ERR_STATUS, "LASER2_STATUS_FAIL");
            }
            return resp_format(resp,
                               resp_len,
                               "OK LASER2 DIST=%u CONF=%u COMM=%u VALID=%u ONLINE=%u STATE=%s",
                               (unsigned int)status.raw_distance_mm,
                               (unsigned int)status.raw_confidence,
                               status.communication_online ? 1U : 0U,
                               status.data_valid ? 1U : 0U,
                               status.online ? 1U : 0U,
                               LaserRange2_StateText(status.state));
        }

        case CMD_ACTION_LASER2_DIAG:
        {
            LaserRangeStatus_t status;

            if (!LaserRange2_GetStatus(&status)) {
                return resp_err(resp, resp_len, CMD_ERR_STATUS, "LASER2_DIAG_FAIL");
            }
            return resp_format(resp,
                               resp_len,
                               "OK LASER2 DIAG PORT=UART4 PINS=PA0/PA1 MODE=MODBUS ADDR=%u STATE=%s REQ=%lu RESP=%lu TIMEOUT=%lu CRC_ERR=%lu PROTO_ERR=%lu EXC=%lu REJECT=%lu DIST_REJECT=%lu CONF_REJECT=%lu UART_ERR=%lu TX_ERR=%lu COMM_ONLINE=%u DATA_VALID=%u ONLINE=%u RAW_DIST=%u RAW_CONF=%u DIST=%u CONF=%u",
                               (unsigned int)status.address,
                               LaserRange2_StateText(status.state),
                               (unsigned long)status.request_count,
                               (unsigned long)status.response_count,
                               (unsigned long)status.timeout_count,
                               (unsigned long)status.crc_error_count,
                               (unsigned long)status.protocol_error_count,
                               (unsigned long)status.exception_count,
                               (unsigned long)status.rejected_count,
                               (unsigned long)status.distance_reject_count,
                               (unsigned long)status.confidence_reject_count,
                               (unsigned long)status.uart_error_count,
                               (unsigned long)status.tx_error_count,
                               status.communication_online ? 1U : 0U,
                               status.data_valid ? 1U : 0U,
                               status.online ? 1U : 0U,
                               (unsigned int)status.raw_distance_mm,
                               (unsigned int)status.raw_confidence,
                               (unsigned int)status.distance_mm,
                               (unsigned int)status.confidence);
        }

        case CMD_ACTION_LASER2_STREAM_ON:
            LaserRange2_SetStreamEnabled(true);
            return resp_format(resp,
                               resp_len,
                               "OK LASER2 STREAM=ON PERIOD=200MS DEBUG_REQUIRED=1");

        case CMD_ACTION_LASER2_STREAM_OFF:
            LaserRange2_SetStreamEnabled(false);
            return resp_format(resp, resp_len, "OK LASER2 STREAM=OFF");

        case CMD_ACTION_LASER2_STREAM_STATUS:
            return resp_format(resp,
                               resp_len,
                               "OK LASER2 STREAM=%s PERIOD=200MS TEXT=%u",
                               LaserRange2_GetStreamEnabled() ? "ON" : "OFF",
                               UartConsole_GetTextOutputEnabled() ? 1U : 0U);

        default:
            return resp_err(resp, resp_len, CMD_ERR_BAD_TARGET_ACTION, "ACTION_NOT_ALLOWED_FOR_SYSTEM");
    }
}

CmdResult_t CmdDispatch_Execute(const ParsedCmd_t *cmd, char *resp, uint16_t resp_len)
{
    /*
     * 命令执行总入口。
     * ParsedCmd_t.target 决定进入哪个处理函数；所有处理函数都把响应写入 resp。
     */
    if ((cmd == 0) || (resp == 0) || (resp_len == 0U)) {
        return CMD_RESULT_ERR;
    }

    switch (cmd->target) {
        case CMD_TARGET_MOTOR:
            return handle_motor_cmd(cmd, resp, resp_len);

        case CMD_TARGET_ALL:
            return handle_all_cmd(cmd, resp, resp_len);

        case CMD_TARGET_AXIS:
            return handle_axis_cmd(cmd, resp, resp_len);

        case CMD_TARGET_ENCODER:
            return handle_encoder_cmd(cmd, resp, resp_len);

        case CMD_TARGET_SYSTEM:
            return handle_system_cmd(cmd, resp, resp_len);

        default:
            return resp_err(resp, resp_len, CMD_ERR_BAD_ARG, "BAD_TARGET");
    }
}
