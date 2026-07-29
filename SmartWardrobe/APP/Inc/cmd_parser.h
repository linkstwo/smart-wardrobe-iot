#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "motor_hw.h"

typedef enum {
    /* 命令解析结果。 */
    CMD_PARSE_OK = 0,
    CMD_PARSE_EMPTY,
    CMD_PARSE_BAD_TARGET,
    CMD_PARSE_BAD_ACTION,
    CMD_PARSE_BAD_ARG
} CmdParseResult_t;

typedef enum {
    /* 命令目标类型：系统、电机、全部电机、三轴、编码器。 */
    CMD_TARGET_SYSTEM = 0,
    CMD_TARGET_MOTOR,
    CMD_TARGET_ALL,
    CMD_TARGET_AXIS,
    CMD_TARGET_ENCODER
} CmdTarget_t;

typedef enum {
    /* 命令动作类型，具体能否用于某个目标由 cmd_dispatch.c 判断。 */
    CMD_ACTION_NONE = 0,
    CMD_ACTION_HELP,
    CMD_ACTION_VER,
    CMD_ACTION_PING,
    CMD_ACTION_DEBUG_ON,
    CMD_ACTION_DEBUG_OFF,
    CMD_ACTION_DEBUG_STATUS,
    CMD_ACTION_LASER_STATUS,
    CMD_ACTION_LASER_DIAG,
    CMD_ACTION_LASER2_STATUS,
    CMD_ACTION_LASER2_DIAG,
    CMD_ACTION_LASER2_STREAM_ON,
    CMD_ACTION_LASER2_STREAM_OFF,
    CMD_ACTION_LASER2_STREAM_STATUS,
    CMD_ACTION_MAGNET_ON,
    CMD_ACTION_MAGNET_OFF,
    CMD_ACTION_MAGNET_STATUS,

    CMD_ACTION_RUN,
    CMD_ACTION_STOP,
    CMD_ACTION_ESTOP,
    CMD_ACTION_SPEED,
    CMD_ACTION_ACCEL,
    CMD_ACTION_DECEL,
    CMD_ACTION_MOVE,
    CMD_ACTION_RECIP,
    CMD_ACTION_RECIP_INF,
    CMD_ACTION_REV_EX1,
    CMD_ACTION_REV_EX2,
    CMD_ACTION_REV_EX3,
    CMD_ACTION_REV_EX4,
    CMD_ACTION_REV_EXKEY,
    CMD_ACTION_CFG,
    CMD_ACTION_STATUS,

    CMD_ACTION_ENC_STATUS,
    CMD_ACTION_ENC_CLR,
    CMD_ACTION_ENC_MOVE,
    CMD_ACTION_ABS_STATUS,
    CMD_ACTION_ABS_MOVE,
    CMD_ACTION_ABS_ZERO,
    CMD_ACTION_CLR,

    CMD_ACTION_CURVE_ON,
    CMD_ACTION_CURVE_OFF,
    CMD_ACTION_CURVE_STATUS,
    CMD_ACTION_ROT_STATUS,
    CMD_ACTION_ROT_ZERO,
    CMD_ACTION_ROT_SET,
    CMD_ACTION_ROT_SAVE,
    CMD_ACTION_ROT_GET,
    CMD_ACTION_ROT_LIMIT_SET,
    CMD_ACTION_ROT_LIMIT_GET,

    CMD_ACTION_LIMIT_SET,
    CMD_ACTION_LIMIT_GET,
    CMD_ACTION_PROTECT_ON,
    CMD_ACTION_PROTECT_OFF,
    CMD_ACTION_PROTECT_STATUS,
    CMD_ACTION_ZERO,
    CMD_ACTION_PLACE_CLOTH,
    CMD_ACTION_TAKE_CLOTH
} CmdAction_t;

typedef struct {
    /* 每个参数都用 has_xxx 标记是否在命令行中出现。 */
    bool has_dir;
    MotorDir_t dir;

    bool has_rpm;
    float rpm;

    bool has_to;
    float to_rpm;

    bool has_time_ms;
    uint32_t time_ms;

    bool has_step;
    uint32_t step;

    bool has_cycle;
    uint32_t cycle;

    bool has_ppr;
    uint32_t ppr;

    bool has_acc;
    float acc;

    bool has_dec;
    float dec;

    bool has_stop_ms;
    uint32_t stop_ms;

    bool has_rate;
    uint32_t rate;

    bool has_x;
    uint32_t x;

    bool has_y;
    uint32_t y;

    bool has_z;
    uint32_t z;

    bool has_slot;
    uint32_t slot;

    bool has_err;
    uint32_t err;

    bool has_tol;
    uint32_t tol;

    bool has_deg;
    float deg;

    bool has_reg;
    uint32_t reg;

    bool has_val;
    uint32_t val;
} CmdArgs_t;

typedef struct {
    /* 解析后的完整命令。 */
    CmdTarget_t target;
    MotorId_t motor_id;
    CmdAction_t action;
    CmdArgs_t args;
} ParsedCmd_t;

CmdParseResult_t CmdParser_Parse(const char *line, ParsedCmd_t *out_cmd);
/* 解析结果转文本，便于调试输出。 */
const char *CmdParser_ResultText(CmdParseResult_t result);

#ifdef __cplusplus
}
#endif

#endif
