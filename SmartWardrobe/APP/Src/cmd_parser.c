#include "cmd_parser.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define CMD_PARSER_MAX_TOKENS 16U
#define CMD_PARSER_LINE_BUF_SIZE 160U

/*
 * 文本命令解析器：
 * 把 USART3 收到的 ASCII 命令行解析为 ParsedCmd_t。
 * 这里只负责“识别命令和参数”，真正执行电机、坐标、夹爪动作在 cmd_dispatch.c 中完成。
 */
static int str_ieq(const char *a, const char *b)
{
    char ca;
    char cb;

    /* 大小写不敏感字符串比较，方便上位机输入 run/RUN/Run 等命令。 */
    if ((a == 0) || (b == 0)) {
        return 0;
    }

    while ((*a != '\0') && (*b != '\0')) {
        ca = (char)toupper((unsigned char)*a);
        cb = (char)toupper((unsigned char)*b);
        if (ca != cb) {
            return 0;
        }
        a++;
        b++;
    }

    return ((*a == '\0') && (*b == '\0')) ? 1 : 0;
}

static int parse_u32(const char *text, uint32_t *out)
{
    char *end = 0;
    unsigned long value;

    /* 解析无符号十进制整数；负数、空字符串、尾部带杂字符都判定失败。 */
    if ((text == 0) || (out == 0)) {
        return 0;
    }

    if ((*text == '-') || (*text == '\0')) {
        return 0;
    }

    value = strtoul(text, &end, 10);
    if ((end == text) || (*end != '\0') || (value > 0xFFFFFFFFUL)) {
        return 0;
    }

    *out = (uint32_t)value;
    return 1;
}

static int parse_float(const char *text, float *out)
{
    char *end = 0;
    float value;

    /* 解析浮点参数，例如 RPM=55.5 或 DEG=90.0。 */
    if ((text == 0) || (out == 0)) {
        return 0;
    }

    value = strtof(text, &end);
    if ((end == text) || (*end != '\0')) {
        return 0;
    }

    *out = value;
    return 1;
}

static CmdAction_t parse_action(const char *token)
{
    /*
     * 命令动作关键字映射。
     * 这里保留了一些别名，例如 HOME_EX1 等同 REV_EX1，ANGLE_MOVE 等同 ABS_MOVE。
     */
    if (str_ieq(token, "RUN")) return CMD_ACTION_RUN;
    if (str_ieq(token, "STOP")) return CMD_ACTION_STOP;
    if (str_ieq(token, "ESTOP")) return CMD_ACTION_ESTOP;
    if (str_ieq(token, "SPEED")) return CMD_ACTION_SPEED;
    if (str_ieq(token, "ACCEL")) return CMD_ACTION_ACCEL;
    if (str_ieq(token, "DECEL")) return CMD_ACTION_DECEL;
    if (str_ieq(token, "MOVE")) return CMD_ACTION_MOVE;
    if (str_ieq(token, "RECIP")) return CMD_ACTION_RECIP;
    if (str_ieq(token, "RECIP_INF")) return CMD_ACTION_RECIP_INF;
    if (str_ieq(token, "REV_EX1")) return CMD_ACTION_REV_EX1;
    if (str_ieq(token, "REV_EX2")) return CMD_ACTION_REV_EX2;
    if (str_ieq(token, "REV_EX3")) return CMD_ACTION_REV_EX3;
    if (str_ieq(token, "REV_EX4")) return CMD_ACTION_REV_EX4;
    if (str_ieq(token, "REV_EXKEY")) return CMD_ACTION_REV_EXKEY;
    if (str_ieq(token, "HOME_EX1")) return CMD_ACTION_REV_EX1;
    if (str_ieq(token, "HOME_EX2")) return CMD_ACTION_REV_EX2;
    if (str_ieq(token, "HOME_EX3")) return CMD_ACTION_REV_EX3;
    if (str_ieq(token, "HOME_EX4")) return CMD_ACTION_REV_EX4;
    if (str_ieq(token, "HOME_EXKEY")) return CMD_ACTION_REV_EXKEY;
    if (str_ieq(token, "HOME")) return CMD_ACTION_REV_EXKEY;
    if (str_ieq(token, "CFG")) return CMD_ACTION_CFG;
    if (str_ieq(token, "STATUS")) return CMD_ACTION_STATUS;

    if (str_ieq(token, "ENC_STATUS")) return CMD_ACTION_ENC_STATUS;
    if (str_ieq(token, "ENC_CLR")) return CMD_ACTION_ENC_CLR;
    if (str_ieq(token, "ENC_MOVE")) return CMD_ACTION_ENC_MOVE;
    if (str_ieq(token, "ABS_STATUS")) return CMD_ACTION_ABS_STATUS;
    if (str_ieq(token, "ABS_MOVE")) return CMD_ACTION_ABS_MOVE;
    if (str_ieq(token, "ABS_ZERO")) return CMD_ACTION_ABS_ZERO;
    if (str_ieq(token, "ABS_ZERO_SET")) return CMD_ACTION_ABS_ZERO;
    if (str_ieq(token, "ANGLE_STATUS")) return CMD_ACTION_ABS_STATUS;
    if (str_ieq(token, "ANGLE_MOVE")) return CMD_ACTION_ABS_MOVE;
    if (str_ieq(token, "ANGLE_ZERO")) return CMD_ACTION_ABS_ZERO;
    if (str_ieq(token, "CLR")) return CMD_ACTION_CLR;

    if (str_ieq(token, "CURVE_ON")) return CMD_ACTION_CURVE_ON;
    if (str_ieq(token, "CURVE_OFF")) return CMD_ACTION_CURVE_OFF;
    if (str_ieq(token, "CURVE_STATUS")) return CMD_ACTION_CURVE_STATUS;
    if (str_ieq(token, "ROT_STATUS")) return CMD_ACTION_ROT_STATUS;
    if (str_ieq(token, "ROT_ZERO")) return CMD_ACTION_ROT_ZERO;
    if (str_ieq(token, "ROT_SET")) return CMD_ACTION_ROT_SET;
    if (str_ieq(token, "ROT_SAVE")) return CMD_ACTION_ROT_SAVE;
    if (str_ieq(token, "ROT_GET")) return CMD_ACTION_ROT_GET;
    if (str_ieq(token, "ROT_LIMIT_SET")) return CMD_ACTION_ROT_LIMIT_SET;
    if (str_ieq(token, "ROT_LIMIT_GET")) return CMD_ACTION_ROT_LIMIT_GET;
    if (str_ieq(token, "LIMIT_SET")) return CMD_ACTION_LIMIT_SET;
    if (str_ieq(token, "LIMIT_GET")) return CMD_ACTION_LIMIT_GET;
    if (str_ieq(token, "PROTECT_ON")) return CMD_ACTION_PROTECT_ON;
    if (str_ieq(token, "PROTECT_OFF")) return CMD_ACTION_PROTECT_OFF;
    if (str_ieq(token, "PROTECT_STATUS")) return CMD_ACTION_PROTECT_STATUS;
    if (str_ieq(token, "ZERO")) return CMD_ACTION_ZERO;
    if (str_ieq(token, "PLACE_CLOTH")) return CMD_ACTION_PLACE_CLOTH;
    if (str_ieq(token, "TAKE_CLOTH")) return CMD_ACTION_TAKE_CLOTH;

    if (str_ieq(token, "HELP")) return CMD_ACTION_HELP;
    if (str_ieq(token, "VER")) return CMD_ACTION_VER;
    if (str_ieq(token, "PING")) return CMD_ACTION_PING;

    return CMD_ACTION_NONE;
}

const char *CmdParser_ResultText(CmdParseResult_t result)
{
    /* 解析结果转文本，供调试响应使用。 */
    switch (result) {
        case CMD_PARSE_OK: return "OK";
        case CMD_PARSE_EMPTY: return "EMPTY";
        case CMD_PARSE_BAD_TARGET: return "BAD_TARGET";
        case CMD_PARSE_BAD_ACTION: return "BAD_ACTION";
        case CMD_PARSE_BAD_ARG: return "BAD_ARG";
        default: return "UNKNOWN";
    }
}

CmdParseResult_t CmdParser_Parse(const char *line, ParsedCmd_t *out_cmd)
{
    char work[CMD_PARSER_LINE_BUF_SIZE];
    char *tokens[CMD_PARSER_MAX_TOKENS];
    char *token;
    char *eq;
    uint32_t i = 0U;
    uint32_t token_count = 0U;
    uint32_t arg_start_idx = 0U;

    /*
     * 命令基本格式示例：
     * M1 RUN DIR=CW RPM=60
     * AXIS MOVE X=1000 Y=2000 Z=3000 RPM=55
     * SYS STATUS
     * 参数统一采用 KEY=VALUE 形式。
     */
    if ((line == 0) || (out_cmd == 0)) {
        return CMD_PARSE_BAD_ARG;
    }

    memset(out_cmd, 0, sizeof(*out_cmd));
    out_cmd->motor_id = MOTOR_ID_INVALID;

    strncpy(work, line, sizeof(work) - 1U);
    work[sizeof(work) - 1U] = '\0';

    /* 按空格/Tab 分词，最多接收 CMD_PARSER_MAX_TOKENS 个 token。 */
    token = strtok(work, " \t");
    while ((token != 0) && (token_count < CMD_PARSER_MAX_TOKENS)) {
        tokens[token_count++] = token;
        token = strtok(0, " \t");
    }

    if (token_count == 0U) {
        return CMD_PARSE_EMPTY;
    }

    /*
     * 第一个 token 表示目标：
     * M1~M5 表示单个电机；
     * ALL 表示所有电机；
     * AXIS 表示三轴坐标；
     * EC1~EC5 表示编码器；
     * SYS 表示系统命令。
     */
    if ((tokens[0][0] == 'M' || tokens[0][0] == 'm') &&
        (tokens[0][1] >= '1') && (tokens[0][1] <= '5') &&
        (tokens[0][2] == '\0')) {
        out_cmd->target = CMD_TARGET_MOTOR;
        out_cmd->motor_id = (MotorId_t)(tokens[0][1] - '1');
    } else if (str_ieq(tokens[0], "ALL")) {
        out_cmd->target = CMD_TARGET_ALL;
    } else if (str_ieq(tokens[0], "AXIS")) {
        out_cmd->target = CMD_TARGET_AXIS;
    } else if ((tokens[0][0] == 'E' || tokens[0][0] == 'e') &&
               (tokens[0][1] == 'C' || tokens[0][1] == 'c') &&
               (tokens[0][2] >= '1') && (tokens[0][2] <= '5') &&
               (tokens[0][3] == '\0')) {
        out_cmd->target = CMD_TARGET_ENCODER;
        out_cmd->motor_id = (MotorId_t)(tokens[0][2] - '1');
    } else if (str_ieq(tokens[0], "SYS")) {
        out_cmd->target = CMD_TARGET_SYSTEM;
    } else if (str_ieq(tokens[0], "DEBUG")) {
        if (token_count != 2U) {
            return CMD_PARSE_BAD_ARG;
        }
        out_cmd->target = CMD_TARGET_SYSTEM;
        if (str_ieq(tokens[1], "ON")) {
            out_cmd->action = CMD_ACTION_DEBUG_ON;
        } else if (str_ieq(tokens[1], "OFF")) {
            out_cmd->action = CMD_ACTION_DEBUG_OFF;
        } else if (str_ieq(tokens[1], "STATUS")) {
            out_cmd->action = CMD_ACTION_DEBUG_STATUS;
        } else {
            return CMD_PARSE_BAD_ARG;
        }
        return CMD_PARSE_OK;
    } else if (str_ieq(tokens[0], "LASER")) {
        if (token_count != 2U) {
            return CMD_PARSE_BAD_ARG;
        }
        out_cmd->target = CMD_TARGET_SYSTEM;
        if (str_ieq(tokens[1], "STATUS")) {
            out_cmd->action = CMD_ACTION_LASER_STATUS;
        } else if (str_ieq(tokens[1], "DIAG")) {
            out_cmd->action = CMD_ACTION_LASER_DIAG;
        } else {
            return CMD_PARSE_BAD_ARG;
        }
        return CMD_PARSE_OK;
    } else if (str_ieq(tokens[0], "LASER2")) {
        out_cmd->target = CMD_TARGET_SYSTEM;
        if ((token_count == 2U) && str_ieq(tokens[1], "STATUS")) {
            out_cmd->action = CMD_ACTION_LASER2_STATUS;
        } else if ((token_count == 2U) && str_ieq(tokens[1], "DIAG")) {
            out_cmd->action = CMD_ACTION_LASER2_DIAG;
        } else if ((token_count == 3U) && str_ieq(tokens[1], "STREAM")) {
            if (str_ieq(tokens[2], "ON")) {
                out_cmd->action = CMD_ACTION_LASER2_STREAM_ON;
            } else if (str_ieq(tokens[2], "OFF")) {
                out_cmd->action = CMD_ACTION_LASER2_STREAM_OFF;
            } else if (str_ieq(tokens[2], "STATUS")) {
                out_cmd->action = CMD_ACTION_LASER2_STREAM_STATUS;
            } else {
                return CMD_PARSE_BAD_ARG;
            }
        } else {
            return CMD_PARSE_BAD_ARG;
        }
        return CMD_PARSE_OK;
    } else if (str_ieq(tokens[0], "MAGNET")) {
        if (token_count != 2U) {
            return CMD_PARSE_BAD_ARG;
        }
        out_cmd->target = CMD_TARGET_SYSTEM;
        if (str_ieq(tokens[1], "ON")) {
            out_cmd->action = CMD_ACTION_MAGNET_ON;
        } else if (str_ieq(tokens[1], "OFF")) {
            out_cmd->action = CMD_ACTION_MAGNET_OFF;
        } else if (str_ieq(tokens[1], "STATUS")) {
            out_cmd->action = CMD_ACTION_MAGNET_STATUS;
        } else {
            return CMD_PARSE_BAD_ARG;
        }
        return CMD_PARSE_OK;
    } else if (str_ieq(tokens[0], "STATUS")) {
        if (token_count != 1U) {
            return CMD_PARSE_BAD_ARG;
        }
        out_cmd->target = CMD_TARGET_SYSTEM;
        out_cmd->action = CMD_ACTION_STATUS;
        return CMD_PARSE_OK;
    } else if (str_ieq(tokens[0], "HELP")) {
        if (token_count != 1U) {
            return CMD_PARSE_BAD_ARG;
        }
        out_cmd->target = CMD_TARGET_SYSTEM;
        out_cmd->action = CMD_ACTION_HELP;
        return CMD_PARSE_OK;
    } else if (str_ieq(tokens[0], "VER")) {
        if (token_count != 1U) {
            return CMD_PARSE_BAD_ARG;
        }
        out_cmd->target = CMD_TARGET_SYSTEM;
        out_cmd->action = CMD_ACTION_VER;
        return CMD_PARSE_OK;
    } else if (str_ieq(tokens[0], "PING")) {
        if (token_count != 1U) {
            return CMD_PARSE_BAD_ARG;
        }
        out_cmd->target = CMD_TARGET_SYSTEM;
        out_cmd->action = CMD_ACTION_PING;
        return CMD_PARSE_OK;
    } else {
        return CMD_PARSE_BAD_TARGET;
    }

    if (token_count < 2U) {
        return CMD_PARSE_BAD_ACTION;
    }

    /* AXIS PROTECT ON/OFF/STATUS 是特殊三段式命令，单独解析。 */
    if ((out_cmd->target == CMD_TARGET_AXIS) && str_ieq(tokens[1], "PROTECT")) {
        if (token_count != 3U) {
            return CMD_PARSE_BAD_ARG;
        }
        if (str_ieq(tokens[2], "ON")) {
            out_cmd->action = CMD_ACTION_PROTECT_ON;
        } else if (str_ieq(tokens[2], "OFF")) {
            out_cmd->action = CMD_ACTION_PROTECT_OFF;
        } else if (str_ieq(tokens[2], "STATUS")) {
            out_cmd->action = CMD_ACTION_PROTECT_STATUS;
        } else {
            return CMD_PARSE_BAD_ARG;
        }
        arg_start_idx = 3U;
    } else {
        out_cmd->action = parse_action(tokens[1]);
        if (out_cmd->action == CMD_ACTION_NONE) {
            return CMD_PARSE_BAD_ACTION;
        }
        arg_start_idx = 2U;
    }

    /* 后续参数必须全部是 KEY=VALUE 格式。 */
    for (i = arg_start_idx; i < token_count; i++) {
        eq = strchr(tokens[i], '=');
        if ((eq == 0) || (eq == tokens[i]) || (eq[1] == '\0')) {
            return CMD_PARSE_BAD_ARG;
        }

        *eq = '\0';
        eq++;

        /* 根据参数名写入 args 对应字段，并设置 has_xxx 标志供分发层做参数校验。 */
        if (str_ieq(tokens[i], "DIR")) {
            if (str_ieq(eq, "CW")) {
                out_cmd->args.dir = MOTOR_DIR_CW;
            } else if (str_ieq(eq, "CCW")) {
                out_cmd->args.dir = MOTOR_DIR_CCW;
            } else {
                return CMD_PARSE_BAD_ARG;
            }
            out_cmd->args.has_dir = true;
        } else if (str_ieq(tokens[i], "RPM")) {
            if (!parse_float(eq, &out_cmd->args.rpm)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_rpm = true;
        } else if (str_ieq(tokens[i], "TO")) {
            if (!parse_float(eq, &out_cmd->args.to_rpm)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_to = true;
        } else if (str_ieq(tokens[i], "T")) {
            if (!parse_u32(eq, &out_cmd->args.time_ms)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_time_ms = true;
        } else if (str_ieq(tokens[i], "STEP") || str_ieq(tokens[i], "CNT")) {
            if (!parse_u32(eq, &out_cmd->args.step)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_step = true;
        } else if (str_ieq(tokens[i], "CYCLE")) {
            if (!parse_u32(eq, &out_cmd->args.cycle)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_cycle = true;
        } else if (str_ieq(tokens[i], "PPR")) {
            if (!parse_u32(eq, &out_cmd->args.ppr)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_ppr = true;
        } else if (str_ieq(tokens[i], "ACC")) {
            if (!parse_float(eq, &out_cmd->args.acc)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_acc = true;
        } else if (str_ieq(tokens[i], "DEC")) {
            if (!parse_float(eq, &out_cmd->args.dec)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_dec = true;
        } else if (str_ieq(tokens[i], "STOP")) {
            if (!parse_u32(eq, &out_cmd->args.stop_ms)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_stop_ms = true;
        } else if (str_ieq(tokens[i], "RATE")) {
            if (!parse_u32(eq, &out_cmd->args.rate)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_rate = true;
        } else if (str_ieq(tokens[i], "X")) {
            if (!parse_u32(eq, &out_cmd->args.x)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_x = true;
        } else if (str_ieq(tokens[i], "Y")) {
            if (!parse_u32(eq, &out_cmd->args.y)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_y = true;
        } else if (str_ieq(tokens[i], "Z")) {
            if (!parse_u32(eq, &out_cmd->args.z)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_z = true;
        } else if (str_ieq(tokens[i], "SLOT")) {
            if (!parse_u32(eq, &out_cmd->args.slot)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_slot = true;
        } else if (str_ieq(tokens[i], "ERR")) {
            if (!parse_u32(eq, &out_cmd->args.err)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_err = true;
        } else if (str_ieq(tokens[i], "TOL")) {
            if (!parse_u32(eq, &out_cmd->args.tol)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_tol = true;
        } else if (str_ieq(tokens[i], "DEG") || str_ieq(tokens[i], "ANGLE")) {
            if (!parse_float(eq, &out_cmd->args.deg)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_deg = true;
        } else if (str_ieq(tokens[i], "REG")) {
            if (!parse_u32(eq, &out_cmd->args.reg)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_reg = true;
        } else if (str_ieq(tokens[i], "VAL")) {
            if (!parse_u32(eq, &out_cmd->args.val)) return CMD_PARSE_BAD_ARG;
            out_cmd->args.has_val = true;
        } else {
            return CMD_PARSE_BAD_ARG;
        }
    }

    return CMD_PARSE_OK;
}
