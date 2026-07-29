#ifndef CMD_DISPATCH_H
#define CMD_DISPATCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "cmd_parser.h"

typedef enum {
    /* 命令执行结果，只表示分发层是否成功生成响应。 */
    CMD_RESULT_OK = 0,
    CMD_RESULT_ERR = -1
} CmdResult_t;

typedef enum {
    /* 取/放衣流程事件，main.c 会把这些事件转换成云端响应帧。 */
    CMD_CLOTH_EVENT_NONE = 0,
    CMD_CLOTH_EVENT_PLACE_DONE,
    CMD_CLOTH_EVENT_TAKE_DONE,
    CMD_CLOTH_EVENT_PLACE_FAULT,
    CMD_CLOTH_EVENT_TAKE_FAULT
} CmdClothEvent_t;

/* 执行一条解析后的文本命令，并把响应写入 resp。 */
CmdResult_t CmdDispatch_Execute(const ParsedCmd_t *cmd, char *resp, uint16_t resp_len);
/* 生成系统状态响应。 */
CmdResult_t CmdDispatch_FormatSystemStatus(char *resp, uint16_t resp_len);
/* 1ms 安全调度入口：推进取放衣流程、回零监控、编码器目标监控等。 */
void CmdDispatch_SafetyTick1ms(void);
/* M1/M3/M4 的 ALL HOME 启动成功后，请求执行 M2 上电回零。 */
void CmdDispatch_RequestM2BootHome(void);
/* 轮询取/放衣完成或故障事件。 */
bool CmdDispatch_PollClothEvent(CmdClothEvent_t *event, uint8_t *fault_code);

#ifdef __cplusplus
}
#endif

#endif
