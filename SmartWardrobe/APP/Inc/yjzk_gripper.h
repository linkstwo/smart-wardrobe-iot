#ifndef YJZK_GRIPPER_H
#define YJZK_GRIPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 夹爪默认 Modbus 地址、状态寄存器、动作寄存器和开合动作值。 */
#define YJZK_GRIPPER_DEFAULT_ADDR 1U
#define YJZK_GRIPPER_STATUS_REG 64U
#define YJZK_GRIPPER_DEFAULT_ACTION_REG 61U
#define YJZK_GRIPPER_DEFAULT_CLOSE_VALUE 1U
#define YJZK_GRIPPER_DEFAULT_OPEN_VALUE 0U

enum {
    /* 夹爪 Modbus 通信错误码。 */
    YJZK_GRIPPER_ERR_ARG = -1,
    YJZK_GRIPPER_ERR_TX = -2,
    YJZK_GRIPPER_ERR_RX = -3,
    YJZK_GRIPPER_ERR_FRAME = -4,
    YJZK_GRIPPER_ERR_CRC = -5,
    YJZK_GRIPPER_ERR_EXCEPTION = -6
};

typedef struct {
    /* 夹爪寄存器读回结果。 */
    uint8_t addr;
    uint16_t reg;
    uint16_t value;
} YjzkGripperReg_t;

/* 初始化夹爪串口接口。 */
void YjzkGripper_Init(void);
/* 读写夹爪寄存器。 */
int YjzkGripper_ReadReg(uint8_t addr, uint16_t reg, uint16_t *out_value);
int YjzkGripper_WriteReg(uint8_t addr, uint16_t reg, uint16_t value);
/* 读取默认夹爪状态，执行闭合或打开动作。 */
int YjzkGripper_ReadStatus(YjzkGripperReg_t *out);
int YjzkGripper_Close(void);
int YjzkGripper_Open(void);
const char *YjzkGripper_ErrorText(int err);

#ifdef __cplusplus
}
#endif

#endif
