#ifndef RS485_ABS_ENCODER_H
#define RS485_ABS_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* M2 绝对值编码器 Modbus 地址和一圈计数。 */
#define RS485_ABS_ENCODER_M2_ADDR 1U
#define RS485_ABS_ENCODER_CPR 1024U

typedef struct {
    /* 编码器 Modbus 地址。 */
    uint8_t addr;
    /* 原始一圈内计数。 */
    uint16_t raw;
    /* 换算后的角度，范围约 0~360。 */
    float deg;
    /* 读取时刻 HAL tick。 */
    uint32_t tick_ms;
} Rs485AbsEncoderStatus_t;

/* 初始化 RS485 绝对值编码器接口。 */
void Rs485AbsEncoder_Init(void);
/* 供 1ms 控制路径使用的 M2 非阻塞位置读取接口。 */
int Rs485AbsEncoder_RequestM2Read(void);
int Rs485AbsEncoder_PollM2Read(Rs485AbsEncoderStatus_t *out);
void Rs485AbsEncoder_Service(void);
void Rs485AbsEncoder_CancelM2Read(void);
void Rs485AbsEncoder_OnUartRxEvent(uint16_t size);
void Rs485AbsEncoder_OnUartError(void);
/* 读取指定地址或默认 M2 地址的绝对值编码器。 */
int Rs485AbsEncoder_Read(uint8_t addr, Rs485AbsEncoderStatus_t *out);
int Rs485AbsEncoder_ReadM2(Rs485AbsEncoderStatus_t *out);
/* 设置当前角度为编码器零点。 */
int Rs485AbsEncoder_SetCurrentZero(uint8_t addr);
int Rs485AbsEncoder_SetM2CurrentZero(void);
const char *Rs485AbsEncoder_ErrorText(int err);

#ifdef __cplusplus
}
#endif

#endif
