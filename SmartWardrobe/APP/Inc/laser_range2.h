#ifndef LASER_RANGE2_H
#define LASER_RANGE2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "laser_range.h"

/*
 * EC5 second ranging sensor.
 * It uses the same Modbus data format as LaserRange, but owns an independent
 * UART4 state machine, receive buffer, sample queue, and diagnostic counters.
 */
void LaserRange2_Init(void);
void LaserRange2_Service(void);
void LaserRange2_OnUartRxEvent(uint16_t size);
void LaserRange2_OnUartTxComplete(void);
void LaserRange2_OnUartError(void);
bool LaserRange2_GetLatest(LaserRangeSample_t *out);
bool LaserRange2_PollSample(LaserRangeSample_t *out);
void LaserRange2_FlushSamples(void);
bool LaserRange2_IsOnline(uint32_t timeout_ms);
bool LaserRange2_IsCommunicationOnline(uint32_t timeout_ms);
bool LaserRange2_GetStatus(LaserRangeStatus_t *out);
const char *LaserRange2_StateText(LaserRangeState_t state);
void LaserRange2_SetStreamEnabled(bool enable);
bool LaserRange2_GetStreamEnabled(void);
void LaserRange2_StreamService(void);

#ifdef __cplusplus
}
#endif

#endif
