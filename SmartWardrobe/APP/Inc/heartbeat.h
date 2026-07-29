#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* 心跳默认周期和允许范围，单位 ms。 */
#define HEARTBEAT_PERIOD_DEFAULT_MS 3000U
#define HEARTBEAT_PERIOD_MIN_MS 100U
#define HEARTBEAT_PERIOD_MAX_MS 60000U

typedef struct {
    /* 是否开启心跳。 */
    bool enabled;
    /* 当前心跳周期。 */
    uint16_t period_ms;
    /* 已发送心跳序号。 */
    uint32_t sequence;
    /* 因队列满等原因丢失的心跳数量。 */
    uint32_t lost;
} HeartbeatStatus_t;

/* 初始化心跳模块，默认开启。 */
void Heartbeat_Init(void);
/* 1ms 调度入口，用于按周期输出心跳。 */
void Heartbeat_Tick1ms(void);
int Heartbeat_Set(bool enable, uint16_t period_ms);
int Heartbeat_GetStatus(HeartbeatStatus_t *out);

#ifdef __cplusplus
}
#endif

#endif
