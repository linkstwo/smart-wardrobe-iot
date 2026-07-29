#include "heartbeat.h"

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "uart_console.h"

/*
 * 心跳模块运行状态：
 * enabled   控制是否周期性输出心跳；
 * period_ms 心跳周期；
 * elapsed_ms 用于 1ms tick 累计；
 * sequence  心跳序号，便于上位机判断是否连续；
 * lost      发送队列满或丢弃旧消息时累计丢失次数。
 */
typedef struct {
    uint8_t enabled;
    uint16_t period_ms;
    uint16_t elapsed_ms;
    uint32_t sequence;
    uint32_t lost;
} HeartbeatRuntime_t;

static HeartbeatRuntime_t g_hb;
static char g_hb_line[UART_CONSOLE_TX_MSG_MAX];

static void heartbeat_emit(void)
{
    bool dropped = false;

    /* 每次发送心跳前递增序号，消息里同时带系统 tick 和丢失计数。 */
    g_hb.sequence++;
    (void)snprintf(g_hb_line,
                   sizeof(g_hb_line),
                   "HB TS=%lu SEQ=%lu LOST=%lu",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)g_hb.sequence,
                   (unsigned long)g_hb.lost);

    /*
     * 心跳走串口文本输出队列。
     * 如果队列完全无法写入，或者为了写入新心跳而丢弃了旧消息，都累计 lost。
     */
    if (!UartConsole_QueueLineDropOldest(g_hb_line, &dropped)) {
        g_hb.lost++;
        return;
    }

    if (dropped) {
        g_hb.lost++;
    }
}

void Heartbeat_Init(void)
{
    /* 默认开启心跳，周期使用头文件中的 HEARTBEAT_PERIOD_DEFAULT_MS。 */
    g_hb.enabled = 1U;
    g_hb.period_ms = HEARTBEAT_PERIOD_DEFAULT_MS;
    g_hb.elapsed_ms = 0U;
    g_hb.sequence = 0U;
    g_hb.lost = 0U;
}

void Heartbeat_Tick1ms(void)
{
    /* 由主循环或 SysTick 路径每 1ms 调用一次，用软件计时决定何时输出心跳。 */
    if (g_hb.enabled == 0U) {
        return;
    }

    g_hb.elapsed_ms++;
    if (g_hb.elapsed_ms >= g_hb.period_ms) {
        g_hb.elapsed_ms = 0U;
        heartbeat_emit();
    }
}

int Heartbeat_Set(bool enable, uint16_t period_ms)
{
    /* period_ms 为 0 时表示使用默认周期，非 0 时必须落在允许范围内。 */
    if (period_ms == 0U) {
        period_ms = HEARTBEAT_PERIOD_DEFAULT_MS;
    }
    if ((period_ms < HEARTBEAT_PERIOD_MIN_MS) || (period_ms > HEARTBEAT_PERIOD_MAX_MS)) {
        return -1;
    }

    g_hb.period_ms = period_ms;
    g_hb.elapsed_ms = 0U;
    g_hb.enabled = enable ? 1U : 0U;
    return 0;
}

int Heartbeat_GetStatus(HeartbeatStatus_t *out)
{
    /* 输出当前心跳状态，供系统状态命令查询。 */
    if (out == 0) {
        return -1;
    }

    out->enabled = (g_hb.enabled != 0U);
    out->period_ms = g_hb.period_ms;
    out->sequence = g_hb.sequence;
    out->lost = g_hb.lost;
    return 0;
}
