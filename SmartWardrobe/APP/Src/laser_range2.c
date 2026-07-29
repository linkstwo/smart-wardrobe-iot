#include "laser_range2.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "uart_console.h"
#include "usart.h"

#define LASER2_MODBUS_ADDRESS 0x01U
#define LASER2_MODBUS_FUNC_READ_HOLDING 0x03U
#define LASER2_MODBUS_FUNC_EXCEPTION 0x83U
#define LASER2_MODBUS_RESPONSE_BYTES 4U
#define LASER2_MODBUS_NORMAL_FRAME_SIZE 9U
#define LASER2_MODBUS_EXCEPTION_FRAME_SIZE 5U
#define LASER2_UART_RX_CHUNK_SIZE 16U
#define LASER2_RX_RING_SIZE 64U
#define LASER2_RESPONSE_BUFFER_SIZE LASER2_MODBUS_NORMAL_FRAME_SIZE
#define LASER2_POLL_PERIOD_MS 50U
#define LASER2_RESPONSE_TIMEOUT_MS 100U
#define LASER2_STATUS_ONLINE_TIMEOUT_MS 300U
#define LASER2_DISTANCE_MIN_MM 50U
#define LASER2_DISTANCE_MAX_MM 4000U
#define LASER2_CONFIDENCE_MIN 70U
#define LASER2_CONFIDENCE_MAX 100U
#define LASER2_SAMPLE_QUEUE_DEPTH 8U
#define LASER2_STREAM_PERIOD_MS 200U /* 串口助手连续上报周期，单位ms，即每秒5次。 */

static const uint8_t g_laser2_modbus_request[8] = {
    LASER2_MODBUS_ADDRESS,
    LASER2_MODBUS_FUNC_READ_HOLDING,
    0x00U, 0x03U,
    0x00U, 0x02U,
    0x34U, 0x0BU
};

static uint8_t g_laser2_uart_rx_chunk[LASER2_UART_RX_CHUNK_SIZE];
static volatile uint8_t g_laser2_rx_ring[LASER2_RX_RING_SIZE];
static volatile uint16_t g_laser2_rx_head;
static volatile uint16_t g_laser2_rx_tail;
static volatile uint8_t g_laser2_recovery_pending;

static uint8_t g_laser2_response[LASER2_RESPONSE_BUFFER_SIZE];
static uint8_t g_laser2_response_len;
static volatile LaserRangeState_t g_laser2_state;
static uint32_t g_laser2_state_started_ms;
static uint32_t g_laser2_next_request_ms;

static LaserRangeSample_t g_laser2_latest;
static uint8_t g_laser2_latest_valid;
static uint32_t g_laser2_sequence;

static LaserRangeSample_t g_laser2_sample_queue[LASER2_SAMPLE_QUEUE_DEPTH];
static uint8_t g_laser2_sample_head;
static uint8_t g_laser2_sample_tail;
static uint8_t g_laser2_sample_count;
static LaserRangeStatus_t g_laser2_status;
static uint8_t g_laser2_stream_enabled;
static uint32_t g_laser2_stream_last_ms;
static char g_laser2_stream_msg[128];

static uint16_t laser2_modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static HAL_StatusTypeDef laser2_restart_receive(void)
{
    HAL_StatusTypeDef rc;

    rc = HAL_UARTEx_ReceiveToIdle_IT(&huart4,
                                     g_laser2_uart_rx_chunk,
                                     LASER2_UART_RX_CHUNK_SIZE);
    if (rc != HAL_OK) {
        g_laser2_recovery_pending = 1U;
    }
    return rc;
}

static void laser2_queue_sample(const LaserRangeSample_t *sample)
{
    if (g_laser2_sample_count >= LASER2_SAMPLE_QUEUE_DEPTH) {
        g_laser2_sample_head =
            (uint8_t)((g_laser2_sample_head + 1U) % LASER2_SAMPLE_QUEUE_DEPTH);
        g_laser2_sample_count--;
    }

    g_laser2_sample_queue[g_laser2_sample_tail] = *sample;
    g_laser2_sample_tail =
        (uint8_t)((g_laser2_sample_tail + 1U) % LASER2_SAMPLE_QUEUE_DEPTH);
    g_laser2_sample_count++;
}

static void laser2_finish_request(uint32_t now_ms, uint8_t keep_period)
{
    g_laser2_state = LASER_RANGE_STATE_IDLE;
    g_laser2_state_started_ms = now_ms;
    g_laser2_response_len = 0U;
    if ((keep_period == 0U) ||
        ((int32_t)(now_ms - g_laser2_next_request_ms) >= 0)) {
        g_laser2_next_request_ms = now_ms + LASER2_POLL_PERIOD_MS;
    }
}

static void laser2_accept_normal_response(uint32_t now_ms)
{
    LaserRangeSample_t sample;
    uint16_t distance_mm;
    uint16_t confidence;
    uint8_t distance_valid;
    uint8_t confidence_valid;

    g_laser2_status.response_count++;
    distance_mm =
        (uint16_t)(((uint16_t)g_laser2_response[3] << 8U) |
                   g_laser2_response[4]);
    confidence =
        (uint16_t)(((uint16_t)g_laser2_response[5] << 8U) |
                   g_laser2_response[6]);
    g_laser2_status.raw_distance_mm = distance_mm;
    g_laser2_status.raw_confidence = confidence;
    distance_valid = ((distance_mm >= LASER2_DISTANCE_MIN_MM) &&
                      (distance_mm <= LASER2_DISTANCE_MAX_MM)) ? 1U : 0U;
    confidence_valid = ((confidence >= LASER2_CONFIDENCE_MIN) &&
                        (confidence <= LASER2_CONFIDENCE_MAX)) ? 1U : 0U;
    g_laser2_status.data_valid = (distance_valid != 0U);

    if (distance_valid == 0U) {
        g_laser2_status.distance_reject_count++;
    }
    if (confidence_valid == 0U) {
        g_laser2_status.confidence_reject_count++;
    }
    if (distance_valid == 0U) {
        g_laser2_status.rejected_count++;
    }

    memset(&sample, 0, sizeof(sample));
    sample.distance_mm = distance_mm;
    sample.confidence =
        (confidence <= 0xFFU) ? (uint8_t)confidence : 0xFFU;
    sample.distance_valid = (distance_valid != 0U);
    sample.timestamp_ms = now_ms;
    g_laser2_sequence++;
    sample.sequence = g_laser2_sequence;
    laser2_queue_sample(&sample);

    if (distance_valid != 0U) {
        g_laser2_latest = sample;
        g_laser2_latest_valid = 1U;
    }
    laser2_finish_request(now_ms, 1U);
}

static void laser2_accept_response(uint8_t frame_len)
{
    uint16_t expected_crc;
    uint16_t received_crc;
    uint32_t now_ms = HAL_GetTick();

    if ((g_laser2_state != LASER_RANGE_STATE_TX_REQUEST) &&
        (g_laser2_state != LASER_RANGE_STATE_WAIT_RESPONSE)) {
        g_laser2_status.protocol_error_count++;
        g_laser2_response_len = 0U;
        return;
    }

    expected_crc =
        laser2_modbus_crc16(g_laser2_response, (uint16_t)(frame_len - 2U));
    received_crc =
        (uint16_t)g_laser2_response[frame_len - 2U] |
        ((uint16_t)g_laser2_response[frame_len - 1U] << 8U);
    if (expected_crc != received_crc) {
        g_laser2_status.crc_error_count++;
        laser2_finish_request(now_ms, 0U);
        return;
    }

    g_laser2_status.last_response_ms = now_ms;

    if (g_laser2_response[1] == LASER2_MODBUS_FUNC_EXCEPTION) {
        g_laser2_status.data_valid = false;
        g_laser2_status.exception_count++;
        laser2_finish_request(now_ms, 1U);
        return;
    }

    laser2_accept_normal_response(now_ms);
}

static void laser2_response_restart(uint8_t byte)
{
    g_laser2_response_len = 0U;
    if (byte == LASER2_MODBUS_ADDRESS) {
        g_laser2_response[0] = byte;
        g_laser2_response_len = 1U;
    }
}

static void laser2_parse_byte(uint8_t byte)
{
    uint8_t expected_len = 0U;

    if (g_laser2_response_len == 0U) {
        if (byte == LASER2_MODBUS_ADDRESS) {
            g_laser2_response[g_laser2_response_len++] = byte;
        }
        return;
    }

    if (g_laser2_response_len >= LASER2_RESPONSE_BUFFER_SIZE) {
        g_laser2_status.protocol_error_count++;
        laser2_response_restart(byte);
        return;
    }

    g_laser2_response[g_laser2_response_len++] = byte;
    if (g_laser2_response_len == 2U) {
        if ((g_laser2_response[1] != LASER2_MODBUS_FUNC_READ_HOLDING) &&
            (g_laser2_response[1] != LASER2_MODBUS_FUNC_EXCEPTION)) {
            g_laser2_status.protocol_error_count++;
            laser2_response_restart(byte);
        }
        return;
    }

    if ((g_laser2_response[1] == LASER2_MODBUS_FUNC_READ_HOLDING) &&
        (g_laser2_response_len == 3U) &&
        (g_laser2_response[2] != LASER2_MODBUS_RESPONSE_BYTES)) {
        g_laser2_status.protocol_error_count++;
        laser2_response_restart(byte);
        return;
    }

    if (g_laser2_response[1] == LASER2_MODBUS_FUNC_READ_HOLDING) {
        expected_len = LASER2_MODBUS_NORMAL_FRAME_SIZE;
    } else if (g_laser2_response[1] == LASER2_MODBUS_FUNC_EXCEPTION) {
        expected_len = LASER2_MODBUS_EXCEPTION_FRAME_SIZE;
    }

    if ((expected_len != 0U) &&
        (g_laser2_response_len == expected_len)) {
        laser2_accept_response(expected_len);
    }
}

static void laser2_start_request(uint32_t now_ms)
{
    HAL_StatusTypeDef rc;

    g_laser2_response_len = 0U;
    g_laser2_state = LASER_RANGE_STATE_TX_REQUEST;
    g_laser2_state_started_ms = now_ms;
    g_laser2_next_request_ms = now_ms + LASER2_POLL_PERIOD_MS;
    rc = HAL_UART_Transmit_IT(&huart4,
                              (uint8_t *)g_laser2_modbus_request,
                              (uint16_t)sizeof(g_laser2_modbus_request));
    if (rc == HAL_OK) {
        g_laser2_status.request_count++;
        return;
    }

    g_laser2_status.tx_error_count++;
    laser2_finish_request(now_ms, 0U);
}

void LaserRange2_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    memset(g_laser2_uart_rx_chunk, 0, sizeof(g_laser2_uart_rx_chunk));
    memset((void *)g_laser2_rx_ring, 0, sizeof(g_laser2_rx_ring));
    memset(g_laser2_response, 0, sizeof(g_laser2_response));
    memset(&g_laser2_latest, 0, sizeof(g_laser2_latest));
    memset(g_laser2_sample_queue, 0, sizeof(g_laser2_sample_queue));
    memset(&g_laser2_status, 0, sizeof(g_laser2_status));

    g_laser2_rx_head = 0U;
    g_laser2_rx_tail = 0U;
    g_laser2_recovery_pending = 0U;
    g_laser2_response_len = 0U;
    g_laser2_state = LASER_RANGE_STATE_IDLE;
    g_laser2_state_started_ms = now_ms;
    g_laser2_next_request_ms = now_ms;
    g_laser2_latest_valid = 0U;
    g_laser2_sequence = 0U;
    g_laser2_sample_head = 0U;
    g_laser2_sample_tail = 0U;
    g_laser2_sample_count = 0U;
    g_laser2_status.address = LASER2_MODBUS_ADDRESS;
    g_laser2_stream_enabled = 0U;
    g_laser2_stream_last_ms = now_ms;
    memset(g_laser2_stream_msg, 0, sizeof(g_laser2_stream_msg));
    (void)laser2_restart_receive();
}

void LaserRange2_OnUartRxEvent(uint16_t size)
{
    uint16_t i;

    if (size > LASER2_UART_RX_CHUNK_SIZE) {
        size = LASER2_UART_RX_CHUNK_SIZE;
    }

    for (i = 0U; i < size; i++) {
        uint16_t next =
            (uint16_t)((g_laser2_rx_head + 1U) % LASER2_RX_RING_SIZE);
        if (next == g_laser2_rx_tail) {
            g_laser2_status.protocol_error_count++;
            break;
        }
        g_laser2_rx_ring[g_laser2_rx_head] = g_laser2_uart_rx_chunk[i];
        g_laser2_rx_head = next;
    }

    (void)laser2_restart_receive();
}

void LaserRange2_OnUartTxComplete(void)
{
    if (g_laser2_state == LASER_RANGE_STATE_TX_REQUEST) {
        g_laser2_state = LASER_RANGE_STATE_WAIT_RESPONSE;
        g_laser2_state_started_ms = HAL_GetTick();
    }
}

void LaserRange2_OnUartError(void)
{
    g_laser2_status.uart_error_count++;
    g_laser2_recovery_pending = 1U;
}

void LaserRange2_Service(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (g_laser2_recovery_pending != 0U) {
        (void)HAL_UART_AbortTransmit(&huart4);
        (void)HAL_UART_AbortReceive(&huart4);
        g_laser2_recovery_pending = 0U;
        g_laser2_response_len = 0U;
        g_laser2_rx_head = 0U;
        g_laser2_rx_tail = 0U;
        g_laser2_state = LASER_RANGE_STATE_IDLE;
        g_laser2_state_started_ms = now_ms;
        g_laser2_next_request_ms = now_ms + LASER2_POLL_PERIOD_MS;
        (void)laser2_restart_receive();
    }

    while (g_laser2_rx_tail != g_laser2_rx_head) {
        uint8_t byte = g_laser2_rx_ring[g_laser2_rx_tail];
        g_laser2_rx_tail =
            (uint16_t)((g_laser2_rx_tail + 1U) % LASER2_RX_RING_SIZE);
        laser2_parse_byte(byte);
    }

    now_ms = HAL_GetTick();
    if (((g_laser2_state == LASER_RANGE_STATE_TX_REQUEST) ||
         (g_laser2_state == LASER_RANGE_STATE_WAIT_RESPONSE)) &&
        ((uint32_t)(now_ms - g_laser2_state_started_ms) >=
         LASER2_RESPONSE_TIMEOUT_MS)) {
        if (g_laser2_state == LASER_RANGE_STATE_TX_REQUEST) {
            (void)HAL_UART_AbortTransmit(&huart4);
            g_laser2_status.tx_error_count++;
        } else {
            g_laser2_status.timeout_count++;
        }
        laser2_finish_request(now_ms, 0U);
    }

    if ((g_laser2_state == LASER_RANGE_STATE_IDLE) &&
        ((int32_t)(now_ms - g_laser2_next_request_ms) >= 0)) {
        laser2_start_request(now_ms);
    }
}

bool LaserRange2_GetLatest(LaserRangeSample_t *out)
{
    if ((out == 0) || (g_laser2_latest_valid == 0U)) {
        return false;
    }
    *out = g_laser2_latest;
    return true;
}

bool LaserRange2_PollSample(LaserRangeSample_t *out)
{
    if ((out == 0) || (g_laser2_sample_count == 0U)) {
        return false;
    }

    *out = g_laser2_sample_queue[g_laser2_sample_head];
    g_laser2_sample_head =
        (uint8_t)((g_laser2_sample_head + 1U) % LASER2_SAMPLE_QUEUE_DEPTH);
    g_laser2_sample_count--;
    return true;
}

void LaserRange2_FlushSamples(void)
{
    g_laser2_sample_head = 0U;
    g_laser2_sample_tail = 0U;
    g_laser2_sample_count = 0U;
}

bool LaserRange2_IsOnline(uint32_t timeout_ms)
{
    if ((g_laser2_latest_valid == 0U) || (timeout_ms == 0U)) {
        return false;
    }
    return ((uint32_t)(HAL_GetTick() - g_laser2_latest.timestamp_ms) <=
            timeout_ms);
}

bool LaserRange2_IsCommunicationOnline(uint32_t timeout_ms)
{
    if ((g_laser2_status.last_response_ms == 0U) || (timeout_ms == 0U)) {
        return false;
    }
    return ((uint32_t)(HAL_GetTick() -
                       g_laser2_status.last_response_ms) <= timeout_ms);
}

bool LaserRange2_GetStatus(LaserRangeStatus_t *out)
{
    if (out == 0) {
        return false;
    }

    *out = g_laser2_status;
    out->state = g_laser2_state;
    out->communication_online =
        LaserRange2_IsCommunicationOnline(LASER2_STATUS_ONLINE_TIMEOUT_MS);
    if (!out->communication_online) {
        out->data_valid = false;
    }
    out->online = LaserRange2_IsOnline(LASER2_STATUS_ONLINE_TIMEOUT_MS);
    if (g_laser2_latest_valid != 0U) {
        out->distance_mm = g_laser2_latest.distance_mm;
        out->confidence = g_laser2_latest.confidence;
    }
    return true;
}

const char *LaserRange2_StateText(LaserRangeState_t state)
{
    switch (state) {
        case LASER_RANGE_STATE_IDLE:
            return "IDLE";
        case LASER_RANGE_STATE_TX_REQUEST:
            return "TX";
        case LASER_RANGE_STATE_WAIT_RESPONSE:
            return "WAIT";
        default:
            return "UNKNOWN";
    }
}

void LaserRange2_SetStreamEnabled(bool enable)
{
    g_laser2_stream_enabled = enable ? 1U : 0U;
    g_laser2_stream_last_ms = HAL_GetTick();
}

bool LaserRange2_GetStreamEnabled(void)
{
    return (g_laser2_stream_enabled != 0U);
}

void LaserRange2_StreamService(void)
{
    LaserRangeStatus_t status;
    uint32_t now_ms;
    int written;

    if ((g_laser2_stream_enabled == 0U) ||
        !UartConsole_GetTextOutputEnabled()) {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_laser2_stream_last_ms) <
        LASER2_STREAM_PERIOD_MS) {
        return;
    }
    g_laser2_stream_last_ms = now_ms;

    if (!LaserRange2_GetStatus(&status)) {
        return;
    }

    written = snprintf(g_laser2_stream_msg,
                       sizeof(g_laser2_stream_msg),
                       "LASER2 DATA DIST=%u CONF=%u COMM=%u VALID=%u ONLINE=%u SEQ=%lu",
                       (unsigned int)status.raw_distance_mm,
                       (unsigned int)status.raw_confidence,
                       status.communication_online ? 1U : 0U,
                       status.data_valid ? 1U : 0U,
                       status.online ? 1U : 0U,
                       (unsigned long)g_laser2_sequence);
    if ((written <= 0) ||
        ((uint32_t)written >= (uint32_t)sizeof(g_laser2_stream_msg))) {
        return;
    }

    /*
     * 流数据只使用普通队列。队列忙时丢弃本次测距输出，不覆盖电机、
     * 回零或故障日志。
     */
    (void)UartConsole_QueueLine(g_laser2_stream_msg);
}
