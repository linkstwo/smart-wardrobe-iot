#include "laser_range.h"

#include <string.h>
#include "main.h"
#include "usart.h"

#define LASER_MODBUS_ADDRESS 0x01U
#define LASER_MODBUS_FUNC_READ_HOLDING 0x03U
#define LASER_MODBUS_FUNC_EXCEPTION 0x83U
#define LASER_MODBUS_RESPONSE_BYTES 4U
#define LASER_MODBUS_NORMAL_FRAME_SIZE 9U
#define LASER_MODBUS_EXCEPTION_FRAME_SIZE 5U
#define LASER_UART_RX_CHUNK_SIZE 16U
#define LASER_RX_RING_SIZE 64U
#define LASER_RESPONSE_BUFFER_SIZE LASER_MODBUS_NORMAL_FRAME_SIZE
#define LASER_POLL_PERIOD_MS 50U
#define LASER_RESPONSE_TIMEOUT_MS 100U
#define LASER_STATUS_ONLINE_TIMEOUT_MS 300U
#define LASER_DISTANCE_MIN_MM 50U
#define LASER_DISTANCE_MAX_MM 4000U
#define LASER_CONFIDENCE_MIN 70U
#define LASER_CONFIDENCE_MAX 100U
#define LASER_SAMPLE_QUEUE_DEPTH 8U

static const uint8_t g_modbus_request[8] = {
    LASER_MODBUS_ADDRESS,
    LASER_MODBUS_FUNC_READ_HOLDING,
    0x00U, 0x03U,
    0x00U, 0x02U,
    0x34U, 0x0BU
};

static uint8_t g_uart_rx_chunk[LASER_UART_RX_CHUNK_SIZE];
static volatile uint8_t g_rx_ring[LASER_RX_RING_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint8_t g_recovery_pending;

static uint8_t g_response[LASER_RESPONSE_BUFFER_SIZE];
static uint8_t g_response_len;
static volatile LaserRangeState_t g_state;
static uint32_t g_state_started_ms;
static uint32_t g_next_request_ms;

static LaserRangeSample_t g_latest;
static uint8_t g_latest_valid;
static uint32_t g_sequence;

static LaserRangeSample_t g_sample_queue[LASER_SAMPLE_QUEUE_DEPTH];
static uint8_t g_sample_head;
static uint8_t g_sample_tail;
static uint8_t g_sample_count;
static LaserRangeStatus_t g_status;

static uint16_t laser_modbus_crc16(const uint8_t *data, uint16_t len)
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

static HAL_StatusTypeDef laser_restart_receive(void)
{
    HAL_StatusTypeDef rc;

    rc = HAL_UARTEx_ReceiveToIdle_IT(&huart1,
                                     g_uart_rx_chunk,
                                     LASER_UART_RX_CHUNK_SIZE);
    if (rc != HAL_OK) {
        g_recovery_pending = 1U;
    }
    return rc;
}

static void laser_queue_sample(const LaserRangeSample_t *sample)
{
    if (g_sample_count >= LASER_SAMPLE_QUEUE_DEPTH) {
        g_sample_head = (uint8_t)((g_sample_head + 1U) % LASER_SAMPLE_QUEUE_DEPTH);
        g_sample_count--;
    }

    g_sample_queue[g_sample_tail] = *sample;
    g_sample_tail = (uint8_t)((g_sample_tail + 1U) % LASER_SAMPLE_QUEUE_DEPTH);
    g_sample_count++;
}

static void laser_finish_request(uint32_t now_ms, uint8_t keep_period)
{
    g_state = LASER_RANGE_STATE_IDLE;
    g_state_started_ms = now_ms;
    g_response_len = 0U;
    if ((keep_period == 0U) || ((int32_t)(now_ms - g_next_request_ms) >= 0)) {
        g_next_request_ms = now_ms + LASER_POLL_PERIOD_MS;
    }
}

static void laser_accept_normal_response(uint32_t now_ms)
{
    LaserRangeSample_t sample;
    uint16_t distance_mm;
    uint16_t confidence;
    uint8_t distance_valid;
    uint8_t confidence_valid;

    g_status.response_count++;
    distance_mm = (uint16_t)(((uint16_t)g_response[3] << 8U) | g_response[4]);
    confidence = (uint16_t)(((uint16_t)g_response[5] << 8U) | g_response[6]);
    g_status.raw_distance_mm = distance_mm;
    g_status.raw_confidence = confidence;
    distance_valid = ((distance_mm >= LASER_DISTANCE_MIN_MM) &&
                      (distance_mm <= LASER_DISTANCE_MAX_MM)) ? 1U : 0U;
    confidence_valid = ((confidence >= LASER_CONFIDENCE_MIN) &&
                        (confidence <= LASER_CONFIDENCE_MAX)) ? 1U : 0U;
    g_status.data_valid = (distance_valid != 0U);

    if (distance_valid == 0U) {
        g_status.distance_reject_count++;
    }
    if (confidence_valid == 0U) {
        g_status.confidence_reject_count++;
    }
    if (distance_valid == 0U) {
        g_status.rejected_count++;
    }

    memset(&sample, 0, sizeof(sample));
    sample.distance_mm = distance_mm;
    sample.confidence = (confidence <= 0xFFU) ? (uint8_t)confidence : 0xFFU;
    sample.distance_valid = (distance_valid != 0U);
    sample.timestamp_ms = now_ms;
    g_sequence++;
    sample.sequence = g_sequence;
    laser_queue_sample(&sample);

    if (distance_valid != 0U) {
        g_latest = sample;
        g_latest_valid = 1U;
    }
    laser_finish_request(now_ms, 1U);
}

static void laser_accept_response(uint8_t frame_len)
{
    uint16_t expected_crc;
    uint16_t received_crc;
    uint32_t now_ms = HAL_GetTick();

    if ((g_state != LASER_RANGE_STATE_TX_REQUEST) &&
        (g_state != LASER_RANGE_STATE_WAIT_RESPONSE)) {
        g_status.protocol_error_count++;
        g_response_len = 0U;
        return;
    }

    expected_crc = laser_modbus_crc16(g_response, (uint16_t)(frame_len - 2U));
    received_crc = (uint16_t)g_response[frame_len - 2U] |
                   ((uint16_t)g_response[frame_len - 1U] << 8U);
    if (expected_crc != received_crc) {
        g_status.crc_error_count++;
        laser_finish_request(now_ms, 0U);
        return;
    }

    g_status.last_response_ms = now_ms;

    if (g_response[1] == LASER_MODBUS_FUNC_EXCEPTION) {
        g_status.data_valid = false;
        g_status.exception_count++;
        laser_finish_request(now_ms, 1U);
        return;
    }

    laser_accept_normal_response(now_ms);
}

static void laser_response_restart(uint8_t byte)
{
    g_response_len = 0U;
    if (byte == LASER_MODBUS_ADDRESS) {
        g_response[0] = byte;
        g_response_len = 1U;
    }
}

static void laser_parse_byte(uint8_t byte)
{
    uint8_t expected_len = 0U;

    if (g_response_len == 0U) {
        if (byte == LASER_MODBUS_ADDRESS) {
            g_response[g_response_len++] = byte;
        }
        return;
    }

    if (g_response_len >= LASER_RESPONSE_BUFFER_SIZE) {
        g_status.protocol_error_count++;
        laser_response_restart(byte);
        return;
    }

    g_response[g_response_len++] = byte;
    if (g_response_len == 2U) {
        if ((g_response[1] != LASER_MODBUS_FUNC_READ_HOLDING) &&
            (g_response[1] != LASER_MODBUS_FUNC_EXCEPTION)) {
            g_status.protocol_error_count++;
            laser_response_restart(byte);
        }
        return;
    }

    if ((g_response[1] == LASER_MODBUS_FUNC_READ_HOLDING) &&
        (g_response_len == 3U) &&
        (g_response[2] != LASER_MODBUS_RESPONSE_BYTES)) {
        g_status.protocol_error_count++;
        laser_response_restart(byte);
        return;
    }

    if (g_response[1] == LASER_MODBUS_FUNC_READ_HOLDING) {
        expected_len = LASER_MODBUS_NORMAL_FRAME_SIZE;
    } else if (g_response[1] == LASER_MODBUS_FUNC_EXCEPTION) {
        expected_len = LASER_MODBUS_EXCEPTION_FRAME_SIZE;
    }

    if ((expected_len != 0U) && (g_response_len == expected_len)) {
        laser_accept_response(expected_len);
    }
}

static void laser_start_request(uint32_t now_ms)
{
    HAL_StatusTypeDef rc;

    g_response_len = 0U;
    g_state = LASER_RANGE_STATE_TX_REQUEST;
    g_state_started_ms = now_ms;
    g_next_request_ms = now_ms + LASER_POLL_PERIOD_MS;
    rc = HAL_UART_Transmit_IT(&huart1,
                              (uint8_t *)g_modbus_request,
                              (uint16_t)sizeof(g_modbus_request));
    if (rc == HAL_OK) {
        g_status.request_count++;
        return;
    }

    g_status.tx_error_count++;
    laser_finish_request(now_ms, 0U);
}

void LaserRange_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    memset(g_uart_rx_chunk, 0, sizeof(g_uart_rx_chunk));
    memset((void *)g_rx_ring, 0, sizeof(g_rx_ring));
    memset(g_response, 0, sizeof(g_response));
    memset(&g_latest, 0, sizeof(g_latest));
    memset(g_sample_queue, 0, sizeof(g_sample_queue));
    memset(&g_status, 0, sizeof(g_status));

    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_recovery_pending = 0U;
    g_response_len = 0U;
    g_state = LASER_RANGE_STATE_IDLE;
    g_state_started_ms = now_ms;
    g_next_request_ms = now_ms;
    g_latest_valid = 0U;
    g_sequence = 0U;
    g_sample_head = 0U;
    g_sample_tail = 0U;
    g_sample_count = 0U;
    g_status.address = LASER_MODBUS_ADDRESS;
    (void)laser_restart_receive();
}

void LaserRange_OnUartRxEvent(uint16_t size)
{
    uint16_t i;

    if (size > LASER_UART_RX_CHUNK_SIZE) {
        size = LASER_UART_RX_CHUNK_SIZE;
    }

    for (i = 0U; i < size; i++) {
        uint16_t next = (uint16_t)((g_rx_head + 1U) % LASER_RX_RING_SIZE);
        if (next == g_rx_tail) {
            g_status.protocol_error_count++;
            break;
        }
        g_rx_ring[g_rx_head] = g_uart_rx_chunk[i];
        g_rx_head = next;
    }

    (void)laser_restart_receive();
}

void LaserRange_OnUartTxComplete(void)
{
    if (g_state == LASER_RANGE_STATE_TX_REQUEST) {
        g_state = LASER_RANGE_STATE_WAIT_RESPONSE;
        g_state_started_ms = HAL_GetTick();
    }
}

void LaserRange_OnUartError(void)
{
    g_status.uart_error_count++;
    g_recovery_pending = 1U;
}

void LaserRange_Service(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (g_recovery_pending != 0U) {
        (void)HAL_UART_AbortTransmit(&huart1);
        (void)HAL_UART_AbortReceive(&huart1);
        g_recovery_pending = 0U;
        g_response_len = 0U;
        g_rx_head = 0U;
        g_rx_tail = 0U;
        g_state = LASER_RANGE_STATE_IDLE;
        g_state_started_ms = now_ms;
        g_next_request_ms = now_ms + LASER_POLL_PERIOD_MS;
        (void)laser_restart_receive();
    }

    while (g_rx_tail != g_rx_head) {
        uint8_t byte = g_rx_ring[g_rx_tail];
        g_rx_tail = (uint16_t)((g_rx_tail + 1U) % LASER_RX_RING_SIZE);
        laser_parse_byte(byte);
    }

    now_ms = HAL_GetTick();
    if (((g_state == LASER_RANGE_STATE_TX_REQUEST) ||
         (g_state == LASER_RANGE_STATE_WAIT_RESPONSE)) &&
        ((uint32_t)(now_ms - g_state_started_ms) >= LASER_RESPONSE_TIMEOUT_MS)) {
        if (g_state == LASER_RANGE_STATE_TX_REQUEST) {
            (void)HAL_UART_AbortTransmit(&huart1);
            g_status.tx_error_count++;
        } else {
            g_status.timeout_count++;
        }
        laser_finish_request(now_ms, 0U);
    }

    if ((g_state == LASER_RANGE_STATE_IDLE) &&
        ((int32_t)(now_ms - g_next_request_ms) >= 0)) {
        laser_start_request(now_ms);
    }
}

bool LaserRange_GetLatest(LaserRangeSample_t *out)
{
    if ((out == 0) || (g_latest_valid == 0U)) {
        return false;
    }
    *out = g_latest;
    return true;
}

bool LaserRange_PollSample(LaserRangeSample_t *out)
{
    if ((out == 0) || (g_sample_count == 0U)) {
        return false;
    }

    *out = g_sample_queue[g_sample_head];
    g_sample_head = (uint8_t)((g_sample_head + 1U) % LASER_SAMPLE_QUEUE_DEPTH);
    g_sample_count--;
    return true;
}

void LaserRange_FlushSamples(void)
{
    g_sample_head = 0U;
    g_sample_tail = 0U;
    g_sample_count = 0U;
}

bool LaserRange_IsOnline(uint32_t timeout_ms)
{
    if ((g_latest_valid == 0U) || (timeout_ms == 0U)) {
        return false;
    }
    return ((uint32_t)(HAL_GetTick() - g_latest.timestamp_ms) <= timeout_ms);
}

bool LaserRange_IsCommunicationOnline(uint32_t timeout_ms)
{
    if ((g_status.last_response_ms == 0U) || (timeout_ms == 0U)) {
        return false;
    }
    return ((uint32_t)(HAL_GetTick() - g_status.last_response_ms) <= timeout_ms);
}

bool LaserRange_GetStatus(LaserRangeStatus_t *out)
{
    if (out == 0) {
        return false;
    }

    *out = g_status;
    out->state = g_state;
    out->communication_online = LaserRange_IsCommunicationOnline(LASER_STATUS_ONLINE_TIMEOUT_MS);
    if (!out->communication_online) {
        out->data_valid = false;
    }
    out->online = LaserRange_IsOnline(LASER_STATUS_ONLINE_TIMEOUT_MS);
    if (g_latest_valid != 0U) {
        out->distance_mm = g_latest.distance_mm;
        out->confidence = g_latest.confidence;
    }
    return true;
}

const char *LaserRange_StateText(LaserRangeState_t state)
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
