#ifndef LASER_RANGE_H
#define LASER_RANGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t firmware_version;
    uint16_t distance_mm;
    uint8_t confidence;
    /* False marks a normal Modbus response with no usable target distance. */
    bool distance_valid;
    uint32_t timestamp_ms;
    uint32_t sequence;
} LaserRangeSample_t;

typedef enum {
    LASER_RANGE_STATE_IDLE = 0,
    LASER_RANGE_STATE_TX_REQUEST,
    LASER_RANGE_STATE_WAIT_RESPONSE
} LaserRangeState_t;

typedef struct {
    uint8_t address;
    LaserRangeState_t state;
    uint32_t request_count;
    uint32_t response_count;
    uint32_t timeout_count;
    uint32_t crc_error_count;
    uint32_t protocol_error_count;
    uint32_t exception_count;
    uint32_t rejected_count;
    uint32_t distance_reject_count;
    uint32_t confidence_reject_count;
    uint32_t uart_error_count;
    uint32_t tx_error_count;
    uint32_t last_response_ms;
    bool communication_online;
    bool data_valid;
    bool online;
    uint16_t raw_distance_mm;
    uint16_t raw_confidence;
    uint16_t distance_mm;
    uint8_t confidence;
} LaserRangeStatus_t;

void LaserRange_Init(void);
void LaserRange_Service(void);
void LaserRange_OnUartRxEvent(uint16_t size);
void LaserRange_OnUartTxComplete(void);
void LaserRange_OnUartError(void);
bool LaserRange_GetLatest(LaserRangeSample_t *out);
/* Returns every normal Modbus response, including distance_valid=false background samples. */
bool LaserRange_PollSample(LaserRangeSample_t *out);
void LaserRange_FlushSamples(void);
bool LaserRange_IsOnline(uint32_t timeout_ms);
bool LaserRange_IsCommunicationOnline(uint32_t timeout_ms);
bool LaserRange_GetStatus(LaserRangeStatus_t *out);
const char *LaserRange_StateText(LaserRangeState_t state);

#ifdef __cplusplus
}
#endif

#endif
