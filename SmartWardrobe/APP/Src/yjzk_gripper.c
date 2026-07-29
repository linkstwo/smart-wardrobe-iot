#include "yjzk_gripper.h"

#include "usart.h"

/*
 * 因卓克/宇晶中控夹爪驱动：
 * 通过 UART4 使用 Modbus RTU 协议读写夹爪寄存器。
 * 上层取放衣服流程主要调用 YjzkGripper_Close() 和 YjzkGripper_Open()。
 */
#define MODBUS_FUNC_READ_HOLDING 0x03U
#define MODBUS_FUNC_WRITE_SINGLE 0x06U
#define MODBUS_EXCEPTION_MASK 0x80U
#define YJZK_UART_TIMEOUT_MS 120U
#define YJZK_WRITE_TIMEOUT_MS 300U

static uint16_t yjzk_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    /* Modbus RTU CRC16，低字节先发送。 */
    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1U);
            }
        }
    }

    return crc;
}

static void yjzk_append_crc(uint8_t *frame, uint16_t len_without_crc)
{
    uint16_t crc = yjzk_crc16(frame, len_without_crc);

    /* 在帧尾追加 CRC_L 和 CRC_H。 */
    frame[len_without_crc] = (uint8_t)(crc & 0xFFU);
    frame[len_without_crc + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);
}

static void yjzk_drain_rx(void)
{
    uint8_t ch;
    uint8_t guard = 0U;

    /* 每次发送新命令前清掉 UART4 中残留字节，避免把旧响应当成新响应解析。 */
    while ((guard < 32U) && (HAL_UART_Receive(&huart4, &ch, 1U, 1U) == HAL_OK)) {
        guard++;
    }
}

void YjzkGripper_Init(void)
{
    /* 初始化阶段只做接收缓存清理，寄存器动作由上层命令触发。 */
    yjzk_drain_rx();
}

int YjzkGripper_ReadReg(uint8_t addr, uint16_t reg, uint16_t *out_value)
{
    uint8_t tx[8];
    uint8_t rx[7];
    uint16_t crc_rx;
    uint16_t crc_calc;

    /*
     * 读单个保持寄存器：
     * 请求：addr 0x03 reg_hi reg_lo 0x00 0x01 CRC_L CRC_H
     * 正常响应：addr 0x03 0x02 data_hi data_lo CRC_L CRC_H
     */
    if ((addr == 0U) || (out_value == 0)) {
        return YJZK_GRIPPER_ERR_ARG;
    }

    tx[0] = addr;
    tx[1] = MODBUS_FUNC_READ_HOLDING;
    tx[2] = (uint8_t)((reg >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(reg & 0xFFU);
    tx[4] = 0x00U;
    tx[5] = 0x01U;
    yjzk_append_crc(tx, 6U);

    yjzk_drain_rx();

    if (HAL_UART_Transmit(&huart4, tx, sizeof(tx), YJZK_UART_TIMEOUT_MS) != HAL_OK) {
        return YJZK_GRIPPER_ERR_TX;
    }

    if (HAL_UART_Receive(&huart4, rx, 3U, YJZK_UART_TIMEOUT_MS) != HAL_OK) {
        return YJZK_GRIPPER_ERR_RX;
    }

    if (rx[0] != addr) {
        return YJZK_GRIPPER_ERR_FRAME;
    }
    /* 功能码最高位为 1 表示 Modbus 异常响应。 */
    if ((rx[1] & MODBUS_EXCEPTION_MASK) != 0U) {
        if (HAL_UART_Receive(&huart4, &rx[3], 2U, YJZK_UART_TIMEOUT_MS) != HAL_OK) {
            return YJZK_GRIPPER_ERR_RX;
        }
        crc_rx = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8U);
        crc_calc = yjzk_crc16(rx, 3U);
        if (crc_rx != crc_calc) {
            return YJZK_GRIPPER_ERR_CRC;
        }
        return YJZK_GRIPPER_ERR_EXCEPTION;
    }
    if ((rx[1] != MODBUS_FUNC_READ_HOLDING) || (rx[2] != 0x02U)) {
        return YJZK_GRIPPER_ERR_FRAME;
    }

    if (HAL_UART_Receive(&huart4, &rx[3], 4U, YJZK_UART_TIMEOUT_MS) != HAL_OK) {
        return YJZK_GRIPPER_ERR_RX;
    }

    crc_rx = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8U);
    crc_calc = yjzk_crc16(rx, 5U);
    if (crc_rx != crc_calc) {
        return YJZK_GRIPPER_ERR_CRC;
    }

    *out_value = (uint16_t)(((uint16_t)rx[3] << 8U) | rx[4]);
    return 0;
}

int YjzkGripper_WriteReg(uint8_t addr, uint16_t reg, uint16_t value)
{
    uint8_t tx[8];
    uint8_t rx[8];
    uint16_t crc_rx;
    uint16_t crc_calc;
    uint16_t i;

    /*
     * 写单个寄存器：
     * 请求和正常响应格式相同，响应会回显寄存器地址和值。
     */
    if (addr == 0U) {
        return YJZK_GRIPPER_ERR_ARG;
    }

    tx[0] = addr;
    tx[1] = MODBUS_FUNC_WRITE_SINGLE;
    tx[2] = (uint8_t)((reg >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(reg & 0xFFU);
    tx[4] = (uint8_t)((value >> 8U) & 0xFFU);
    tx[5] = (uint8_t)(value & 0xFFU);
    yjzk_append_crc(tx, 6U);

    yjzk_drain_rx();

    if (HAL_UART_Transmit(&huart4, tx, sizeof(tx), YJZK_UART_TIMEOUT_MS) != HAL_OK) {
        return YJZK_GRIPPER_ERR_TX;
    }

    if (HAL_UART_Receive(&huart4, rx, sizeof(rx), YJZK_WRITE_TIMEOUT_MS) != HAL_OK) {
        return YJZK_GRIPPER_ERR_RX;
    }

    crc_rx = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8U);
    crc_calc = yjzk_crc16(rx, 6U);
    if (crc_rx != crc_calc) {
        return YJZK_GRIPPER_ERR_CRC;
    }
    if (rx[0] != addr) {
        return YJZK_GRIPPER_ERR_FRAME;
    }
    if ((rx[1] & MODBUS_EXCEPTION_MASK) != 0U) {
        return YJZK_GRIPPER_ERR_EXCEPTION;
    }
    if (rx[1] != MODBUS_FUNC_WRITE_SINGLE) {
        return YJZK_GRIPPER_ERR_FRAME;
    }
    /* 校验响应是否完整回显了本次写入的寄存器和值。 */
    for (i = 2U; i < 6U; i++) {
        if (rx[i] != tx[i]) {
            return YJZK_GRIPPER_ERR_FRAME;
        }
    }

    return 0;
}

int YjzkGripper_ReadStatus(YjzkGripperReg_t *out)
{
    uint16_t value;
    int rc;

    /* 读取默认夹爪地址的状态寄存器。 */
    if (out == 0) {
        return YJZK_GRIPPER_ERR_ARG;
    }

    rc = YjzkGripper_ReadReg(YJZK_GRIPPER_DEFAULT_ADDR, YJZK_GRIPPER_STATUS_REG, &value);
    if (rc != 0) {
        return rc;
    }

    out->addr = YJZK_GRIPPER_DEFAULT_ADDR;
    out->reg = YJZK_GRIPPER_STATUS_REG;
    out->value = value;
    return 0;
}

int YjzkGripper_Close(void)
{
    /* 向默认动作寄存器写入闭合值，执行夹爪夹紧。 */
    return YjzkGripper_WriteReg(YJZK_GRIPPER_DEFAULT_ADDR,
                                YJZK_GRIPPER_DEFAULT_ACTION_REG,
                                YJZK_GRIPPER_DEFAULT_CLOSE_VALUE);
}

int YjzkGripper_Open(void)
{
    /* 向默认动作寄存器写入打开值，执行夹爪松开。 */
    return YjzkGripper_WriteReg(YJZK_GRIPPER_DEFAULT_ADDR,
                                YJZK_GRIPPER_DEFAULT_ACTION_REG,
                                YJZK_GRIPPER_DEFAULT_OPEN_VALUE);
}

const char *YjzkGripper_ErrorText(int err)
{
    /* 错误码转文本，方便串口响应和调试输出。 */
    switch (err) {
        case 0: return "OK";
        case YJZK_GRIPPER_ERR_ARG: return "ARG";
        case YJZK_GRIPPER_ERR_TX: return "TX";
        case YJZK_GRIPPER_ERR_RX: return "RX_TIMEOUT";
        case YJZK_GRIPPER_ERR_FRAME: return "FRAME";
        case YJZK_GRIPPER_ERR_CRC: return "CRC";
        case YJZK_GRIPPER_ERR_EXCEPTION: return "EXCEPTION";
        default: return "UNKNOWN";
    }
}
