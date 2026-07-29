#include "main.h"

#include <stdio.h>
#include <string.h>

#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "cmd_dispatch.h"
#include "cmd_parser.h"
#include "axis_coord.h"
#include "axis_limit.h"
#include "axis_planner.h"
#include "encoder_ctrl.h"
#include "electromagnet.h"
#include "heartbeat.h"
#include "laser_range.h"
#include "laser_range2.h"
#include "motor_ctrl.h"
#include "rs485_abs_encoder.h"
#include "telemetry_curve.h"
#include "uart_console.h"

/*
 * 主程序入口：
 * 负责初始化所有外设和业务模块，并在主循环中处理三类任务：
 * 1. 1ms 周期任务：电机控制、编码器、坐标、规划器、取放衣状态机；
 * 2. ESP32 二进制帧：来自阿里云的存衣/取衣/回零/状态查询命令；
 * 3. 串口文本命令：调试用的 M1/AXIS/SYS 等 ASCII 命令。
 */
#define CMD_RESP_QUEUE_RETRY_MAX 5U

/* ESP32 -> STM32 的衣柜业务命令字。 */
#define WARDROBE_CMD_STORE 0x10U
#define WARDROBE_CMD_TAKE 0x11U
#define WARDROBE_CMD_RESET_HOME 0x12U
#define WARDROBE_CMD_READ_STATUS 0x20U
/* STM32 -> ESP32 的衣柜业务响应命令字。 */
#define WARDROBE_RESP_SUCCESS 0x90U
#define WARDROBE_RESP_FAULT 0x91U
#define WARDROBE_RESP_STATUS 0xA0U
/* 衣柜业务故障码。 */
#define WARDROBE_FAULT_COMM_TIMEOUT 0x05U
#define WARDROBE_FAULT_UNKNOWN 0xFFU
/* 云端触发回零时使用的默认转速。 */
#define WARDROBE_HOME_RPM 60.0f
/* 上电自动回零：M1/M3/M4 执行 ALL HOME；M2 由 cmd_dispatch.c 的绝对值编码器 boot-home 回 DEG=0。 */
#define WARDROBE_BOOT_HOME_ENABLE 1U
#define WARDROBE_BOOT_HOME_DELAY_MS 2000U
/* 周期性状态上报间隔，默认复用心跳周期。 */
#define WARDROBE_STATUS_HEARTBEAT_MS HEARTBEAT_PERIOD_DEFAULT_MS
#define WARDROBE_STATUS_BOOT_MUTE_MS 3000U
#define WARDROBE_TICK_CATCH_UP_MAX 10U
/* 云端运行模式：处理 ESP32 下发的二进制协议帧。 */
#define WARDROBE_BINARY_PROTOCOL_ENABLE 1U
/* 每隔 WARDROBE_STATUS_HEARTBEAT_MS 主动向 ESP32 上报一次二进制状态帧。 */
#define WARDROBE_BINARY_STATUS_HEARTBEAT_ENABLE 1U
/* 云端二进制协议与调试文本共用 USART3，关闭文本输出避免污染帧数据流。 */
#define WARDROBE_TEXT_OUTPUT_ENABLE 0U
/* 云端 position 直接作为第 N 块激光定位板编号，取衣和放衣均支持 1..255。 */

/* 1ms 软件调度基准。 */
static uint32_t g_last_tick_ms = 0U;
/* 文本命令接收缓冲区和响应缓冲区。 */
static char g_cmd_line[UART_CONSOLE_LINE_MAX];
static char g_cmd_resp[UART_CONSOLE_TX_MSG_MAX];
/* USART3 收到的二进制帧缓存。 */
static UartConsoleFrame_t g_uart_frame;
/* 上电自动 ALL HOME 状态，独立于云端 pending，避免向阿里云误报一次手动回零成功。 */
static uint8_t g_wardrobe_boot_home_pending = WARDROBE_BOOT_HOME_ENABLE;
static uint32_t g_wardrobe_boot_home_ms = 0U;

void SystemClock_Config(void);

#ifdef ENCODER_TEST_ONLY

static int32_t ec3_delta(uint16_t now, uint16_t prev)
{
  return (int32_t)(int16_t)(uint16_t)(now - prev);
}

static void ec3_send_text(const char *text)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

static void ec3_send_line(const char *text)
{
  ec3_send_text(text);
  ec3_send_text("\r\n");
}

static int ec3_line_is(const char *line, const char *cmd)
{
  while ((*line != '\0') && (*cmd != '\0')) {
    char a = *line;
    char b = *cmd;
    if ((a >= 'a') && (a <= 'z')) {
      a = (char)(a - 'a' + 'A');
    }
    if ((b >= 'a') && (b <= 'z')) {
      b = (char)(b - 'a' + 'A');
    }
    if (a != b) {
      return 0;
    }
    line++;
    cmd++;
  }
  return ((*line == '\0') && (*cmd == '\0')) ? 1 : 0;
}

static uint8_t ec3_poll_line(char *line, uint16_t max_len)
{
  static uint16_t pos = 0U;
  uint8_t ch;

  if ((line == 0) || (max_len < 2U)) {
    return 0U;
  }

  while (HAL_UART_Receive(&huart3, &ch, 1U, 0U) == HAL_OK) {
    if ((ch == '\r') || (ch == '\n')) {
      if (pos > 0U) {
        while ((pos > 0U) && ((line[pos - 1U] == ' ') || (line[pos - 1U] == '\t'))) {
          pos--;
        }
        line[pos] = '\0';
        pos = 0U;
        return 1U;
      }
    } else if (pos < (uint16_t)(max_len - 1U)) {
      line[pos++] = (char)ch;
    } else {
      pos = 0U;
    }
  }

  return 0U;
}

static void ec3_format_i32(char **p, int32_t value)
{
  char tmp[12];
  uint32_t v;
  uint8_t i = 0U;

  if (value < 0) {
    *(*p)++ = '-';
    v = (uint32_t)(-value);
  } else {
    v = (uint32_t)value;
  }

  do {
    tmp[i++] = (char)('0' + (v % 10U));
    v /= 10U;
  } while (v > 0U);

  while (i > 0U) {
    *(*p)++ = tmp[--i];
  }
}

static void ec3_format_u32(char **p, uint32_t value)
{
  char tmp[10];
  uint8_t i = 0U;

  do {
    tmp[i++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value > 0U);

  while (i > 0U) {
    *(*p)++ = tmp[--i];
  }
}

static void ec3_status_response(int32_t count_total, char *resp, uint16_t resp_len)
{
  uint32_t raw = __HAL_TIM_GET_COUNTER(&htim3);
  uint8_t a = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
  int32_t deg_x100 = (count_total * 2500) / 100;
  char *p = resp;
  (void)resp_len;

  memcpy(p, "EC3 CNT=", 8U);
  p += 8U;
  ec3_format_i32(&p, count_total);
  memcpy(p, " RAW=", 5U);
  p += 5U;
  ec3_format_u32(&p, raw);
  memcpy(p, " A=", 3U);
  p += 3U;
  *p++ = (char)('0' + a);
  memcpy(p, " B=", 3U);
  p += 3U;
  *p++ = (char)('0' + b);
  memcpy(p, " DEG=", 5U);
  p += 5U;
  ec3_format_i32(&p, deg_x100 / 100);
  *p++ = '.';
  {
    int32_t frac = deg_x100 % 100;
    if (frac < 0) {
      frac = -frac;
    }
    *p++ = (char)('0' + (frac / 10));
    *p++ = (char)('0' + (frac % 10));
  }
  memcpy(p, " CPR=1440", 9U);
  p += 9U;
  *p = '\0';
}

int main(void)
{
  char line[32];
  char resp[96];
  uint32_t last_tick;
  uint16_t raw_prev;
  int32_t count_total = 0;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();

  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  raw_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  last_tick = HAL_GetTick();

  ec3_send_line("OK EC3 Encoder Test Ready");
  ec3_send_line("CMD: EC3 STATUS / EC3 CLR");

  while (1)
  {
    uint16_t raw_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t delta = ec3_delta(raw_now, raw_prev);
    raw_prev = raw_now;
    count_total += delta;

    if (ec3_poll_line(line, sizeof(line)) != 0U) {
      if (ec3_line_is(line, "EC3 STATUS")) {
        ec3_status_response(count_total, resp, sizeof(resp));
        ec3_send_line(resp);
      } else if (ec3_line_is(line, "STATUS")) {
        ec3_status_response(count_total, resp, sizeof(resp));
        ec3_send_line(resp);
      } else if (ec3_line_is(line, "EC3 CLR")) {
        count_total = 0;
        __HAL_TIM_SET_COUNTER(&htim3, 0U);
        raw_prev = 0U;
        ec3_send_line("OK EC3 CLR");
      } else if (ec3_line_is(line, "CLR")) {
        count_total = 0;
        __HAL_TIM_SET_COUNTER(&htim3, 0U);
        raw_prev = 0U;
        ec3_send_line("OK EC3 CLR");
      } else if (ec3_line_is(line, "PING")) {
        ec3_send_line("OK PONG");
      } else {
        ec3_send_line("ERR BAD_CMD");
      }
    }

    if ((HAL_GetTick() - last_tick) >= 1000U) {
      last_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      ec3_status_response(count_total, resp, sizeof(resp));
      ec3_send_line(resp);
    }
  }
}

#else

#define EX_KEY_DEBOUNCE_MS 20U
#define EX_KEY_COUNT 4U

typedef struct {
  /* EX_KEY 输入 GPIO 以及触发/解除时输出的提示文本。 */
  GPIO_TypeDef *port;
  uint16_t pin;
  const char *blocked_text;
  const char *clear_text;
} ExKeyConfig_t;

typedef struct {
  /* stable_blocked 是消抖后的稳定状态，last_sample 是最近一次采样状态。 */
  uint8_t stable_blocked;
  uint8_t last_sample;
  uint32_t changed_ms;
} ExKeyState_t;

static const ExKeyConfig_t g_ex_key_cfg[EX_KEY_COUNT] = {
  {EX_KEY1_GPIO_Port, EX_KEY1_Pin, "提示：EX_KEY1检测到物体遮挡", "提示：EX_KEY1遮挡解除"},
  {EX_KEY2_GPIO_Port, EX_KEY2_Pin, "提示：EX_KEY2检测到物体遮挡", "提示：EX_KEY2遮挡解除"},
  {EX_KEY3_GPIO_Port, EX_KEY3_Pin, "提示：EX_KEY3检测到物体遮挡", "提示：EX_KEY3遮挡解除"},
  {EX_KEY4_GPIO_Port, EX_KEY4_Pin, "提示：EX_KEY4检测到物体遮挡", "提示：EX_KEY4遮挡解除"}
};

static ExKeyState_t g_ex_key_state[EX_KEY_COUNT];

typedef struct {
  /* 云端取/放衣命令挂起状态，用于等待 CmdDispatch 的取/放衣完成事件。 */
  uint8_t active;
  uint8_t action;
  uint8_t position;
  uint8_t motion_slot;
} WardrobePendingCmd_t;

static WardrobePendingCmd_t g_wardrobe_pending;
static uint16_t g_wardrobe_clothing_count = 0U;
static uint32_t g_wardrobe_status_last_tx_ms = 0U;
static uint8_t g_wardrobe_status_boot_mute_done = 0U;

typedef enum {
  WARDROBE_USART3_MODE_BINARY = 0,
  WARDROBE_USART3_MODE_TO_DEBUG,
  WARDROBE_USART3_MODE_DEBUG,
  WARDROBE_USART3_MODE_TO_BINARY
} WardrobeUsart3Mode_t;

typedef enum {
  WARDROBE_DEBUG_REQUEST_NONE = 0,
  WARDROBE_DEBUG_REQUEST_ENABLE,
  WARDROBE_DEBUG_REQUEST_REPORT_OFF,
  WARDROBE_DEBUG_REQUEST_REPORT_BUSY
} WardrobeDebugRequest_t;

static WardrobeUsart3Mode_t g_wardrobe_usart3_mode = WARDROBE_USART3_MODE_BINARY;
static WardrobeDebugRequest_t g_wardrobe_debug_request = WARDROBE_DEBUG_REQUEST_NONE;

static bool Wardrobe_QueueTextResponse(const char *text)
{
  uint32_t retry = CMD_RESP_QUEUE_RETRY_MAX;

  while (retry > 0U)
  {
    if (UartConsole_QueueLine(text))
    {
      return true;
    }
    UartConsole_Tick();
    retry--;
  }

  return false;
}

static void Wardrobe_StartDebugTransition(WardrobeDebugRequest_t request)
{
  g_wardrobe_debug_request = request;
  g_wardrobe_usart3_mode = WARDROBE_USART3_MODE_TO_DEBUG;
}

static uint8_t Wardrobe_HandleDebugCommand(const ParsedCmd_t *cmd)
{
  if ((cmd == 0) || (cmd->target != CMD_TARGET_SYSTEM))
  {
    return 0U;
  }
  if ((cmd->action != CMD_ACTION_DEBUG_ON) &&
      (cmd->action != CMD_ACTION_DEBUG_OFF) &&
      (cmd->action != CMD_ACTION_DEBUG_STATUS))
  {
    return 0U;
  }

  if (cmd->action == CMD_ACTION_DEBUG_ON)
  {
    if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_DEBUG)
    {
      (void)Wardrobe_QueueTextResponse("OK DEBUG ON BINARY=PAUSED");
    }
    else if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_BINARY)
    {
      Wardrobe_StartDebugTransition((g_wardrobe_pending.active != 0U) ?
                                    WARDROBE_DEBUG_REQUEST_REPORT_BUSY :
                                    WARDROBE_DEBUG_REQUEST_ENABLE);
    }
    return 1U;
  }

  if (cmd->action == CMD_ACTION_DEBUG_OFF)
  {
    if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_DEBUG)
    {
      if (Wardrobe_QueueTextResponse("OK DEBUG OFF BINARY=RESUMED"))
      {
        UartConsole_SetTextOutputEnabled(false);
        g_wardrobe_usart3_mode = WARDROBE_USART3_MODE_TO_BINARY;
      }
    }
    else if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_BINARY)
    {
      Wardrobe_StartDebugTransition(WARDROBE_DEBUG_REQUEST_REPORT_OFF);
    }
    return 1U;
  }

  if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_DEBUG)
  {
    (void)Wardrobe_QueueTextResponse("OK DEBUG ON BINARY=PAUSED");
  }
  else if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_BINARY)
  {
    Wardrobe_StartDebugTransition(WARDROBE_DEBUG_REQUEST_REPORT_OFF);
  }
  return 1U;
}

static void Wardrobe_DebugModeService(void)
{
  const char *reply = 0;
  uint8_t enter_debug = 0U;

  if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_TO_DEBUG)
  {
    if (!UartConsole_IsTxIdle())
    {
      return;
    }

    UartConsole_SetTextOutputEnabled(true);
    switch (g_wardrobe_debug_request)
    {
      case WARDROBE_DEBUG_REQUEST_ENABLE:
        reply = "OK DEBUG ON BINARY=PAUSED";
        enter_debug = 1U;
        break;

      case WARDROBE_DEBUG_REQUEST_REPORT_BUSY:
        reply = "ERR DEBUG BUSY CLOUD_TASK";
        break;

      case WARDROBE_DEBUG_REQUEST_REPORT_OFF:
      default:
        reply = "OK DEBUG OFF BINARY=RESUMED";
        break;
    }

    if (!Wardrobe_QueueTextResponse(reply))
    {
      return;
    }

    g_wardrobe_debug_request = WARDROBE_DEBUG_REQUEST_NONE;
    if (enter_debug != 0U)
    {
      g_wardrobe_usart3_mode = WARDROBE_USART3_MODE_DEBUG;
    }
    else
    {
      UartConsole_SetTextOutputEnabled(false);
      g_wardrobe_usart3_mode = WARDROBE_USART3_MODE_TO_BINARY;
    }
    return;
  }

  if ((g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_TO_BINARY) &&
      UartConsole_IsTxIdle())
  {
    UartConsole_SetTextOutputEnabled(false);
    g_wardrobe_usart3_mode = WARDROBE_USART3_MODE_BINARY;
    g_wardrobe_status_last_tx_ms = HAL_GetTick();
    g_wardrobe_status_boot_mute_done = 1U;
  }
}

static uint8_t ExKey_IsBlocked(uint32_t index)
{
  /* EX_KEY 为低电平时表示被遮挡/触发。 */
  if (index >= EX_KEY_COUNT) {
    return 0U;
  }

  return (HAL_GPIO_ReadPin(g_ex_key_cfg[index].port, g_ex_key_cfg[index].pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void ExKey_InitState(void)
{
  uint32_t i;
  uint32_t now_ms = HAL_GetTick();

  /* 启动时把当前传感器状态作为稳定初始状态，避免上电立即误报一次变化。 */
  for (i = 0U; i < EX_KEY_COUNT; i++)
  {
    uint8_t blocked = ExKey_IsBlocked(i);
    g_ex_key_state[i].stable_blocked = blocked;
    g_ex_key_state[i].last_sample = blocked;
    g_ex_key_state[i].changed_ms = now_ms;
  }
}

static void ExKey_Tick(uint32_t now_ms)
{
  uint32_t i;

  /* EX_KEY 软件消抖，状态稳定超过 EX_KEY_DEBOUNCE_MS 后才输出提示。 */
  for (i = 0U; i < EX_KEY_COUNT; i++)
  {
    uint8_t blocked = ExKey_IsBlocked(i);

    if (blocked != g_ex_key_state[i].last_sample)
    {
      g_ex_key_state[i].last_sample = blocked;
      g_ex_key_state[i].changed_ms = now_ms;
      continue;
    }

    if ((blocked != g_ex_key_state[i].stable_blocked) &&
        ((now_ms - g_ex_key_state[i].changed_ms) >= EX_KEY_DEBOUNCE_MS))
    {
      g_ex_key_state[i].stable_blocked = blocked;
      if (blocked != 0U)
      {
        (void)UartConsole_QueueLineDropOldest(g_ex_key_cfg[i].blocked_text, 0);
      }
      else
      {
        (void)UartConsole_QueueLineDropOldest(g_ex_key_cfg[i].clear_text, 0);
      }
    }
  }
}

static uint8_t Wardrobe_CalcCrc(uint8_t cmd, uint8_t len, const uint8_t *data)
{
  /*
   * 自定义衣柜二进制帧 CRC：
   * CRC = CMD ^ LEN ^ DATA[0] ^ DATA[1] ...
   * 需要和 ESP32 的 UART 帧解析保持一致。
   */
  uint8_t crc = (uint8_t)(cmd ^ len);
  uint8_t i;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
  }

  return crc;
}

static bool Wardrobe_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
#if WARDROBE_BINARY_PROTOCOL_ENABLE
  uint8_t frame[UART_CONSOLE_FRAME_MAX];
  uint16_t frame_len;

  if (g_wardrobe_usart3_mode != WARDROBE_USART3_MODE_BINARY)
  {
    return false;
  }

  /* 按 A5 CMD LEN DATA CRC 5A 组帧，通过 USART3 发给 ESP32。 */
  if (len > UART_CONSOLE_FRAME_DATA_MAX)
  {
    return false;
  }
  if ((data == 0) && (len > 0U))
  {
    return false;
  }

  frame[0] = UART_CONSOLE_FRAME_SOF;
  frame[1] = cmd;
  frame[2] = len;
  if ((data != 0) && (len > 0U))
  {
    memcpy(&frame[3], data, len);
  }
  frame[3U + len] = Wardrobe_CalcCrc(cmd, len, data);
  frame[4U + len] = UART_CONSOLE_FRAME_EOF;
  frame_len = (uint16_t)(5U + len);

  return UartConsole_QueueBytes(frame, frame_len);
#else
  /* 纯串口调试模式：屏蔽所有 A5 ... 5A 二进制回复，避免 ASCII 串口窗口刷屏。 */
  (void)cmd;
  (void)data;
  (void)len;
  return true;
#endif
}

static void Wardrobe_SendSuccess(uint8_t position)
{
  /* 成功响应载荷为 1 字节 position。 */
  uint8_t data[1] = { position };

  (void)Wardrobe_SendFrame(WARDROBE_RESP_SUCCESS, data, 1U);
}

static void Wardrobe_SendFault(uint8_t fault_code, uint8_t position)
{
  /* 故障响应载荷为 fault_code + position。 */
  uint8_t data[2] = { fault_code, position };

  (void)Wardrobe_SendFrame(WARDROBE_RESP_FAULT, data, 2U);
}

static void Wardrobe_SendStatus(void)
{
  /*
   * 状态响应载荷共 6 字节：
   * [0..1] 当前衣物数量；
   * [2..3] 运行天数；
   * [4] 小时；
   * [5] 分钟。
   */
  uint32_t uptime_s = HAL_GetTick() / 1000U;
  uint16_t days = (uint16_t)(uptime_s / 86400U);
  uint8_t hours;
  uint8_t minutes;
  uint8_t data[6];

  uptime_s %= 86400U;
  hours = (uint8_t)(uptime_s / 3600U);
  uptime_s %= 3600U;
  minutes = (uint8_t)(uptime_s / 60U);

  data[0] = (uint8_t)(g_wardrobe_clothing_count >> 8);
  data[1] = (uint8_t)(g_wardrobe_clothing_count & 0xFFU);
  data[2] = (uint8_t)(days >> 8);
  data[3] = (uint8_t)(days & 0xFFU);
  data[4] = hours;
  data[5] = minutes;

  (void)Wardrobe_SendFrame(WARDROBE_RESP_STATUS, data, 6U);
}

/* 云端 position 直接作为内部运动 slot，即越过门槛后的第 N 块有效定位板。 */
static void Wardrobe_StatusHeartbeat_Tick1ms(void)
{
#if WARDROBE_BINARY_STATUS_HEARTBEAT_ENABLE
  uint32_t now_ms = HAL_GetTick();

  if (g_wardrobe_status_boot_mute_done == 0U)
  {
    if (now_ms < WARDROBE_STATUS_BOOT_MUTE_MS)
    {
      g_wardrobe_status_last_tx_ms = now_ms;
      return;
    }
    g_wardrobe_status_boot_mute_done = 1U;
    g_wardrobe_status_last_tx_ms = now_ms;
    return;
  }

  /* 周期性发送衣柜状态给 ESP32，再由 ESP32 转成阿里云属性/用户消息。 */
  if ((uint32_t)(now_ms - g_wardrobe_status_last_tx_ms) >= WARDROBE_STATUS_HEARTBEAT_MS)
  {
    g_wardrobe_status_last_tx_ms = now_ms;
    Wardrobe_SendStatus();
  }
#else
  /* 调试模式：不主动发送二进制状态帧，避免普通串口助手被 11 Bytes 状态帧刷屏。 */
  g_wardrobe_status_last_tx_ms = HAL_GetTick();
  g_wardrobe_status_boot_mute_done = 0U;
#endif
}

static void Wardrobe_StartClothAction(uint8_t action, uint8_t position)
{
  ParsedCmd_t cmd;
  uint8_t motion_slot;

  /*
   * 启动存衣/取衣业务：
   * 1. 若已有云端任务挂起，则返回忙/超时故障；
   * 2. 将 position 直接作为内部 motion_slot；
   * 3. 构造 AXIS PLACE_CLOTH 或 AXIS TAKE_CLOTH 命令交给 CmdDispatch 执行；
   * 4. 记录 pending，等待取放衣状态机完成事件。
   */
  if (g_wardrobe_pending.active != 0U)
  {
    Wardrobe_SendFault(WARDROBE_FAULT_COMM_TIMEOUT, position);
    return;
  }

  if (position == 0U)
  {
    Wardrobe_SendFault(WARDROBE_FAULT_UNKNOWN, position);
    return;
  }

  /* Store and take both use cloud position as the Nth laser marker. */
  motion_slot = position;

  memset(&cmd, 0, sizeof(cmd));
  cmd.target = CMD_TARGET_AXIS;
  cmd.motor_id = MOTOR_ID_INVALID;
  cmd.action = (action == WARDROBE_CMD_STORE) ? CMD_ACTION_PLACE_CLOTH : CMD_ACTION_TAKE_CLOTH;
  cmd.args.has_slot = true;
  cmd.args.slot = motion_slot;

  if (CmdDispatch_Execute(&cmd, g_cmd_resp, sizeof(g_cmd_resp)) != CMD_RESULT_OK)
  {
    Wardrobe_SendFault(WARDROBE_FAULT_UNKNOWN, position);
    return;
  }

  g_wardrobe_boot_home_pending = 0U;
  g_wardrobe_pending.active = 1U;
  g_wardrobe_pending.action = action;
  g_wardrobe_pending.position = position;
  g_wardrobe_pending.motion_slot = motion_slot;
}

static void Wardrobe_StartResetHome(void)
{
  ParsedCmd_t cmd;

  /* 云端回零命令会转换成 ALL HOME，使用 WARDROBE_HOME_RPM 默认转速。 */
  if (g_wardrobe_pending.active != 0U)
  {
    Wardrobe_SendFault(WARDROBE_FAULT_COMM_TIMEOUT, 0xFFU);
    return;
  }

  memset(&cmd, 0, sizeof(cmd));
  cmd.target = CMD_TARGET_ALL;
  cmd.motor_id = MOTOR_ID_INVALID;
  cmd.action = CMD_ACTION_REV_EXKEY;
  cmd.args.has_rpm = true;
  cmd.args.rpm = WARDROBE_HOME_RPM;

  if (CmdDispatch_Execute(&cmd, g_cmd_resp, sizeof(g_cmd_resp)) != CMD_RESULT_OK)
  {
    Wardrobe_SendFault(WARDROBE_FAULT_UNKNOWN, 0xFFU);
    return;
  }

  g_wardrobe_boot_home_pending = 0U;
  Wardrobe_SendSuccess(0U);
}

static void Wardrobe_BootHome_Tick1ms(void)
{
  ParsedCmd_t cmd;

  /*
   * 上电自动回零只负责启动一次 ALL HOME，不向云端返回成功帧。
   * 若用户在延时期间已经下发取/放衣或回零命令，对应命令会自行处理回零，这里不再重复触发。
   */
  if (g_wardrobe_boot_home_pending == 0U)
  {
    return;
  }

  if (g_wardrobe_boot_home_ms < 0xFFFFFFFFUL)
  {
    g_wardrobe_boot_home_ms++;
  }

  if (g_wardrobe_boot_home_ms < WARDROBE_BOOT_HOME_DELAY_MS)
  {
    return;
  }

  if (g_wardrobe_pending.active != 0U)
  {
    return;
  }

  memset(&cmd, 0, sizeof(cmd));
  cmd.target = CMD_TARGET_ALL;
  cmd.motor_id = MOTOR_ID_INVALID;
  cmd.action = CMD_ACTION_REV_EXKEY;
  cmd.args.has_rpm = true;
  cmd.args.rpm = WARDROBE_HOME_RPM;

  /* 一次性消费上电回零请求：无论启动成功或失败，都不再周期重试。 */
  g_wardrobe_boot_home_pending = 0U;
  if (CmdDispatch_Execute(&cmd, g_cmd_resp, sizeof(g_cmd_resp)) == CMD_RESULT_OK)
  {
    CmdDispatch_RequestM2BootHome();
    (void)UartConsole_QueueLineDropOldest(g_cmd_resp, 0);
    (void)UartConsole_QueueLineDropOldest("OK BOOT ALL_HOME START", 0);
  }
  else
  {
    (void)UartConsole_QueueLineDropOldest(g_cmd_resp, 0);
  }
}

static void Wardrobe_HandleFrame(const UartConsoleFrame_t *frame)
{
  /*
   * 处理 ESP32 发来的衣柜二进制帧。
   * STORE/TAKE 的载荷必须是 1 字节 position；
   * RESET_HOME/READ_STATUS 没有载荷。
   */
  if (frame == 0)
  {
    return;
  }

  switch (frame->cmd)
  {
    case WARDROBE_CMD_STORE:
    case WARDROBE_CMD_TAKE:
      if (frame->len != 1U)
      {
        Wardrobe_SendFault(WARDROBE_FAULT_UNKNOWN, 0xFFU);
        return;
      }
      Wardrobe_StartClothAction(frame->cmd, frame->data[0]);
      return;

    case WARDROBE_CMD_RESET_HOME:
      if (frame->len != 0U)
      {
        Wardrobe_SendFault(WARDROBE_FAULT_UNKNOWN, 0xFFU);
        return;
      }
      Wardrobe_StartResetHome();
      return;

    case WARDROBE_CMD_READ_STATUS:
      if (frame->len != 0U)
      {
        Wardrobe_SendFault(WARDROBE_FAULT_UNKNOWN, 0xFFU);
        return;
      }
      Wardrobe_SendStatus();
      return;

    default:
      Wardrobe_SendFault(WARDROBE_FAULT_UNKNOWN, 0xFFU);
      return;
  }
}

static void Wardrobe_HandleClothEvents(void)
{
  CmdClothEvent_t event;
  uint8_t fault_code;

  /*
   * CmdDispatch 完成取/放衣状态机后会产生事件。
   * 这里把事件转换成云端响应帧，并维护当前衣物数量。
   */
  while (CmdDispatch_PollClothEvent(&event, &fault_code))
  {
    if (g_wardrobe_pending.active == 0U)
    {
      continue;
    }

    if (((event == CMD_CLOTH_EVENT_PLACE_DONE) && (g_wardrobe_pending.action == WARDROBE_CMD_STORE)) ||
        ((event == CMD_CLOTH_EVENT_TAKE_DONE) && (g_wardrobe_pending.action == WARDROBE_CMD_TAKE)))
    {
      if (g_wardrobe_pending.action == WARDROBE_CMD_STORE)
      {
        if (g_wardrobe_clothing_count < 0xFFFFU)
        {
          g_wardrobe_clothing_count++;
        }
      }
      else if (g_wardrobe_clothing_count > 0U)
      {
        g_wardrobe_clothing_count--;
      }
      Wardrobe_SendSuccess(g_wardrobe_pending.position);
      g_wardrobe_pending.active = 0U;
      continue;
    }

    if (((event == CMD_CLOTH_EVENT_PLACE_FAULT) && (g_wardrobe_pending.action == WARDROBE_CMD_STORE)) ||
        ((event == CMD_CLOTH_EVENT_TAKE_FAULT) && (g_wardrobe_pending.action == WARDROBE_CMD_TAKE)))
    {
      Wardrobe_SendFault(fault_code, g_wardrobe_pending.position);
      g_wardrobe_pending.active = 0U;
    }
  }
}

int main(void)
{
  /* HAL 和系统时钟初始化。 */
  HAL_Init();
  SystemClock_Config();

  /* 外设初始化：GPIO、DMA、编码器定时器、串口、步进电机 PWM 定时器。 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Infrared_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();

  /* 继电器控制先建立确定的释放状态，再启动业务模块。 */
  Electromagnet_Init();

  /* 业务模块初始化，顺序上先底层控制，再坐标/限位/规划/通信。 */
  MotorCtrl_Init();
  EncoderCtrl_Init();
  AxisCoord_Init();
  AxisLimit_Init();
  AxisPlanner_Init();
  LaserRange_Init();
  LaserRange2_Init();
  Rs485AbsEncoder_Init();
  UartConsole_Init();
  UartConsole_SetTextOutputEnabled(WARDROBE_TEXT_OUTPUT_ENABLE != 0U);
  TelemetryCurve_Init();
  Heartbeat_Init();
  (void)Heartbeat_Set(false, HEARTBEAT_PERIOD_DEFAULT_MS);
  ExKey_InitState();
  memset(&g_wardrobe_pending, 0, sizeof(g_wardrobe_pending));
  g_wardrobe_status_last_tx_ms = HAL_GetTick();
  g_wardrobe_status_boot_mute_done = 0U;
  g_wardrobe_usart3_mode = WARDROBE_USART3_MODE_BINARY;
  g_wardrobe_debug_request = WARDROBE_DEBUG_REQUEST_NONE;

  /* 主循环 1ms 调度基准和启动提示。 */
  g_last_tick_ms = HAL_GetTick();
  (void)UartConsole_QueueLine("OK SmartWardrobe Ready");

  while (1)
  {
    uint32_t now_ms;
    uint32_t catch_up_count = 0U;

    /* Parse both independent ranging UARTs before advancing control tasks. */
    LaserRange_Service();
    LaserRange2_Service();
    LaserRange2_StreamService();
    now_ms = HAL_GetTick();

    /*
     * 1ms 周期任务补偿循环：
     * 如果主循环某次执行超过 1ms，会补跑缺失的 tick，减少时间漂移。
     */
    while ((g_last_tick_ms != now_ms) &&
           (catch_up_count < WARDROBE_TICK_CATCH_UP_MAX))
    {
      g_last_tick_ms++;
      catch_up_count++;
      MotorCtrl_Tick1ms();
      EncoderCtrl_Tick1ms();
      AxisCoord_Tick1ms();
      /* 运行时轴限位巡检按当前需求关闭；限位参数和查询接口仍保留。 */
      AxisPlanner_Tick1ms();
      TelemetryCurve_Tick1ms();
      Wardrobe_StatusHeartbeat_Tick1ms();
      Wardrobe_BootHome_Tick1ms();
      ExKey_Tick(g_last_tick_ms);
      CmdDispatch_SafetyTick1ms();
    }

    if (g_last_tick_ms != now_ms)
    {
      /* 长时间阻塞后丢弃过期控制 tick，避免集中回放历史任务。 */
      g_last_tick_ms = now_ms;
    }

    Rs485AbsEncoder_Service();
    Wardrobe_DebugModeService();

#if WARDROBE_BINARY_PROTOCOL_ENABLE
    /* ESP32 二进制帧，也就是云端下发的衣柜业务命令。 */
    while (UartConsole_PollFrame(&g_uart_frame))
    {
      if (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_BINARY)
      {
        Wardrobe_HandleFrame(&g_uart_frame);
      }
    }
#else
    /* 纯串口调试模式：丢弃可能残留的二进制帧，不执行云端协议命令。 */
    while (UartConsole_PollFrame(&g_uart_frame))
    {
      /* discard */
    }
#endif
    if ((g_wardrobe_pending.active == 0U) ||
        (g_wardrobe_usart3_mode == WARDROBE_USART3_MODE_BINARY))
    {
      Wardrobe_HandleClothEvents();
    }

    /* 处理普通 ASCII 文本调试命令。 */
    if (UartConsole_PollLine(g_cmd_line, sizeof(g_cmd_line)))
    {
      ParsedCmd_t parsed;
      CmdParseResult_t parse_result = CmdParser_Parse(g_cmd_line, &parsed);
      uint8_t response_pending = 1U;

      if (parse_result != CMD_PARSE_OK)
      {
        (void)snprintf(g_cmd_resp,
                       sizeof(g_cmd_resp),
                       "ERR 1%02u %s",
                       (unsigned int)parse_result,
                       CmdParser_ResultText(parse_result));
      }
      else
      {
        /* 文本命令解析成功后交给命令分发层执行。 */
        if (Wardrobe_HandleDebugCommand(&parsed) != 0U)
        {
          response_pending = 0U;
        }
        else
        {
          (void)CmdDispatch_Execute(&parsed, g_cmd_resp, sizeof(g_cmd_resp));
        }
      }

      if (response_pending != 0U)
      {
        (void)Wardrobe_QueueTextResponse(g_cmd_resp);
      }
    }

    UartConsole_Tick();
  }
}

#endif

void SystemClock_Config(void)
{
  /* 使用 HSI + PLL 配置系统主频，APB1 分频 4，APB2 分频 2。 */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* 严重初始化错误时关中断并停在死循环，便于调试定位。 */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
