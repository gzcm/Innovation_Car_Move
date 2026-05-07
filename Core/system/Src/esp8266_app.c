/*
 * esp8266_app.c — ESP8266 Wi-Fi 模块应用层
 *
 * 功能：
 *   上电后依次发送 AT 指令将 ESP8266 配置为 AP + TCP Server 模式，
 *   初始化完成后持续接收其他节点通过 TCP 发来的指令（如 "a1"/"b2"），
 *   解析后存入待消费队列，供上层在路口转弯时读取方向。
 *
 * 使用方式：
 *   App_Init()   → ESP8266_App_Init(&huartX)   初始化串口并启动 AT 序列
 *   App_Update() → ESP8266_App_Run()            驱动 AT 序列状态机
 *   路口触发时   → ESP8266_App_PopCommand()     弹出待执行指令
 *
 * AT 初始化序列：
 *   CWMODE=2（AP 模式）→ RST → CWSAP（建热点）→ CIPMUX=1 → CIPSERVER=1,8086 → CIFSR
 *   各步骤间按 g_cmd_delay_ms 等待，不解析 OK/ERROR 响应（简化处理）
 *
 * 数据帧格式（ESP8266 透传）：
 *   +IPD,<link_id>,<len>:<payload>\r\n
 *   payload 示例："a1" "b3"，对应 ESP8266_CommandState 枚举
 */

#include "esp8266_app.h"

#include <stdio.h>
#include <string.h>

/* 行缓冲区最大长度，需容纳完整 +IPD 帧 */
#define ESP8266_RX_LINE_BUF_SIZE      (96U)
/* payload 缓冲区长度，"a1"\0 只需 3 字节，留足余量 */
#define ESP8266_CMD_PAYLOAD_BUF_SIZE  (9U)

/* ---------- 串口句柄 ---------- */
static UART_HandleTypeDef *s_huart = NULL;

/* ---------- 串口接收缓冲 ---------- */
static uint8_t  g_rx_byte = 0U;                          /* 中断逐字节接收临时变量 */
static uint8_t  g_rx_line_buf[ESP8266_RX_LINE_BUF_SIZE]; /* 行缓冲，遇 \n 或满后处理 */
static uint16_t g_rx_line_len = 0U;

/* ---------- AT 指令序列 ---------- */
static uint8_t g_at_cmd1[] = "AT+CWMODE=2\r\n";                    /* 设置 AP 模式 */
static uint8_t g_at_cmd2[] = "AT+RST\r\n";                         /* 软复位，应用新模式 */
static uint8_t g_at_cmd3[] = "AT+CWSAP=\"Car\",\"12345678\",1,4\r\n"; /* 创建热点 */
static uint8_t g_at_cmd4[] = "AT+CIPMUX=1\r\n";                    /* 多连接模式 */
static uint8_t g_at_cmd5[] = "AT+CIPSERVER=1,8086\r\n";            /* 开启 TCP Server */
static uint8_t g_at_cmd6[] = "AT+CIFSR\r\n";                       /* 查询 IP（调试用）*/

static uint8_t *g_at_cmd_table[] = {
    g_at_cmd1, g_at_cmd2, g_at_cmd3, g_at_cmd4, g_at_cmd5, g_at_cmd6
};

/* 每条指令发出后需等待的时间（ms），RST 后需较长时间等模块重启 */
static uint16_t g_cmd_delay_ms[] = {200U, 5000U, 5000U, 200U, 200U, 200U};

static volatile uint8_t  g_cmd_index    = 0U;
static const uint8_t     g_cmd_total    = (uint8_t)(sizeof(g_at_cmd_table) / sizeof(g_at_cmd_table[0]));
static volatile uint8_t  g_tx_done_flag = 0U;  /* TX 完成中断置位，Run() 检测后发下一条 */
static volatile uint32_t g_last_tx_tick = 0U;  /* 上次发送完成时的 tick，用于延时计算 */
static uint8_t g_initialized   = 0U;           /* Init() 已调用标志，防止 Run() 空跑 */
static uint8_t g_init_finished = 0U;           /* AT 序列全部发完标志 */

/* ---------- IPD 解析临时缓冲 ---------- */
static char g_ipd_link_buf[2]                        = {0}; /* 连接 ID（单字符+""\0"） */
static char g_ipd_payload_buf[ESP8266_CMD_PAYLOAD_BUF_SIZE] = {0};

/* ---------- 指令状态 ---------- */
static volatile ESP8266_CommandState g_current_state  = ESP8266_STATE_IDLE; /* 最后收到的指令 */
static volatile uint8_t              g_command_pending = 0U;                /* 待消费标志 */
static volatile ESP8266_CommandState g_pending_state   = ESP8266_STATE_IDLE; /* 待消费的指令值 */

/* ================================================================
 *  内部函数
 * ================================================================ */

/* 重新启动单字节中断接收 */
static void ESP8266_StartReceive(void)
{
  (void)HAL_UART_Receive_IT(s_huart, &g_rx_byte, 1);
}

/* 清空行缓冲区，准备接收下一行 */
static void ESP8266_ResetLineBuffer(void)
{
  g_rx_line_len = 0U;
  g_rx_line_buf[0] = '\0';
}

/* 将 payload 字符串映射到指令枚举，未识别返回 IDLE */
static ESP8266_CommandState ESP8266_ParseCommandState(const char *payload)
{
  if (strcmp(payload, "a1") == 0) return ESP8266_STATE_A1;
  if (strcmp(payload, "a2") == 0) return ESP8266_STATE_A2;
  if (strcmp(payload, "a3") == 0) return ESP8266_STATE_A3;
  if (strcmp(payload, "b1") == 0) return ESP8266_STATE_B1;
  if (strcmp(payload, "b2") == 0) return ESP8266_STATE_B2;
  if (strcmp(payload, "b3") == 0) return ESP8266_STATE_B3;
  return ESP8266_STATE_IDLE;
}

/*
 * 从行缓冲中解析 +IPD 帧，提取 link_id 和 payload
 * 帧格式：+IPD,<link_id>,<len>:<payload>
 * 成功返回 1，格式错误或无数据返回 0
 */
static uint8_t ESP8266_ParseIpdFrame(const uint8_t *buffer, uint16_t length,
    char link_buf[2], char payload_buf[ESP8266_CMD_PAYLOAD_BUF_SIZE])
{
  uint16_t start_index = 0U;
  uint16_t index;
  uint16_t payload_length = 0U;
  uint16_t payload_index  = 0U;
  uint8_t  digit_count    = 0U;

  /* 在缓冲区中搜索 "+IPD," 起始标志 */
  for (start_index = 0U; (start_index + 5U) < length; start_index++)
  {
    if ((buffer[start_index]      == '+') &&
        (buffer[start_index + 1U] == 'I') &&
        (buffer[start_index + 2U] == 'P') &&
        (buffer[start_index + 3U] == 'D') &&
        (buffer[start_index + 4U] == ','))
    {
      break;
    }
  }

  if ((start_index + 5U) >= length) return 0U; /* 未找到帧头 */

  /* 解析 link_id（单个十进制数字，后跟逗号） */
  index = start_index + 5U;
  if ((index + 1U) >= length ||
      (buffer[index] < '0') ||
      (buffer[index] > '9') ||
      (buffer[index + 1U] != ','))
  {
    return 0U;
  }

  link_buf[0] = (char)buffer[index];
  link_buf[1] = '\0';

  /* 解析 payload 长度字段 */
  index += 2U;
  while ((index < length) && (buffer[index] >= '0') && (buffer[index] <= '9'))
  {
    payload_length = (uint16_t)(payload_length * 10U + (uint16_t)(buffer[index] - '0'));
    index++;
    digit_count++;
  }

  if ((digit_count == 0U) || (index >= length) || (buffer[index] != ':')) return 0U;

  /* 提取 payload，遇到行尾或超出长度时停止 */
  index++;
  while ((index < length) &&
         (payload_index < payload_length) &&
         (payload_index < (ESP8266_CMD_PAYLOAD_BUF_SIZE - 1U)) &&
         (buffer[index] != '\r') &&
         (buffer[index] != '\n') &&
         (buffer[index] != '\0'))
  {
    payload_buf[payload_index] = (char)buffer[index];
    payload_index++;
    index++;
  }

  if (payload_index == 0U) return 0U;

  payload_buf[payload_index] = '\0';
  return 1U;
}

/* ================================================================
 *  公开接口
 * ================================================================ */

/*
 * 初始化：绑定串口句柄，发送第一条 AT 指令，启动接收中断
 * 必须在 App_Init() 中调用，早于 ESP8266_App_Run()
 */
void ESP8266_App_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  ESP8266_ResetLineBuffer();
  (void)HAL_UART_Transmit_IT(s_huart, g_at_cmd_table[0], strlen((char *)g_at_cmd_table[0]));
  ESP8266_StartReceive();
  g_initialized = 1U;
}

/*
 * 主循环轮询：驱动 AT 指令序列状态机
 * 每条指令 TX 完成后等待对应延时，再发下一条；序列结束后置 g_init_finished
 */
void ESP8266_App_Run(void)
{
  if (g_initialized == 0U) return;

  if (g_tx_done_flag != 0U)
  {
    if ((HAL_GetTick() - g_last_tx_tick) > g_cmd_delay_ms[g_cmd_index])
    {
      g_tx_done_flag = 0U;
      g_cmd_index++;

      if (g_cmd_index < g_cmd_total)
      {
        (void)HAL_UART_Transmit_IT(s_huart, g_at_cmd_table[g_cmd_index],
            strlen((char *)g_at_cmd_table[g_cmd_index]));
      }
      else
      {
        g_init_finished = 1U; /* AT 序列全部发完，模块就绪 */
      }
    }
  }
}

/* 返回模块是否已完成初始化（AT 序列发送完毕） */
bool ESP8266_App_IsReady(void)
{
  return (g_init_finished != 0U);
}

/*
 * 弹出一条待消费指令（one-shot）
 * 有新指令返回 true 并写入 *out_command，否则返回 false
 * 典型用法：在路口全黑检测后调用，取得本次转弯方向
 */
bool ESP8266_App_PopCommand(ESP8266_CommandState *out_command)
{
  if ((out_command == NULL) || (g_command_pending == 0U)) return false;

  *out_command = g_pending_state;
  g_command_pending = 0U;
  return true;
}

/* 返回最后收到的指令状态（持久保留，不消费） */
ESP8266_CommandState ESP8266_App_GetState(void)
{
  return g_current_state;
}

/* 返回当前指令状态的可读字符串，用于 OLED 显示 */
const char *ESP8266_App_GetStateName(void)
{
  switch (g_current_state)
  {
    case ESP8266_STATE_A1: return "A1";
    case ESP8266_STATE_A2: return "A2";
    case ESP8266_STATE_A3: return "A3";
    case ESP8266_STATE_B1: return "B1";
    case ESP8266_STATE_B2: return "B2";
    case ESP8266_STATE_B3: return "B3";
    case ESP8266_STATE_IDLE:
    default:               return "IDLE";
  }
}

/*
 * 将指令解码为两个路口的转向方向
 * 字母（A/B）→ dir1，数字（1/2/3）→ dir2
 * A系：dir1=-1（左转）；B系：dir1=+1（右转）
 * 1=右转(+1)，2=直行(0)，3=左转(-1)
 */
void ESP8266_App_DecodeDirs(ESP8266_CommandState state, int8_t *dir1, int8_t *dir2)
{
  switch (state)
  {
    case ESP8266_STATE_A1: *dir1 = -1; *dir2 = +1; break;
    case ESP8266_STATE_A2: *dir1 = -1; *dir2 =  0; break;
    case ESP8266_STATE_A3: *dir1 = -1; *dir2 = -1; break;
    case ESP8266_STATE_B1: *dir1 = +1; *dir2 = +1; break;
    case ESP8266_STATE_B2: *dir1 = +1; *dir2 =  0; break;
    case ESP8266_STATE_B3: *dir1 = +1; *dir2 = -1; break;
    default:               *dir1 =  0; *dir2 =  0; break;
  }
}

/* ================================================================
 *  UART 中断回调（由 stm32f1xx_it.c 分发器调用）
 * ================================================================ */

/* TX 完成：记录时间戳，Run() 据此计算延时后发送下一条 */
void ESP8266_App_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (s_huart != NULL && huart->Instance == s_huart->Instance)
  {
    g_tx_done_flag = 1U;
    g_last_tx_tick = HAL_GetTick();
  }
}

/*
 * RX 完成：逐字节追加到行缓冲
 * 遇到 \n 或缓冲区满时尝试解析 +IPD 帧；解析成功则更新指令状态
 */
void ESP8266_App_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (s_huart == NULL || huart->Instance != s_huart->Instance) return;

  /* 追加字节到行缓冲，防溢出 */
  if (g_rx_line_len < (ESP8266_RX_LINE_BUF_SIZE - 1U))
  {
    g_rx_line_buf[g_rx_line_len] = g_rx_byte;
    g_rx_line_len++;
    g_rx_line_buf[g_rx_line_len] = '\0';
  }

  /* 行结束或缓冲区满：尝试解析并重置缓冲 */
  if ((g_rx_byte == '\n') || (g_rx_line_len >= (ESP8266_RX_LINE_BUF_SIZE - 1U)))
  {
    if ((g_init_finished != 0U) &&
        ESP8266_ParseIpdFrame(g_rx_line_buf, g_rx_line_len, g_ipd_link_buf, g_ipd_payload_buf) != 0U)
    {
      ESP8266_CommandState next_state = ESP8266_ParseCommandState(g_ipd_payload_buf);
      if (next_state != ESP8266_STATE_IDLE)
      {
        g_current_state  = next_state;
        g_pending_state  = next_state;
        g_command_pending = 1U;
      }
    }
    ESP8266_ResetLineBuffer();
  }

  /* 重新挂载单字节接收，保持中断持续 */
  ESP8266_StartReceive();
}
