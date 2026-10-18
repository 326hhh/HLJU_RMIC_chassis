/**
  ******************************************************************************
  * @file    can_test.c
  * @brief   CAN1 通信测试模块实现
  *
  * 依赖：CubeMX 生成的 Core/Src/can.c / Core/Inc/can.h（提供 hcan1 句柄）
  *
  * ⚠️ F407 上三个经典坑（本文件都已处理）：
  *
  *   1) 滤波器不配 = 一帧都收不到
  *      HAL 不会自动给默认滤波器。CubeMX 生成的 can.c 里只有 Init，没有
  *      ConfigFilter，所以必须自己配。这里用「掩码全 0 = 全通过」。
  *
  *   2) CAN1 / CAN2 共用滤波器组
  *      F407 共 14 组（0~13 给 CAN1，14~27 给 CAN2），且用 CAN2 前必须先
  *      初始化 CAN1。本模块只用 CAN1，占第 0 组。
  *
  *   3) 位时序必须自己强制配
  *      本工程 .ioc 里没有 Prescaler/BS1/BS2/SJW 这几个键，重新生成时会被
  *      CubeMX 的默认值覆盖（原来是 Prescaler=16/BS1=1TQ/BS2=1TQ，一位只有
  *      3 个 TQ、采样点 66.7%，CAN 协议下根本不可能稳定通信）。
  *      所以 can_test_init() 会拿 can_test_config.h 里的值再 Init 一次。
  ******************************************************************************
  */

#include "can_test.h"

#include <stdio.h>
#include <string.h>

#include "can_test_config.h"
#include "can.h"        /* CubeMX 生成，声明 hcan1 */
#include "main.h"       /* LED_Pin / LED_GPIO_Port（PB2，板上 LED） */

#if CANTEST_SERIAL_DEBUG
#include "usart.h"      /* 自诊断打印用 */
#endif

/* ============================== 内部状态 ================================== */
static volatile uint32_t s_tx_count   = 0;
static volatile uint32_t s_rx_count   = 0;
static volatile uint32_t s_last_error = 0;
static volatile bool     s_bus_off    = false;

static uint32_t s_last_hb_ms   = 0;
static uint32_t s_last_dbg_ms  = 0;
static uint32_t s_last_st_ms   = 0;     /* 自检帧发送时刻 */
static uint8_t  s_hb_seq     = 0;
static bool     s_started    = false;
static bool     s_led_on     = false;   /* PB2 的逻辑状态（与物理电平的对应关系见 CANTEST_LED_ACTIVE_HIGH） */

/* ============================== 内部函数 ================================== */

/** 把 hcan1 的位时序强制成 can_test_config.h 里的值，然后重新 Init 一次。
 *  @note 因为 MX_CAN1_Init() 已经跑过，hcan1.State 不是 RESET，
 *        HAL_CAN_Init() 不会重复调用 MspInit（时钟/GPIO 已就绪），
 *        所以这里只是覆盖位时序，安全。 */
static void can_test_force_timing(void)
{
    hcan1.Init.Prescaler             = CANTEST_PRESCALER;
#if CANTEST_LOOPBACK_SELFTEST
    /* 自检模式：控制器自己发自己收，帧不出总线口，不需要任何外部节点 ACK */
    hcan1.Init.Mode                  = CAN_MODE_SILENT_LOOPBACK;
#else
    hcan1.Init.Mode                  = CAN_MODE_NORMAL;
#endif
    hcan1.Init.SyncJumpWidth         = CANTEST_SJW;
    hcan1.Init.TimeSeg1              = CANTEST_BS1;
    hcan1.Init.TimeSeg2              = CANTEST_BS2;
    hcan1.Init.TimeTriggeredMode     = DISABLE;
    hcan1.Init.AutoBusOff            = ENABLE;   /* 出总线故障自动恢复 */
    hcan1.Init.AutoWakeUp            = DISABLE;
    hcan1.Init.AutoRetransmission    = CANTEST_AUTO_RETRANSMIT ? ENABLE : DISABLE;
    hcan1.Init.ReceiveFifoLocked     = DISABLE;
    hcan1.Init.TransmitFifoPriority  = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        s_last_error = HAL_CAN_GetError(&hcan1);
        Error_Handler();
    }
}

/** 配置全通过滤波器：接收所有标准帧和扩展帧 */
static void can_test_config_filter(void)
{
    CAN_FilterTypeDef f;

    memset(&f, 0, sizeof(f));

    f.FilterBank           = 0;   /* CAN1 的第 0 组 */
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;   /* 掩码全 0 = 这几位不关心 = 全通过 */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;       /* CAN2 的起始组（本模块不用 CAN2） */

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
    {
        s_last_error = HAL_CAN_GetError(&hcan1);
        Error_Handler();
    }
}

/** 从 CAN_BTR 寄存器反读实际生效的位速率。
 *  比"相信配置"可靠 —— CubeMX 配错了或者 Init 没生效都能立刻发现。 */
static uint32_t can_test_read_bitrate(void)
{
    const uint32_t btr = hcan1.Instance->BTR;

    const uint32_t brp = (btr & CAN_BTR_BRP) + 1U;                  /* 位 0-9  */
    const uint32_t ts1 = ((btr & CAN_BTR_TS1) >> 16) + 1U;          /* 位 16-19 */
    const uint32_t ts2 = ((btr & CAN_BTR_TS2) >> 20) + 1U;          /* 位 20-22 */
    const uint32_t tq  = 1U + ts1 + ts2;                            /* 同步 + BS1 + BS2 */

    return CANTEST_APB1_HZ / brp / tq;
}

#if CANTEST_ECHO_ENABLE
/** 取一帧就回一帧（ID + CANTEST_ECHO_OFFSET） */
static void can_test_echo(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data)
{
    if (hdr->IDE == CAN_ID_STD)
    {
        const uint16_t echo_id =
            (uint16_t)((hdr->StdId + CANTEST_ECHO_OFFSET) & 0x7FFU);
        (void)can_test_send(echo_id, data, (uint8_t)hdr->DLC);
    }
    else
    {
        /* 扩展帧（云台 ZDT 就是扩展帧）统一用固定标准帧 ID 回，方便识别 */
        (void)can_test_send(CANTEST_EXT_ECHO_ID, data, (uint8_t)hdr->DLC);
    }
}

#endif /* CANTEST_ECHO_ENABLE */

/** 把逻辑状态写到 PB2。
 *  物理电平由 CANTEST_LED_ACTIVE_HIGH 决定 —— 极性猜错了只改那一个宏。 */
static void can_test_led_write(bool on)
{
#if CANTEST_LED_ACTIVE_HIGH
    const GPIO_PinState level = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    const GPIO_PinState level = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, level);
    s_led_on = on;
}

/** 收帧总入口：控制帧（LED）自己处理，其余走回显。
 *  @note 轮询模式和中断模式都走这里，保证两种模式下行为一致。 */
static void can_test_handle_rx(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data)
{
    /* ---------- LED 控制帧：只认标准帧 + CANTEST_LED_ID ---------- */
    if ((hdr->IDE == CAN_ID_STD) && (hdr->StdId == CANTEST_LED_ID))
    {
        /* 没数据就当翻转请求（方便手工 cansend can0 200#） */
        const uint8_t cmd = (hdr->DLC > 0U) ? data[0]
                                            : (uint8_t)CANTEST_LED_CMD_TOGGLE;

        switch (cmd)
        {
        case CANTEST_LED_CMD_OFF:
            can_test_led_write(false);
            break;

        case CANTEST_LED_CMD_ON:
            can_test_led_write(true);
            break;

        case CANTEST_LED_CMD_TOGGLE:
            can_test_led_write(!s_led_on);
            break;

        default:
            /* 未知命令：不改灯，但照样回 ACK 报告当前状态 */
            break;
        }

        /* ACK = [cmd, 当前状态, 已收帧数低 8 位, 0xA5 魔数]
         * 0xA5 是给 Jetson 侧认「这是 LED 应答，不是普通回显」用的 */
        {
            uint8_t ack[4];

            ack[0] = cmd;
            ack[1] = s_led_on ? 1U : 0U;
            ack[2] = (uint8_t)(s_rx_count & 0xFFU);
            ack[3] = 0xA5U;

            (void)can_test_send(CANTEST_LED_ACK_ID, ack, 4U);
        }

        return;   /* 不再回显，省得 Jetson 收到两帧 */
    }

    /* ---------- 其他帧：原样回显（ID + 偏移） ---------- */
#if CANTEST_ECHO_ENABLE
    can_test_echo(hdr, data);
#else
    (void)hdr;
    (void)data;
#endif
}

/* ============================== 对外接口 ================================== */

void can_test_init(void)
{
    s_tx_count   = 0;
    s_rx_count   = 0;
    s_last_error = 0;
    s_bus_off    = false;
    s_hb_seq     = 0;

    /* 顺序必须是：强制位时序 -> 配滤波器 -> Start -> (可选)开中断
     * 反过来 ConfigFilter 会返回 HAL_ERROR。 */
    can_test_force_timing();
    can_test_config_filter();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        s_last_error = HAL_CAN_GetError(&hcan1);
        Error_Handler();
    }

#if CANTEST_USE_RX_INTERRUPT
    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        s_last_error = HAL_CAN_GetError(&hcan1);
        Error_Handler();
    }
#endif

    s_last_hb_ms  = HAL_GetTick();
    s_last_dbg_ms = HAL_GetTick();
    s_last_st_ms  = HAL_GetTick();
    s_started     = true;

    /* 上电先确保灯是灭的（MX_GPIO_Init 已经把引脚拉到低，这里再过一遍逻辑状态） */
    can_test_led_write(false);
}

bool can_test_send(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t            mailbox = 0;
    uint8_t             buf[8];

    if (!s_started)
    {
        return false;
    }
    if (len > 8U)
    {
        len = 8U;
    }

    memset(buf, 0, sizeof(buf));
    if ((data != NULL) && (len > 0U))
    {
        memcpy(buf, data, len);
    }

    hdr.StdId              = (uint16_t)(std_id & 0x7FFU);
    hdr.ExtId              = 0;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = len;
    hdr.TransmitGlobalTime = DISABLE;

    /* 三个发送邮箱都满时返回 HAL_ERROR，这里直接丢一帧（测试场景可接受） */
    if (HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox) == HAL_OK)
    {
        s_tx_count++;
        return true;
    }

    s_last_error = HAL_CAN_GetError(&hcan1);
    return false;
}

void can_test_poll(void)
{
    uint32_t now;

    if (!s_started)
    {
        return;
    }

    /* ---------------- 1. 心跳：每 CANTEST_HEARTBEAT_MS 一帧 ---------------- */
    now = HAL_GetTick();
#if CANTEST_HEARTBEAT_ENABLE
    if ((uint32_t)(now - s_last_hb_ms) >= CANTEST_HEARTBEAT_MS)
    {
        uint8_t d[4];

        s_last_hb_ms = now;

        /* 数据 = [序号, 0xCA, 0xFE, 已收帧数低 8 位]
         * 0xCA 0xFE 是给 Jetson 侧一眼认出「这是 F407 的心跳」用的魔数 */
        d[0] = s_hb_seq++;
        d[1] = 0xCAU;
        d[2] = 0xFEU;
        d[3] = (uint8_t)(s_rx_count & 0xFFU);

        (void)can_test_send(CANTEST_HEARTBEAT_ID, d, 4U);
    }
#endif /* CANTEST_HEARTBEAT_ENABLE */

#if CANTEST_LOOPBACK_SELFTEST
    /* ---------------- 1b. 自检：自己发一帧 0x200，靠回环走到 RX ---------------- */
    if ((uint32_t)(now - s_last_st_ms) >= CANTEST_LOOPBACK_SELFTEST_MS)
    {
        const uint8_t cmd = (uint8_t)CANTEST_LED_CMD_TOGGLE;

        s_last_st_ms = HAL_GetTick();
        (void)can_test_send(CANTEST_LED_ID, &cmd, 1U);
    }
#endif /* CANTEST_LOOPBACK_SELFTEST */

    /* ---------------- 2. 收帧并回显 ---------------- */
#if CANTEST_USE_RX_INTERRUPT
    /* 中断模式：收帧在 HAL_CAN_RxFifo0MsgPendingCallback() 里处理，这里不做 */
#else
    /* 轮询模式：把 FIFO0 取空 */
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef hdr;
        uint8_t             data[8];

        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, data) != HAL_OK)
        {
            s_last_error = HAL_CAN_GetError(&hcan1);
            break;
        }

        s_rx_count++;
        can_test_handle_rx(&hdr, data);
    }
#endif

    /* ---------------- 3. 串口自诊断 ---------------- */
#if CANTEST_SERIAL_DEBUG
    if ((uint32_t)(now - s_last_dbg_ms) >= CANTEST_DEBUG_PERIOD_MS)
    {
        char line[160];
        int  n;

        /* 从 CAN_ESR 反读硬件错误计数器。
         * HAL 的 ErrorCode 只反映「HAL 调用层面」的软件错误（比如邮箱满），
         * 看不到总线层的 TEC/REC/LEC —— 而排查速率不一致恰恰只能靠它。 */
        const uint32_t esr = hcan1.Instance->ESR;
        const uint32_t tec = (esr >> 16) & 0xFFU;   /* 发送错误计数 */
        const uint32_t rec = (esr >> 24) & 0xFFU;   /* 接收错误计数 */
        const uint32_t lec = esr & 0x7U;            /* 最后一次错误类型 */

        static const char *const lec_name[8] = {
            "none", "STUFF", "FORM", "ACK",
            "BIT-REC", "BIT-DOM", "CRC", "?"
        };

        s_last_dbg_ms = now;

        n = snprintf(line, sizeof(line),
                     "CAN %lubps | tx=%lu rx=%lu err=0x%08lX bof=%d | TEC=%lu REC=%lu LEC=%s\r\n",
                     (unsigned long)can_test_read_bitrate(),
                     (unsigned long)s_tx_count,
                     (unsigned long)s_rx_count,
                     (unsigned long)s_last_error,
                     (int)s_bus_off,
                     (unsigned long)tec,
                     (unsigned long)rec,
                     lec_name[lec & 0x7U]);

        if (n > 0)
        {
            (void)HAL_UART_Transmit(&CANTEST_DEBUG_UART,
                                    (uint8_t *)line, (uint16_t)n, 20u);
        }
    }
#endif

    /* ---------------- 4. Bus-Off 自动恢复 ---------------- */
#if CANTEST_AUTO_BUSOFF_RECOVER
    if ((HAL_CAN_GetError(&hcan1) & HAL_CAN_ERROR_BOF) != 0U)
    {
        s_bus_off = true;

        /* Bus-Off 意味着总线上连续出错。排查方向：
         *   · 没有别的节点应答（对面没上电 / 线没接好 / 终端电阻缺失）
         *   · 两边位速率不一致
         */
        (void)HAL_CAN_Stop(&hcan1);
        (void)HAL_CAN_Start(&hcan1);
    }
#endif
}

can_test_state_t can_test_get_state(void)
{
    can_test_state_t st;

    st.started      = s_started;
    st.tx_count     = s_tx_count;
    st.rx_count     = s_rx_count;
    st.last_error   = s_last_error;
    st.bus_off_seen = s_bus_off;
    st.bitrate_hz   = can_test_read_bitrate();

    return st;
}

bool can_test_is_started(void)
{
    return s_started;
}

uint32_t can_test_last_error(void)
{
    return s_last_error;
}

/* ============================== 中断回调 ================================== */
#if CANTEST_USE_RX_INTERRUPT

/** 收到一帧的中断回调（HAL 弱函数，只在中断模式下编译进来）
 *  ⚠️ 如果工程里别处也定义了这个函数，会链接冲突，合并即可。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t             data[8];

    if (hcan->Instance != CAN1)
    {
        return;
    }

    /* 一次中断可能积压多帧，取空为止 */
    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK)
    {
        s_rx_count++;
        can_test_handle_rx(&hdr, data);
    }
}

#endif /* CANTEST_USE_RX_INTERRUPT */
