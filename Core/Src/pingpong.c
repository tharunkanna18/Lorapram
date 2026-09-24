/**
 ******************************************************************************
 * @file    pingpong.c
 * @brief   PingPong LoRa application.
 *
 *  Role selection
 *  ──────────────
 *  Define MASTER_BOARD in your IDE preprocessor (Project → C/C++ Build →
 *  Settings → MCU GCC Compiler → Preprocessor) for the board that
 *  initiates communication by sending "PING".
 *  Leave it undefined for the board that listens first and replies "PONG".
 *
 *  State machine (MASTER)
 *  ──────────────────────
 *  IDLE → TX "PING" → wait TX_DONE → RX → wait RX_DONE
 *       → if payload == "PONG" : log, repeat
 *       → else                 : ignore, retry
 *
 *  State machine (SLAVE)
 *  ──────────────────────
 *  IDLE → RX → wait RX_DONE
 *       → if payload == "PING" : TX "PONG" → wait TX_DONE → back to RX
 *       → else                 : back to RX
 *
 *  UART debug (115 200 baud, PB6 TX / PB7 RX – adjust for your board)
 *  Prints: "[PING-TX]", "[PONG-RX] RSSI=-65 SNR=7", "[RX TIMEOUT]", etc.
 ******************************************************************************
 */

#include "pingpong.h"
#include "subghz_phy.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ─── UART handle for debug output ──────────────────────────────────────── */
UART_HandleTypeDef huart1;  /* USART1 for debug output */

/* ─── Tx packet buffer ───────────────────────────────────────────────────── */
static uint8_t TxPacket[PINGPONG_BUFFER_SIZE];
static uint8_t TxLen;

/* ─── Rx buffer ──────────────────────────────────────────────────────────── */
static uint8_t RxBuffer[PINGPONG_BUFFER_SIZE + 1]; /* +1 for NUL terminator */
static uint8_t RxSize;

/* ─── Internal state ─────────────────────────────────────────────────────── */
typedef enum {
    APP_IDLE = 0,
    APP_TX,
    APP_RX,
    APP_WAIT,
} AppState_t;

static AppState_t AppState = APP_IDLE;

/* ─── Debug UART ─────────────────────────────────────────────────────────── */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE; /* Disabled! */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void DBG_Print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str,
                      (uint16_t)strlen(str), 100);
}

static void DBG_Printf(const char *fmt, ...)
{
    char buf[80];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    DBG_Print(buf);
}

/* ─── Radio param helpers ─────────────────────────────────────────────────── */
static uint16_t bw_to_khz(uint8_t bw)
{
    if (bw == 5) return 250;
    if (bw == 6) return 500;
    return 125; /* bw == 4 */
}

static const char* cr_to_str(uint8_t cr)
{
    switch(cr) {
        case 2: return "4/6";
        case 3: return "4/7";
        case 4: return "4/8";
        default: return "4/5";
    }
}

/* ─── Build TX packet ─────────────────────────────────────────────────────── */
static void Build_PingPacket(void)
{
    TxLen = (uint8_t)snprintf((char*)TxPacket, sizeof(TxPacket),
        "$I AM BATMAN,%lu,SF%d,%dkHz,CR%s;CRC1",
        (unsigned long)Radio_GetCurrentFreq(),
        (int)Radio_GetCurrentSf(),
        (int)bw_to_khz(Radio_GetCurrentBw()),
        cr_to_str(Radio_GetCurrentCr()));
}

static void Build_PongPacket(void)
{
    TxLen = (uint8_t)snprintf((char*)TxPacket, sizeof(TxPacket),
        "$I AM IRONMAN,%lu,SF%d,%dkHz,CR%s;CRC1",
        (unsigned long)Radio_GetCurrentFreq(),
        (int)Radio_GetCurrentSf(),
        (int)bw_to_khz(Radio_GetCurrentBw()),
        cr_to_str(Radio_GetCurrentCr()));
}

/* ─── Read received buffer from radio ───────────────────────────────────── */
static void ReadRxPayload(void)
{
    uint8_t offset = 0;
    uint8_t rxStatus[2] = {0};
    /* GetRxBufferStatus returns: [0]=PayloadLengthRx, [1]=RxStartBufferPointer */
    HAL_SUBGHZ_ExecGetCmd(&hsubghz, 0x13, rxStatus, 2);
    RxSize  = rxStatus[0];
    offset  = rxStatus[1];
    if (RxSize > PINGPONG_BUFFER_SIZE) RxSize = PINGPONG_BUFFER_SIZE;
    memset(RxBuffer, 0, sizeof(RxBuffer));
    HAL_SUBGHZ_ReadBuffer(&hsubghz, offset, RxBuffer, RxSize);
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void PingPong_Init(void)
{
    MX_USART1_UART_Init();

    DBG_Print("\r\n=============================\r\n");
    DBG_Print(" LoRa PingPong — Booting...\r\n");
    DBG_Print(" UART debug initialized on PB6/PB7.\r\n");
    DBG_Print(" Initializing LoRa Radio...\r\n");
    DBG_Print(" >> CP1: UART OK <<\r\n");
    HAL_Delay(200);

    Radio_Init();

    DBG_Print(" >> CP2: Radio OK <<\r\n");
    DBG_Print(" LoRa Radio initialized successfully!\r\n");
#ifdef MASTER_BOARD
    DBG_Print(" Role : MASTER (PING)\r\n");
#else
    DBG_Print(" Role : SLAVE  (PONG)\r\n");
#endif
    DBG_Print(" Default Mode : 3 (Balanced General)\r\n");
    DBG_Print(" Freq : 865000000 Hz\r\n");
    DBG_Print("=============================\r\n\r\n");

    AppState = APP_IDLE;
}

void PingPong_Process(void)
{
    static uint32_t last_heartbeat = 0;
    static uint32_t state_time = 0;
    if (HAL_GetTick() - last_heartbeat > 2000)
    {
        last_heartbeat = HAL_GetTick();
        DBG_Print("=== UART Heartbeat: PB6/PB7 @ 115200 Baud ===\r\n");
    }

    /* Check for UART command (1-7) */
    static uint8_t mode_switch_pending = 0;
    static uint32_t mode_switch_time = 0;

    uint8_t cmd;
    if (HAL_UART_Receive(&huart1, &cmd, 1, 0) == HAL_OK)
    {
        if (cmd >= '1' && cmd <= '7')
        {
            uint8_t mode = cmd - '0';
            static const char *mode_names[] = {
                "",
                "1: Ultra Long Range",
                "2: Fast Data",
                "3: Balanced General",
                "4: Indoor Penetration",
                "5: Low Power Battery",
                "6: High Noise Environment",
                "7: Sensor Streaming"
            };
            DBG_Printf("\r\n=== Switching to Mode %s ===\r\n", mode_names[mode]);
            Radio_ApplyMode(mode);
            /* Park state machine. Master will wait 1s for Slave to re-enter RX */
            AppState = APP_WAIT;
            mode_switch_pending = 1;
            mode_switch_time = HAL_GetTick();
            DBG_Print("[MODE] Radio reconfigured. Waiting 1s for remote to sync...\r\n");
        }
    }
    else
    {
        /* Clear potential Overrun error that could block future reception */
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
        {
            __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
            cmd = (uint8_t)(huart1.Instance->RDR & 0xFF);
        }
    }

    /* After mode switch: wait 1s so both sides finish reconfiguring */
    if (mode_switch_pending && (HAL_GetTick() - mode_switch_time >= 1000))
    {
        mode_switch_pending = 0;
        DBG_Print("[MODE] Sync done. Restarting communication.\r\n");
        AppState = APP_IDLE;
    }

    switch (AppState)
    {
    /* ── IDLE ─────────────────────────────────────────────────────────── */
    case APP_IDLE:
#ifdef MASTER_BOARD
        /* Master sends PING first */
        Build_PingPacket();
        Radio_SetTxConfig();
        DBG_Printf("[TX] Sending: %.*s\r\n", TxLen, TxPacket);
        Radio_Send(TxPacket, TxLen);
        state_time = HAL_GetTick();
        AppState = APP_TX;
#else
        /* Slave listens first */
        Radio_SetRxConfig();
        Radio_Rx(RX_TIMEOUT_VALUE);
        DBG_Print("[RX] Listening...\r\n");
        state_time = HAL_GetTick();
        AppState = APP_RX;
#endif
        break;

    /* ── TX: wait for transmission complete ───────────────────────────── */
    case APP_TX:
        if (RadioState == RADIO_TX_DONE)
        {
            DBG_Print("[TX] Done. Switching to RX...\r\n");
            Radio_SetRxConfig();
            Radio_Rx(RX_TIMEOUT_VALUE);
            state_time = HAL_GetTick();
            AppState = APP_RX;
        }
        else if (RadioState == RADIO_TX_TIMEOUT || (HAL_GetTick() - state_time > 5000))
        {
            DBG_Print("[TX] TIMEOUT! Retrying...\r\n");
            AppState = APP_IDLE;
        }
        break;

    /* ── RX: wait for packet ──────────────────────────────────────────── */
    case APP_RX:
        if (RadioState == RADIO_RX_DONE)
        {
            ReadRxPayload();

#ifdef MASTER_BOARD
            /* Master expects packet starting with "$I AM IRONMAN" */
            if (RxSize > 0 && strncmp((char*)RxBuffer, "$I AM IRONMAN", 13) == 0)
            {
                RxBuffer[RxSize] = '\0'; /* null-terminate for printing */
                DBG_Printf("[RX] Received: %s\r\n", (char*)RxBuffer);
                DBG_Printf("[RX] RSSI=%d dBm  SNR=%d dB\r\n",
                           (int)Radio_GetLastRssi(), (int)Radio_GetLastSnr());
                HAL_Delay(500);
                AppState = APP_IDLE;
            }
            else
            {
                DBG_Print("[RX] Unknown payload. Ignoring.\r\n");
                Radio_Rx(RX_TIMEOUT_VALUE);
            }
#else
            /* Slave expects packet starting with "$I AM BATMAN" */
            if (RxSize > 0 && strncmp((char*)RxBuffer, "$I AM BATMAN", 12) == 0)
            {
                RxBuffer[RxSize] = '\0'; /* null-terminate for printing */
                DBG_Printf("[RX] Received: %s\r\n", (char*)RxBuffer);
                DBG_Printf("[RX] RSSI=%d dBm  SNR=%d dB\r\n",
                           (int)Radio_GetLastRssi(), (int)Radio_GetLastSnr());
                Build_PongPacket();
                DBG_Printf("[TX] Sending: %.*s\r\n", TxLen, TxPacket);
                Radio_SetTxConfig();
                Radio_Send(TxPacket, TxLen);
                state_time = HAL_GetTick();
                AppState = APP_TX;
            }
            else
            {
                DBG_Print("[RX] Unknown payload. Ignoring.\r\n");
                Radio_Rx(RX_TIMEOUT_VALUE);
                state_time = HAL_GetTick();
            }
#endif
        }
        else if (RadioState == RADIO_RX_TIMEOUT || (HAL_GetTick() - state_time > RX_TIMEOUT_VALUE + 100))
        {
            DBG_Print("[RX] Timeout.\r\n");
            AppState = APP_IDLE;  /* restart */
        }
        else if (RadioState == RADIO_RX_ERROR)
        {
            DBG_Print("[RX] CRC Error!\r\n");
            Radio_Rx(RX_TIMEOUT_VALUE);
            state_time = HAL_GetTick();
        }
        break;

    default:
        AppState = APP_IDLE;
        break;
    }
}

/* ─── HAL UART MSP (GPIO + clock) ────────────────────────────────────────── */
/*
 * USART1 pins for Seeed Studio LoRa-E5 module:
 *   PB6  → USART1_TX  (AF7)
 *   PB7  → USART1_RX  (AF7)
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    }
}
