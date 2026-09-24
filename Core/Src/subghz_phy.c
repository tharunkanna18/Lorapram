/**
 ******************************************************************************
 * @file    subghz_phy.c
 * @brief   STM32WL SubGHz (SX126x-compatible) radio driver for LoRa PingPong.
 *
 *  Architecture
 *  ─────────────
 *  The STM32WL embeds an SX1262 radio core that is accessed through the
 *  SUBGHZ peripheral (an internal SPI-like bus).  The HAL driver exposes it
 *  via HAL_SUBGHZ_ExecSetCmd / HAL_SUBGHZ_ExecGetCmd / ...Mem interfaces.
 *
 *  This file wraps those primitives into a small set of helper functions that
 *  the PingPong layer calls.  It also implements the HAL interrupt callbacks
 *  (HAL_SUBGHZ_TxCpltCallback, HAL_SUBGHZ_RxCpltCallback …) so the radio
 *  state machine is updated from the ISR context.
 *
 *  SX126x opcodes and register addresses are taken from the SX1262 datasheet
 *  (DS.SX1261-2.W.APP – rev 2.1) and are identical for the embedded radio
 *  inside the STM32WL5x.
 ******************************************************************************
 */

#include "subghz_phy.h"
#include <string.h>

/* ─── Handle & state ─────────────────────────────────────────────────────── */
SUBGHZ_HandleTypeDef     hsubghz;
volatile Radio_States_t  RadioState = RADIO_LOWPOWER;

/* Last packet info */
static int16_t  LastRssi = 0;
static int8_t   LastSnr  = 0;

/* Current RF Configuration */
static uint32_t current_freq = 865000000U;
static uint8_t  current_sf = 9;
static uint8_t  current_bw = 4; // 125 kHz
static uint8_t  current_cr = 1; // 4/5
static int8_t   current_tx_pow = 14;

/* ─── SX126x opcodes ─────────────────────────────────────────────────────── */
#define RADIO_SET_SLEEP              0x84U
#define RADIO_SET_STANDBY            0x80U
#define RADIO_SET_FS                 0xC1U
#define RADIO_SET_TX                 0x83U
#define RADIO_SET_RX                 0x82U
#define RADIO_SET_RFFREQUENCY        0x86U
#define RADIO_SET_PACKETTYPE         0x8AU
#define RADIO_SET_MODULATIONPARAMS   0x8BU
#define RADIO_SET_PACKETPARAMS       0x8CU
#define RADIO_SET_TXPARAMS           0x8EU
#define RADIO_CFG_DIOIRQ             0x08U
#define RADIO_CLR_IRQSTATUS          0x02U
#define RADIO_GET_IRQSTATUS          0x12U
#define RADIO_GET_RXBUFFERSTATUS     0x13U
#define RADIO_GET_PACKETSTATUS       0x14U
#define RADIO_READ_BUFFER            0x1EU
#define RADIO_WRITE_BUFFER           0x0EU
#define RADIO_SET_BUFFERBASEADDRESS  0x8FU
#define RADIO_SET_REGULATORMODE      0x96U
#define RADIO_SET_PACONFIG           0x95U
#define RADIO_SET_RXTXFALLBACKMODE   0x93U
#define RADIO_CALIBRATEIMAGE         0x98U
#define RADIO_WRITE_REGISTER         0x0DU
#define RADIO_READ_REGISTER          0x1DU

/* SX126x register addresses */
#define REG_XTA_TRIM                 0x0911U
#define REG_OCP                      0x08E7U
#define REG_TX_CLAMP_CFG             0x08D8U

/* Packet type */
#define PACKET_TYPE_LORA             0x01U

/* IRQ masks */
#define IRQ_TX_DONE                  0x0001U
#define IRQ_RX_DONE                  0x0002U
#define IRQ_RX_TX_TIMEOUT            0x0200U
#define IRQ_CRC_ERROR                0x0040U
#define IRQ_ALL                      0x03FFU

/* ─── Internal helpers ───────────────────────────────────────────────────── */

static void SubGHz_SetStandby(uint8_t mode)
{
    /* mode 0 = RC, 1 = XOSC */
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &mode, 1);
}

static void SubGHz_SetPacketType(uint8_t pktType)
{
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETTYPE, &pktType, 1);
}

static void SubGHz_SetRfFrequency(uint32_t freq)
{
    /* fRF = freq * 2^25 / 32e6  (TCXO / XTAL reference) */
    uint32_t fInt  = (uint32_t)((double)freq / (double)32000000.0 * (double)(1 << 25) + 0.5);
    uint8_t  buf[4];
    buf[0] = (uint8_t)((fInt >> 24) & 0xFF);
    buf[1] = (uint8_t)((fInt >> 16) & 0xFF);
    buf[2] = (uint8_t)((fInt >>  8) & 0xFF);
    buf[3] = (uint8_t)( fInt        & 0xFF);
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, buf, 4);
}

static void SubGHz_SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax,
                                uint8_t deviceSel,   uint8_t paLut)
{
    uint8_t buf[4] = { paDutyCycle, hpMax, deviceSel, paLut };
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACONFIG, buf, 4);
}

static void SubGHz_SetTxParams(int8_t power, uint8_t rampTime)
{
    uint8_t buf[2] = { (uint8_t)power, rampTime };
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, buf, 2);
}

static void SubGHz_SetModulationParams(uint8_t sf, uint8_t bw,
                                        uint8_t cr, uint8_t ldro)
{
    uint8_t buf[4] = { sf, bw, cr, ldro };
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, buf, 4);
}

static void SubGHz_SetPacketParams(uint16_t preambleLen, uint8_t headerType,
                                    uint8_t payloadLen,   uint8_t crcMode,
                                    uint8_t invertIq)
{
    uint8_t buf[6];
    buf[0] = (uint8_t)(preambleLen >> 8);
    buf[1] = (uint8_t)(preambleLen & 0xFF);
    buf[2] = headerType;
    buf[3] = payloadLen;
    buf[4] = crcMode;
    buf[5] = invertIq;
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS, buf, 6);
}

static void SubGHz_SetBufferBaseAddress(uint8_t txAddr, uint8_t rxAddr)
{
    uint8_t buf[2] = { txAddr, rxAddr };
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

static void SubGHz_SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask,
                                    uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)(irqMask  >> 8); buf[1] = (uint8_t)(irqMask  & 0xFF);
    buf[2] = (uint8_t)(dio1Mask >> 8); buf[3] = (uint8_t)(dio1Mask & 0xFF);
    buf[4] = (uint8_t)(dio2Mask >> 8); buf[5] = (uint8_t)(dio2Mask & 0xFF);
    buf[6] = (uint8_t)(dio3Mask >> 8); buf[7] = (uint8_t)(dio3Mask & 0xFF);
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CFG_DIOIRQ, buf, 8);
}

static uint16_t SubGHz_GetIrqStatus(void)
{
    uint8_t buf[2] = {0, 0};
    HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_IRQSTATUS, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void SubGHz_ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[2] = { (uint8_t)(irqMask >> 8), (uint8_t)(irqMask & 0xFF) };
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_IRQSTATUS, buf, 2);
}

static void SubGHz_WriteRegister(uint16_t addr, uint8_t value)
{
    uint8_t buf[3];
    buf[0] = (uint8_t)(addr >> 8);
    buf[1] = (uint8_t)(addr & 0xFF);
    buf[2] = value;
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_WRITE_REGISTER, buf, 3);
}

static void SubGHz_CalibrateImage(uint32_t freq)
{
    uint8_t buf[2];
    if      (freq > 900000000U) { buf[0] = 0xE1; buf[1] = 0xE9; }
    else if (freq > 850000000U) { buf[0] = 0xD7; buf[1] = 0xDB; }
    else if (freq > 770000000U) { buf[0] = 0xC1; buf[1] = 0xC5; }
    else if (freq > 460000000U) { buf[0] = 0x75; buf[1] = 0x81; }
    else                        { buf[0] = 0x6B; buf[1] = 0x6F; }
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CALIBRATEIMAGE, buf, 2);
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void Radio_Config(void)
{
    /* Standby using RC13M */
    SubGHz_SetStandby(0);

    /* Use DC-DC regulator (saves ~10 mA vs LDO) */
    uint8_t regMode = 0x01;
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_REGULATORMODE, &regMode, 1);

    /* LoRa packet type */
    SubGHz_SetPacketType(PACKET_TYPE_LORA);

    /* Calibrate image for the chosen band */
    SubGHz_CalibrateImage(current_freq);

    /* RF frequency */
    SubGHz_SetRfFrequency(current_freq);

    /* PA config for HP PA (Wio-E5 / LoRa-E5 uses HP path exclusively) */
    /* paDutyCycle=0x04, hpMax=0x07, deviceSel=0x00 (HP PA), paLut=0x01 */
    SubGHz_SetPaConfig(0x04, 0x07, 0x00, 0x01);

    /* Set LoRa Public SyncWord (0x3444) */
    SubGHz_WriteRegister(0x0740, 0x34);
    SubGHz_WriteRegister(0x0741, 0x44);

    /* Tx parameters: power, ramp time 200 µs */
    SubGHz_SetTxParams(current_tx_pow, 0x04);

    /* OCP (over-current protection) — 140 mA for HP PA */
    SubGHz_WriteRegister(REG_OCP, 0x38);

    /* TX clamp config — recommended fix per SX1262 errata */
    uint8_t txClamp = 0x1E;
    SubGHz_WriteRegister(REG_TX_CLAMP_CFG, txClamp);

    /* Buffer base addresses: Tx=0x00, Rx=0x00 (separate ops, no overlap) */
    SubGHz_SetBufferBaseAddress(0x00, 0x00);

    /* Modulation params: SF, BW, CR, LowDataRateOptimize */
    uint8_t ldro = ((current_sf >= 11) && (current_bw == 4)) ? 1 : 0;
    SubGHz_SetModulationParams(current_sf,
                                current_bw,
                                current_cr,
                                ldro);

    /* Packet params: preamble, header type, payload, CRC, IQ */
    SubGHz_SetPacketParams((uint16_t)LORA_PREAMBLE_LENGTH,
                            (uint8_t)LORA_FIX_LENGTH_PAYLOAD,
                            (uint8_t)PINGPONG_BUFFER_SIZE,
                            0x01,   /* CRC on */
                            (uint8_t)LORA_IQ_INVERSION);

    /* Route all IRQs to DIO1 (SUBGHZ_IRQ line) */
    SubGHz_SetDioIrqParams(IRQ_ALL, IRQ_ALL, 0x0000, 0x0000);

    RadioState = RADIO_LOWPOWER;
}

void Radio_Init(void)
{
    /* Initialise HAL SUBGHZ peripheral */
    hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
    if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK) {
        Error_Handler();
    }

    Radio_Config();
}

void Radio_SetChannel(uint32_t freq)
{
    SubGHz_SetRfFrequency(freq);
}

void Radio_SetTxConfig(void)
{
    SubGHz_SetPacketType(PACKET_TYPE_LORA);
    uint8_t ldro = ((current_sf >= 11) && (current_bw == 4)) ? 1 : 0;
    SubGHz_SetModulationParams(current_sf,
                                current_bw,
                                current_cr,
                                ldro);
    SubGHz_SetPacketParams((uint16_t)LORA_PREAMBLE_LENGTH,
                            (uint8_t)LORA_FIX_LENGTH_PAYLOAD,
                            (uint8_t)PINGPONG_BUFFER_SIZE,
                            0x01, (uint8_t)LORA_IQ_INVERSION);
}

void Radio_SetRxConfig(void)
{
    Radio_SetTxConfig(); /* same mod params for Rx */
}

void Radio_Send(uint8_t *buffer, uint8_t size)
{
    RadioState = RADIO_TX;

    /* RF Switch TX High Power: PA4=0, PA5=1 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    SubGHz_SetBufferBaseAddress(0x00, 0x00);
    HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, buffer, size);

    /* Update payload length in packet params */
    SubGHz_SetPacketParams((uint16_t)LORA_PREAMBLE_LENGTH,
                            (uint8_t)LORA_FIX_LENGTH_PAYLOAD,
                            size, 0x01,
                            (uint8_t)LORA_IQ_INVERSION);

    /* Timeout: 0xFFFFFF = no timeout (rely on TX_DONE IRQ) */
    uint8_t timeout[3] = { 0xFF, 0xFF, 0xFF };
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, timeout, 3);
}

void Radio_Rx(uint32_t timeout)
{
    RadioState = RADIO_RX;

    /* RF Switch RX: PA4=1, PA5=0 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    SubGHz_SetBufferBaseAddress(0x00, 0x00);

    /* timeout in ms → SX126x units (timeout × 15.625 µs steps) */
    uint32_t rxTimeout;
    if (timeout == 0) {
        rxTimeout = 0xFFFFFF; /* continuous */
    } else {
        rxTimeout = (uint32_t)((double)timeout * 1000.0 / 15.625);
        if (rxTimeout > 0xFFFFFF) rxTimeout = 0xFFFFFF;
    }
    uint8_t buf[3];
    buf[0] = (uint8_t)(rxTimeout >> 16);
    buf[1] = (uint8_t)(rxTimeout >>  8);
    buf[2] = (uint8_t)(rxTimeout       );
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, buf, 3);
}

void Radio_Sleep(void)
{
    RadioState = RADIO_LOWPOWER;
    
    /* RF Switch OFF: PA4=0, PA5=0 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    uint8_t sleepConfig = 0x00; /* cold sleep */
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, &sleepConfig, 1);
}

void Radio_ApplyMode(uint8_t mode)
{
    /* Do not sleep, just switch state */
    SubGHz_SetStandby(0);

    switch(mode) {
        case 1: /* Ultra Long Range */
            current_freq = 865000000U;
            current_sf = 12;
            current_bw = 4; /* 125 kHz */
            current_cr = 4; /* 4/8 */
            current_tx_pow = 20;
            break;
        case 2: /* Fast Data */
            current_freq = 865000000U;
            current_sf = 7;
            current_bw = 6; /* 500 kHz */
            current_cr = 1; /* 4/5 */
            current_tx_pow = 14;
            break;
        case 3: /* Balanced General Mode */
            current_freq = 865000000U;
            current_sf = 9;
            current_bw = 4; /* 125 kHz */
            current_cr = 1; /* 4/5 */
            current_tx_pow = 14;
            break;
        case 4: /* Indoor Penetration Mode */
            current_freq = 865000000U;
            current_sf = 11;
            current_bw = 4; /* 125 kHz */
            current_cr = 3; /* 4/7 */
            current_tx_pow = 14;
            break;
        case 5: /* Low Power Battery Mode */
            current_freq = 865000000U;
            current_sf = 7;
            current_bw = 4; /* 125 kHz */
            current_cr = 1; /* 4/5 */
            current_tx_pow = 2;
            break;
        case 6: /* High Noise Environment */
            current_freq = 865000000U;
            current_sf = 10;
            current_bw = 4; /* 125 kHz */
            current_cr = 4; /* 4/8 */
            current_tx_pow = 14;
            break;
        case 7: /* Sensor Streaming Mode */
            current_freq = 865000000U;
            current_sf = 8;
            current_bw = 5; /* 250 kHz */
            current_cr = 1; /* 4/5 */
            current_tx_pow = 14;
            break;
        default:
            return;
    }
    
    /* Re-apply entire radio configuration to ensure all params correctly set */
    Radio_Config();
}

int16_t Radio_GetLastRssi(void)      { return LastRssi;      }
int8_t  Radio_GetLastSnr(void)       { return LastSnr;       }

uint32_t Radio_GetCurrentFreq(void)  { return current_freq;  }
uint8_t  Radio_GetCurrentSf(void)    { return current_sf;    }
uint8_t  Radio_GetCurrentBw(void)    { return current_bw;    }
uint8_t  Radio_GetCurrentCr(void)    { return current_cr;    }
int8_t   Radio_GetCurrentTxPow(void) { return current_tx_pow; }

/* ─── HAL SUBGHZ Interrupt Callbacks ────────────────────────────────────── */
/*
 * The SUBGHZ HAL calls HAL_SUBGHZ_IRQHandler() from the
 * SUBGHZ_Radio_IRQHandler() ISR (defined in stm32wlxx_it.c).
 * HAL then calls HAL_SUBGHZ_TxCpltCallback / RxCpltCallback / ErrorCallback
 * based on the IRQ flags it reads internally.
 *
 * We read the IRQ status register ourselves here because the HAL's callback
 * design for STM32WL may vary by CubeMX version.  Reading manually is robust.
 */

void HAL_SUBGHZ_TxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz_arg)
{
    (void)hsubghz_arg;
    RadioState = RADIO_TX_DONE;
}

void HAL_SUBGHZ_RxCpltCallback(SUBGHZ_HandleTypeDef *hsubghz_arg)
{
    (void)hsubghz_arg;
    /* Read RSSI / SNR */
    uint8_t pktStatus[3] = {0};
    HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETSTATUS, pktStatus, 3);
    /* LoRa packet status: [0]=RssiPkt, [1]=SnrPkt, [2]=SignalRssiPkt */
    LastRssi = -(int16_t)pktStatus[0] / 2;
    LastSnr  = (int8_t)pktStatus[1] / 4;
    RadioState = RADIO_RX_DONE;
}

void HAL_SUBGHZ_CRCErrorCallback(SUBGHZ_HandleTypeDef *hsubghz_arg)
{
    (void)hsubghz_arg;
    RadioState = RADIO_RX_ERROR;
}

void HAL_SUBGHZ_RxTxTimeoutCallback(SUBGHZ_HandleTypeDef *hsubghz_arg)
{
    (void)hsubghz_arg;
    if (RadioState == RADIO_TX) {
        RadioState = RADIO_TX_TIMEOUT;
    } else {
        RadioState = RADIO_RX_TIMEOUT;
    }
}
