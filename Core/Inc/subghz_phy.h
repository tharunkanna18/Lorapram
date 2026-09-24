/**
 ******************************************************************************
 * @file    subghz_phy.h
 * @brief   SubGHz radio (SX126x) driver wrapper for PingPong example.
 *          Configures LoRa modulation, exposes Tx / Rx helpers and the
 *          interrupt callbacks required by the HAL_SUBGHZ driver.
 ******************************************************************************
 */
#ifndef SUBGHZ_PHY_H
#define SUBGHZ_PHY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ─── LoRa RF parameters ─────────────────────────────────────────────────── */
/* Frequency and power will be configured dynamically */
/* These are now imported from pingpong.h */
#include "pingpong.h"

/* ─── Payload ────────────────────────────────────────────────────────────── */
#define PINGPONG_BUFFER_SIZE       64           /* custom packet format      */

/* ─── Timeout ────────────────────────────────────────────────────────────── */
#define TX_TIMEOUT_VALUE           3000U        /* ms            */

/* ─── Radio state ────────────────────────────────────────────────────────── */
typedef enum {
    RADIO_LOWPOWER = 0,
    RADIO_RX,
    RADIO_TX,
    RADIO_TX_DONE,
    RADIO_RX_DONE,
    RADIO_TX_TIMEOUT,
    RADIO_RX_TIMEOUT,
    RADIO_RX_ERROR,
} Radio_States_t;

extern volatile Radio_States_t RadioState;
extern SUBGHZ_HandleTypeDef     hsubghz;

/* ─── API ────────────────────────────────────────────────────────────────── */
void Radio_Init(void);
void Radio_SetChannel(uint32_t freq);
void Radio_SetTxConfig(void);
void Radio_SetRxConfig(void);
void Radio_Send(uint8_t *buffer, uint8_t size);
void Radio_Rx(uint32_t timeout);
void Radio_Sleep(void);
void Radio_ApplyMode(uint8_t mode);

/* Last received packet info */
int16_t  Radio_GetLastRssi(void);
int8_t   Radio_GetLastSnr(void);

/* Current RF config getters */
uint32_t Radio_GetCurrentFreq(void);
uint8_t  Radio_GetCurrentSf(void);
uint8_t  Radio_GetCurrentBw(void);
uint8_t  Radio_GetCurrentCr(void);
int8_t   Radio_GetCurrentTxPow(void);

#ifdef __cplusplus
}
#endif

#endif /* SUBGHZ_PHY_H */
