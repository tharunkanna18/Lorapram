/**
 ******************************************************************************
 * @file    pingpong.h
 * @brief   PingPong application layer header.
 *          Manages the MASTER (sends PING) / SLAVE (replies PONG) role,
 *          the protocol state-machine, and UART debug output.
 ******************************************************************************
 */
#ifndef PINGPONG_H
#define PINGPONG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ─── Board role ─────────────────────────────────────────────────────────── */
/*
 * Define MASTER_BOARD in your project preprocessor symbols (or uncomment
 * below) for the board that sends PING first.
 * Leave undefined (or comment out) for the board that replies with PONG.
 */
#define MASTER_BOARD

/* ─── LoRa Settings (Defaults moved to subghz_phy.c) ─────────────────────── */
#define LORA_PREAMBLE_LENGTH       8         /* Same for Tx and Rx */
#define LORA_FIX_LENGTH_PAYLOAD    0         /* Variable    */
#define LORA_IQ_INVERSION          0         /* False       */

/* ─── Timings ────────────────────────────────────────────────────────────── */
#define RX_TIMEOUT_VALUE           3000U     /* ms            */

/* ─── API ────────────────────────────────────────────────────────────────── */
void PingPong_Init(void);
void PingPong_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* PINGPONG_H */
