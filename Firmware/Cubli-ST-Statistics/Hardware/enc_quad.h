#ifndef __ENC_QUAD_H__
#define __ENC_QUAD_H__
#include "main.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#ifndef ENC_USE_HW_QUAD
#define ENC_USE_HW_QUAD 0  /* 0=I2C absolute, 1=TIM2/TIM5/TIM8 hardware quadrature */
#endif
#if ENC_USE_HW_QUAD != 0 && ENC_USE_HW_QUAD != 1
#error ENC_USE_HW_QUAD_must_be_0_or_1
#endif

/* MT6701 AB pulses/revolution BEFORE x4 decoding. Standard-module working
 * assumption: 1024 PPR -> 4096 counter steps/rev. This does not program EEPROM.
 * Override the common PPR or individual axes to match the physical module. */
#ifndef ENC_QUAD_PPR
#define ENC_QUAD_PPR 1024U
#endif
/* Legacy common CPR override remains supported; prefer configuring PPR. */
#ifndef ENC_QUAD_COUNTS_PER_REV
#define ENC_QUAD_COUNTS_PER_REV (4U * ENC_QUAD_PPR)
#endif
#if ENC_QUAD_COUNTS_PER_REV < 4 || ENC_QUAD_COUNTS_PER_REV > 4096 || (ENC_QUAD_COUNTS_PER_REV % 4) != 0
#error ENC_QUAD_COUNTS_PER_REV_must_be_4_times_PPR_1_to_1024
#endif
#ifndef ENC_QUAD_PPR_M1
#define ENC_QUAD_PPR_M1 (ENC_QUAD_COUNTS_PER_REV / 4U)
#endif
#ifndef ENC_QUAD_PPR_M2
#define ENC_QUAD_PPR_M2 (ENC_QUAD_COUNTS_PER_REV / 4U)
#endif
#ifndef ENC_QUAD_PPR_M3
#define ENC_QUAD_PPR_M3 (ENC_QUAD_COUNTS_PER_REV / 4U)
#endif
#if ENC_QUAD_PPR_M1 < 1 || ENC_QUAD_PPR_M1 > 1024 || ENC_QUAD_PPR_M2 < 1 || ENC_QUAD_PPR_M2 > 1024 || ENC_QUAD_PPR_M3 < 1 || ENC_QUAD_PPR_M3 > 1024
#error MT6701_PPR_must_be_in_1_to_1024
#endif
#define ENC_QUAD_CPR_M1 (4U * ENC_QUAD_PPR_M1)
#define ENC_QUAD_CPR_M2 (4U * ENC_QUAD_PPR_M2)
#define ENC_QUAD_CPR_M3 (4U * ENC_QUAD_PPR_M3)

/* Netlist: M1 PA5/PB3=TIM2 AF1, M2 PA0/PA1=TIM5 AF2,
 *          M3 PC6/PC7=TIM8 AF3. Only A/B are wired; no Z index is read.
 * TIM5 is reserved for M2 in QUAD builds; micros() uses TIM7 instead. */
void ENC_Quad_Init(void);
uint16_t ENC_Quad_CountsPerRev(uint8_t idx);
uint16_t ENC_CountsPerRev(uint8_t idx); /* active method: I2C=16384 or per-axis QUAD CPR */
uint8_t ENC_GetAngle(uint8_t idx, int16_t *raw, float *deg);
uint8_t ENC_I2C_GetAngle(uint8_t idx, int16_t *raw, float *deg);
/* 0=counter configured/running, 1=invalid argument/uninitialized/bad registers.
 * AB-only hardware cannot distinguish a stationary encoder from a disconnected
 * module. Counter availability is NOT a physical encoder-online diagnosis.
 * Angle is relative to initialization; FOC electrical alignment is mandatory. */
uint8_t ENC_Quad_GetAngle(uint8_t idx, int16_t *raw, float *deg);
#ifdef __cplusplus
}
#endif
#endif
