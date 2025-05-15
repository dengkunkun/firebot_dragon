#ifndef __DORAEMON_PACK_H_
#define __DORAEMON_PACK_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include "stdint.h"
#include "stddef.h"

#define DP_GET_U64(x) *((uint64_t *)(&x))
#define DP_GET_U32(x) *((uint32_t *)(&x))
#define DP_GET_U16(x) *((uint16_t *)(&x))
#define DP_GET_U8(x) *((uint8_t *)(&x))

#define DP_UINT64_B7(x) ((uint8_t)(((x) >> 56) & 0x00000000000000ff))
#define DP_UINT64_B6(x) ((uint8_t)(((x) >> 48) & 0x00000000000000ff))
#define DP_UINT64_B5(x) ((uint8_t)(((x) >> 40) & 0x00000000000000ff))
#define DP_UINT64_B4(x) ((uint8_t)(((x) >> 32) & 0x00000000000000ff))
#define DP_UINT64_B3(x) ((uint8_t)(((x) >> 24) & 0x00000000000000ff))
#define DP_UINT64_B2(x) ((uint8_t)(((x) >> 16) & 0x00000000000000ff))
#define DP_UINT64_B1(x) ((uint8_t)(((x) >> 8) & 0x00000000000000ff))
#define DP_UINT64_B0(x) ((uint8_t)((x) & 0x00000000000000ff))

#define DP_UINT32_B3(x) ((uint8_t)(((x) >> 24) & 0x000000ff))
#define DP_UINT32_B2(x) ((uint8_t)(((x) >> 16) & 0x000000ff))
#define DP_UINT32_B1(x) ((uint8_t)(((x) >> 8) & 0x000000ff))
#define DP_UINT32_B0(x) ((uint8_t)((x) & 0x000000ff))

#define DP_UINT16_H(x) ((uint8_t)(((x) >> 8) & 0x00ff))
#define DP_UINT16_L(x) ((uint8_t)((x) & 0x00ff))

uint64_t dp_u8_2_u64_msb(uint8_t *dat);
uint64_t dp_u8_2_u64_lsb(uint8_t *dat);
uint32_t dp_u8_2_u32_msb(uint8_t *dat);
uint32_t dp_u8_2_u32_lsb(uint8_t *dat);
uint16_t dp_u8_2_u16_msb(uint8_t *dat);
uint16_t dp_u8_2_u16_lsb(uint8_t *dat);

#if defined(__cplusplus)
}
#endif

#endif



