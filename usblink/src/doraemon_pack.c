#include "doraemon_pack.h"

union u64u8_u{
	uint64_t n;
	uint8_t a[8];
};

union u32u8_u{
	uint32_t n;
	uint8_t a[4];
};

union u16u8_u{
	uint16_t n;
	uint8_t a[2];
};

uint64_t dp_u8_2_u64_lsb(uint8_t *dat)
{
	union u64u8_u tmp;

	tmp.a[0] = dat[0];
	tmp.a[1] = dat[1];
	tmp.a[2] = dat[2];
	tmp.a[3] = dat[3];
	tmp.a[4] = dat[4];
	tmp.a[5] = dat[5];
	tmp.a[6] = dat[6];
	tmp.a[7] = dat[7];

	return tmp.n;
}

uint64_t dp_u8_2_u64_msb(uint8_t *dat)
{
	union u64u8_u tmp;

	tmp.a[0] = dat[7];
	tmp.a[1] = dat[6];
	tmp.a[2] = dat[5];
	tmp.a[3] = dat[4];
	tmp.a[4] = dat[3];
	tmp.a[5] = dat[2];
	tmp.a[6] = dat[1];
	tmp.a[7] = dat[0];

	return tmp.n;
}

uint32_t dp_u8_2_u32_lsb(uint8_t *dat)
{
	union u32u8_u tmp;

	tmp.a[0] = dat[0];
	tmp.a[1] = dat[1];
	tmp.a[2] = dat[2];
	tmp.a[3] = dat[3];

	return tmp.n;
}

uint32_t dp_u8_2_u32_msb(uint8_t *dat)
{
	union u32u8_u tmp;

	tmp.a[0] = dat[3];
	tmp.a[1] = dat[2];
	tmp.a[2] = dat[1];
	tmp.a[3] = dat[0];

	return tmp.n;
}

uint16_t dp_u8_2_u16_lsb(uint8_t *dat)
{
	union u16u8_u tmp;

	tmp.a[0] = dat[0];
	tmp.a[1] = dat[1];

	return tmp.n;
}

uint16_t dp_u8_2_u16_msb(uint8_t *dat)
{
	union u16u8_u tmp;

	tmp.a[0] = dat[1];
	tmp.a[1] = dat[0];

	return tmp.n;
}






