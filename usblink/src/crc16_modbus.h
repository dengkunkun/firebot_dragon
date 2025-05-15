#ifndef __CRC16_MODBUS_H_
#define __CRC16_MODBUS_H_

#if defined (__cplusplus)
extern "C" {
#endif

unsigned short calc_modbus_crc16(const unsigned char *p, int n);

#if defined (__cplusplus)
}
#endif

#endif

