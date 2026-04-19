#ifndef CRC_H
#define CRC_H

#include <stdint.h>

// CRC8 计算（多项式 G(x)=x8+x5+x4+1）
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
// 验证帧头 CRC8
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
// 在数据末尾追加 CRC8
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

// CRC16 计算（多项式 0x8005，初始值 0xFFFF）
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
// 验证整包 CRC16
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
// 在数据末尾追加 CRC16（小端）
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
