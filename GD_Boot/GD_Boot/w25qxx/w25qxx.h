#ifndef W25QXX_H_
#define W25QXX_H_

#include "projectconfig.h"
#include "core/ssp/ssp.h"
#include "core/systick/systick.h"

/* Hardware Specific Defines */
#define SPI_PORT           0
#define W25Q_PAGESIZE      256
#define W25Q_SECTORSIZE    4096

/* List of Avaliable Commands */
#define W25Q_WR_ENABLE    0x06
#define W25Q_WR_DISABLE   0x04
#define W25Q_RD_STAT1     0x05
#define W25Q_RD_STAT2     0x35
#define W25Q_RD_STAT3     0x15
#define W25Q_WR_STAT1     0x01
#define W25Q_WR_STAT2     0x31
#define W25Q_WR_STAT3     0x11
#define W25Q_RD_PAGE      0x03
#define W25Q_WR_PAGE      0x02
#define W25Q_ERASE_SCT    0x20
#define W25Q_ERASE_BLK    0x52
#define W25Q_ERASE_CHP    0xC7
#define W25Q_PWR_DOWN     0xB9
#define W25Q_PWR_UP       0xAB
#define W25Q_RD_ID        0x90
#define W25Q_DEVICE_ID    0xEF

uint8_t w25qReadDeviceID();
void w25qInit();
void w25qReadPage(uint8_t rdBuf[],  uint32_t address);
uint8_t w25qReadByte(uint32_t address, uint8_t offset);
void w25qWritePage(uint8_t wrBuf[], uint32_t address, uint16_t length);
void w25qEraseSector(uint32_t address);
void w25qEraseBlock(uint32_t address);
void w25qEraseChip();

#endif
