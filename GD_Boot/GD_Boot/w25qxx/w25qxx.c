/**************************************************************************/
/*!
    @file     w25qxx.c
    @author   Matt Epperson

    @section Overview

    This file allows working with winbond w25qxx class of flash memory. Allows
    reading, writing, and erasing.
*/
/**************************************************************************/


#include "w25qxx.h"

/**************************************************************************/
/*!
	@brief Enables/Disables the write bit in Status Register 1
*/
/**************************************************************************/
static void _w25qWriteEnableLatch(bool value)
{
   uint8_t command = W25Q_WR_ENABLE;

   if (!value)
      command = W25Q_WR_DISABLE;

   ssp0Select();
   sspSend(SPI_PORT, &command, 1);
   ssp0Deselect();
}


/**************************************************************************/
/*!
	@brief Powers up or down the flash memory chip. Power down mode
          can be used to save power. The only command that is accepted
          during power down is the power up command.

   NOTE: Could probably make _w25qSendCommand(cmd, buffer, len)
*/
/**************************************************************************/
static void _w25qSetPowerMode(uint8_t cmd)
{
   uint8_t command = cmd;

   ssp0Select();
   sspSend(SPI_PORT, &command, 1);
   ssp0Deselect();
}


/**************************************************************************/
/*!
	@brief Read a status register (1-3) and return the value

   @param[in] cmd
              The cmd to send

   @return buf
           The value read from the status register
*/
/**************************************************************************/
static uint8_t _w25qReadStatusReg(uint8_t cmd)
{
	uint8_t buf = cmd;

	ssp0Select();
	sspSend(SPI_PORT, &buf, 1);
	sspReceive(SPI_PORT, &buf, 1);
	ssp0Deselect();

	return buf;
}


/**************************************************************************/
/*!
	@brief Polls the busy bit continuesly untill it has been cleared
*/
/**************************************************************************/
static void _w25qPollBusyBit()
{
   while ((_w25qReadStatusReg(W25Q_RD_STAT1) & 0x01) != 0);
}


/**************************************************************************/
/*!
	@brief Write a byte of data to a status register (1-3)
*/
/**************************************************************************/
static void _w25qWriteStatusReg(uint8_t cmd, uint8_t data)
{
	uint8_t command[2] = {cmd, data};

   _w25qPollBusyBit();
   _w25qWriteEnableLatch(true);

   ssp0Select();
   sspSend(SPI_PORT, command, 2);
   ssp0Deselect();
}


/**************************************************************************/
/*!
   @brief Initializes the SPI and Winbond device

   1) Set up SPI to mode 3
   2) Power up the chip just in case
   3) Disable Enable latch also just in case
   4) Set the output power to 25% (Default)
*/
/**************************************************************************/
void w25qInit()
{
   sspInit(0, sspClockPolarity_Low, sspClockPhase_RisingEdge);
   systickDelay(CFG_SYSTICK_100MS_DELAY);

   _w25qSetPowerMode(W25Q_PWR_UP);
   systickDelay(CFG_SYSTICK_100MS_DELAY);

   _w25qWriteEnableLatch(false);
   systickDelay(CFG_SYSTICK_100MS_DELAY);

   _w25qWriteStatusReg(W25Q_WR_STAT3, 0x60);
   systickDelay(CFG_SYSTICK_100MS_DELAY);
}


/**************************************************************************/
/*!
   @brief Reads the manufactures ID from the W25QXX

   @return  command[0]
            The result from the read operation. (Expect 0xEF)
*/
/**************************************************************************/
uint8_t w25qReadDeviceID()
{
	uint8_t command[4] = {W25Q_RD_ID, 0, 0, 0};

   _w25qPollBusyBit();

	ssp0Select();
	sspSend(0, command, 4);
	sspReceive(0, command, 2);
	ssp0Deselect();

	return command[0];
}

/*********************************d*****************************************/
/*!
   @brief Reads a Page (256 Bytes) of data

   @param[in]  rdBuf
 			      Buffer array to store data

   @param[in]  address
               Address to start reading from. Only uses first three bytes
*/
/**************************************************************************/
void w25qReadPage(uint8_t rdBuf[], uint32_t address)
{
   uint8_t command[4] = { W25Q_RD_PAGE,
                          address>>16 & 0xFF,
                          address>>8 & 0xFF,
                          address & 0xFF};

   _w25qPollBusyBit();

	ssp0Select();
   sspSend(SPI_PORT, command, 4);
   sspReceive(SPI_PORT, rdBuf, W25Q_PAGESIZE);
   ssp0Deselect();
}


/**************************************************************************/
/*!
   @brief Reads a single byte from a full page. Full page is read
          since the hardware enforces this anyways.

   NOTE: This is really just a convience function that SHOULD be removed
   if code size becomes an issue.
*/
/**************************************************************************/
uint8_t w25qReadByte(uint32_t address, uint8_t offset)
{
   uint8_t buffer[W25Q_PAGESIZE];
   w25qReadPage(buffer, address);
   return buffer[offset];
}


/**************************************************************************/
/*!
   @brief Writes a Page (256 Bytes) of data to the memory chip. Note that
   in order to write data the section must first be erased (set to 0xFF).
   Writing beyond the start of a page will cause it to roll over to beginning
   of the page and overwrite data there.

   @param[in]  wrBuf
 				   Buffer array to write to flash

   @param[in]  address
               Address to start writing at. Does not need to be page
               aligned.
*/
/**************************************************************************/
void w25qWritePage(uint8_t wrBuf[], uint32_t address, uint16_t length)
{
   uint8_t command[4] = { W25Q_WR_PAGE,
                          address>>16 & 0xFF,
                          address>>8 & 0xFF,
                          address & 0xFF };

   _w25qPollBusyBit();
   _w25qWriteEnableLatch(true);

   ssp0Select();
   sspSend(SPI_PORT, command, 4);
   sspSend(SPI_PORT, wrBuf, length);
   ssp0Deselect();
}

/**************************************************************************/
/*!
   @brief Erases a Sector(4k) of memory. Must be done in order to rewrite

   @param[in]  address
               Address to the start of the sector to erase.
*/
/**************************************************************************/
void w25qEraseSector(uint32_t address)
{
   uint8_t command[4] = { W25Q_ERASE_SCT,
                          address>>16 & 0xFF,
                          address>>8 & 0xFF,
                          address & 0xFF };

   _w25qPollBusyBit();
   _w25qWriteEnableLatch(true);

   ssp0Select();
   sspSend(SPI_PORT, command, 4);
   ssp0Deselect();
}

/**************************************************************************/
/*!
   @brief Erases a Block(32k) of memory. Must be done in order to rewrite

   @param[in]  address
               Address to the start of sector to erase
*/
/**************************************************************************/
void w25qEraseBlock(uint32_t address)
{
   uint8_t command[4] = { W25Q_ERASE_BLK,
                          address>>16 & 0xFF,
                          address>>8 & 0xFF,
                          address & 0xFF };

   _w25qPollBusyBit();
   _w25qWriteEnableLatch(true);

   ssp0Select();
   sspSend(SPI_PORT, command, 4);
   ssp0Deselect();
}

/**************************************************************************/
/*!
   @brief Erases entire chip

   @param[in]  address
               Address to the start of sector to erase
*/
/**************************************************************************/
void w25qEraseChip()
{
   uint8_t command = W25Q_ERASE_CHP;

   _w25qPollBusyBit();
   _w25qWriteEnableLatch(true);

   ssp0Select();
   sspSend(SPI_PORT, &command, 1);
   ssp0Deselect();
}
