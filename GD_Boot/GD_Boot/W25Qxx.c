/* --------------------------------------------------------------------------------------------------------------------------------------- */
/* Includes ------------------------------------------------------------------------------------------------------------------------------ */
/* ....................................................................................................................................... */

#include "W25Qxx.h"
//#include "io.h"
//#include "Device/SPI1/SPI1.h"
//#include "pcb_config.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "main.h"

/* --------------------------------------------------------------------------------------------------------------------------------------- */
/* Defines ------------------------------------------------------------------------------------------------------------------------------- */
/* ....................................................................................................................................... */

void Delay_ms(uint8_t ms)
{	
	while (ms > 0)
	{
		ms++;
	}
}

#define		W25qxx_Delay(delay)							HAL_Delay(delay)
#define		W25QXX_Enable()								(HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET))
#define		W25QXX_Disable()							(HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_SET))



/* --------------------------------------------------------------------------------------------------------------------------------------- */
/* Public -------------------------------------------------------------------------------------------------------------------------------- */
/* ....................................................................................................................................... */
SPI_HandleTypeDef hspi_mem;
w25qxx_t	w25qxx;

bool W25qxx_Init(void);
void W25qxx_ReadOnlyEnable(void);

void W25qxx_EraseChip(void);
void W25qxx_EraseSector(uint32_t SectorAddr);
void W25qxx_EraseBlock(uint32_t BlockAddr);

uint32_t W25qxx_PageToSector(uint32_t PageAddress);
uint32_t W25qxx_PageToBlock(uint32_t PageAddress);
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress);
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress);
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress);

bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize);
bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize);
bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize);

void W25qxx_WriteByte(uint8_t pBuffer, uint32_t Bytes_Address);
void W25qxx_WritePage(uint8_t* pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize);
void W25qxx_WriteSector(uint8_t* pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize);
void W25qxx_WriteBlock(uint8_t* pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize);

void W25qxx_ReadByte(uint8_t* pBuffer, uint32_t Bytes_Address);
void W25qxx_ReadBytes(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
void W25qxx_ReadPage(uint8_t* pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);
void W25qxx_ReadSector(uint8_t* pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);
void W25qxx_ReadBlock(uint8_t* pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t	NumByteToRead_up_to_BlockSize);

/* --------------------------------------------------------------------------------------------------------------------------------------- */
/* Private ------------------------------------------------------------------------------------------------------------------------------- */
/* ....................................................................................................................................... */


static uint32_t W25qxx_ReadID(void);
static void W25qxx_ReadUniqID(void);
static void W25qxx_WriteEnable(void);
static void W25qxx_WriteDisable(void);
static void W25qxx_WriteAddress(uint32_t Page_Address);
static uint8_t W25qxx_ReadStatusRegister(uint8_t	SelectStatusRegister_1_2_3);
static void W25qxx_WaitForWriteEnd(void);

/* --------------------------------------------------------------------------------------------------------------------------------------- */
/* Public Implementation ----------------------------------------------------------------------------------------------------------------- */
/* ....................................................................................................................................... */

//###################################################################################################################

uint8_t SPI_TxRx(uint8_t Data)
{
	uint8_t	ret;
	HAL_SPI_TransmitReceive(&hspi_mem, &Data, &ret, 1, 10);
	return ret;	
}

void SPI_Rx(uint8_t* pBuffer, uint16_t size, uint32_t Timeout)
{
	HAL_SPI_Receive(&hspi_mem, pBuffer, size, Timeout);
}


void SPI_Tx(uint8_t * pBuffer, uint16_t Size, uint32_t Timeout)
{
	HAL_SPI_Transmit(&hspi_mem, pBuffer, Size, Timeout);
}
bool W25qxx_Init(void)
{
	SPI_HandleTypeDef* xSPI = &hspi_mem;

	xSPI->Instance = SPI1;
	xSPI->Init.Mode = SPI_MODE_MASTER;
	xSPI->Init.Direction = SPI_DIRECTION_2LINES;
	xSPI->Init.DataSize = SPI_DATASIZE_8BIT;
	xSPI->Init.CLKPolarity = SPI_POLARITY_LOW;
	xSPI->Init.CLKPhase = SPI_PHASE_1EDGE;
	xSPI->Init.NSS = SPI_NSS_SOFT;
	xSPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	xSPI->Init.FirstBit = SPI_FIRSTBIT_MSB;
	xSPI->Init.TIMode = SPI_TIMODE_DISABLE;
	xSPI->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	xSPI->Init.CRCPolynomial = 7;
	
	if (HAL_SPI_Init(xSPI) != HAL_OK)
	{
		Error_Handler();
	}

	W25QXX_Disable();

	GPIO_InitTypeDef init = {
		 .Pin = GPIO_PIN_9,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_HIGH,
	};

	//iHAL_IO_Config((IO_t)IO_MEM_CS1, &init);
	HAL_GPIO_Init(GPIOB, &init);

	w25qxx.Lock = 1;

	while (HAL_GetTick() < 100)	
		W25qxx_Delay(1);

	uint32_t id = W25qxx_ReadID();

	switch (id & 0x0000FFFF)
	{
	case 0x401A:	// 	w25q512
		w25qxx.ID = W25Q512;
		w25qxx.BlockCount = 1024;
		break;
	case 0x4019:	// 	w25q256
		w25qxx.ID = W25Q256;
		w25qxx.BlockCount = 512;
		break;
	case 0x4018:	// 	w25q128
		w25qxx.ID = W25Q128;
		w25qxx.BlockCount = 256;
		break;
	case 0x4017:	//	w25q64
		w25qxx.ID = W25Q64;
		w25qxx.BlockCount = 128;
		break;
	case 0x4016:	//	w25q32
		w25qxx.ID = W25Q32;
		w25qxx.BlockCount = 64;
		break;
	case 0x4015:	//	w25q16
		w25qxx.ID = W25Q16;
		w25qxx.BlockCount = 32;
		break;
	case 0x4014:	//	w25q80
		w25qxx.ID = W25Q80;
		w25qxx.BlockCount = 16;
		break;
	case 0x4013:	//	w25q40
		w25qxx.ID = W25Q40;
		w25qxx.BlockCount = 8;
		break;
	case 0x4012:	//	w25q20
		w25qxx.ID = W25Q20;
		w25qxx.BlockCount = 4;
		break;
	case 0x4011:	//	w25q10
		w25qxx.ID = W25Q10;
		w25qxx.BlockCount = 2;
		break;
	default:
		w25qxx.Lock = 0;
		return false;
	}

	w25qxx.PageSize = 256;
	w25qxx.SectorSize = 0x1000;
	w25qxx.SectorCount = w25qxx.BlockCount * 16;
	w25qxx.PageCount = (w25qxx.SectorCount * w25qxx.SectorSize) / w25qxx.PageSize;
	w25qxx.BlockSize = w25qxx.SectorSize * 16;
	w25qxx.CapacityInKiloByte = (w25qxx.SectorCount * w25qxx.SectorSize) / 1024;
	W25qxx_ReadUniqID();
	W25qxx_ReadStatusRegister(1);
	W25qxx_ReadStatusRegister(2);
	W25qxx_ReadStatusRegister(3);

	w25qxx.Lock = 0;
	return true;
}

//###################################################################################################################
void W25qxx_ReadOnlyEnable(void)
{
	W25qxx_WriteDisable();
}

//###################################################################################################################
void W25qxx_EraseChip(void)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);
	w25qxx.Lock = 1;

	W25qxx_WriteEnable();

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_CHIP_ERASE);
	}
	W25QXX_Disable();

	W25qxx_WaitForWriteEnd();

	//W25qxx_Delay(10);
	w25qxx.Lock = 0;
}

//###################################################################################################################
void W25qxx_EraseSector(uint32_t SectorAddr)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);
	w25qxx.Lock = 1;

	W25qxx_WaitForWriteEnd();

	SectorAddr = SectorAddr * w25qxx.SectorSize;

	W25qxx_WriteEnable();

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_SECTOR_ERASE_4KB);
		W25qxx_WriteAddress(SectorAddr);
	}
	W25QXX_Disable();

	W25qxx_WaitForWriteEnd();

	//W25qxx_Delay(1);
	w25qxx.Lock = 0;
}

//###################################################################################################################
void W25qxx_EraseBlock(uint32_t BlockAddr)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);
	w25qxx.Lock = 1;

	W25qxx_WaitForWriteEnd();

	BlockAddr = BlockAddr * w25qxx.SectorSize * 16;

	W25qxx_WriteEnable();

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_BLOCK_ERASE_64KB);

		W25qxx_WriteAddress(BlockAddr);
	}
	W25QXX_Disable();

	W25qxx_WaitForWriteEnd();

	//W25qxx_Delay(1);
	w25qxx.Lock = 0;
}

//###################################################################################################################

uint32_t W25qxx_PageToSector(uint32_t PageAddress) { return ((PageAddress * w25qxx.PageSize) / w25qxx.SectorSize); }
uint32_t W25qxx_PageToBlock(uint32_t PageAddress) { return ((PageAddress * w25qxx.PageSize) / w25qxx.BlockSize); }
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress) { return ((SectorAddress * w25qxx.SectorSize) / w25qxx.BlockSize); }
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress) { return (SectorAddress * w25qxx.SectorSize) / w25qxx.PageSize; }
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress) { return (BlockAddress * w25qxx.BlockSize) / w25qxx.PageSize; }

//###################################################################################################################




bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);
	w25qxx.Lock = 1;

	if (((NumByteToCheck_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || (NumByteToCheck_up_to_PageSize == 0))
		NumByteToCheck_up_to_PageSize = w25qxx.PageSize - OffsetInByte;

	uint8_t	pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;

	for (i = OffsetInByte; i < w25qxx.PageSize; i += sizeof(pBuffer))
	{
		W25QXX_Enable();
		{
			WorkAddress = (i + Page_Address * w25qxx.PageSize);
			SPI_TxRx(COMMAND_FAST_READ);
			W25qxx_WriteAddress(WorkAddress);
			SPI_TxRx(COMMAND_DUMMY_BYTE_0);
			SPI_Rx(pBuffer, sizeof(pBuffer), 100);
			
			
		}
		W25QXX_Disable();

		for (uint8_t x = 0; x < sizeof(pBuffer); x++) if (pBuffer[x] != 0xFF) goto NOT_EMPTY;
	}

	if ((w25qxx.PageSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.PageSize; i++)
		{
			W25QXX_Enable();
			{
				WorkAddress = (i + Page_Address * w25qxx.PageSize);
				SPI_TxRx(COMMAND_FAST_READ);
				W25qxx_WriteAddress(WorkAddress);
				SPI_TxRx(COMMAND_DUMMY_BYTE_0);
				SPI_Rx(pBuffer, 1, 100);
			}
			W25QXX_Disable();

			if (pBuffer[0] != 0xFF)	goto NOT_EMPTY;
		}
	}

	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:

	w25qxx.Lock = 0;
	return false;
}

//###################################################################################################################
bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);
	w25qxx.Lock = 1;

	if ((NumByteToCheck_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToCheck_up_to_SectorSize == 0))
		NumByteToCheck_up_to_SectorSize = w25qxx.SectorSize;

	uint8_t	pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;

	for (i = OffsetInByte; i < w25qxx.SectorSize; i += sizeof(pBuffer))
	{
		W25QXX_Enable();
		{
			WorkAddress = (i + Sector_Address * w25qxx.SectorSize);

			SPI_TxRx(COMMAND_FAST_READ);

			W25qxx_WriteAddress(WorkAddress);

			SPI_TxRx(COMMAND_DUMMY_BYTE_0);

			SPI_Rx(pBuffer, sizeof(pBuffer), 100);
		}
		W25QXX_Disable();

		for (uint8_t x = 0; x < sizeof(pBuffer); x++) if (pBuffer[x] != 0xFF) goto NOT_EMPTY;
	}

	if ((w25qxx.SectorSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.SectorSize; i++)
		{
			W25QXX_Enable();
			{
				WorkAddress = (i + Sector_Address * w25qxx.SectorSize);

				SPI_TxRx(COMMAND_FAST_READ);

				W25qxx_WriteAddress(WorkAddress);

				SPI_TxRx(COMMAND_DUMMY_BYTE_0);

				SPI_Rx(pBuffer, 1, 100);
			}
			W25QXX_Disable();

			if (pBuffer[0] != 0xFF)	goto NOT_EMPTY;
		}
	}

	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:

	w25qxx.Lock = 0;
	return false;
}

//###################################################################################################################
bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);
	w25qxx.Lock = 1;

	if ((NumByteToCheck_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToCheck_up_to_BlockSize == 0))
		NumByteToCheck_up_to_BlockSize = w25qxx.BlockSize;

	uint8_t	pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;

	for (i = OffsetInByte; i < w25qxx.BlockSize; i += sizeof(pBuffer))
	{
		W25QXX_Enable();
		{
			WorkAddress = (i + Block_Address * w25qxx.BlockSize);

			SPI_TxRx(COMMAND_FAST_READ);

			W25qxx_WriteAddress(WorkAddress);

			SPI_TxRx(COMMAND_DUMMY_BYTE_0);

			SPI_Rx(pBuffer, sizeof(pBuffer), 100);
		}
		W25QXX_Disable();

		for (uint8_t x = 0; x < sizeof(pBuffer); x++) if (pBuffer[x] != 0xFF) goto NOT_EMPTY;
	}

	if ((w25qxx.BlockSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.BlockSize; i++)
		{
			W25QXX_Enable();
			{
				WorkAddress = (i + Block_Address * w25qxx.BlockSize);
				SPI_TxRx(COMMAND_FAST_READ);

				W25qxx_WriteAddress(WorkAddress);

				SPI_TxRx(COMMAND_DUMMY_BYTE_0);

				SPI_Rx(pBuffer, 1, 100);
			}
			W25QXX_Disable();
			if (pBuffer[0] != 0xFF)	goto NOT_EMPTY;
		}
	}

	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:

	w25qxx.Lock = 0;
	return false;
}

//###################################################################################################################
void W25qxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);

	w25qxx.Lock = 1;

	W25qxx_WaitForWriteEnd();

	W25qxx_WriteEnable();

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_PAGE_PROGRAM);

		W25qxx_WriteAddress(WriteAddr_inBytes);

		SPI_TxRx(pBuffer);
	}
	W25QXX_Disable();

	W25qxx_WaitForWriteEnd();

	w25qxx.Lock = 0;
}

//###################################################################################################################
void W25qxx_WritePage(uint8_t* pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);

	w25qxx.Lock = 1;

	if (((OffsetInByte + NumByteToWrite_up_to_PageSize) > w25qxx.PageSize) || (NumByteToWrite_up_to_PageSize == 0))
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;

	//	if ((OffsetInByte + NumByteToWrite_up_to_PageSize) > w25qxx.PageSize)
	//		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;

	W25qxx_WaitForWriteEnd();

	W25qxx_WriteEnable();

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_PAGE_PROGRAM);

		Page_Address = (Page_Address * w25qxx.PageSize) + OffsetInByte;

		W25qxx_WriteAddress(Page_Address);

		SPI_Tx(pBuffer, NumByteToWrite_up_to_PageSize, 100);
	}
	W25QXX_Disable();

	W25qxx_WaitForWriteEnd();

	//W25qxx_Delay(1);
	w25qxx.Lock = 0;
}

//###################################################################################################################
void W25qxx_WriteSector(uint8_t* pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize)
{
	if (OffsetInByte >= w25qxx.SectorSize) return;

	if ((NumByteToWrite_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToWrite_up_to_SectorSize == 0))
		NumByteToWrite_up_to_SectorSize = w25qxx.SectorSize;


	int32_t	BytesToWrite = ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize) ?
		w25qxx.SectorSize - OffsetInByte :
		NumByteToWrite_up_to_SectorSize;

	uint32_t StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	uint32_t LocalOffset = OffsetInByte % w25qxx.PageSize;

	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
}

//###################################################################################################################
void W25qxx_WriteBlock(uint8_t* pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t	NumByteToWrite_up_to_BlockSize)
{
	if (OffsetInByte >= w25qxx.BlockSize) return;

	if ((NumByteToWrite_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToWrite_up_to_BlockSize == 0))
		NumByteToWrite_up_to_BlockSize = w25qxx.BlockSize;

	int32_t BytesToWrite = ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize) ?
		w25qxx.BlockSize - OffsetInByte :
		NumByteToWrite_up_to_BlockSize;

	uint32_t StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	uint32_t LocalOffset = OffsetInByte % w25qxx.PageSize;

	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
}

//###################################################################################################################
void W25qxx_ReadByte(uint8_t* pBuffer, uint32_t Bytes_Address)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);

	w25qxx.Lock = 1;
	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_FAST_READ);

		W25qxx_WriteAddress(Bytes_Address);

		SPI_TxRx(COMMAND_DUMMY_BYTE_0);

		*pBuffer = SPI_TxRx(COMMAND_DUMMY_BYTE);
	}
	W25QXX_Disable();

	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadBytes(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);

	w25qxx.Lock = 1;

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_FAST_READ);

		W25qxx_WriteAddress(ReadAddr);

		SPI_TxRx(COMMAND_DUMMY_BYTE_0);

		SPI_Rx(pBuffer, NumByteToRead, 2000);
	}
	W25QXX_Disable();

	W25qxx_Delay(1);
	w25qxx.Lock = 0;
}

//###################################################################################################################
void W25qxx_ReadPage(uint8_t* pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
	while (w25qxx.Lock == 1) W25qxx_Delay(1);

	w25qxx.Lock = 1;
	if ((NumByteToRead_up_to_PageSize > w25qxx.PageSize) || (NumByteToRead_up_to_PageSize == 0))
		NumByteToRead_up_to_PageSize = w25qxx.PageSize;
	if ((OffsetInByte + NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
		NumByteToRead_up_to_PageSize = w25qxx.PageSize - OffsetInByte;

	Page_Address = Page_Address * w25qxx.PageSize + OffsetInByte;

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_FAST_READ);

		W25qxx_WriteAddress(Page_Address);

		SPI_TxRx(COMMAND_DUMMY_BYTE_0);

		SPI_Rx(pBuffer, NumByteToRead_up_to_PageSize, 100);
	}
	W25QXX_Disable();

	//W25qxx_Delay(1);
	//HAL_Delay(1);
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadSector(uint8_t* pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
	if (OffsetInByte >= w25qxx.SectorSize) return;

	if ((NumByteToRead_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToRead_up_to_SectorSize == 0))
		NumByteToRead_up_to_SectorSize = w25qxx.SectorSize;

	int32_t	BytesToRead = ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize) ?
		w25qxx.SectorSize - OffsetInByte :
		NumByteToRead_up_to_SectorSize;

	uint32_t StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	uint32_t LocalOffset = OffsetInByte % w25qxx.PageSize;

	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize;
		LocalOffset = 0;
	} while (BytesToRead > 0);
}

//###################################################################################################################
void W25qxx_ReadBlock(uint8_t* pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t	NumByteToRead_up_to_BlockSize)
{
	if (OffsetInByte >= w25qxx.BlockSize) return;

	if ((NumByteToRead_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToRead_up_to_BlockSize == 0))
		NumByteToRead_up_to_BlockSize = w25qxx.BlockSize;

	int32_t BytesToRead = ((OffsetInByte + NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize) ?
		w25qxx.BlockSize - OffsetInByte :
		NumByteToRead_up_to_BlockSize;

	uint32_t StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	uint32_t LocalOffset = OffsetInByte % w25qxx.PageSize;

	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize;
		LocalOffset = 0;
	} while (BytesToRead > 0);
}

/* --------------------------------------------------------------------------------------------------------------------------------------- */
/* Private Implementation ---------------------------------------------------------------------------------------------------------------- */
/* ....................................................................................................................................... */

//###################################################################################################################
static uint32_t W25qxx_ReadID(void)
{
	uint32_t temp = 0;

	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_READ_ID);
		temp |= SPI_TxRx(COMMAND_DUMMY_BYTE) << 16;
		temp |= SPI_TxRx(COMMAND_DUMMY_BYTE) << 8;
		temp |= SPI_TxRx(COMMAND_DUMMY_BYTE);
	}
	W25QXX_Disable();

	return temp;
}

//###################################################################################################################
static void W25qxx_ReadUniqID(void)
{
	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_READ_UNIQ_ID);
		for (uint8_t i = 0; i < 4; i++)	SPI_TxRx(COMMAND_DUMMY_BYTE);
		for (uint8_t i = 0; i < 8; i++)	w25qxx.UniqID[i] = SPI_TxRx(COMMAND_DUMMY_BYTE);
	}
	W25QXX_Disable();
}

//###################################################################################################################
static void W25qxx_WriteEnable(void)
{
	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_WRITE_ENABLE);
	}
	W25QXX_Disable();

	//W25qxx_Delay(1);
}

//###################################################################################################################
static void W25qxx_WriteDisable(void)
{
	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_WRITE_DISABLE);
	}
	W25QXX_Disable();

	//W25qxx_Delay(1);
}

//###################################################################################################################
static void W25qxx_WriteAddress(uint32_t Page_Address)
{
	if (w25qxx.ID >= W25Q256)
		SPI_TxRx((Page_Address & 0xFF000000) >> 24);
	SPI_TxRx((Page_Address & 0xFF0000) >> 16);
	SPI_TxRx((Page_Address & 0xFF00) >> 8);
	SPI_TxRx(Page_Address & 0xFF);
}

//###################################################################################################################
static uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3)
{
	uint8_t	status = 0;

	W25QXX_Enable();
	{
		if (SelectStatusRegister_1_2_3 == 1)
		{
			SPI_TxRx(COMMAND_READ_STATUS_REG_1);
			status = SPI_TxRx(COMMAND_DUMMY_BYTE);
			w25qxx.StatusRegister1 = status;
		}
		else if (SelectStatusRegister_1_2_3 == 2)
		{
			SPI_TxRx(COMMAND_READ_STATUS_REG_2);
			status = SPI_TxRx(COMMAND_DUMMY_BYTE);
			w25qxx.StatusRegister2 = status;
		}
		else
		{
			SPI_TxRx(COMMAND_READ_STATUS_REG_3);
			status = SPI_TxRx(COMMAND_DUMMY_BYTE);
			w25qxx.StatusRegister3 = status;
		}
	}
	W25QXX_Disable();
	return status;
}

//###################################################################################################################
//static void W25qxx_WriteStatusRegister(uint8_t SelectStatusRegister_1_2_3, uint8_t Data)
//{
//	HAL_GPIO_WritePin(_W25QXX_CS_GPIO, _W25QXX_CS_PIN, GPIO_PIN_RESET);
//	if (SelectStatusRegister_1_2_3 == 1)
//	{
//		W25qxx_Spi(0x01);
//		w25qxx.StatusRegister1 = Data;
//	}
//	else if (SelectStatusRegister_1_2_3 == 2)
//	{
//		W25qxx_Spi(0x31);
//		w25qxx.StatusRegister2 = Data;
//	}
//	else
//	{
//		W25qxx_Spi(0x11);
//		w25qxx.StatusRegister3 = Data;
//	}
//	W25qxx_Spi(Data);
//	HAL_GPIO_WritePin(_W25QXX_CS_GPIO, _W25QXX_CS_PIN, GPIO_PIN_SET);
//}

//###################################################################################################################
static void W25qxx_WaitForWriteEnd(void)
{
	//W25qxx_Delay(1);
	W25QXX_Enable();
	{
		SPI_TxRx(COMMAND_READ_STATUS_REG_1);
		do
		{
			w25qxx.StatusRegister1 = SPI_TxRx(COMMAND_DUMMY_BYTE);
			//W25qxx_Delay(1);
		} while ((w25qxx.StatusRegister1 & STATUS_REG_BUSY) == STATUS_REG_BUSY);
	}
	W25QXX_Disable();
}

//###################################################################################################################
