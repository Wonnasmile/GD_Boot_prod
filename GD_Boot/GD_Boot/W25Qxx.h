#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

	
#define NO_OFFSET 0
#define BLOCK_SIZE 4096	

	typedef enum W25QXX_ID_t {
		W25Q10 = 1,
		W25Q20,
		W25Q40,
		W25Q80,
		W25Q16,
		W25Q32,
		W25Q64,
		W25Q128,
		W25Q256,
		W25Q512,
	}W25QXX_ID_t;

	typedef enum W25QXX_Command_e {
		COMMAND_DUMMY_BYTE = 0xA5,
		COMMAND_DUMMY_BYTE_0 = 0x00,
		COMMAND_WRITE_ENABLE = 0x06,
		COMMAND_WRITE_DISABLE = 0x04,
		COMMAND_READ_STATUS_REG_1 = 0x05,
		COMMAND_READ_STATUS_REG_2 = 0x35,
		COMMAND_READ_STATUS_REG_3 = 0x15,
		COMMAND_READ_ID = 0x9F,
		COMMAND_READ_UNIQ_ID = 0x4B,
		COMMAND_FAST_READ = 0x0B,
		COMMAND_PAGE_PROGRAM = 0x02,
		COMMAND_SECTOR_ERASE_4KB = 0x20,
		COMMAND_BLOCK_ERASE_32KB = 0x52,
		COMMAND_BLOCK_ERASE_64KB = 0xD8,
		COMMAND_CHIP_ERASE = 0xC7,
	}W25QXX_Command_e;

	typedef enum W25QXX_STATUS_REG_1_Bits_e {
		STATUS_REG_BUSY = 0x01,
		STATUS_REG_WRITE_ENABLE_LATCH = 0x02,
		STATUS_REG_BLOCK_PROTECTION_BITS = 0x1C,
		STATUS_REG_TOP_OR_BOTTOM_PROTECT = 0x20,
		STATUS_REG_SECTOR_PROTECT = 0x40,
		STATUS_REG_PROTECT_0 = 0x80,
	}W25QXX_STATUS_REG_1_Bits_e;

	typedef enum W25QXX_STATUS_REG_2_Bits_e	{
		STATUS_REG_PROTECT_1 = 0x01,
		STATUS_REG_QUAD_ENABLE = 0x02,
		STATUS_REG_RESERVED = 0x04,
		STATUS_REG_SECURITY_REG_LOCK_BITS = 0x38,
		STATUS_REG_COMPLEMENT_PROTECT = 0x40,
		STATUS_REG_SUSPEND_STATUS = 0x80,
	}W25QXX_STATUS_REG_2_Bits_e;

	typedef struct w25qxx_t {
		W25QXX_ID_t	ID;
		uint8_t	UniqID[8];
		uint16_t PageSize;
		uint32_t PageCount;
		uint32_t SectorSize;
		uint32_t SectorCount;
		uint32_t BlockSize;
		uint32_t BlockCount;
		uint32_t CapacityInKiloByte;
		uint8_t StatusRegister1;
		uint8_t StatusRegister2;
		uint8_t StatusRegister3;
		uint8_t Lock;
	}w25qxx_t;

	extern w25qxx_t	w25qxx;

	//############################################################################
	// in Page,Sector and block read/write functions, can put 0 to read maximum bytes 
	//############################################################################
	
	bool W25qxx_Init(void);

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
	//############################################################################
#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */
