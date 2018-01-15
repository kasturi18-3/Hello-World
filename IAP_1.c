#include "IAP.h"

#define INVALID_RESULT 0xFFFFFFFF
#define INITIAL_SECTOR_INDEX 0	 

typedef const struct
{
	unsigned char 	sec_num; 	/* sector number of the sector group */
	unsigned char 	sec_size;    /* each sector size in KB */
	unsigned char 	sec_index;	/* starting index of the sector group */
} SECTOR_DESC_T;

 SECTOR_DESC_T sector_desc[] = 
{
	{8,  4,  0}, 	// 8 4KB sectors, index: 0..3
	{14, 32, 8},	// 14 32KB sectors, index: 8..21
	{6,  4, 22},	// 6 4KB sectors, index: 22..27
	{0,  0,  0}		// should be zero
};
unsigned long command[5] = {0,0,0,0,0};
unsigned long result[3]= {0,0,0};


/*************************************************************************
 * Function Name: IAP_PrepareSec
 * Parameters: 	unsigned long StartSecNum -- Start Sector Number
 *			 	unsigned long EndSecNum -- End Sector Number
 * Return: unsigned long -- Status Code 
 *
 * Description: This command must be executed before executing "Copy RAM to Flash" or
 *			"Erase Sector(s)" command.
 *
 *************************************************************************/
unsigned long IAP_PrepareSec (unsigned long StartSecNum,  unsigned long EndSecNum)
{
	if (EndSecNum < StartSecNum)
		return IAP_STA_INVALD_PARAM;

	command[0] = IAP_CMD_PrepareSec;
	command[1] = StartSecNum;
	command[2] = EndSecNum;
	iap_entry(command, result);

	return result[0];
}

/*************************************************************************
 * Function Name: IAP_CopyRAMToFlash
 * Parameters: 	unsigned long dst -- Destination Flash address, should be a 256 byte boundary.
 *			 	unsigned long src -- Source RAM address, should be a word boundary
 *				unsigned long number -- 256 | 512 |1024 |4096			
 * Return: unsigned long -- Status Code 
 *
 * Description: This command is used to program the flash memory.
 *
 *************************************************************************/
unsigned long IAP_CopyRAMToFlash (unsigned long dst,  unsigned long src, 
	unsigned long number)
{
	command[0] = IAP_CMD_CopyRAMToFlash;
	command[1] = dst;
	command[2] = src;
	command[3] = number;
	command[4] = IAP_CLK / 1000;	// Fcclk in KHz
	iap_entry(command, result);

	return result[0];
}


/*************************************************************************
 * Function Name: IAP_EraseSec
 * Parameters: 	unsigned long StartSecNum -- Start Sector Number
 *			 	unsigned long EndSecNum -- End Sector Number
 * Return: unsigned long -- Status Code 
 *
 * Description: This command is used to erase a sector or multiple sectors of on-chip Flash
 *			 memory.
 *
 *************************************************************************/
unsigned long IAP_EraseSec (unsigned long StartSecNum,  unsigned long EndSecNum)
{
	if (EndSecNum < StartSecNum)
		return IAP_STA_INVALD_PARAM;

	command[0] = IAP_CMD_EraseSec;
	command[1] = StartSecNum;
	command[2] = EndSecNum;
	command[3] = IAP_CLK / 1000;
	iap_entry(command, result);

	return result[0];
}
void  IAP_ReinvokeISP(void)
{
	command[0] = IAP_CMD_REINVOKEISP;					
	iap_entry(command, result);;	
}

unsigned long getSectorIndex(unsigned long addr)
{
	SECTOR_DESC_T *psg = &sector_desc[0];
	unsigned long tmp, size_acc, sector_index, size_addr;

	size_acc = 0;
	size_addr = addr>>10;
	sector_index = 0; 
	while (psg->sec_num)
	{
		tmp = size_addr - size_acc;  //KB
		if (psg->sec_size*psg->sec_num > tmp)
		{
			sector_index += tmp/psg->sec_size;
			return sector_index;	

		}
		else
		{
			sector_index += psg->sec_num;
			size_acc += psg->sec_size*psg->sec_num;
		}
		psg++;
	}
	
	return INVALID_RESULT;	
}

