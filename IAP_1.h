#ifndef __IAP_H
#define	__IAP_H

/* IAP Command */
#define		IAP_CMD_PrepareSec		50		//select sector
#define		IAP_CMD_CopyRAMToFlash	51		//copy data from ram to flash
#define		IAP_CMD_EraseSec		52		//erase sector
#define		IAP_CMD_BlankChkSec		53		//check if sector is blank
#define		IAP_CMD_ReadParID		54		//read chip ID
#define		IAP_CMD_ReadBootVer		55		//read BOOT version
#define		IAP_CMD_Compare			56		//compare
#define		IAP_CMD_REINVOKEISP		57		//reinvoke ISP command

/* IAP Status Codes */
#define IAP_STA_CMD_SUCCESS 								0
#define IAP_STA_INVALID_COMMAND 							1
#define IAP_STA_SRC_ADDR_ERROR 								2
#define IAP_STA_DST_ADDR_ERROR 								3
#define IAP_STA_SRC_ADDR_NOT_MAPPED 						4
#define IAP_STA_DST_ADDR_NOT_MAPPED 						5
#define IAP_STA_COUNT_ERROR 								6
#define IAP_STA_INVALID_SECTOR 								7
#define IAP_STA_SECTOR_NOT_BLANK							8
#define IAP_STA_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 	9
#define IAP_STA_COMPARE_ERROR 								10
#define IAP_STA_BUSY 										11

#define IAP_STA_INVALD_PARAM 								12

unsigned long IAP_PrepareSec (unsigned long,  unsigned long);
unsigned long IAP_CopyRAMToFlash (unsigned long dst,  unsigned long src, 
	unsigned long number);
unsigned long IAP_EraseSec (unsigned long StartSecNum,  unsigned long EndSecNum);
extern unsigned long IAP_BlankChkSec (unsigned long StartSecNum,  unsigned long EndSecNum,
					unsigned long * pResult);
extern unsigned long IAP_ReadParID (unsigned long * PartID);				
extern unsigned long IAP_ReadBootVer (unsigned long * MajorVer, unsigned long * MinorVer);
extern unsigned long IAP_Compare (unsigned long dst,  unsigned long src, 
	   unsigned long number, unsigned long *offset);
extern void  IAP_ReinvokeISP(void);	
unsigned long getSectorIndex(unsigned long addr);
#define IAP_CLK 48000000

#define IAP_LOCATION 0x7FFFFFF1
#define iap_entry(a, b)     ((void (*)())(IAP_LOCATION))(a, b)

#endif
