/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "HSP_SDCARD.h" /* Use GD driver */

/* Definitions of physical drive number for each drive */
#define DEV_SD		   0	/* Example: Map MMC/SD card to physical drive 0 */
#define DEV_FLASH		1	/* Example: Map QSPI Flash  to physical drive 1 */

#define SD_CARD_BLOCK_SIZE 1
extern sd_card_info_struct sd_cardinfo;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
      case DEV_SD :
         // result = RAM_disk_status();

         // translate the reslut code here

         return 0;

      case DEV_FLASH :
         // result = MMC_disk_status();

         // translate the reslut code here

         return stat;
	}
   
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
      case DEV_SD :
         // result = RAM_disk_initialize();
         // translate the reslut code here
         stat &= ~STA_NOINIT;
         //return stat;
         return 0;

      case DEV_FLASH :
         // result = MMC_disk_initialize();
         // translate the reslut code here

         return stat;
	}
	return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;
   uint32_t *ptrd, *btrd;
   sd_error_enum SD_stat = SD_OK;

	switch (pdrv) {
      case DEV_SD :
         if(count > 1)
         {
            SD_stat = sd_multiblocks_read((uint32_t *)buff, sector*sd_cardinfo.card_blocksize, sd_cardinfo.card_blocksize, count);
         }
         else
         {
            SD_stat = sd_block_read((uint32_t *)buff, sector*sd_cardinfo.card_blocksize, sd_cardinfo.card_blocksize);
         }
         if(SD_stat == SD_OK)
         {
            res = RES_OK;
         }
         else
         {
            res = RES_ERROR;
         }
         return res;
         
      case DEV_FLASH :
         // translate the arguments here
         // result = MMC_disk_read(buff, sector, count);
         // translate the reslut code here

         return res;
	}

	return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;
   uint32_t address;
   sd_error_enum SD_stat = SD_OK;
   

	switch (pdrv) {
      case DEV_SD :
         if(count > 1)
         {
            SD_stat = sd_multiblocks_write((uint32_t *)buff, sector*sd_cardinfo.card_blocksize, sd_cardinfo.card_blocksize, count);
         }
         else
         {
            SD_stat = sd_block_write((uint32_t *)buff, sector*sd_cardinfo.card_blocksize, sd_cardinfo.card_blocksize);
         }
         if(SD_stat == SD_OK)
         {
            res = RES_OK;
         }
         else
         {
            res = RES_ERROR;
         }
         return res;
         
      case DEV_FLASH :
         // translate the arguments here
         // result = MMC_disk_write(buff, sector, count);
         // translate the reslut code here

         return res;
	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
      case DEV_SD :
         // Process of the command for the RAM drive
         switch(cmd)
         {
            case GET_SECTOR_COUNT:     // sector number
               *(DWORD *)buff = sd_cardinfo.card_capacity / (sd_cardinfo.card_blocksize);
               break;
            case GET_SECTOR_SIZE:      // each sector size
               *(DWORD *)buff = sd_cardinfo.card_blocksize;
               break;
            case GET_BLOCK_SIZE:       // smallest unit of erased sector (unit 1)
               *(DWORD *)buff = SD_CARD_BLOCK_SIZE;
               break;
         }

        return res;
      case DEV_FLASH :
         // Process of the command for the MMC/SD card

         return res;
	}

	return RES_PARERR;
}

// return the time stamp
DWORD get_fattime(void)
{
   return 0;
}