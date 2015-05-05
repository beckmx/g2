/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/* It mostly follows FatFs sample code, but has been modified            */
/* to use the Motate SPI functionality.                                  */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "motate/utility/SamSPI.h"
#include "motate/MotateTimers.h"

#include <stdio.h>
#include <memory>

/* Definitions of physical drive number for each drive */
#define SD0		0	/* Example: Map MMC/SD card to drive number 1 */

/* MMC/SD command (SPI mode) */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define CMD13	(13)		/* SEND_STATUS */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

/* Card type flags (CardType) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

#define SD_INIT_SPEED 400000
#define SD_ACTIVE_SPEED 8000000

std::shared_ptr<Motate::SPI<10>> spi;
Motate::Pin<119> cdPin(Motate::kInput);

/*--------------------------------------------------------------------------
 
 Module Private Functions
 
 ---------------------------------------------------------------------------*/

static volatile
DSTATUS Stat = STA_NOINIT;	/* Disk status */

static
BYTE CardType;			/* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */


/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static
int rcvr_datablock (	/* 1:OK, 0:Failed */
                    BYTE *buff,			/* Data buffer to store received data */
                    UINT btr			/* Byte count (must be even number) */
)
{
	volatile INT token;
    
    uint32_t start = Motate::SysTickTimer.getValue();
	do {							/* Wait for data packet in timeout of 100ms */
		token = spi->read(true, 0xFF);
	} while ((token != 0xFE) && Motate::SysTickTimer.getValue()-start < 100);
	if(token != 0xFE) {
        return 0;		/* If not valid data token, retutn with error */
    }
    
    spi->read((uint8_t*)buff, btr);
    
	spi->read(false, 0xFF);					/* Discard CRC */
	spi->read(false, 0xFF);
    
	return 1;						/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
static
int xmit_datablock (	/* 1:OK, 0:Failed */
                    const BYTE *buff,	/* 512 byte data block to be transmitted */
                    BYTE token			/* Data/Stop token */
)
{
	BYTE resp;
    
    bool stop = (token == 0xFD);
	spi->write(token, stop);					/* Xmit data token */
	if (token != 0xFD) {	/* Is data token */
        spi->write(buff, 512);
		spi->write(0xFF);					/* CRC (Dummy) */
		spi->write(0xFF, true);
        do {
            resp = spi->read() & 0x1F;				/* Reveive data response */
        } while (resp == 0 || resp == 0x1F);
		if (resp != 0x05)		/* If not accepted, return with error */
			return 0;
	}
    
	return 1;
}
#endif



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
uint8_t send_cmd (		/* Returns command response (bit7==1:Send failed)*/
    uint8_t cmd,		/* Command byte */
    uint32_t arg,		/* Argument */
    uint8_t* ocr = nullptr

)
{
    uint8_t res = 0xFF;
    
    if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}
    
    // wait until card is ready to receive command
    uint32_t start = Motate::SysTickTimer.getValue();
    while ((spi->read(true, 0xFF) != 0xFF) && Motate::SysTickTimer.getValue() - start < 1000) {};

    // choose the command CRC
	uint8_t n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
    
    // send command packet
    uint8_t data[6] = {
        (uint8_t)(0x40 | cmd),
        (uint8_t)((arg >> 24) & 0xFF),
        (uint8_t)((arg >> 16) & 0xFF),
        (uint8_t)((arg >> 8) & 0xFF),
        (uint8_t)(arg & 0xFF),
        n
    };
    spi->write(data, 6, false);
    
	// receive response
	if (cmd == CMD12) spi->read(false, 0xFF);		/* Skip a stuff byte when stop reading */
	uint8_t tries = 0xFF;							/* Wait for a valid response in timeout of 10 attempts */
	do
		res = spi->read(false, 0xFF);
	while ((res & 0x80) && --tries);                // 7th bit is low in all valid responses
    
    if (ocr)
        spi->read(ocr, 4);
    
	return res;			/* Return with the response value */
}

uint8_t send_cmd_until_specific_response(uint8_t command, uint32_t args, uint8_t response,
                                         uint8_t* ocr = nullptr, uint16_t attempts = 40)
{
    uint8_t resp = 0;
    do {
        resp = send_cmd(command, args, ocr);
    } while (resp != response && attempts--);
    
    return (resp == response);
}


/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
                         BYTE drv		/* Physical drive nmuber (0) */
)
{
	BYTE n, ty, cmd, ocr[4];
    
	if (drv) return STA_NOINIT;			/* Supports only single drive */
	if (disk_status(drv) & STA_NODISK) return Stat;	/* No card in the socket */
    
    // initialization has to be performed at a slower speed
    spi.reset(new Motate::SPI<10>(SD_INIT_SPEED));
    
	for (n = 10; n; n--) spi->read();	/* 80 dummy clocks */
    
	ty = 0;
    if (send_cmd_until_specific_response(CMD0, 0, 1)) {     /* Enter Idle state */
		if (send_cmd(CMD8, 0x1AA, &ocr[0]) == 1) {	/* SDv2? */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* The card can work at vdd range of 2.7-3.6V */
                if (send_cmd_until_specific_response(ACMD41, 1UL << 30, 0, nullptr, 1000)) { /* Wait for leaving idle state (ACMD41 with HCS bit) */
                    if (send_cmd_until_specific_response(CMD58, 0, 0, &ocr[0])) {
                        ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
                    }
                }
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
            bool success = false;
            if (send_cmd_until_specific_response(cmd, 0, 0, nullptr, 1000)) { /* Wait for leaving idle state */
                if (send_cmd(CMD16, 512) == 0) /* Set R/W block length to 512 */
                    success = true;
            }
            if (!success)
                ty = 0;
		}
	}
	CardType = ty;
    
	if (ty) {			/* Initialization succeded */
		Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
	}
    
    spi.reset(new Motate::SPI<10>(SD_ACTIVE_SPEED));
    
	return Stat;
}


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber to identify the drive */
)
{
    if (drv != SD0) return STA_NOINIT;
    
    if (cdPin.get()) {
        Stat = STA_NODISK | STA_NOINIT;
    } else {
        Stat &= ~STA_NODISK;
    }

    return Stat;
}




/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	if (drv || !count) return RES_PARERR;
    if (disk_status(drv) & (STA_NODISK | STA_NOINIT)) return RES_NOTRDY;
    
	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */
    
    BYTE cmd;
	cmd = count > 1 ? CMD18 : CMD17;			/*  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK */
    
	if (send_cmd_until_specific_response(cmd, sector, 0)) {
		do {
			if (!rcvr_datablock(buff, 512)) break;
			buff += 512;
		} while (--count);
		if (cmd == CMD18) send_cmd(CMD12, 0);	/* STOP_TRANSMISSION */
	}
    
	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
    if (drv || !count) return RES_PARERR;
    
    if (disk_status(drv) & (STA_NODISK | STA_NOINIT)) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;
    
	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */
    
	if (count == 1) {	/* Single block write */
        if (send_cmd_until_specific_response(CMD24, sector, 0)
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
    
	return count ? RES_ERROR : RES_OK;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
                    BYTE drv,		/* Physical drive nmuber (0) */
                    BYTE ctrl,		/* Control code */
                    void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	BYTE n, csd[16], *ptr = (BYTE*)buff;
	DWORD cs;
    
    
	if (drv) return RES_PARERR;
    if (disk_status(drv) & (STA_NODISK | STA_NOINIT)) return RES_NOTRDY;
    
	res = RES_ERROR;
	switch (ctrl) {
        case CTRL_SYNC :		/* Make sure that no pending write process */
            res = RES_OK;
            break;
            
        case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
                if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
                    cs = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                    *(DWORD*)buff = cs << 10;
                } else {					/* SDC ver 1.XX or MMC */
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    cs = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                    *(DWORD*)buff = cs << (n - 9);
                }
                res = RES_OK;
            }
            break;
            
        case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
            if (CardType & CT_SD2) {			/* SDv2 */
                if (send_cmd(ACMD13, 0) == 0) {		/* Read SD status */
                    spi->read();
                    if (rcvr_datablock(csd, 16)) {				/* Read partial block */
                        for (n = 64 - 16; n; n--) spi->read();	/* Purge trailing data */
                        *(DWORD*)buff = 16UL << (csd[10] >> 4);
                        res = RES_OK;
                    }
                }
            } else {					/* SDv1 or MMCv3 */
                if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
                    if (CardType & CT_SD1) {	/* SDv1 */
                        *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                    } else {					/* MMC */
                        *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                    }
                    res = RES_OK;
                }
            }
            break;
            
        case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
            *ptr = CardType;
            res = RES_OK;
            break;
            
        case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
            if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
                && rcvr_datablock(ptr, 16))
                res = RES_OK;
            break;
            
        case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
            if (send_cmd(CMD10, 0) == 0		/* READ_CID */
                && rcvr_datablock(ptr, 16))
                res = RES_OK;
            break;
            
        case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 bytes) */
            if (send_cmd(CMD58, 0, ptr) == 0) {	/* READ_OCR */
                res = RES_OK;
            }
            break;
            
        case MMC_GET_SDSTAT :	/* Receive SD statsu as a data block (64 bytes) */
            if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
                spi->read();
                if (rcvr_datablock(ptr, 64))
                    res = RES_OK;
            }
            break;
            
        default:
            res = RES_PARERR;
	}
    
//	deselect();
    
	return res;
}
#endif
