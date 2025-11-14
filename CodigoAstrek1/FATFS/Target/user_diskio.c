/* USER CODE BEGIN Header */
/**
 * user_diskio.c - VERSIÓN CORREGIDA DE ORDEN Y ERRORES
 */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "ff_gen_drv.h"
#include "user_diskio.h"
#include "main.h"

/* === CONFIGURACIÓN DE PINES === */
#define CS_PORT    GPIOB
#define CS_PIN     GPIO_PIN_12

/* === REFERENCIAS EXTERNAS === */
extern SPI_HandleTypeDef hspi2;
#define HSPI_SD &hspi2

extern void Serial_PrintString(const char* str);

/* === MACROS DE CONTROL === */
#define SD_CS_LOW()     HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static BYTE CardType;

/* === COMANDOS SD === */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		0x06		/* SD */
#define CT_BLOCK	0x08		/* Block addressing */
#define CMD0    (0)
#define CMD1    (1)
#define ACMD41  (0x80+41)
#define CMD8    (8)
#define CMD9    (9)
#define CMD10   (10)
#define CMD12   (12)
#define ACMD13  (0x80+13)
#define CMD16   (16)
#define CMD17   (17)
#define CMD18   (18)
#define CMD23   (23)
#define ACMD23  (0x80+23)
#define CMD24   (24)
#define CMD25   (25)
#define CMD55   (55)
#define CMD58   (58)

/* Intercambio SPI */
static BYTE xchg_spi (BYTE dat) {
    BYTE rx_dat;
    if(HAL_SPI_TransmitReceive(HSPI_SD, &dat, &rx_dat, 1, 100) != HAL_OK) {
        return 0xFF;
    }
    return rx_dat;
}

/* Esperar tarjeta lista */
static int wait_ready (void) {
    BYTE d;
    UINT tmr;
    for (tmr = 5000; tmr; tmr--) {
        d = xchg_spi(0xFF);
        if (d == 0xFF) return 1;
        HAL_Delay(1);
    }
    return 0;
}

static void deselect (void) {
    SD_CS_HIGH();
    xchg_spi(0xFF);
}

static int select (void) {
    SD_CS_LOW();
    xchg_spi(0xFF);
    if (wait_ready()) return 1;
    deselect();
    return 0;
}

/* Recibir bloque de datos */
static int rcvr_datablock (BYTE *buff, UINT btr) {
    BYTE token;
    UINT tmr;
    for (tmr = 2000; tmr; tmr--) {
        token = xchg_spi(0xFF);
        if (token != 0xFF) break;
        HAL_Delay(1);
    }
    if (token != 0xFE) return 0;

    for(UINT i=0; i<btr; i++) buff[i] = xchg_spi(0xFF);

    xchg_spi(0xFF); /* Descartar CRC */
    xchg_spi(0xFF);
    return 1;
}

/* Transmitir bloque de datos */
#if _USE_WRITE
static int xmit_datablock (const BYTE *buff, BYTE token) {
    BYTE resp;
    if (!wait_ready()) return 0;

    xchg_spi(token);
    if (token != 0xFD) {
        for(UINT i=0; i<512; i++) xchg_spi(buff[i]);
        xchg_spi(0xFF);
        xchg_spi(0xFF);
        resp = xchg_spi(0xFF);
        if ((resp & 0x1F) != 0x05) return 0;
    }
    return 1;
}
#endif

/* Enviar comando */
static BYTE send_cmd (BYTE cmd, DWORD arg) {
    BYTE n, res;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    deselect();
    if (!select()) return 0xFF;

    xchg_spi(0x40 | cmd);
    xchg_spi((BYTE)(arg >> 24));
    xchg_spi((BYTE)(arg >> 16));
    xchg_spi((BYTE)(arg >> 8));
    xchg_spi((BYTE)arg);

    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;
    xchg_spi(n);

    if (cmd == CMD12) xchg_spi(0xFF);

    n = 10;
    do {
        res = xchg_spi(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	BYTE n, ty, ocr[4];
	char msg[100];

	if (pdrv) return STA_NOINIT;

	Serial_PrintString("\r\n--- SD DEBUG INIT START ---\r\n");

	SD_CS_HIGH();
	for (n = 0; n < 10; n++) xchg_spi(0xFF);

	ty = 0;

	Serial_PrintString("Enviando CMD0... ");
	n = send_cmd(CMD0, 0);

	sprintf(msg, "Resp: 0x%02X\r\n", n);
	Serial_PrintString(msg);

	if (n == 1) {
		Serial_PrintString("SD entro en IDLE. Enviando CMD8...\r\n");
		if (send_cmd(CMD8, 0x1AA) == 1) {
			Serial_PrintString("CMD8 OK (SD v2). Leyendo OCR...\r\n");
			for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);

			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
				Serial_PrintString("Voltaje OK. Inicializando (ACMD41)...\r\n");
				UINT tmr = 1000;
				while (tmr-- && send_cmd(ACMD41, 1UL << 30));

				if (tmr && send_cmd(CMD58, 0) == 0) {
					Serial_PrintString("Init OK. Checkeando CCS...\r\n");
					for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
					sprintf(msg, "Tipo detectado: 0x%X\r\n", ty);
					Serial_PrintString(msg);
				} else {
					Serial_PrintString("Fallo en Init o CMD58\r\n");
				}
			}
		} else {
			Serial_PrintString("CMD8 fallo. Probando SD v1 / MMC...\r\n");
		}
	} else {
		Serial_PrintString("ERROR: CMD0 fallo. Tarjeta no responde.\r\n");
	}

	CardType = ty;
	deselect();

	if (ty) {
		Stat &= ~STA_NOINIT;
		Serial_PrintString("--- SD INIT EXITOSO ---\r\n");
	} else {
		Serial_PrintString("--- SD INIT FALLIDO ---\r\n");
	}

	return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
	Stat = STA_NOINIT;
	return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	if (pdrv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) sector *= 512;

	if (count == 1) {
		if (send_cmd(CMD17, sector) == 0) {
			if (rcvr_datablock(buff, 512)) count = 0;
		}
	} else {
		if (send_cmd(CMD18, sector) == 0) {
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
	/* USER CODE HERE */
	if (pdrv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!(CardType & CT_BLOCK)) sector *= 512;

	if (count == 1) {
		if (send_cmd(CMD24, sector) == 0) {
			if (xmit_datablock(buff, 0xFE)) count = 0;
		}
	} else {
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD)) count = 1;
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	DRESULT res = RES_ERROR;
	BYTE n, csd[16];
	DWORD csize;

	if (pdrv) return RES_PARERR;
	res = RES_ERROR;
	if (cmd == CTRL_POWER) return RES_OK;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	switch (cmd) {
	case CTRL_SYNC:
		if (select()) res = RES_OK;
		break;
	case GET_SECTOR_COUNT:
		if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
			if ((csd[0] >> 6) == 1) {
				csize = csd[9] + ((WORD)csd[8] << 8) + 1;
				*(DWORD*)buff = csize << 10;
			} else {
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(DWORD*)buff = csize << (n - 9);
			}
			res = RES_OK;
		}
		break;
	case GET_BLOCK_SIZE:
		*(DWORD*)buff = 128;
		res = RES_OK;
		break;
	default:
		res = RES_PARERR;
	}
	deselect();
	return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

