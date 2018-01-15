/******************************Points to Remember******************************
1)	Assign the Pin No.s to FLCS, SCLK, SDA and RESET in this Header file.
2)	This Program assumes that SO and SI pins are shorted and named as SDA.
3)	Pull the RESET, FLCS, SDA and SCLK pins of Data Flash to High in the Main Program during Initialisation.
4)	Change the include file depending on the 8051 derivative used.
5)	Define a Buffer (SPIBuff[]) in idata of required length in the Main Program.
*/
#ifndef __SSP_H__
#define __SSP_H__
#include "tydef.h"
#endif
/* SPI read and write buffer size */
#define SSPBUFSIZE		64
#define FIFOSIZE		8

/* SSP select pin */
#define SSP1_SEL		1 << 4
/* SSP Status register */
#define SSPSR_TFE		1 << 0
#define SSPSR_TNF		1 << 1 
#define SSPSR_RNE		1 << 2
#define SSPSR_RFF		1 << 3 
#define SSPSR_BSY		1 << 4
 
/* SSP0 CR0 register */
#define SSPCR0_DSS		1 << 0
#define SSPCR0_FRF		1 << 4
#define SSPCR0_SPO		1 << 6
#define SSPCR0_SPH		1 << 7
#define SSPCR0_SCR		1 << 8

/* SSP0 CR1 register */
#define SSPCR1_LBM		1 << 0
#define SSPCR1_SSE		1 << 1
#define SSPCR1_MS		1 << 2
#define SSPCR1_SOD		1 << 3

/* SSP0 Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM	1 << 0
#define SSPIMSC_RTIM	1 << 1
#define SSPIMSC_RXIM	1 << 2
#define SSPIMSC_TXIM	1 << 3

/* SSP0 Interrupt Status register */
#define SSPRIS_RORRIS	1 << 0
#define SSPRIS_RTRIS	1 << 1
#define SSPRIS_RXRIS	1 << 2
#define SSPRIS_TXRIS	1 << 3

/* SSP0 Masked Interrupt register */
#define SSPMIS_RORMIS	1 << 0
#define SSPMIS_RTMIS	1 << 1
#define SSPMIS_RXMIS	1 << 2
#define SSPMIS_TXMIS	1 << 3

/* SSP0 Interrupt clear register */
#define SSPICR_RORIC	1 << 0
#define SSPICR_RTIC		1 << 1

/* If RX_INTERRUPT is enabled, the SSP RX will be handled in the ISR
SSP0Receive() will not be needed. */
extern DWORD SSP0Init( void );
void spTxRx(char *bufr,int len);

