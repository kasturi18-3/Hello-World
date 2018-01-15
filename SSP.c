#include <lpc23xx.h> 
#include "SSP.h"
#include "tydef.h"
#include "pinsel.h"
/*****************************************************************************
** Function name:		SSP1Init
**
** Descriptions:		SSP1 port initialization routine
**				
** parameters:			None
** Returned value:		true or false, if the interrupt handler
**						can't be installed correctly, return false.
** 
*****************************************************************************/
DWORD SSP0Init( void )
{
  BYTE i, Dummy;
  PCONP |= (1 << 21);	/* by default, it's enabled already, for safety reason */
  
	PINSEL0 |= 0x80000000;    /* Port 0.15 SPI SCK, port0.16 uses GPIO SPI_SEL, 	  */
	PINSEL1 |= 0x00000028; //port0.17 MISO, port0.18 MOSI */

	FIO2DIR0 |= 0x0A;   		// FOR SLAVE SELECT SSP1	 rst
	FIO2SET0 |= 0x0A;			 //cs for tdc1, 2
	/* Set DSS data to 8-bit, Frame format SPI, CPOL = 0, CPHA = 1, and SCR is 15 */
	SSP0CR0 = 0x0187;
	
	/* SSPnCPSR clock prescale register, master mode, minimum divisor is 0x04 */
	SSP0CPSR = 0x08; 
	
	for ( i = 0; i < FIFOSIZE; i++ )
	{
		Dummy = SSP0DR;		/* clear the RxFIFO */
	}
	
	/* Device select as master, SSP Enabled, Disable loopback operational mode */
	SSP0CR1 = SSPCR1_SSE;
	return( TRUE );
}
 
void spTxRx(char *bufr,int len)
{
	int i;  
	chipselecttdcPinLO; // Chip Select Low
	 	
	for( i=0; i<len; i++)
	{
		while (!(SSP0SR & SSPSR_TNF) );
		SSP0DR =  bufr[i]; 
		while( !(SSP0SR & SSPSR_RNE) );
		bufr[i] = SSP0DR;
	}
	chipselecttdcPinHI; // Chip Select High
	return;
}	
/*
  description when Clock rate changes

	value					read Time(for 24 bytes of Data)			write Time(for 24 bytes of Data) 
	  07					39 miliseconds						  	40/41 miliseconds
    47					85/86 miliseconds			   			nearly 74/75 milliseconds
    87					123/124 miliseconds			   			nearly 121/120 milliseconds
    C7					154/155 miliseconds			   			nearly 153/151 milliseconds

  */
