/******************************************************************************/
/* SERIAL.C: Low Level Serial Routines                                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <LPC23xx.H>                     /* LPC23xx definitions               */
#include <stdio.h>
#include "tydef.h"
#include "serial.h"
#include "irq.h"
#define maxuartBuff	0x40
#define DLE	0x10
#define STX 0x02
BYTE dacRxbuff[maxuartBuff];
BYTE dacTxbuff[maxuartBuff];
BYTE transmitIdx;
BYTE transmitLen;
BYTE receiveIdx;
BYTE rxchar;
void outputDAC(int DACQ_flow);
void outputDAC2(int DACQ_flow, BYTE unitID);
void UART2Handler (void) __irq;
void ProcessFrame(void);
void GetData(void);
extern BYTE channelId; //to indicate and differentiate time sync correction

BYTE init_serial (BYTE Uart)  
{               /* Initialize Serial Interface       */
		if (Uart == 0x02)
		{
			PINSEL0 |= 0x00500000;                /* Enable TxD2 and RxD2              */

			U2FDR    = 0x85;                        /* Fractional divider 85 for 115200  */
			U2LCR    = 0x87;                        /* 8 bits, no Parity, 2 Stop bit     */
			U2DLM 	 = 0;							
			U2DLL 	 = 4;							/*for 115200 DLL is 4 */
			U2LCR 	 = 0x07;						/* DLAB = 0, 2 stop, 8-bit */
 			if ( install_irq(UART2_INT, (void *)UART2Handler, HIGHEST_PRIORITY ) == FALSE )
			{
			  return (FALSE);
			}
			U2IER = IER_RBR | IER_THRE | IER_RLS;	//  Enable UART0 interrupt 
 			U2FCR = 0x07; //Enable FIFO BUFFER OF UART	 00FOR1  01FOR4   10FOR8   11FOR14 
			receiveIdx =0;
			return (TRUE);
		}
		if (Uart == 0x00)
		{
			PINSEL0 |= 0x00000050;                /* Enable TxD0 and RxD0              */
			U0FDR    = 0x85;                        /* Fractional divider 85 for 115200  */
			U0LCR    = 0x83;                        /* 8 bits, no Parity, 1 Stop bit     */
			U0DLM 	 = 0;				
			U0DLL 	 = 4;				/*for 115200 DLL is 4 */
			U0LCR 	 = 0x03;			/* DLAB = 0 */
/*	 		if ( install_irq(UART0_INT, (void *)UART0Handler, HIGHEST_PRIORITY ) == FALSE )
			{
			  return (FALSE);
			}
			U0IER = IER_RBR | IER_THRE | IER_RLS;	//  Enable UART0 interrupt 
*/			U0FCR = 0x07; //Enable FIFO BUFFER OF UART	 00FOR1  01FOR4   10FOR8   11FOR14 
			return (TRUE);
		}

return (FALSE);
}

int sendchar (BYTE ch) /* Implementation of putchar (also used by printf function to output data)*/
{
  while (!(U0LSR & 0x20));
  return (U0THR = ch);
}

int getkey(void)  // Read character from Serial Port Blocking Type 
{ 
  while (!(U0LSR & 0x01));
  return (U0RBR);
}
void UART2Handler (void) __irq 	 //used for DAC Output
{
  BYTE IIRValue, LSRValue;
  BYTE Dummy = Dummy;
  IIRValue = U2IIR;
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
 
  if (IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = U2LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  Dummy = U2RBR;		/* Dummy read on RX to clear 
	  						interrupt, then bail out */
	  VICVectAddr = 0;		/* Acknowledge Interrupt */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
		/* Receive Data Available */
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	while(U2LSR & 0x1) // read data until empty
	{
			rxchar = U2RBR;
			dacRxbuff[receiveIdx++] = rxchar;	//received STX

			if (receiveIdx >= maxuartBuff)
			{
				receiveIdx = 0;
			}
	}
  }
  else if (IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
 	while(U2LSR & 0x1) // read data until empty
	{
		
	}
  }
  else if (IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
		if (U2LSR & LSR_THRE )
		{
		  	  if (transmitIdx < transmitLen)
			  {
					U2THR = dacTxbuff[transmitIdx];
					transmitIdx = (transmitIdx+1) & (maxuartBuff -1);
			  }
		}
 	}
    
  VICVectAddr = 0;		/* Acknowledge Interrupt */
  return;
}
void outputDAC2(int DACQ_flow, BYTE unitID)
{
	dacTxbuff[0]  = (((DACQ_flow>>8)&0xf)|(unitID<<4)); //take MSB & set the ID
	dacTxbuff[1]  = (DACQ_flow&0xff); //take LSB
	dacTxbuff[2]  = ~(dacTxbuff[0]+dacTxbuff[1]); //calc checksum and compliment
	transmitLen = 3;
	transmitIdx = 1;
	receiveIdx =0; //to prepare for input from slave
	U2THR = dacTxbuff[0];  // To Generate a Trasmit Interrupt
   return;
}
void outputDAC(int DACQ_flow)
{
	BYTE cs;

	dacTxbuff[3]  = (DACQ_flow>>8) + 0x30;
	if( dacTxbuff[3] > '9') 
		dacTxbuff[3]+=7;
		dacTxbuff[2]  = ((DACQ_flow>>4)&0x0f) + 0x30;
		
	if( dacTxbuff[2] > '9') 
		dacTxbuff[2]+=7;
		dacTxbuff[1]  = (DACQ_flow&0x0f) + 0x30;
		
	if( dacTxbuff[1] > '9') dacTxbuff[1]+=7;
		cs = (BYTE)~(DACQ_flow&0xff | DACQ_flow>>8); //calc CS
		dacTxbuff[4]  = ((cs>>4)&0x0f) + 0x30;
		
	if( dacTxbuff[4] > '9') 
		dacTxbuff[4]+=7;
		dacTxbuff[5]  = (cs&0x0f) + 0x30;
		
	if( dacTxbuff[5] > '9') 
		dacTxbuff[5]+=7;

		dacTxbuff[0] = (channelId>>1)+0x30; //STX
		
		transmitLen = 6;
		transmitIdx = 1;
		receiveIdx =0;
		U2THR = dacTxbuff[0];  // To Generate a Trasmit Interrupt

   return;
}
void GetData(void)
{
	return;
}
