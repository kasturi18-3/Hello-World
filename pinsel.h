#include <LPC23xx.H>
#define P0LOW(x)	(FIO0CLR |= x)
#define P0HIGH(x)	(FIO0SET |= x)
#define P1LOW(x)	(FIO1CLR |= x)
#define P1HIGH(x)	(FIO1SET |= x)
#define P2LOW(x)	(FIO2CLR |= x)
#define P2HIGH(x)	(FIO2SET |= x)
#define P3LOW(x)	(FIO3CLR |= x)
#define P3HIGH(x)	(FIO3SET |= x)
#define P4LOW(x)	(FIO4CLR |= x)
#define P4HIGH(x)	(FIO4SET |= x)


#define W4LO			(FIO0CLR |= (1<<0));	 //P0.0
#define W4HI			(FIO0SET |= (1<<0));	 
#define W2LO			(FIO4CLR |= (1<<3));	 //P4.3
#define W2HI			(FIO4SET |= (1<<3));	 
#define simu10pPin			(1<<4)	//P0.4
#define simu50pPin			(1<<5)	//P0.5
#define measurementPin		(1<<7)	//P0.7
#define simu100pPin			(1<<8)	//P0.8
#define zeroFlowPin		(1<<9)	//P0.9	//interchange by santosh toggle swich and press switch is interchanged

#define interrupttdcPin		(1<<19)	//P0.19
#define overrangePin		(1<<29)	//P0.29
#define healthstatusPin		((unsigned long)(1<<31))	//P0.31

#define triggerFPin			(1<<18)	//P1.18
#define triggerRPin			(1<<20)	//P1.20

#define validatelatchPinHI	(FIO2SET |= (1<<0));	//P2.0
#define validatelatchPinLO	(FIO2CLR |= (1<<0));
#define chipselecttdcPinHI	(FIO2SET |= (1<<1));	//P2.1
#define chipselecttdcPinLO	(FIO2CLR |= (1<<1));
#define W3LO			(FIO0CLR |= (1<<1));	 //P0.1
#define W3HI			(FIO0SET |= (1<<1));	 

#define resettdcPinHI	(FIO4SET |=	(1<<12));	//P4.12
#define resettdcPinLO	(FIO4CLR |=	(1<<12));
#define channel5Pin			(1<<3) //p2.3

#define healthCheckPin		(1<<0)	//P4.0
#define W1LO			(FIO2CLR |= (1<<13));	 //P2.13
#define W1HI			(FIO2SET |= (1<<13));	 
#define channel7Pin			(1<<4)  //p2.4
#define zcdSilenceHI	(FIO4SET |= (1<<9));	//P4.9
#define zcdSilenceLO	(FIO4CLR |= (1<<9));
#define startlatchPinHI	(FIO4SET |=	(1<<10));	//P4.10
#define startlatchPinLO	(FIO4CLR |=	(1<<10));
#define enablestartPinHI (FIO4SET |= (1<<13));	//P4.13
#define enablestartPinLO (FIO4CLR |= (1<<13));
#define  zeroConfirmPin			(1<<14)	//P4.14//interchange by santosh toggle swich and press switch is interchanged

