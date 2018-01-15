#include <LPC23xx.h>                        /* LPC23xx/24xx definitions */
#include <stdlib.h>
#include "tydef.h"
#include "pwm.h"
#include "irq.h"
extern void PWM0MasterHandler( void ) __irq;

//PWM  o/p pins - 2 for capture inputs and 1 for giving pwm pulse to another controllers for sync

//pwm_init
//	P1.28 		PCAP1.0	pinsel 10
//	P1.29 		PCAP1.1	pinsel 10
//	P2.6  		PCAP1.0	pinsel 01
//	P3.23 	    PCAP1.0	pinsel 11
DWORD PWM_Init()
{
	PINSEL4 |= 0x00000010; //p2.2 pwm3  output
	PINSEL3 |= 0x0A000000;  //for capture input
	PWM1CCR = 0x3f;		// INTERRUPT ON RISING/falling EDGE for cap0 cap1
	PWM1TCR = TCR_RESET;	/* Counter Reset */ 
	PWM1MCR = 0x0B;		//  RESET ON MATCH MR0	INTERRUPT on mr1
		
	PWM1PR = 0x00;			/* count frequency:Fpclk */
								//	TC if PWM0 matches */				
	/* all PWM latch enabled */
	PWM1LER = LER0_EN  | LER1_EN | LER2_EN | LER3_EN;
   if ( install_irq( PWM0_1_INT, (void *)PWM0MasterHandler, HIGHEST_PRIORITY ) == FALSE )
  {
	return( FALSE );
  }
 	return(TRUE);
}

//pwm_start
//total period 720000 --> 60ms
//duty cycle   180000 --> 15ms
//correction time at count 700000 --> 58 ms
void PWM_Start()   //
{			
	PWM1MR0 = 720000;		/* set PWM cycle */
	PWM1MR1 = 700000;
	PWM1LER = LER0_EN | LER1_EN;//| LER3_EN | LER4_EN | LER5_EN | LER6_EN;
	PWM1PCR = PWMENA1 |PWMENA3 ;//  PWMENA2 | | PWMENA4 | PWMENA5 | PWMENA6;
	PWM1TCR = TCR_CNT_EN | TCR_PWM_EN ;	/* counter enable, PWM enable */
    return;
}
//pwm_set
void PWM_Set(DWORD cycle)
{			
//	PWM1MR0 = 720000;		/* set PWM cycle */
//	PWM1MR1 = 700000;
//	PWM1MR2 = 9000;
	PWM1MR3 = 180000;
	PWM1LER = LER0_EN | LER3_EN | LER1_EN;// | LER5_EN | LER6_EN;
  return;
}	
void PWM_Stop()
{
 	PWM1MR1 = 0;
 	PWM1MR2 = 0;
 	PWM1MR3 = 0;
	PWM1LER = LER3_EN | LER1_EN | LER2_EN ;//| LER3_EN | LER4_EN | LER5_EN | LER6_EN;
  return;
}
extern BYTE delaypwm;
extern BYTE channelId; //to be set based on port pin setting of chan id
extern int startScan; //indicates to main tio start new scan
struct channelInfo
{
	DWORD NegEdge;	//Neg Edge capture value
	DWORD PosEdge;	//Pos Edge capture value
	DWORD TmHi;		//High Period
	DWORD TmLow;	//Low Period
	DWORD PosPhErr;	//Phase Error in pos direction
	DWORD NegPhErr; //phase error in neg Direction
	BYTE  PosPhErrCnt; //pos direction phase error (number of events) count
	BYTE  NegPhErrCnt; //Neg direction phase error (number of events) count
	BYTE  Captured;	   //flasg to indicate this channel info is captured to be used further
};
void PWM0MasterHandler(void) __irq
{
// variables used in the ISR
static BYTE measCycle;
static struct channelInfo channelB;
static struct channelInfo channelC;
DWORD captureStatus, phCorreection;
long myRefClock;

	captureStatus = PWM1IR; //read cause of interurpt
	myRefClock = PWM1TC; //read event ref. time
	PWM1IR = captureStatus;//CLEAR INTERRUPT
	if ( captureStatus & 0x10 )//capture 0  interrupt for channelB
	{
		captureStatus = 0;
		if(PWM1CCR & 1)//rising edge interrupt 
		{
			channelB.PosEdge   = PWM1CR0;	//save pos edge event time
			PWM1CCR |= 2;//make falling edge triggerred  
			PWM1CCR &= ~1;
			if(channelB.PosEdge  < 239900)
			{
				channelB.NegPhErrCnt++;
				channelB.NegPhErr = 240000 - channelB.PosEdge ;
			}
			else if(channelB.PosEdge  > 240100)
			{
				channelB.PosPhErrCnt++;
				channelB.PosPhErr = abs(channelB.PosEdge  - 240000);
			}
		}
		else if(PWM1CCR & 2)//falling edge interrupt 
		{
			channelB.NegEdge   = PWM1CR0;   //timervalue
			PWM1CCR |= 0x1;//make rising edge triggerred  
			PWM1CCR &= ~0x2;
			if(channelB.NegEdge  < 419900)
			{
				channelB.NegPhErrCnt++;
			}
			else if(channelB.NegEdge  > 420100)
			{
				channelB.PosPhErrCnt++;
			}
			if(channelB.NegEdge  > channelB.PosEdge )
			{
				channelB.TmHi  =  (channelB.NegEdge  - channelB.PosEdge );
				channelB.TmLow  = (720000 - channelB.NegEdge  + channelB.PosEdge );
			}
			else
			{
				channelB.TmHi  =  (720000 - channelB.PosEdge  + channelB.NegEdge );
				channelB.TmLow  = (channelB.PosEdge  - channelB.NegEdge );
			}
			channelB.Captured  = 1;
		}
	}
	else if ( captureStatus & 0x20 )//capture 1  interrupt for channelC
	{
		captureStatus = 0;
		if(PWM1CCR & 8)//rising edge interrupt 
		{
			channelC.PosEdge   = PWM1CR1; //save chanC pos edge event
			PWM1CCR |= 0x10;//make falling edge triggerred  
			PWM1CCR &= ~8;
			if(channelC.PosEdge  < 479900)
			{
				channelC.NegPhErrCnt++;
				channelC.NegPhErr = 480000 - channelB.PosEdge ;
			}
			else if(channelC.PosEdge  > 480100)
			{
				channelC.PosPhErrCnt++;
				channelC.PosPhErr = abs(channelC.PosEdge  - 480000);
			}
		}
		else if(PWM1CCR & 0x10)//falling edge interrupt 
		{
			channelC.NegEdge   = PWM1CR1;//neg edge ref value
			PWM1CCR |= 8;
			PWM1CCR &= ~0x10;//make rising edge triggerred  
			{
				if(channelC.NegEdge  < 659900)
				{
					channelC.NegPhErrCnt++;
				}
				else if(channelC.NegEdge  > 660100)
				{
					channelC.PosPhErrCnt++;
				}
			}
			if(channelC.NegEdge  > channelC.PosEdge )
			{
				channelC.TmHi  =  (channelC.NegEdge  - channelC.PosEdge ); //high period of capture one
				channelC.TmLow  = (720000 - channelC.NegEdge  + channelC.PosEdge );//low period of capture one
			}												
			else
			{
				channelC.TmHi  =  (720000 - channelC.PosEdge  + channelC.NegEdge );//high period of capture one
				channelC.TmLow  = (channelC.PosEdge  - channelC.NegEdge );//low period of capture one
			}
			channelC.Captured = 1;
		}
	}
	else if (captureStatus & 0x01 )	 //mcr0 match interrupt, cycle complete
	{
		captureStatus = 0;
		if(delaypwm==0)
		{
			startScan = 1; //let main start scanning now
			PWM_Set(0); 
		}
	}
	else if ( captureStatus & 0x02 )	 //mcr1 match, time to make correction
	{
		captureStatus = 0;
        if ((measCycle++ % channelId) == 0)      //correction to be made at different times in each channel
	{
		if(delaypwm)//power on correction can be both increase/decrease time period
		{
			delaypwm--;
			if(delaypwm == 0)
			{
				PWM_Set(0); //make self PWM output appear
			}
			else if(((channelB.TmHi > 179820 && channelB.TmHi < 180180) && (channelB.TmLow > 539460 && channelB.TmLow < 540540)) || ((channelC.TmHi > 179820 && channelC.TmHi < 180180) && (channelC.TmLow > 539460 && channelC.TmLow < 540540)))
			{
				if(channelB.NegPhErrCnt)
				{
					if(myRefClock + channelB.NegPhErr > 720000)
						PWM1TC = myRefClock + channelB.NegPhErr - 720000;
					else 
						PWM1TC = myRefClock + channelB.NegPhErr;
				}					
				else if(channelC.NegPhErrCnt)
				{
					if(myRefClock + channelC.NegPhErr > 720000)
						PWM1TC = myRefClock + channelC.NegPhErr - 720000;
					else 
						PWM1TC = myRefClock + channelC.NegPhErr;
				}					
				else if(channelB.PosPhErrCnt)
				{
					PWM1TC = myRefClock - channelB.PosPhErr; 
				}					
				else if(channelC.PosPhErrCnt)
				{
					PWM1TC = myRefClock - channelC.PosPhErr;
				}					
				channelC.PosPhErrCnt = channelB.PosPhErrCnt = channelC.NegPhErrCnt = channelB.NegPhErrCnt = 0;
			}
		}
		phCorreection = 0; //so that below logic sets it to some value
		if(channelC.Captured)//channelC correction
		{
			channelC.Captured = 0; 
			if((channelC.TmHi > 179820 && channelC.TmHi < 180180) && (channelC.TmLow > 539460 && channelC.TmLow < 5405400))			
			{//if high period and low period of chanC are within limits 
				if((channelC.PosEdge  > 480000))
				{
					if((channelC.PosEdge  - 480000) > 1200)
					{
						if((channelC.PosPhErrCnt > 1))
						{
							phCorreection = channelC.PosPhErr; //to be applied later below
							channelC.NegPhErrCnt = channelC.PosPhErrCnt = 0;
						}
					}
				}
			}
		}//correction channelC
		if(channelB.Captured) //channelB correction
		{
			channelB.Captured = 0;
			if((channelB.TmHi > 179820 && channelB.TmHi < 180180) && (channelB.TmLow > 539460 && channelB.TmLow < 5405400))
			{	//if high period and low period of capture 1 are within limits 
				if((channelB.PosEdge  > 240000))
				{   
					if((channelB.PosEdge  - 240000) > 1200)
					{
						if((channelB.PosPhErrCnt > 1))
						{
							if( phCorreection < channelB.PosPhErr) //is this bigger correction than chanC
								phCorreection = channelB.PosPhErr;
							channelB.NegPhErrCnt = channelB.PosPhErrCnt = 0;
						}					
					}
				}
			}
		}//correction cap0
		if( phCorreection) //if above two condition have got something to correct
		{
			PWM1TC = abs(myRefClock - phCorreection);
	  }
 	}//timer correction ends
		/*to make the pwm off*/
	VICVectAddr = 0;                      /* Acknowledge Interrupt              */
	return;
}
/*
Time Synchronisation Logic
The sync logic has following goals
1.	Maintain phase relation between three channels having their clocks within 1% tolerance
2.	Reject any channel which has more deviation than 1% frequency
3.	Reject any channel from sync logic which does not output valid waveform
4.	Provide output to other channels to synchronise
The channels output a PWM waveform of 60mS duration and 15mS positive pulse width. This output is connected to other two channels inputs in a fixed sequence. i.e. each channels 1st input is of the channel lagging in time (say B for channel A) and the 2nd input is of channel leading in time (say C for channel A).
This is achieved by running a PWM cycle of 60mS duration having pulse width of 15mS. The same PWM channel acts as reference to measure other channels phase errors and pulse widths. The other channel timing information is captured using capture logic which uses the same PWM timer. The rising and falling edges of the inputs are captured to get phase and pulse width information.
Each channel has different channel ID, so that
1.	At power on each shall try to synchronise with other channels for different period and at the end of this period output its own PWM waveform.
2.	The correction is done at different cycles (of 60 ms) by each of the channels so that when one channel corrects, the other does not get confused.
3.	The correction is applied such that the current cycle is either extended or shortened suitably.
When a channel corrects self to extend its cycle (60ms), the other should not shorten; hence all channels only will extend their period (60mS). So essentially the system with synchronise to the slowest channel within tolerance limits.
Whereas at power on before the PWM output is activates, the channels make correction in both the directions i.e. (extend the Period or shorten the period). Because at this time the output is not active so other channels will not be correcting based on this channel.
All the above logic is handled by a simple capture interrupt. The logic gets Six interrupts in the 60mS period.
1.	Rising edge of channel B, expected at 20mS
2.	Falling edge of channel B, expected at 35mS
3.	Rising edge of channel C, expected at 40mS
4.	Falling edge of channel C, expected at 55mS
5.	Correction time interrupt, at 58mS. SO that period can be extended.
6.	60mS over interrupt, to start the new measurement cycle.
The software uses following variables
Delaypwm: this is different value in different channels, to start the PWM cycle at different time instant.
startScan: flag to indicate start of new measurement cycle.
CHANID: to know self ID, to make correction in different measurement cycles. They maske correction is 3rd, 5th, 7th measurement cycle.
measCycle : measurement cycle number varies from 0 to 255..0-255. When this matches CHANID multiple, a correction shall be applied. So channel A shall correct every 3,6,9,12,15, channel B shall correct every 5,10,15,20.. and channel C shall apply correction every 7,14,21,28 measCycle. But the measCycle in all the three channels shall not be matching. i.e. all will not have same number/value.
phCorreection : This is the correction applied to the period.
myRefClock : This is the refclock, which is used to register the different events, and has range from 0-59.99ms with a resolution of (1/12 us).
Both the inputs capture and derive following information
	NegEdge:	Negative Edge capture value
	PosEdge:	Positive Edge capture value
	TmHi:		High Period
	TmLow:	Low Period
	PosPhErr:	Phase Error in positive direction
	NegPhErr:          Phase error in negative Direction
	PosPhErrCnt:    Positive direction phase error (number of events) count
	NegPhErrCnt:   Negative direction phase error (number of events) count
	Captured:	flag to indicate this channel info is captured to be used further

*/
}
