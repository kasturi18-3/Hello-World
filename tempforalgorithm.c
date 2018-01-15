#include <stdio.h>
#include <LPC23xx.H>
#include <string.h>
#include <stdlib.h>
#include "tydef.h" 	 
#include "pwm.h"
#include "SSP.h"
#include "serial.h"
#include "pinsel.h"
#include "iap.h"
#include "irq.h"

#define	Kgf  3.24447  //3.243922	//geometric factor			 //*DDP*//
#define Kh   0.93120  	//hydraulic constant
#define Alpha   1.1234  	//Calibration constant as ON:02-05-13					  //*DDP*//	   0.8924 with RED flowmeter
//ch-1 = 1.0962,    ch-2 = 1.1240 ,   ch-3 = 1.1234		 as ON:02-05-13
// ch-1 = 0.8924,    ch-2 = 0.9067 ,   ch-3 = 0.9132
#define AVTCORRECTION      2000 //avarage T Correction
#define	MKfact1	((Kgf * Kh * Alpha)* 262144000) 				 //*DDP*//
#define	MKfact2	1000
#define T2STEEL	413663		   //eqnt of 2*0.789uS =1.578uS  steel sound velo 5700m/s and thickness 4.5mm	/////DDP ON:02-05-13
#define T3RDHIT	 1116790 //eqnt of 4.26 uS for the validation on second rising receiver pulse.(for higher validation threshold)	/////DDP ON:02-05-13
                //668390//eqnt of 2.55 uS for the validation on first rising receiver pulse.			 ///DDP ON:02-05-13
#define	TDCMAXBUFF  16
#define TDC_100US 25029000//it is temp data actual data is = 26214400//25984026//25974026//tdc count coresponding to 100us 25000 ?	 *****	 //*DDP*//
#define CONFIG_ADDR   0x7000 //address for storing parameters in ROM
#define BUFFLONGFILTER	160
#define trans_count  320                       //??????%%%%%%********* DDP*//		

#define RELAYK1ON			(FIO0CLR |= (1<<31));	 //P0.31
#define RELAYK1OFF			(FIO0SET |= (1<<31));	 
#define RELAYK2ON			(FIO0SET |= (1<<29));	 //p0.29
#define RELAYK2OFF			(FIO0CLR |= (1<<29));
#define statusLedOn			(FIO0SET |= (1<<23)); //p0.23
#define statusLedOff		(FIO0CLR |= (1<<23));

extern int getkey(void);
extern  void init_serial(BYTE Uart);
void delayUs(int);
void InitTDC(void);
unsigned int Tdc_for(BYTE mode);
unsigned int Tdc_rev(BYTE mode);
void triggerDelay(void);
void trigger(int channel);
void delayMs(float delayInMs);
void storeParams(void);
extern void outputDAC(int DACQ_flow);
extern void outputDAC2(int DACQ_flow, BYTE unitID);
void quickSort( unsigned int *arr,int elements);
int ReadCorrection[16] __attribute__((at(CONFIG_ADDR))); //3int per channel, with  1 int gap for easy calculation

char SPIBuff[SSPBUFSIZE];
int startScan, delaypwm;
BYTE channelId; //to indicate and differentiate time sync correction
__align(4) BYTE IAP_Buf[48]; //used for IAP storing parameters in ROM, 16 per channel
int faultCountF; //forword fault
int faultCountR; //reverse fault
float finalFlowOutput;
struct fC flowCalc;
int avgTcorr; //should be signed
int avgTcorr_1,avgTcorr_2,avgTcorr_3;		////DDP
unsigned int TDC_forward_2,TDC_forward_3,TDC_reverse_2,TDC_reverse_3;			 /////DDP
unsigned int Diff_Rv_32_1sec[16],Diff_Rv_21_1sec[16],Diff_Fr_32_1sec[16],Diff_Fr_21_1sec[16],TDC_status_r,TDC_status_f;     ///DDP
unsigned int Diff_Rv_32_5sec[160],Diff_Rv_21_5sec[160],Diff_Fr_32_5sec[160],Diff_Fr_21_5sec[160];
unsigned int Diff_Rv_32,Diff_Rv_21,Diff_Fr_32,Diff_Fr_21,Diff_Rv_32_temp,Diff_Rv_21_temp,Diff_Fr_32_temp,Diff_Fr_21_temp;
unsigned int Tcorr_1, Tcorr_2;

BYTE blinkLed;
struct TDC_values
{
	unsigned int TdcFr, TdcRv;
};
struct 
{
	unsigned int bufTf[TDCMAXBUFF];
	unsigned int bufTr[TDCMAXBUFF];
	int idx;
	float flow; //from 1 sec
}flow1Sec; //to be updated in measure mode
unsigned int bufdummy[BUFFLONGFILTER]; //to sort 10 sec image of circular buffer	   //*DDP*//
struct 
{
	unsigned int bufTf[BUFFLONGFILTER];
	unsigned int bufTr[BUFFLONGFILTER];
	int idx;
	int transCounter; //transient counter				 //*DDP*// this variable is not required
	float flow; //from 5 seconds
}flow5Sec; //to be updated in measure mode




struct fC
{
	unsigned int avgTr;	
	unsigned int avgTf;
};

float calcFlow(struct fC *flow);

struct
{
   unsigned	int bufTr[301];	// *
   unsigned	int bufTf[301];	//*
	int idx;
	int ready;
}zeroMode; //to be updated in zeroflowmode

struct 
{
	unsigned int trtf[TDCMAXBUFF];
	int idx;
}healthCheck; //to be updated in healthcheck mode

__packed struct 
{
	char healthCheck;
	char simu10p;
	char simu50p;
	char simu100p;
	char zeroFlow;
	char zeroConfirm;
	char measurement;
}inpBuf; //to be updated every 60mS at end of main loop

enum modes
{
	measureMode,
	healthCheckMode,
	simu10pMode,
	simu50pMode,
	simu100pMode,
	zeroFlowMode
}operMode; //mode of operation, to be modified at end of main loop

unsigned char displayvalue;
void functiondelay100(void);

#define  PCONP_PCTIM0_BIT		(1 << 1)
#define  PCONP_PCTIM1_BIT		(1 << 2)
#define  PCONP_PCUART0_BIT		(1 << 3)
#define  PCONP_PCPWM1_BIT		(1 << 6)
#define  PCONP_PCSSP1_BIT		(1 << 10)
#define  PCONP_PCTIM2_BIT		(1 << 22)
#define  PCONP_PCUART2_BIT		(1 << 24)

extern BYTE dacRxbuff[];
extern BYTE receiveIdx;
BYTE receiveIdxTx;

int main (void) 
{	
	
	
	int faultBits,relayk2disturb,OR_UR=0;	// over range or under range bit	//////DDP ON:02-05-13
	float deltaFlow,deltaFlow_temp;					///DDP
	int i,j=0,k=0;
	BYTE toggle, statusLedBlink;
	unsigned long long sum; 
	struct TDC_values Cur_val;

	unsigned int temp_avgTr_new, temp_avgTr_old, temp_avgTf_new, temp_avgTf_old;//??????%%%%%%********* DDP

	int index_1=0, index_5=0;        /////DDP


	memset(&flow1Sec,0,sizeof( flow1Sec));
	memset(&flow5Sec,0,sizeof( flow5Sec));
	for(i=0; i<16;i++)
	{
		healthCheck.trtf[i] = 100; //100us value
	}
	healthCheck.idx = 0;
	zeroMode.idx = 0;
	zeroMode.ready = 0; //now we cannot check confirm input
	memset(&inpBuf, 0xff, sizeof(inpBuf)); //so that initially 

 	EMC_CTRL = 0x00000001;
	PCONP  = 0;  					// turn on selected peripherals
	PCONP  |= PCONP_PCUART0_BIT | PCONP_PCPWM1_BIT  |PCONP_PCSSP1_BIT|PCONP_PCUART2_BIT|PCONP_PCTIM2_BIT;	//BM
	SCS |= 1; //port0,1 using fast io

/********************** GPIO CONFIGURATION *********************************/
	PINSEL0  = 0x00000000;// P0.0 to P0.15 as GPIO
	PINSEL1  = 0x00000000;// P0.16 to P0.31 as GPIO
	FIO0DIR  = 0xFFFFFFFF;// P0.0 to P0.31 as GPIO	and direction as Output

	PINSEL2  = 0x00000000;// P1.0 to P1.15 as GPIO
	PINSEL3  = 0x00000000;// P1.16 to P1.31 as GPIO
	FIO1DIR  = 0xFFFFFFFF;// P1.0 to P1.31 as GPIO	and direction as Output

	PINSEL4  = 0x00000000;// P2.0 to P2.15 as GPIO
	PINSEL5  = 0x00000000;// P2.16 to P2.31 as GPIO
	FIO2DIR  = 0xFFFFFFFF;// P2.0 to P2.31 as GPIO	and direction as Output

	PINSEL6  = 0x00000000;// P3.0 to P3.15 as GPIO
	PINSEL7  = 0x00000000;// P3.16 to P3.31 as GPIO
	FIO3DIR  = 0xFFFFFFFF;// P3.0 to P3.31 as GPIO	and direction as Output

	PINSEL8  = 0x00000000;// P4.0 to P4.15 as GPIO
	PINSEL9  = 0x00000000;// P4.16 to P4.31 as GPIO
	FIO4DIR  = 0xFFFFFFFF;// P4.0 to P4.31 as GPIO	and direction as Output

	//Set Direction for Input PINS
	FIO0DIR &= ~simu10pPin;
	FIO0DIR &= ~measurementPin;
	FIO0DIR &= ~simu100pPin;
	FIO0DIR &= ~zeroFlowPin;
	//FIO4DIR &= ~zeroFlowPin;
	FIO0DIR &= ~simu50pPin;
	FIO2DIR &= ~channel5Pin;
	FIO4DIR &= ~healthCheckPin;
	FIO4DIR &= ~zeroConfirmPin;
	//FIO0DIR &= ~zeroConfirmPin;
	FIO2DIR &= ~channel7Pin;
/**********************************************************************/
	init_serial(0x00); // for print
	init_serial(0x02); //for MSP430 and DAC
	outputDAC2(0,0xf);//broadcast 0 output, also inits the uart variables
	
	FIO1CLR  |= triggerFPin;
	FIO1CLR  |= triggerRPin;
	PWM_Init();
	SSP0Init();
	statusLedOn; //turn off LED
	resettdcPinLO;		//TDC reset Low
	FIO4SET1 |= 0x34; 		//enable pin's start and stop for Tdc 1 2  rst for tdc1
	validatelatchPinHI;	//stop tdc
	resettdcPinHI;		//TDC reset high
	FIO2CLR0 |= 0x80;			// rst for d-latch

	faultBits = 1; //to begin with TDC fail, let it be cleared by these element
	RELAYK1OFF; //error
	InitTDC();
	PWM_Start();
	W3HI;
	W2HI;
	W4HI;
	W1HI;	
	delaypwm = 30; // defalut correction instant for channel 3
    channelId = 3; // default channel ID

	if(((FIO2PIN & (unsigned long)channel5Pin) == 0)&&( (FIO2PIN & (unsigned long)channel7Pin) == 0))
	{
	channelId = 3; // default channel ID
	}
	else
	{
	if( (FIO2PIN & (unsigned long)channel7Pin) == 0) //check channel as 5 ??
	{
		delaypwm += 10;
        channelId = 5;//this decides the correction instant of each channel 
	}
	if( (FIO2PIN & (unsigned long)channel5Pin) == 0) //check channel as 7 ??
	{
		delaypwm += 20;  
        channelId = 7; //this decides the correction instant of each channel
	}
	}

	startScan = 0; //no scanning till capture isr says
	avgTcorr = AVTCORRECTION; //copy default, then check and get saved if consistant
	i =  (channelId>>1)-1; //make it 0,1,2 from 3,5,7
	i = i<<2; //make int offset of 0,4,8 from 0,1,2, as readcorrection is a four byte int
	if( ReadCorrection[i+0] == ReadCorrection[i+1])
		if( ReadCorrection[i+0] == ~ReadCorrection[i+2])
			avgTcorr = ReadCorrection[i+0];//AVTCORRECTION;

	avgTcorr_1 = ReadCorrection[0];						////DDP
	avgTcorr_2 = ReadCorrection[4];						////DDP
	avgTcorr_3 = ReadCorrection[8];						////DDP


	printf("Correction Read#%d=%d\r\n",i,avgTcorr);
	statusLedOff; //turn on LED
	memset(&inpBuf.healthCheck,0x00,7);
	healthCheck.idx = 0;

	flow5Sec.transCounter = trans_count;		   //??????%%%%%%********* DDP
	relayk2disturb = 150; //after these iterations overrange relay will be off

	while (1)   // Loop forever   
	{                                                
		if(startScan) //set at every 60 Ms
		{
			startScan = 0;
			InitTDC();
		
			SPIBuff[0] = 0x03; 		//calibrate clk		//////*****DDP
			spTxRx(SPIBuff,1);							//////*****DDP
		
//printf("1\n");
			Cur_val.TdcRv  = Tdc_rev(operMode);
//printf("2\n");
/////////////////////////DDP--start////////////////////////////////////////////

			if (operMode == measureMode)
			{
			SPIBuff[0] = 0x81;
			SPIBuff[1] = 0x31;		// 0x21 for 1st hit, 0x31 for 2nd hit and 0x41 for 3rd hit *****  DDP
			SPIBuff[2] = 0x44;	//42		 			//44;
			SPIBuff[3] = 0x00;
			spTxRx(SPIBuff,4); //R1 set meas type from strt to hit1 
			delayUs(5);

			SPIBuff[0] = 0x81;
			SPIBuff[1] = 0x41;		// 0x21 for 1st hit, 0x31 for 2nd hit and 0x41 for 3rd hit *****  DDP
			SPIBuff[2] = 0x44;	//42		 			//44;
			SPIBuff[3] = 0x00;
			spTxRx(SPIBuff,4); //R1 set meas type from strt to hit1 
			delayUs(5);
		
			SPIBuff[0] = 0xB1; 		//read 2nd hit data						   ///DDP
			spTxRx(SPIBuff,5);
			TDC_reverse_2 = ((SPIBuff[1]<<8) | SPIBuff[2]);							  ///DDP
			TDC_reverse_2 = (TDC_reverse_2<<16)|((SPIBuff[3]<<8) | SPIBuff[4]);		  ///DDP

 			SPIBuff[0] = 0xB2; 		//read 2nd hit data								 ///DDP
			spTxRx(SPIBuff,5);															///DDP
			TDC_reverse_3 = ((SPIBuff[1]<<8) | SPIBuff[2]);								///DDP
			TDC_reverse_3 = (TDC_reverse_3<<16)|((SPIBuff[3]<<8) | SPIBuff[4]);			 ///DDP

//printf("3\n");			
			}
/////////////////////////DDP--stop////////////////////////////////////////////

			delayUs(9000);
			InitTDC();
//printf("4\n");
			Cur_val.TdcFr = Tdc_for(operMode);
//printf("5\n");
/////////////////////////DDP--start////////////////////////////////////////////

			if (operMode == measureMode)
			{
  			SPIBuff[0] = 0x81;
			SPIBuff[1] = 0x31;		// 0x21 for 1st hit, 0x31 for 2nd hit and 0x41 for 3rd hit *****  DDP
			SPIBuff[2] = 0x44;	//42		 			//44;
			SPIBuff[3] = 0x00;
			spTxRx(SPIBuff,4); //R1 set meas type from strt to hit1 
			delayUs(5);

			SPIBuff[0] = 0x81;
			SPIBuff[1] = 0x41;		// 0x21 for 1st hit, 0x31 for 2nd hit and 0x41 for 3rd hit *****  DDP
			SPIBuff[2] = 0x44;	//42		 			//44;
			SPIBuff[3] = 0x00;
			spTxRx(SPIBuff,4); //R1 set meas type from strt to hit1 
			delayUs(5);
		
			SPIBuff[0] = 0xB1; 		//read 2nd hit data							 ///DDP
			spTxRx(SPIBuff,5);													 ///DDP
			TDC_forward_2 = ((SPIBuff[1]<<8) | SPIBuff[2]);						 ///DDP
			TDC_forward_2 = (TDC_forward_2<<16)|((SPIBuff[3]<<8) | SPIBuff[4]);	  ///DDP

 			SPIBuff[0] = 0xB2; 		//read 2nd hit data							 ///DDP
			spTxRx(SPIBuff,5);													 ///DDP
			TDC_forward_3 = ((SPIBuff[1]<<8) | SPIBuff[2]);						  ///DDP
			TDC_forward_3 = (TDC_forward_3<<16)|((SPIBuff[3]<<8) | SPIBuff[4]);	  ///DDP

//printf("6\n");

			if( index_1 >= 16)														///DDP
				index_1 = 0;														///DDP
				Diff_Rv_32_1sec[index_1] = (TDC_reverse_3-TDC_reverse_2);			///DDP
				Diff_Rv_21_1sec[index_1] = (TDC_reverse_2-Cur_val.TdcRv);		   ///DDP
				Diff_Fr_32_1sec[index_1] = (TDC_forward_3-TDC_forward_2);		   ///DDP
				Diff_Fr_21_1sec[index_1] = (TDC_forward_2-Cur_val.TdcFr);		   ///DDP
				index_1 = index_1 + 1;											   ///DDP

 				memcpy(&bufdummy[0],&Diff_Rv_32_1sec[0],16*4);//copy 16 int to temp		   ///DDP
				quickSort(bufdummy, 16);												 ///DDP
				for(i = 4,sum = 0; i < 12; i++)											   ///DDP
					sum += bufdummy[i];													   ///DDP
				Diff_Rv_32_temp = (sum >> 3);											  ///DDP
//printf("7\n");

 				memcpy(&bufdummy[0],&Diff_Rv_21_1sec[0],16*4);//copy 16 int to temp			///DDP
				quickSort(bufdummy, 16);												  ///DDP
				for(i = 4,sum = 0; i < 12; i++)											  ///DDP
					sum += bufdummy[i];													  ///DDP
				Diff_Rv_21_temp = (sum >> 3);											  ///DDP
//printf("8\n");
																							///DDP
  				memcpy(&bufdummy[0],&Diff_Fr_32_1sec[0],16*4);//copy 16 int to temp			///DDP
				quickSort(bufdummy, 16);													///DDP
				for(i = 4,sum = 0; i < 12; i++)										///DDP
					sum += bufdummy[i];											   ///DDP
				Diff_Fr_32_temp = (sum >> 3);									   ///DDP
//printf("9\n");

 				memcpy(&bufdummy[0],&Diff_Fr_21_1sec[0],16*4);//copy 16 int to temp		 ///DDP
				quickSort(bufdummy, 16);											   ///DDP
				for(i = 4,sum = 0; i < 12; i++)										   ///DDP
					sum += bufdummy[i];												   ///DDP
				Diff_Fr_21_temp = (sum >> 3);										   ///DDP

//printf("10\n");
				if( index_5 >= BUFFLONGFILTER)									   ///DDP
				index_5 = 0;													   ///DDP

				Diff_Rv_32_5sec[index_5] = Diff_Rv_32_temp;						   ///DDP
				Diff_Rv_21_5sec[index_5] = Diff_Rv_21_temp;						  ///DDP
				Diff_Fr_32_5sec[index_5] = Diff_Fr_32_temp;						  ///DDP
				Diff_Fr_21_5sec[index_5] = Diff_Fr_21_temp;						 ///DDP
				index_5 = index_5 + 1;											 ///DDP

				memcpy(&bufdummy[0],&Diff_Rv_32_5sec[0],160*4);//copy 160 int to temp		   ///DDP
				quickSort(bufdummy, 160);													///DDP
				for(i = 16,sum = 0; i < BUFFLONGFILTER-16; i++)							   ///DDP
					sum += bufdummy[i];													   ///DDP
				Diff_Rv_32_temp = (sum >> 7);										   ///DDP
				Diff_Rv_32 =   0.994*Diff_Rv_32 + 0.006*Diff_Rv_32_temp;		//%%%%%*****$$$$$  DDP
//printf("11\n");
				memcpy(&bufdummy[0],&Diff_Rv_21_5sec[0],160*4);//copy 160 int to temp		///DDP
				quickSort(bufdummy, 160);											   ///DDP
				for(i = 16,sum = 0; i < BUFFLONGFILTER-16; i++)						///DDP
					sum += bufdummy[i];												///DDP
				Diff_Rv_21_temp = (sum >> 7);									   ///DDP
				Diff_Rv_21 =   0.994*Diff_Rv_21 + 0.006*Diff_Rv_21_temp;		//%%%%%*****$$$$$  DDP
//printf("12\n");
				memcpy(&bufdummy[0],&Diff_Fr_32_5sec[0],160*4);//copy 160 int to temp	   ///DDP
				quickSort(bufdummy, 160);											 ///DDP
				for(i = 16,sum = 0; i < BUFFLONGFILTER-16; i++)					   ///DDP
					sum += bufdummy[i];											   ///DDP
				Diff_Fr_32_temp = (sum >> 7);									  ///DDP
				Diff_Fr_32 =   0.994*Diff_Fr_32 + 0.006*Diff_Fr_32_temp;		//%%%%%*****$$$$$  DDP
//printf("13\n");
				memcpy(&bufdummy[0],&Diff_Fr_21_5sec[0],160*4);//copy 160 int to temp ///DDP
				quickSort(bufdummy, 160);									   ///DDP
				for(i = 16,sum = 0; i < BUFFLONGFILTER-16; i++)				   ///DDP
					sum += bufdummy[i];										   ///DDP
				Diff_Fr_21_temp = (sum >> 7);								   ///DDP
				Diff_Fr_21 =   0.994*Diff_Fr_21 + 0.006*Diff_Fr_21_temp;		//%%%%%*****$$$$$  DDP

//printf("14\n");



			//Diff_Rv_32 = (TDC_reverse_3-TDC_reverse_2);
			
//			Diff_Rv_32 = 0.996*Diff_Rv_32 + 0.004*(TDC_reverse_3-TDC_reverse_2);		//%%%%%*****$$$$$  DDP
//			Diff_Rv_21 = 0.996*Diff_Rv_21 + 0.004*(TDC_reverse_2-Cur_val.TdcRv);
//			Diff_Fr_32 = 0.996*Diff_Fr_32 + 0.004*(TDC_forward_3-TDC_forward_2);
//			Diff_Fr_21 = 0.996*Diff_Fr_21 + 0.004*(TDC_forward_2-Cur_val.TdcFr);
//
//			Tcorr_1 = 2*(Diff_Rv_21-Diff_Fr_21)-(Diff_Rv_32-Diff_Fr_32);
//			Tcorr_2 = 3*(Diff_Rv_21-Diff_Fr_21)-(1.5*(Diff_Rv_32-Diff_Fr_32));

			}
/////////////////////////DDP--stop////////////////////////////////////////////

			if( operMode == healthCheckMode)
			{
				if( healthCheck.idx >= 16)
				{
				 	healthCheck.idx = 0;
				}
				healthCheck.trtf[healthCheck.idx] = Cur_val.TdcFr;
				healthCheck.idx = (healthCheck.idx+1);  //buffer of 16
				memcpy(&bufdummy[0],&healthCheck.trtf[0],16*4);//copy 16 int to temp
				quickSort(bufdummy, 16);
				for(i = 4,sum = 0; i < 12; i++)
					sum += bufdummy[i];
				flowCalc.avgTf = (sum >> 3);
		  	 	finalFlowOutput =  31.5 + ((int)flowCalc.avgTf-TDC_100US)*31.5/TDC_100US; /*//100uS  */
				finalFlowOutput = (unsigned int)(finalFlowOutput *3900)/63;	//*
			}
			else
			{	 			
				
				if( flow1Sec.idx >= 16)
					flow1Sec.idx = 0;
				flow1Sec.bufTf[flow1Sec.idx] = Cur_val.TdcFr;
				flow1Sec.bufTr[flow1Sec.idx] = Cur_val.TdcRv;
				flow1Sec.idx = flow1Sec.idx+1;
				if( operMode == zeroFlowMode)
				{
					if( zeroMode.idx >= 300)
					{
						zeroMode.idx = 0;
						zeroMode.ready = 1; //now we can check confirm input
					}
					zeroMode.bufTf[zeroMode.idx] = Cur_val.TdcFr;
					zeroMode.bufTr[zeroMode.idx] = Cur_val.TdcRv;
					zeroMode.idx = zeroMode.idx+1;
				}
				else
				{
				zeroMode.ready = 0;					////DDP
				zeroMode.idx = 0;					//// DDP
				}
				//forward calculation	
				memcpy(&bufdummy[0],&flow1Sec.bufTf[0],16*4);//copy 16 int to temp
				quickSort(bufdummy, 16);
				for(i = 4,sum = 0; i < 12; i++)
					sum += bufdummy[i];
				flowCalc.avgTf = (sum >> 3);
					
				//reverse calculation
				memcpy(&bufdummy[0],&flow1Sec.bufTr[0],16*4);//copy 16 int to temp
				quickSort(bufdummy, 16);
				for(i = 4,sum = 0; i < 12; i++)
					sum += bufdummy[i];
				flowCalc.avgTr = (sum >> 3);
				flow1Sec.flow = calcFlow(&flowCalc); //for 1 sec

//printf("15\n");
				//start calc for 5 second flow
				if( flow5Sec.idx >= BUFFLONGFILTER)
					flow5Sec.idx = 0;
				flow5Sec.bufTf[flow5Sec.idx] = flowCalc.avgTf;	// put media avarage of 1 sec flow forword
				flow5Sec.bufTr[flow5Sec.idx] = flowCalc.avgTr;	// put media avarage of 1 sec flow reversed
				flow5Sec.idx = flow5Sec.idx+1;
				memcpy(&bufdummy[0],&flow5Sec.bufTf[0],160*4);//copy 160 int to temp
				quickSort(bufdummy, 160);
				for(i = 16,sum = 0; i < BUFFLONGFILTER-16; i++)
					sum += bufdummy[i];
//				flowCalc.avgTf = (sum / (BUFFLONGFILTER-32));						//%%%%%*****$$$$$  DDP
				
				temp_avgTf_new = (sum / (BUFFLONGFILTER-32));						//%%%%%*****$$$$$  DDP

				flowCalc.avgTf =   0.994*temp_avgTf_old + 0.006*temp_avgTf_new;		//%%%%%*****$$$$$  DDP
				temp_avgTf_old = flowCalc.avgTf;									//%%%%%*****$$$$$  DDP
					
				memcpy(&bufdummy[0],&flow5Sec.bufTr[0],160*4);//copy 160 int to temp
				quickSort(bufdummy, 160);
				for(i = 16,sum = 0; i < BUFFLONGFILTER-16; i++)
					sum += bufdummy[i];

//				flowCalc.avgTr = (sum / (BUFFLONGFILTER-32));						//%%%%%*****$$$$$  DDP
				temp_avgTr_new = (sum / (BUFFLONGFILTER-32));						//%%%%%*****$$$$$  DDP

				flowCalc.avgTr =   0.994*temp_avgTr_old + 0.006*temp_avgTr_new;		//%%%%%*****$$$$$  DDP
				temp_avgTr_old = flowCalc.avgTr;									//%%%%%*****$$$$$  DDP
//printf("16\n");


				flow5Sec.flow = calcFlow(&flowCalc); //for 5 sec

				if(operMode == simu10pMode)
				{
					finalFlowOutput = 390;
				}
				else if(operMode == simu50pMode)
				{
					finalFlowOutput = 1950;
				}
				else if(operMode == simu100pMode)
				{
					finalFlowOutput = 3900;
				}
//				else if( abs(flow1Sec.flow - flow5Sec.flow) > 19) //30%
//				{
//					finalFlowOutput = flow1Sec.flow;
//						finalFlowOutput = (unsigned int)(finalFlowOutput *3900)/63;	//*					  
//					flow5Sec.transCounter = 0;
//					displayvalue = 0x01;
//				}
//				else if (flow5Sec.transCounter < 320)
//				{
//					finalFlowOutput = flow1Sec.flow;
//					finalFlowOutput = (unsigned int)(finalFlowOutput *3900)/63;//*						  
//					flow5Sec.transCounter++;
//					displayvalue = 0x02;
//				}
				else
				{	deltaFlow = (flow1Sec.flow - flow5Sec.flow)/63;
					if (deltaFlow < 0)
					 { deltaFlow = -1 * deltaFlow;
					 }
	   				if (flow5Sec.transCounter < trans_count)     
					//  if transient counter is less than trans_count i.e. process is not steady
					{ 					
						deltaFlow = 50*deltaFlow;	   // 100/50 = 2% error band
						
					}
					else   //  transient counter is equal to trans_count i.e. process is steady within 2% since last trans_count time
					{ 					
						deltaFlow = 20*deltaFlow;	   // 100/20 = 5% error band
					}
					
					if (deltaFlow >= 1)
					 { deltaFlow = 1;
					   flow5Sec.transCounter = 0;
					   displayvalue = 0x03;
					 }
					else
					{  flow5Sec.transCounter++;
						if (flow5Sec.transCounter >=  trans_count)
						{
						flow5Sec.transCounter =  trans_count;
						deltaFlow = deltaFlow * deltaFlow; //make sqr of %diff
						displayvalue = 0x01;
						}
						else
						{
						deltaFlow = 1;
						displayvalue = 0x02;
						}
					 }
					finalFlowOutput = flow5Sec.flow*(1-deltaFlow) +flow1Sec.flow*deltaFlow;
					finalFlowOutput = (unsigned int)(finalFlowOutput *3900)/63;		//*				  
//					flow5Sec.transCounter = 320; //clamp
					
				}
			}

			if( finalFlowOutput < 0)
			{	
				finalFlowOutput = -1*finalFlowOutput;
				if( finalFlowOutput > 150)
				OR_UR = 1;     /// DDP if underrange (i.e. reverse flow) by 5% i.e. 150 counts then set Overrange_underrange bit to BLINK LED	 06/05/13

				finalFlowOutput = 0;
			}
			else if( finalFlowOutput >= 3997)	//102.5%		 ///////// DDP

//			if( finalFlowOutput >= 3910)	//102.5%		 ///////// DDP
			//if( finalFlowOutput >= 4095)			 ///////// DDP
			{
				RELAYK2ON;
				relayk2disturb = 10*16; //10 seconds flow should be stable before removing overrange indication
										// 16 pulses per seconds
				finalFlowOutput = 3997;		/// 102.5%	   ///////// DDP

//				finalFlowOutput = 3910;		/// 102.5%	   ///////// DDP
				//finalFlowOutput = 4095;                  ///////// DDP
				OR_UR = 1;     /// DDP if overrange -> set Overrange_underrange bit to BLINK LED	 06/05/13
			}
			else if(relayk2disturb)
			{
				relayk2disturb--;
				if(relayk2disturb==0)
					{
					RELAYK2OFF; //turnoff overrange relay after flow is stable for 15 seconds
					OR_UR = 0;
					}
			 }
		
			else
			 {
			 OR_UR = 0;
			 }

			inpBuf.healthCheck = (inpBuf.healthCheck<<1)|(~((FIO4PIN & healthCheckPin)>>0) & 0x01);
			inpBuf.zeroFlow    = (inpBuf.zeroFlow<<1)   |(~((FIO0PIN & zeroFlowPin)   >>9)& 0x01);
			//inpBuf.zeroFlow    = (inpBuf.zeroFlow<<1)   |(~((FIO4PIN & zeroFlowPin)   >>14)& 0x01);
			inpBuf.simu10p     = (inpBuf.simu10p<<1)    |(~((FIO0PIN & simu10pPin)    >>4) & 0x01);
			inpBuf.simu50p     = (inpBuf.simu50p<<1)    |(~((FIO0PIN & simu50pPin)    >>5) & 0x01);
			inpBuf.measurement = (inpBuf.measurement<<1)|(~((FIO0PIN & measurementPin)>>7) & 0x01);
			inpBuf.simu100p    = (inpBuf.simu100p<<1)   |(~((FIO0PIN & simu100pPin)   >>8) & 0x01);
			inpBuf.zeroConfirm = (inpBuf.zeroConfirm<<1)|(~((FIO4PIN & zeroConfirmPin)>>14) & 0x01);
			//inpBuf.zeroConfirm = (inpBuf.zeroConfirm<<1)|(~((FIO0PIN & zeroConfirmPin)>>9) & 0x01);

			if(inpBuf.zeroFlow == 0xFF)
			{
				operMode = zeroFlowMode;
//				statusLedOff;							  /////##### DDP
//				statusLedBlink = 0;						  /////##### DDP
				if (zeroMode.ready == 0)
				{
				 statusLedOff;
				 statusLedBlink = 0;
				}
				else
				{
				 statusLedBlink = 1;
				}										  /////##### DDP


				if((inpBuf.zeroConfirm == 0xFF) && (zeroMode.ready)) //use in zero cal mode
				{
					//j=10;

					zeroMode.ready = 0;
					zeroMode.idx = 0; //next zero time we will set ready after proper time
					memset(&inpBuf.healthCheck,0x00,7);
					for(i = 0,sum = 0; i <300 ; i++)
						sum += zeroMode.bufTf[i];
					flowCalc.avgTf = (unsigned int )( sum /300); //find average flowCalc.avgTf

					for(i = 0,sum = 0; i <300 ; i++)
						sum += zeroMode.bufTr[i];
					flowCalc.avgTr = (unsigned int )(sum /300); //find average flowCalc.avgTr
					avgTcorr = flowCalc.avgTr - flowCalc.avgTf;	//find and apply correction
					printf("\r\n0 Flow confirmation OK%d avTr:%d avTf %d\r\n",avgTcorr,flowCalc.avgTr,flowCalc.avgTf);
					//copy param data in IAP_Buf to store in ROM
					i = (channelId>>1)-1; //get chaid 0,1,2 from 3,5,7
					i = i<<4; //multiply channel id by 16 to get 0,16,32 byte offset for ch 0,1,2
					memcpy(IAP_Buf, ReadCorrection,48); //copy all channels data, and modify current channel data
				    IAP_Buf[i+0] =IAP_Buf[i+4] = avgTcorr & 0xFF; 
				    IAP_Buf[i+1] =IAP_Buf[i+5] = (avgTcorr >>8 ) & 0xFF; 
				    IAP_Buf[i+2] =IAP_Buf[i+6] = (avgTcorr >>16) & 0xFF; 
				    IAP_Buf[i+3] =IAP_Buf[i+7] = (avgTcorr >>24) & 0xFF; 
					IAP_Buf[i+8] = (~avgTcorr)&0xff;
					IAP_Buf[i+9] = ((~avgTcorr)>>8)&0xff;
					IAP_Buf[i+10] = ((~avgTcorr)>>16)&0xff;
					IAP_Buf[i+11] = ((~avgTcorr)>>24)&0xff;
				   storeParams();	//store correction
				}
				memset(&inpBuf.healthCheck,0x00,7);
			}
 			if(inpBuf.healthCheck == 0xFF)
			{
				operMode = healthCheckMode;
				memset(&inpBuf.healthCheck,0x00,7);
				statusLedBlink=1;
			}
			else if(inpBuf.simu10p == 0xFF)
			{
				operMode = simu10pMode;
				memset(&inpBuf.healthCheck,0x00,7);
				statusLedBlink=1;
			}
			else if(inpBuf.simu50p == 0xFF)
			{
				operMode = simu50pMode;
				memset(&inpBuf.healthCheck,0x00,7);
				statusLedBlink=1;
			}
			else if(inpBuf.simu100p == 0xFF)
			{
				operMode = simu100pMode;
				memset(&inpBuf.healthCheck,0x00,7);
				statusLedBlink=1;
			}
			else if(inpBuf.measurement == 0xFF)
			{
				operMode = measureMode;
				memset(&inpBuf.healthCheck,0x00,7);
			}
			if((faultCountF < 10) && (faultCountR < 10)) //no fault detected
			{
				faultBits &= ~1; //TDC ok
			}
			else if(( faultCountF >= 10) ||(faultCountR >=10)) 
			{
				faultCountF = 10; //clamp
				faultCountR = 10; //clamp
				faultBits |= 1; //TDC fail
			}
			if((operMode == measureMode)&& (faultBits == 0))
			{
				RELAYK1ON; //measureMode and no fault
				statusLedOn;
				if(OR_UR == 0)
				statusLedBlink=0;
				else
				statusLedBlink=1;
			}
			else
			{
				RELAYK1OFF; //error
			}
			if( faultBits )//any fault
			{
				statusLedBlink = 0;
				statusLedOff;
				finalFlowOutput = 0;            ///////   DDP
			}
			
			
			if (statusLedBlink)
			{
				if (j>=1)
				{
					statusLedOff;
					j--;
				}
				else
				{
				if (toggle)
				{
					toggle = 0;	
					statusLedOff;	
				}
				else
				{
					toggle = 1;
					statusLedOn;	
				}
				}
			}
 		   
		   if(k < 16)
		   { 
		   	 finalFlowOutput = 0;
			 k++;
		   }
		 
			 outputDAC2(finalFlowOutput,0xf);//send outputdata to DAC
			 if( receiveIdxTx != receiveIdx)
			 {	
				i =dacRxbuff[receiveIdxTx++];  //response received from DAC
//				if(((i&0xf0) == 0x0)&&(i&0xf)) //some error
//					faultBits |= 2; //DAC fault
//				else
//					faultBits &= ~2;
				if(receiveIdxTx > 0x40)
					receiveIdxTx = 0;
			 }


//               printf("%d \t %d\r\n",flowCalc.avgTf,TDC_100US);
//			printf("%d  %d  %d \r\n",TDC_status_f,TDC_status_r,(faultCountF+faultCountR));			
			printf("%4.2f  %4.2f  %d  %d  %d  %d  %d %d %d %02X\r\n",((finalFlowOutput*63)/3900),flow1Sec.flow,(flowCalc.avgTr-flowCalc.avgTf-avgTcorr),(Diff_Rv_32+Diff_Rv_21-Diff_Fr_32-Diff_Fr_21),Diff_Rv_32,Diff_Fr_32,avgTcorr,faultCountF,faultCountR,displayvalue);

//			printf("%4.2f  %4.2f  %d  %d  %d  %d  %d  %d  %d %d %d %02X\r\n",((finalFlowOutput*63)/3900),flow1Sec.flow,(flowCalc.avgTr-flowCalc.avgTf-avgTcorr),(Diff_Rv_32+Diff_Rv_21-Diff_Fr_32-Diff_Fr_21),Diff_Rv_32,Diff_Rv_21,Diff_Fr_32,Diff_Fr_21,avgTcorr,faultCountF,faultCountR,displayvalue);
//			printf(" %4.2f  %4.2f  %d  %d  %d  %d  %d  %d\r\n",((finalFlowOutput*63)/3900),flow1Sec.flow,(flowCalc.avgTr-flowCalc.avgTf-avgTcorr),Tcorr_1,Tcorr_2,(Diff_Rv_32+Diff_Rv_21-Diff_Fr_32-Diff_Fr_21),(Diff_Rv_32+Diff_Rv_21),(Diff_Fr_32+Diff_Fr_21));
//			printf(" %4.2f\t%4.2f\t%4.2f\t%4.2f\t%d\t%d\t%d\t%d\t%d\t%d\r\n",((finalFlowOutput*63)/3900),flow1Sec.flow,flow5Sec.flow,(100*deltaFlow),(flowCalc.avgTr-flowCalc.avgTf-avgTcorr),(flowCalc.avgTr-flowCalc.avgTf),avgTcorr_1,avgTcorr_2,avgTcorr_3,channelId);	
//			printf(" %4.2f\t%4.2f\t%4.2f\t%4.2f\t%d\t%d\t%d\t%d\t%d\r\n",((finalFlowOutput*63)/3900),flow1Sec.flow,flow5Sec.flow,(100*deltaFlow),(flowCalc.avgTr-flowCalc.avgTf-avgTcorr),(Cur_val.TdcRv-Cur_val.TdcFr),flow5Sec.transCounter,(faultCountF+faultCountR),channelId);	
//			printf(" %4.2f  %4.2f  %4.2f  %4.2f  %d  %d  %d  %d  %02X\r\n",((finalFlowOutput*63)/3900),flow1Sec.flow,flow5Sec.flow,(deltaFlow_temp*100),flowCalc.avgTr,flowCalc.avgTf,(flowCalc.avgTr-flowCalc.avgTf-avgTcorr),(Cur_val.TdcRv-Cur_val.TdcFr-avgTcorr),operMode);	
			
//			printf(" %4.2f\t%d\t%d\t%d\t%02X\r\n",((finalFlowOutput*63)/3900),Cur_val.TdcRv,Cur_val.TdcFr,(Cur_val.TdcRv-Cur_val.TdcFr),operMode);	
		}//if(startScan) //set at every 60 Ms
    }//while 1
}

void delayUs(int delayInUs)
{
	/* setup timer #1 for delay	*/
	T2TCR = 0x02;		/* reset timer */
	T2PR  = 0x00;		/* set prescaler to zero */
	T2MR0 = delayInUs * 12;//(12000000 / 1000000-1);// (delayInUs * 12000000 / 1000000)-1;
	T2IR  = 0xff;		/* reset all interrrupts */
	T2MCR = 0x04;		/* stop timer on match */
	T2TCR = 0x01;		/* start timer */
	/* wait until delay time has elapsed */
	while (T2TCR & 0x01);
  return;
}
void delayMs(float delayInMs)
{
	/*	* setup timer #1 for delay	*/
	T1TCR = 0x02;		/* reset timer */
	T1PR  = 0x00;		/* set prescaler to zero */
	T1MR0 = delayInMs;	//* (12000000 / 1000-1);
	T1IR  = 0xff;		/* reset all interrrupts */
	T1MCR = 0x04;		/* stop timer on match */
	T1TCR = 0x01;		/* start timer */
	/* wait until delay time has elapsed */
	while (T1TCR & 0x01);
  return;
}
void InitTDC(void)
{
	SPIBuff[0] = 0x50;
	spTxRx(SPIBuff,1); //reset

	SPIBuff[0] = 0x80;
	SPIBuff[1] = 0x33;
	SPIBuff[2] = 0x04;   //8A
	SPIBuff[3] = 0x68;	// 0x68 ( 6a for testing  ip 500 us pulse from cro start - rising edge  stop - falling edge)	//78;
	spTxRx(SPIBuff,4); //r0 define edge, clk select, 

	SPIBuff[0] = 0x81;
	SPIBuff[1] = 0x21;		// 0x21 for 1st hit, 0x31 for 2nd hit and 0x41 for 3rd hit *****  DDP
	SPIBuff[2] = 0x44;	//42		 			//44;
	SPIBuff[3] = 0x00;
	spTxRx(SPIBuff,4); //R1 set meas type from strt to hit1 

	SPIBuff[0] = 0x82;
	SPIBuff[1] = 0xE0;				//E0;
	SPIBuff[2] = 0x00;				 //32
	SPIBuff[3] = 0x00;
	spTxRx(SPIBuff,4); //expeceted stop pulse delay

	SPIBuff[0] = 0x83;
	SPIBuff[1] = 0x38;	//18			//08;
	SPIBuff[2] = 0x01;		//18		 //33
	SPIBuff[3] = 0x00;
	spTxRx(SPIBuff,4); //expeceted stop pulse delay hit2

	SPIBuff[0] = 0x84;
	SPIBuff[1] = 0x20;
	SPIBuff[2] = 0x02;				 //34
	SPIBuff[3] = 0x00;
	spTxRx(SPIBuff,4); //expeceted stop pulse delay hit3

	SPIBuff[0] = 0x85;
	SPIBuff[1] = 0x08;	  //   DIS_PHASE_NOISE   enabled     *******DDP
	SPIBuff[2] = 0x00;
	SPIBuff[3] = 0x00;
	spTxRx(SPIBuff,4); //fire, temp meas, etc

//	SPIBuff[0] = 0x03; 		//calibrate clk		///***DDP
//	spTxRx(SPIBuff,1); 
// NOT here otherwise it will introduce ref clock jitter in to the delta T as init_TDC is placed between TDC_rev and TDC_for routine.

	return;
}  
unsigned int Tdc_rev(BYTE mode)
{
unsigned int  Tdc_reverse;
//		W3LO;		//w3 LOW				                 // ***$$$$DDP
	
	SPIBuff[0] = 0x70;	//init opcode, start measurement
	spTxRx(SPIBuff,1);
	W4LO;		// w4 Low 
	delayUs(4);		//delay for o occur
	startlatchPinHI;	//rst for d-latch
	delayUs(1);		//delay for   occur
	enablestartPinHI;	//en_start
	trigger(2);		//REVERSE TRIGGER PULSE
	W4HI;		// w4 High start pulse for analog switch for reverse time measurement
	startlatchPinLO;	//rst latch 
	enablestartPinLO;		//en_start
	delayUs(240);		//delay for stop to occur
	W3LO;		//w3 LOW				                 // ***$$$$DDP
	delayUs(4);		//delay for stop to occur
	zcdSilenceLO;	//zcdSilence, enable comparator setpoint 
	validatelatchPinHI;	//en_stop tdc
	delayUs(90);		//delay for stop to occur   (1020)	bm
	W3HI;		//w3 High						                 // ***$$$$DDP
	validatelatchPinLO;	//en_stop tdc
	delayUs(60);		//delay for stop to occur
	SPIBuff[0] = 0xB4; 	//cstatus
	SPIBuff[1] = 0xff; 	//cstatus
	SPIBuff[2] = 0xff; 	//cstatus
	spTxRx(SPIBuff,3); 	//read status, wait for meas over
	TDC_status_r = SPIBuff[2];				//// *********DDP
	if((SPIBuff[2] & 0x38) == 0x20)			//// *********DDP
//	if((SPIBuff[2] & 0x3f) == 0x21)//error routine to be added if no stop hits received 
	{										//diagnostic feature for no hit received 
		SPIBuff[0] = 0xB0; 		//cread data 1st hit
		spTxRx(SPIBuff,5);
		Tdc_reverse = ((SPIBuff[1]<<8) | SPIBuff[2]);
		Tdc_reverse = (Tdc_reverse<<16)|((SPIBuff[3]<<8) | SPIBuff[4]);
		faultCountR = 0;
	}
	else
		faultCountR++;		
	zcdSilenceHI;	//zcdSilence, disable comparator setpoint 
	return Tdc_reverse;
}
unsigned int Tdc_for(BYTE mode)
{
unsigned int  Tdc_forward;
//		W2LO;		// w2 low                                    // ***$$$$DDP
	SPIBuff[0] = 0x70;	//init opcode, start measurement
	spTxRx(SPIBuff,1);
	W1LO;		// W1 Low
	delayUs(4);		//delay for stop to occur
	startlatchPinHI;	//rst for d-latch
	delayUs(1);		//delay for occur
	enablestartPinHI;	//en_start 
	trigger(1);		//FORWARD TRIGGER PULSE
	W1HI;		// W1  start pulse for analog switch for forward time measurement
	startlatchPinLO;	//rst latch 
	enablestartPinLO;		//en_start
	if(mode == healthCheckMode)
		delayUs(76);   //total 100us from trigger
	else
		delayUs(240);	//delay for stop to occur
	W2LO;		// w2 low  				                    // ***$$$$DDP
	delayUs(4);		//delay for stop to occur
	zcdSilenceLO;	//zcdSilence, enable comparator setpoint 
	validatelatchPinHI;	//en_stop tdc
	if(mode == healthCheckMode)
	{
		triggerDelay();//delayUs(8);   //total 100us from trigger
		trigger(2); 	//generate reverse trigger ur self
		triggerDelay();
		trigger(2); 	//we may need delay bet 2 trigger to pass though analog section & crystals
		triggerDelay();
		trigger(2);
		delayUs(10);
	}
	else
		delayUs(90);	//delay for stop to occur
	W2HI;		// w2 high stop pulse for analog switch for forward time measurement                // ***$$$$DDP		
	validatelatchPinLO;	//en_stop tdc
	delayUs(60);		//delay for stop to occur
	SPIBuff[0] = 0xB4; 	//cstatus
	SPIBuff[1] = 0xff; 	//cstatus
	SPIBuff[2] = 0xff; 	//cstatus
	spTxRx(SPIBuff,3); 	//read status, wait for meas over
	TDC_status_f = SPIBuff[2];				//// *********DDP
	if((SPIBuff[2] & 0x38) == 0x20)		 /////////*****DDP
//	if((SPIBuff[2] & 0x3f) == 0x21)//error routine to be added if no stop hits received 
	{										//diagnostic feature for no hit received 
		SPIBuff[0] = 0xB0; 		//cread data
		spTxRx(SPIBuff,5);
		Tdc_forward = ((SPIBuff[1]<<8) | SPIBuff[2]);
		Tdc_forward = (Tdc_forward<<16)|((SPIBuff[3]<<8) | SPIBuff[4]);
		faultCountF = 0;		
	} 
	else
		faultCountF++;		
	zcdSilenceHI;	//zcdSilence, disable comparator setpoint 
	return Tdc_forward;
}

float calcFlow(struct fC *flow) //calculates flow from time diff of TDC
{
	float T_factor,Tx,Q_flow;
	int delta;
	Tx =(flow->avgTf + flow->avgTr )/2.0;
	Tx =(Tx - T3RDHIT - T2STEEL)/6.2426;
	delta =  (flow->avgTr - flow->avgTf  - avgTcorr);
	T_factor = delta;
	T_factor /= (flow->avgTf - 2*Tx - T2STEEL - T3RDHIT );
 	T_factor /= (flow->avgTr - 2*Tx - T2STEEL - T3RDHIT );
	Q_flow = (T_factor *  MKfact1);
	Q_flow = (Q_flow * MKfact2);
	return 	Q_flow;
} 

void triggerDelay(void)
{
			__nop();		//__nop();		__nop();		__nop();
	return;
}
void trigger(int channel)
{
if( channel == 1)
{
	FIO1SET  |= triggerFPin;//trigger pulse for forword
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();	//	__nop();
	FIO1CLR  |= triggerFPin;
}
else if( channel == 2) //trigger pulse for reverse
{
	FIO1SET  |= triggerRPin;//testing purpose delay of 1us to be used instead of pwm trigger
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();	//	__nop();
	FIO1CLR  |= triggerRPin;	  
}
return;
}
 void quickSort(unsigned int *arr, int elements)
{
  #define  MAX_LEVELS  300
  int  piv, beg[MAX_LEVELS], end[MAX_LEVELS], i=0, L, R, swap ;
  beg[0]=0; end[0]=elements;

  	while (i>=0) 
  	{
    	L=beg[i]; 
		R=end[i]-1;
    	if (L<R) 
		{
      		piv=arr[L];
      		while (L<R) 
			{
        		while (arr[R]>=piv && L<R) R--; if (L<R) arr[L++]=arr[R];
        		while (arr[L]<=piv && L<R) L++; if (L<R) arr[R--]=arr[L]; 
			}							  
      		arr[L]=piv; 
			beg[i+1]=L+1; 
			end[i+1]=end[i]; 
			end[i++]=L;
      		if (end[i]-beg[i]>end[i-1]-beg[i-1]) 
			{
        		swap=beg[i]; 
				beg[i]=beg[i-1]; 
				beg[i-1]=swap;
        		swap=end[i]; 
				end[i]=end[i-1]; 
				end[i-1]=swap; 
			}
		}
    	else 
		{
      		i--; 
		}
	}
	return;
}
void storeParams(void)
{
	int writeSec, stat;
	writeSec = getSectorIndex(CONFIG_ADDR);
//	printf( "writesec=%x\r\n", writeSec);
	stat = IAP_PrepareSec(writeSec,writeSec);
//	printf( "PrepSec=%x\r\n", stat);
	VICIntEnClr  = 0xffffffff; /* disable all interrupt     */
	stat = IAP_EraseSec(writeSec,writeSec);
	VICIntEnable = (1<<PWM0_1_INT)|(1<<UART2_INT); /* Enable Interrupt     */
	printf( "EraseSec=%x\r\n", stat);
	stat = IAP_PrepareSec(writeSec,writeSec);
	printf( "PrepSec=%x\r\n", stat);
	VICIntEnClr  = 0xffffffff; /* disable all interrupt     */
	stat = IAP_CopyRAMToFlash(CONFIG_ADDR,(unsigned long)IAP_Buf,256);
	VICIntEnable = (1<<PWM0_1_INT)|(1<<UART2_INT); /* Enable Interrupt     */
	printf( "CopytoFlash=%x\r\n", stat);
	return;
}
void functiondelay100(void)
{
	char i;
	for (i=0;i<81;i++)
	{
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
			__nop();		__nop();		__nop();		__nop();
		
	}
	return;
}
