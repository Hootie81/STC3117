/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : stc3117_Driver.h
* Author             : AMS - IMS application, C.Huitema updated from 3115 to suit 3117
* Version            : V01
* Date               : 30 July 2014, updated 17 June 2016
* Description        : STC3117 driver definition
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.

* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/


/* Define to prevent recursive inclusion ---------------------------------------------- */
#ifndef __STC3117_H
#define __STC3117_H

/* Private define --------------------------------------------------------------------- */
#define STC3117_SLAVE_ADDRESS            0xE0    /* STC31xx 8-bit address byte			*/

/*STC3117 registers define ------------------------------------------------------------ */
#define STC3117_REG_MODE                 0x00    /* Mode Register             			*/
#define STC3117_REG_CTRL                 0x01    /* Control and Status Register 		*/
#define STC3117_REG_SOC                  0x02    /* SOC Data (2 bytes) 					*/
#define STC3117_REG_COUNTER              0x04    /* Number of Conversion (2 bytes) 		*/
#define STC3117_REG_CURRENT              0x06    /* Battery Current (2 bytes) 			*/
#define STC3117_REG_VOLTAGE              0x08    /* Battery Voltage (2 bytes) 			*/
#define STC3117_REG_TEMPERATURE          0x0A    /* Temperature               			*/
#define STC3117_REG_CC_ADJ_HIGH          0x1C    /* CC adjustement     					*/
#define STC3117_REG_CC_ADJ_LOW           0x1B    /* CC adjustement    					*/
#define STC3117_REG_VM_ADJ_HIGH          0x1E    /* VM adjustement     					*/
#define STC3117_REG_VM_ADJ_LOW           0x1D    /* VM adjustement     					*/
#define STC3117_REG_OCV                  0x0D    /* Battery OCV (2 bytes) 				*/
#define STC3117_REG_CC_CNF               0x0F    /* Coulomb Counter CC configuration (2 bytes) */
#define STC3117_REG_VM_CNF               0x11    /* Voltage Mode VM configuration (2 bytes)    */
#define STC3117_REG_ALARM_SOC            0x13    /* SOC alarm level         			*/
#define STC3117_REG_ALARM_VOLTAGE        0x14    /* Low voltage alarm level 			*/
#define STC3117_REG_CURRENT_THRES        0x15    /* Current threshold for relaxation 	*/
#define STC3117_REG_RELAX_COUNT          0x16    /* Voltage relaxation counter   		*/
#define STC3117_REG_RELAX_MAX            0x17    /* Voltage relaxation max count 		*/
#define STC3117_REG_ID					 0x18
#define STC3117_REG_RAM     			 0x20    /* General Purpose RAM Registers 		*/
#define STC3117_REG_OCVTAB               0x30	 /* OCV OFFSET table registers			*/
#define STC3117_REG_SOCTAB				 0x50	 /* SOC OFFSET table registers			*/

/*STC3117_REG_MODE Bit mask definition ------------------------------------ 	*/
#define STC3117_VMODE   	0x01	 	/* Voltage mode bit mask 0:mixed mode 1:voltage mode	*/
#define STC3117_BATD_PU		0x02  		/* BATD internal pullup enable 0:nc 1:pulled up	*/
#define STC3117_FORCE_CD  	0x04  		/* 0:CD driven by logic 1:CD forced high		*/
#define STC3117_ALM_ENA		0x08	 	/* Alarm enable bit mask 0:disabled 1:Enabled	*/
#define STC3117_GG_RUN		0x10	 	/* Alarm enable bit mask 0:standby 1:operating mode		*/
#define STC3117_FORCE_CC	0x20	 	/* Force CC bit mask     					*/
#define STC3117_FORCE_VM	0x40	 	/* Force VM bit mask     					*/

/*STC3117_REG_CTRL Bit mask definition ------------------------------------ */
//ALM TBD
#define STC3117_IO0DATA		0x01		/* alm pin status 							*/
#define STC3117_GG_RST		0x02		/* Convertion counter reset					*/
#define STC3117_GG_VM		0x04		/* STC3117 active mode: cc=0, VM=1			*/
#define STC3117_BATFAIL		0x08		/* Battery presence state					*/
#define STC3117_PORDET		0x10	 	/* W = soft reset, R = POR detect			*/
#define STC3117_ALM_SOC		0x20	 	/* Low SOC alarm event						*/
#define STC3117_ALM_VOLT	0x40	 	/* Low voltage alarm event					*/
#define STC3117_UVLOD		0x80		/* UVLO event detection 					*/

/*STC3117 General purpose define ---------------------------------------------------------- */
#define STC3117_ID          0x16    	/* STC3117 ID 										*/
#define STC3117_RAM_SIZE    16      	/* Total RAM size of STC3117 in bytes 				*/
#define STC3117_OCVTAB_SIZE 32      	/* OCVTAB size of STC3117 in bytes 					*/
#define STC3117_SOCTAB_SIZE	16			/* SOCTAB size of STC3117 in bytes					*/
#define VCOUNT				4       	/* counter value for 1st current/temp measurements	*/
#define VM_MODE 			1           // Voltage Mode
#define CC_MODE 			0           // Coulomb Counter Mode
#define MIXED_MODE			0			// Mixed Mode (Voltage + Current)
#define MAX_HRSOC          	51200  		/* 100% in 1/512% units								*/
#define MAX_SOC            	1000   		/* 100% in 0.1% units 								*/
#define STC3117_OK 			0
#define VoltageFactor  		9011      	/* LSB=2.20mV ~9011/4096 - convert to mV         	*/
#define CurrentFactor		24084		/* LSB=5.88uV/R= ~24084/R/4096 - convert to mA  	*/
#define VOLTAGE_SECURITY_RANGE 200

#define RAM_TESTWORD 		0x53A9		/* STC3117 RAM test word - TBC						*/
#define STC3117_UNINIT    0             /* Gas gauge Not Initialiezd state 					*/
#define STC3117_INIT     'I'			/* Gas gauge Init states 							*/
#define STC3117_RUNNING  'R'			/* Gas gauge Running states 						*/
#define STC3117_POWERDN  'D'			/* Gas gauge Stop states 							*/



/*stc3117 configuration structure --------------------------------------------- */
 typedef struct  {
  int Vmode;       		/* 1=Voltage mode, 0=mixed mode 						*/
  int Alm_SOC;     		/* SOC alarm level in %									*/
  int Alm_Vbat;    		/* Vbat alarm level in mV								*/
  int CC_cnf;      		/* nominal battery CC_cnf 								*/
  int VM_cnf;      		/* nominal battery VM cnf 								*/
  int Cnom;        		/* nominal battery capacity in mAh 						*/
  int Rsense;      		/* sense resistor in mOhms								*/
  int RelaxCurrent; 	/* relaxation current(< C/20) in mA						*/
  unsigned char OCVOffset[32];    /* OCV curve adjustment in 0.55mV				*/
  unsigned char SOCOffset[16];    /* OCV curve adjustment in 1/2%				*/
} STC3117_ConfigData_TypeDef;

/*battery output structure ---------------------------------------------------- */
typedef struct  {
  int StatusWord;		/* STC3117 status registers 							*/
  int HRSOC;			/* battery relative SOC (%) in 1/512% 					*/
  int SOC;            	/* battery relative SOC (%) in 0.1% 					*/
  int Voltage;        	/* battery voltage in mV 								*/
  int Current;        	/* battery current in mA 								*/
  int Temperature;    	/* battery temperature in 0.1°C 						*/
  int ConvCounter;		/* STC3117 convertion counter in 0.5s					*/
  int OCV;				/* battery relax voltage in mV 							*/
  int Presence;			/* battery presence										*/
  int ChargeValue;    	/* battery remaining capacity in mAh 					*/
  int RemTime;        	/* battery remaining operating time during discharge 	*/
 } STC3117_BatteryData_TypeDef;
 
/* stc3117 RAM registers structure -------------------------------------------- */
static union InternalRAM {
  unsigned char db[STC3117_RAM_SIZE];  /* last byte holds the CRC 						*/
  struct {
    short TestWord;       /* 0-1 RAM test word									*/
    short HRSOC;          /* 2-3 SOC backup in (1/512%)							*/
    short CC_cnf;         /* 4-5 current CC_cnf 								*/
    short VM_cnf;         /* 6-7 current VM_cnf 								*/
    char SOC;             /* 8  SOC (in %) 										*/
	char STC3117_State;   /* 9  STC3117 working state							*/
	char unused1;         /* 10  -Bytes upto ..STC3117_RAM_SIZE-2 are free				*/
	char unused2;         /* 11  -Bytes upto ..STC3117_RAM_SIZE-2 are free				*/
	char unused3;         /* 12  -Bytes upto ..STC3117_RAM_SIZE-2 are free				*/
	char unused4;         /* 13  -Bytes upto ..STC3117_RAM_SIZE-2 are free				*/
	char unused5;         /* 14  -Bytes upto ..STC3117_RAM_SIZE-2 are free				*/
	char CRC;             /* 15  last byte STC3117_RAM_SIZE-1 is the CRC				*/
  } reg;
} RAMData;


/* Exported functions prototypes----------------------------------------------- */

#ifdef __cplusplus		//c++
extern "C"				//c++
{						//c++
#endif					//c++


int GasGauge_Initialization(STC3117_ConfigData_TypeDef*, STC3117_BatteryData_TypeDef*);
int GasGauge_Reset(void); 
int GasGauge_Stop(void);
int GasGauge_Task(STC3117_ConfigData_TypeDef*, STC3117_BatteryData_TypeDef*);

int STC3117_SetPowerSavingMode(void);
int STC3117_StopPowerSavingMode(void);

int STC3117_EnableBattDPullup(void);
int STC3117_DisableBattDPullup(void);

int STC3117_EnableCharger(void);
int STC3117_DisableCharger(void);

int STC3117_AlarmSet(void);
int STC3117_AlarmStop(void);
int STC3117_AlarmGet(void);
int STC3117_AlarmClear(void);
int STC3117_AlarmSetVoltageThreshold(STC3117_ConfigData_TypeDef*, int);
int STC3117_AlarmSetSOCThreshold(STC3117_ConfigData_TypeDef*, int);

int STC3117_CheckI2cDeviceId(void);
int STC3117_GetRunningCounter(void);

#ifdef __cplusplus	//c++
}					//c++
#endif				//c++

#endif /* __Gasgauge_H */


 
 
/**** END OF FILE ****/
