/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_uart.h"
#include "fsl_ftm.h"
#include <stdio.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
#define FTM_MOTOR FTM0
#define FTM_CHANNEL_DC_MOTOR kFTM_Chnl_0
#define FTM_CHANNEL_Servo_MOTOR kFTM_Chnl_3
#define TARGET_UART UART4

void setupPWM();
void updatePWM_dutyCycle(ftm_chnl_t, float);
void setupUART();

#define TARGET_UART UART4
volatile int new_char = 0;
volatile int input_DC, input_Servo;
char msg_pwm[] = "\nPlease enter your desired PWM value\r\n";
char msg_servo[] = "\nPlease enter your desired Servo value\r\n";
volatile float dutyCycle_DC, dutyCycle_Servo;
volatile char ch;
volatile char inputbuff[8];

int main(void)
{
	int i, flag;
	
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
	setupPWM();
	setupUART();

	/******* Delay *******/
	for(volatile int i = 0U; i < 10000000; i++)
		__asm("NOP");


	//set PWM, flag is 0 for PWM input and 1 for Servo input
	flag = 0;
	UART_WriteBlocking(TARGET_UART, msg_pwm, sizeof(msg_pwm) - 1);
	printf("%s", msg_pwm);

	i = 0;
	
	while (1) {
		if(new_char) {
			new_char = 0;
			inputbuff[i] = ch;
			i++;
			//signifies end of string
			if ((ch == '!') || (i >= 8)) {
				//flag == 0 means DC input
				if (!flag) {
					input_DC = atoi(inputbuff);
					//swap to fetching servo input
					flag = 1;
					UART_WriteBlocking(TARGET_UART, msg_servo, sizeof(msg_servo) - 1);
					printf("%s", msg_servo);
				}
				//flag == 1 means Servo input
				else {
					input_Servo = atoi(inputbuff);
					//check for valid input and submit values after servo input is complete
					if ((input_DC >= -100) && (input_DC <= 100) && (input_Servo >= -90) && (input_Servo <= 90)) {
						printf("\nInput is valid! (%d DC and %d Servo)\n", input_DC, input_Servo);
						//from part 1
						dutyCycle_DC = input_DC * 0.025f/100.0f + 0.0615;
						dutyCycle_Servo = input_Servo * 0.05f/180.0f + 0.075;
						updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, dutyCycle_DC);
						updatePWM_dutyCycle(FTM_CHANNEL_Servo_MOTOR, dutyCycle_Servo);

						FTM_SetSoftwareTrigger(FTM_MOTOR, true);
					}
					//restart cycle
					flag = 0;
					UART_WriteBlocking(TARGET_UART, msg_pwm, sizeof(msg_pwm) - 1);
					printf("%s", msg_pwm);
				}
				//reset counter
				i = 0;
			}
		}
	}
}

// retrieve bytes one at a time
void UART4_RX_TX_IRQHandler()
{
	printf("\nInterrupt triggered!\n");
	UART_GetStatusFlags(TARGET_UART);
	ch = UART_ReadByte(TARGET_UART);
	printf("%c", ch);
	new_char = 1;
}


void setupUART()
{
	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 57600;
	config.enableTx = true;
	config.enableRx = true;
	config.enableRxRTS = true;
	config.enableTxCTS = true;

	UART_Init(TARGET_UART, &config, CLOCK_GetFreq(kCLOCK_BusClk));
	/******** Enable Interrupts *********/
	UART_EnableInterrupts(TARGET_UART, kUART_RxDataRegFullInterruptEnable);
	EnableIRQ(UART4_RX_TX_IRQn);
}




void setupPWM()
{
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam;
	ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;
	ftmParam.chnlNumber = FTM_CHANNEL_DC_MOTOR;
	ftmParam.level = pwmLevel;
	ftmParam.dutyCyclePercent = 7;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;

	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;

	FTM_Init(FTM_MOTOR, &ftmInfo);
	FTM_SetupPwm(FTM_MOTOR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_BusClk));

	ftmParam.chnlNumber = FTM_CHANNEL_Servo_MOTOR;
	ftmParam.level = pwmLevel;
	ftmParam.dutyCyclePercent = 7;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;

	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;

	FTM_Init(FTM_MOTOR, &ftmInfo);
	FTM_SetupPwm(FTM_MOTOR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_BusClk));

	FTM_StartTimer(FTM_MOTOR, kFTM_SystemClock);
}


void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle)
{
	uint32_t cnv, cnvFirstEdge = 0, mod;
	/* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
	assert(-1 != FSL_FEATURE_FTM_CHANNEL_COUNTn(FTM_MOTOR));
	mod = FTM_MOTOR->MOD;
	if (dutyCycle == 0U)
	{
	/* Signal stays low */
	cnv = 0;
	}
	else
	{
	cnv = mod * dutyCycle;
	/* For 100% duty cycle */
	if (cnv >= mod)
	{
	cnv = mod + 1U;
	}
	}
	FTM_MOTOR->CONTROLS[channel].CnV = cnv;
}
