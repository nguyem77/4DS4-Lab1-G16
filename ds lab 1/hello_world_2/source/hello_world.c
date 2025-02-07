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

void setupUART();
void setupPWM();
void updatePWM_dutyCycle(ftm_chnl_t, float);

int main(void)
{
	int input_DC;
	int input_Servo;
	float dutyCycle_DC;
	float dutyCycle_Servo;
	
	char txbuff[] = "\nPlease enter your desired PWM value\r\n";
	char msg_servo[] = "\nPlease enter your desired Servo value\r\n";
	char ch;
	char inputbuff[8];

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
	setupPWM();
	setupUART();


	/******* Delay *******/
	for(volatile int i = 0U; i < 10000000; i++)
	__asm("NOP");
	PRINTF("%s",txbuff);

	// Control PWM based on UART input
	while (1)
	{
		//set PWM
		UART_WriteBlocking(TARGET_UART, txbuff, sizeof(txbuff) - 1);
		printf("%s", txbuff);
		int i;
		for (i = 0; i < 8; i++) {
			// retrieve bytes one at a time
			UART_ReadBlocking(TARGET_UART, &ch, 1);
			printf("%c", ch);
			if (ch != '!') {
				inputbuff[i] = ch;
			}
			else {
				inputbuff[i] = ch;
				break;
			}
		}
		//convert to integer
		input_DC = atoi(inputbuff);

		
		UART_WriteBlocking(TARGET_UART, msg_servo, sizeof(msg_servo) - 1);
		printf("%s", msg_servo);
		for (i = 0; i < 8; i++) {
			//retrieve bytes one at a time
			UART_ReadBlocking(TARGET_UART, &ch, 1);
			printf("%c", ch);
			if (ch != '!') {
				inputbuff[i] = ch;
			}
			else {
				inputbuff[i] = ch;
				break;
			}
		}
		//convert to integer
		input_Servo = atoi(inputbuff);
		
		//check for valid input
		if ((input_DC >= -100) && (input_DC <= 100) && (input_Servo >= -90) && (input_Servo <= 90)) {
			//from part 1
			dutyCycle_DC = input_DC * 0.025f/100.0f + 0.0615;
			dutyCycle_Servo = input_Servo * 0.05f/180.0f + 0.075;
			updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, dutyCycle_DC);
			updatePWM_dutyCycle(FTM_CHANNEL_Servo_MOTOR, dutyCycle_Servo);

			FTM_SetSoftwareTrigger(FTM_MOTOR, true);
		}
	}
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
