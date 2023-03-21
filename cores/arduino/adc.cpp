/*
BSD 3-Clause License

Copyright (c) 2021-2022 WPI Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "adc.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/




/*******************************************************************************
 * ADC Functions
 ******************************************************************************/
 
void analogReadResolution(lpadc_conversion_resolution_mode_t bitsel)
{
    lpadc_conv_command_config_t lpadcCommand_config;
	  
	LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);
	lpadcCommand_config.channelNumber = 0;
	lpadcCommand_config.conversionResolutionMode = bitsel;
	LPADC_SetConvCommandConfig(ADC0, 1U, &lpadcCommand_config);
}

void analogReference(uint8_t reference)
{
	LPADC_Enable(ADC0,false); 

	ADC0->CFG |= ADC_CFG_REFSEL(reference);
	
	LPADC_Enable(ADC0,true); 	
}


void adc_config()
{
	lpadc_config_t lpadc_config;
	lpadc_conv_command_config_t lpadcCommand_config;
	lpadc_conv_trigger_config_t lpadcTrigger_config;
	
	// GPIO pin config
	CLOCK_EnableClock(kCLOCK_Iocon);
	CLOCK_EnableClock(kCLOCK_Gpio0);
	gpio_pin_config_t gpio0_pin23_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };  
	GPIO_PinInit(GPIO, 0U, 23U, &gpio0_pin23_config);
	IOCON_PinMuxSet(IOCON, 0U, 23U, ADC_PIN_CONFIG);

	// Clock setup
	CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 8U, true);
    CLOCK_AttachClk(kMAIN_CLK_to_ADC_CLK);

    // Disable LDOGPADC power down 
    POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);
		
	// ADC config
	LPADC_GetDefaultConfig(&lpadc_config);
	lpadc_config.enableAnalogPreliminary = true;
	lpadc_config.referenceVoltageSource = kLPADC_ReferenceVoltageAlt2;
	lpadc_config.conversionAverageMode = kLPADC_ConversionAverage128;
	LPADC_Init(ADC0,&lpadc_config);
	
	// ADC calibration
	LPADC_DoOffsetCalibration(ADC0);
	LPADC_SetOffsetValue(ADC0,0x10U,0x10U);
	LPADC_DoAutoCalibration(ADC0);
	
	// ADC command config
	LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);
	lpadcCommand_config.channelNumber = 0;
	lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
	LPADC_SetConvCommandConfig(ADC0, 1U, &lpadcCommand_config);
	
	// Set trigger configuration
	LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
	lpadcTrigger_config.targetCommandId = 1U;
	lpadcTrigger_config.enableHardwareTrigger = false;
	LPADC_SetConvTriggerConfig(ADC0,0U,&lpadcTrigger_config);
}


void adc_init(uint8_t adc_pin)
{
	lpadc_conv_command_config_t lpadcCommand_config;
  	lpadc_conv_trigger_config_t lpadcTrigger_config;
    lpadc_config_t lpadc_config;
	static uint8_t initflag=0;
	gpio_pin_config_t adcGpio_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };

	CLOCK_EnableClock(kCLOCK_Iocon);

	if(initflag==0)
	{
		// Clock setup
		CLOCK_SetClkDiv(kCLOCK_DivAdcAsyncClk, 8U, true);
		CLOCK_AttachClk(kMAIN_CLK_to_ADC_CLK);

		// Disable LDOGPADC power down 
		POWER_DisablePD(kPDRUNCFG_PD_LDOGPADC);
			
		// ADC config
		LPADC_GetDefaultConfig(&lpadc_config);
		lpadc_config.enableAnalogPreliminary = true;
		lpadc_config.referenceVoltageSource = kLPADC_ReferenceVoltageAlt2;
		lpadc_config.conversionAverageMode = kLPADC_ConversionAverage128;
		LPADC_Init(ADC0,&lpadc_config);
		
		// ADC calibration
		LPADC_DoOffsetCalibration(ADC0);
		LPADC_SetOffsetValue(ADC0,0x10U,0x10U);
		LPADC_DoAutoCalibration(ADC0);
		initflag=1;
	}

	switch(adc_pin)
	{
		case 0x17://ADC0_0 P0_23
			CLOCK_EnableClock(kCLOCK_Gpio0);
			GPIO_PinInit(GPIO, 0U, 23U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 0U, 23U, ADC_PIN_CONFIG);

			// ADC command config
			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);					
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
			lpadcCommand_config.channelNumber = 0;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 1U, &lpadcCommand_config);

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 1;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,0,&lpadcTrigger_config);
		break;

		case 0x0A://ADC0_1 P0_10
			CLOCK_EnableClock(kCLOCK_Gpio0);
			GPIO_PinInit(GPIO, 0U, 10U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 0U, 10U, ADC_PIN_CONFIG);

			// ADC command config
			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);					
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
			lpadcCommand_config.channelNumber = 1;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 2U, &lpadcCommand_config);

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 2;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,1,&lpadcTrigger_config);		
		break;

		case 0x0F://ADC0_2 P0_15
			CLOCK_EnableClock(kCLOCK_Gpio0);
			GPIO_PinInit(GPIO, 0U, 15U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 0U, 15U, ADC_PIN_CONFIG);

			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);					
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
			lpadcCommand_config.channelNumber = 2;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 3U, &lpadcCommand_config);	

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 3;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,2,&lpadcTrigger_config);
		break;

		case 0x1F://ADC0_3 P0_31
			CLOCK_EnableClock(kCLOCK_Gpio0);
			GPIO_PinInit(GPIO, 0U, 31U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 0U, 31U, ADC_PIN_CONFIG);

			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);					
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
			lpadcCommand_config.channelNumber = 3;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 4U, &lpadcCommand_config);	

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 4;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,3,&lpadcTrigger_config);
		break;

		case 0x28://ADC0_4 P1_8
			CLOCK_EnableClock(kCLOCK_Gpio1);
			GPIO_PinInit(GPIO, 1U, 8U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 1U, 8U, ADC_PIN_CONFIG);

			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);					
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
			lpadcCommand_config.channelNumber = 4;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 5U, &lpadcCommand_config);	

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 5;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,4,&lpadcTrigger_config);
		break;

		case 0x10://ADC0_8 P0_16
			CLOCK_EnableClock(kCLOCK_Gpio0);
			GPIO_PinInit(GPIO, 0U, 16U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 0U, 16U, ADC_PIN_CONFIG);

			// ADC command config
			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
			lpadcCommand_config.channelNumber = 0;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 9U, &lpadcCommand_config);

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 9;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,8,&lpadcTrigger_config);
		break;

		case 0x20://ADC0_11 P1_0
			CLOCK_EnableClock(kCLOCK_Gpio1);
			GPIO_PinInit(GPIO, 1U, 0U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 1U, 0U, ADC_PIN_CONFIG);

			// ADC command config
			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
			lpadcCommand_config.channelNumber = 3;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 12U, &lpadcCommand_config);

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 12;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,11,&lpadcTrigger_config);
		break;

		case 0x29://ADC0_12 P1_9
			CLOCK_EnableClock(kCLOCK_Gpio1);
			GPIO_PinInit(GPIO, 1U, 9U, &adcGpio_config);
			IOCON_PinMuxSet(IOCON, 1U, 9U, ADC_PIN_CONFIG);

			// ADC command config
			LPADC_GetDefaultConvCommandConfig(&lpadcCommand_config);
			lpadcCommand_config.sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
			lpadcCommand_config.channelNumber = 4;
			lpadcCommand_config.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
			LPADC_SetConvCommandConfig(ADC0, 13U, &lpadcCommand_config);

			// Set trigger configuration
			LPADC_GetDefaultConvTriggerConfig(&lpadcTrigger_config);
			lpadcTrigger_config.targetCommandId = 13;
			lpadcTrigger_config.enableHardwareTrigger = false;
			LPADC_SetConvTriggerConfig(ADC0,12,&lpadcTrigger_config);
		break;

        default:
		break;

	}
	CLOCK_DisableClock(kCLOCK_Iocon);
}

uint32_t analogRead(uint8_t adc_pin)
{
	lpadc_conv_result_t lpadcResult_config;
	uint32_t channelid=0;
    channelid = adc_outtrigger_id(adc_pin);
	LPADC_DoSoftwareTrigger(ADC0, 1<<channelid); /* 1U is trigger0 mask. */
	while (!LPADC_GetConvResult(ADC0, &lpadcResult_config, 0U));
    if(lpadcResult_config.commandIdSource == (channelid+1))
	{
		return (lpadcResult_config.convValue >> 3U);
	}
	ADC0->SWTRIG = 0x00;
	// return 0;//by dylan
}

uint8_t adc_outtrigger_id(const uint8_t adc_pin)
{
	uint8_t out=100;
	switch(adc_pin)
	{
		case 0x17://ADC0_0 P0_23
			out = 0;

		break;

		case 0x0A://ADC0_1 P0_10
			out = 1;
		break;

		case 0x0F://ADC0_2 P0_15
			out = 2;

		break;

		case 0x1F://ADC0_3 P0_31
            out = 3;

		break;

		case 0x28://ADC0_4 P1_8
			out = 4;

		break;

		case 0x10://ADC0_8 P0_16
            out = 8;

		break;

		case 0x0B://ADC0_9 P0_11
            out = 9;

		break;

		case 0x0C://ADC0_10 P0_12
            out = 10;

		break;

		case 0x20://ADC0_11 P1_0
			out = 11;

		break;

		case 0x29://ADC0_12 P1_9
			out = 12;

		break;

        default:

		break;	
	}
	return out;
}


