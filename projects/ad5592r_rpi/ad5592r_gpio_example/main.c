/********************************************************************************
 * Copyright (C) 2023 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include "linux_spi.h"
#include "no_os_spi.h"
#include "ad5592r.h"

//Helper macro for 12-bit full scale values
#define FULL_SCALE_12b      0x0FFFu

//Initialization structure for the SPI peripheral
static struct no_os_spi_init_param spiInitParams = {
	.device_id    = 0,                  //SPI0
	.max_speed_hz = 1000000,            //1MHz
	.chip_select  = 0,                  //CS0
	.mode         = NO_OS_SPI_MODE_2,   //Mode 2
	.bit_order    = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
	.platform_ops = &linux_spi_ops,
	.extra        = NULL,
	.parent       = NULL
};

//SPI bus handle created by the No-Os platform drivers
static struct no_os_spi_desc* my_spi;


static struct ad5592r_dev my_ad5592 = {
	.num_channels = 8,  //AD5592 has 8 channels
	.channel_modes = { 	//Configure the channels as we need:
		CH_MODE_DAC,
		CH_MODE_ADC,
		CH_MODE_GPO,    //Channels 2 and 3 are GP Ouputs
		CH_MODE_GPO,    //for this exercise
		CH_MODE_UNUSED,
		CH_MODE_GPI,
		CH_MODE_GPI,
		CH_MODE_GPI
	}
};
//Initialization parameters. Enable internal reference
static struct ad5592r_init_param ad5592Init = { .int_ref = 1 };


/***** Local Prototypes ******/
static float rawTo_mVolts(uint16_t raw);
static uint16_t mVoltsToRaw(float raw);
static float rawToTemperatureC(uint16_t raw);
static uint16_t dacReadback(uint8_t channel);


/**
 * Main entry point to our application.
 *
 * Note: All of the No-OS and Driver calls return back error codes,
 * which in most applications should be checked and handled. For
 * the purposes of making simple tutorial code, those error checks
 * were omitted for readability and easy code flow.
 */
int main(int argc, char* argv[])
{
	//Initialize the SPI bus
	no_os_spi_init(&my_spi, &spiInitParams);

	//Need to set the spiHandle after its been initialized
	my_ad5592.spi = my_spi;

	//Initialize the AD5592
	ad5592r_init(&my_ad5592, &ad5592Init);

	//Alternate red and green blinkenlichten!

	//So - the curve tracer board has a 47-ohm resistor between CH1 and CH2. This
	//means that we have to wiggle both in tandem if we want to blink the light
	//since the CH1 analog output loads down the GPIO (Yeah, took a while to figure
	//that one out!) CH3 doesn't have this issue, it's only tied to the LED.
	printf("Let's blink some blinkenlichten!\n");
	for( int i = 0; i < 10; i++ ) {
		ad5592r_gpio_set(&my_ad5592, 2, 1);
		ad5592r_write_dac(&my_ad5592, 1, 4095); //Set CH! to full-scale
		ad5592r_gpio_set(&my_ad5592, 3, 0);

		usleep(500000); //500000 usec...or 0.5 seconds

		ad5592r_gpio_set(&my_ad5592, 2, 0);
		ad5592r_write_dac(&my_ad5592, 1, 0); //Set CH1 to 0
		ad5592r_gpio_set(&my_ad5592, 3, 1);

		printf("Blink number %d\n", i + 1);
		usleep(500000);
	}

	printf("Done!\n");

	//Read out state of GPIO inputs.
	printf("\nReading state of GPIO pins:\n");
	printf("GPIO Ch 5: %d\n", ad5592r_gpio_get(&my_ad5592, 5));
	printf("GPIO Ch 6: %d\n", ad5592r_gpio_get(&my_ad5592, 6));
	printf("GPIO Ch 7: %d\n", ad5592r_gpio_get(&my_ad5592, 7));
	printf("\n\n");


	//Clean up the SPI bus
	no_os_spi_remove(my_spi);
	return 0;
}



/**
 *  Converts a raw AD5592 sample value to degrees C.
 *  Equation pulled directly from AD5592 datasheet
 *  @param raw - Raw sample value
 *  @return Temperature in Deg C
 */
float rawToTemperatureC(uint16_t raw)
{
	float workingVal;

	workingVal = ((float)raw - ((0.5f / INTERNAL_VREF_VOLTAGE) * 4095.0f));
	workingVal = workingVal / (2654.0f * (2.5f / INTERNAL_VREF_VOLTAGE));
	workingVal += 25.0f;

	return workingVal;
}

/**
 * Converts a raw AD5592 sample value to Millivolts.
 * Assumes the internal reference used, scale x1
 * @param raw - Raw sample value
 * @return Voltage in mV
 */
float rawTo_mVolts(uint16_t raw)
{
	float workingVal;

	workingVal = (float)raw * INTERNAL_VREF_VOLTAGE;
	workingVal = workingVal / (float)FULL_SCALE_12b;
	workingVal *= 1000.0f;
	return workingVal;
}

/**
 * Converts a value in millivolts into a sample value for the
 * AD5592 DAC.  Assumes internal reference, scale x1 used
 * @param mV - Millivolts to convert
 * @return Equivalent sample value
 */
uint16_t mVoltsToRaw(float mV)
{
	float workingVal;
	workingVal = (mV / 1000.0f) * (float)FULL_SCALE_12b;
	workingVal = workingVal / INTERNAL_VREF_VOLTAGE;
	return workingVal;
}

/**
 * Helper function for reading back a DAC-only channel. The driver doesnt
 * have this capability implemented yet, so we'll do it here.  Per the
 * datasheet: Set the DAC_READBACK_REGISTER (0x1) bits 4:3, and channel in
 * 2:0.  Then do an empty read to get the data
 * @param ch - Channel to readback
 * @return Sample value from channel
 */
uint16_t dacReadback( uint8_t ch )
{
	uint16_t spiData;

	spiData = swab16((uint16_t)(AD5592R_REG_DAC_READBACK << 11) | 0x18 | ch);
	no_os_spi_write_and_read(my_ad5592.spi, (uint8_t*)&spiData, 2 );

	spiData = swab16((uint16_t)(AD5592R_REG_NOOP << 11));
	no_os_spi_write_and_read(my_ad5592.spi, (uint8_t*)&spiData, 2 );

	return swab16(spiData);
}
