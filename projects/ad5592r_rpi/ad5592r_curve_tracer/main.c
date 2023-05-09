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
#include "linux_spi.h"
#include "no_os_spi.h"
#include "ad5592r.h"

//Helper macro for 12-bit full scale values
#define FULL_SCALE_12b      0x0FFFu

/* Curve Tracer Constants */
#define RSENSE      47.0f   // 47 Ohms
#define RBASE       47.0e3f // 47 kOhms
#define VBE         0.7f    //Volts (An approximation, of course....)
#define CH_VBDRIVE  0
#define CH_VCSENSE  1
#define CH_VCDRIVE  2
#define NUM_CURVES  5       //Number of curves to run
#define NUM_STEPS   50      //Number of steps per curve


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
		CH_MODE_DAC_AND_ADC,
		CH_MODE_DAC_AND_ADC,
		CH_MODE_UNUSED,
		CH_MODE_UNUSED,
		CH_MODE_GPI,
		CH_MODE_GPO
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
 * Structure to hold curve tracer data
 */
typedef struct {
	float vc;   // Collector Voltage
	float ic;   // Collector Current
} curve_data_t;

//Actual collection of curve data
static curve_data_t curves[NUM_CURVES][NUM_STEPS];


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
	int cur, step;
	float vb, ib, ic, vc, vcv;
	float vcDrive_meas, vcSense;
	uint16_t sampleValue;

	//Initialize the SPI bus
	no_os_spi_init(&my_spi, &spiInitParams);

	//Need to set the spiHandle after its been initialized
	my_ad5592.spi = my_spi;

	//Initialize the AD5592
	ad5592r_init(&my_ad5592, &ad5592Init);

	//Sweep base voltage from 499mV to 2.5V
	for( cur = 0, vb = 499.0f; cur < NUM_CURVES; cur++, vb += 500.0f ) {
		//Set the base voltage
		ad5592r_write_dac(&my_ad5592, CH_VBDRIVE, mVoltsToRaw(vb));

		//Calculate the base current
		ib = ((vb / 1000.0f) - VBE) / RBASE;

		//Print to stderr...that way we can use stdout for our CSV data
		fprintf(stderr, "Base Drive: %f Volts, %f uA\n", vb, ib * 1.0e6f);

		//Sweep collector drive voltage from 0 to 2.5V in 50mV steps
		for( step = 0, vcv = 0.0f; step < NUM_STEPS; step++, vcv += 50.0f ) {
			//Set collector drive voltage
			ad5592r_write_dac(&my_ad5592, CH_VCDRIVE, mVoltsToRaw(vcv));

			//Measure collector current & voltage
			ad5592r_read_adc(&my_ad5592, CH_VCDRIVE, &sampleValue);
			vcDrive_meas = rawTo_mVolts(sampleValue & FULL_SCALE_12b);
			ad5592r_read_adc(&my_ad5592, CH_VCSENSE, &sampleValue);
			vcSense = rawTo_mVolts(sampleValue & FULL_SCALE_12b);
			ic = (vcDrive_meas - vcSense) / RSENSE;
			vc = vcSense / 1000.0f;

			//Add the measurement to the array
			curves[cur][step].ic = ic;
			curves[cur][step].vc = vc;

			//Print for fun. Use stderr...that way we can use stdout for our CSV data
			fprintf(stderr, "Coll Voltage: %f coll curre: %f\n", vc, ic);
		}
	}

	//Loop through all the curves and steps, creating a CSV output
	//Format will be vc0,ic0,vc1,ic1,vc2,ic2,vc3,ic3,vc4,ic4,
	//and printed to stdout for capture in the terminal or saving to a file
	for( step = 0; step < NUM_STEPS; step++ ) {
		for( cur = 0; cur < NUM_CURVES; cur++ ) {
			printf("%f,%f,", curves[cur][step].vc, curves[cur][step].ic);
		}
		printf("\n"); //End of a row
	}

	//Clean up the SPI
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
