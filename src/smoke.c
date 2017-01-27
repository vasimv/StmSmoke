/* Temperature controller for vaping mod, schematics in Stm-Smoke.sch or Stm-Smoke-Inductor.sch
 *
 *    Copyright (C) 2016  Vasim V.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program. */

#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "u8g_arm.h"

#include "smoke.h"
#include "flash_data.h"

// u8glib stuff
static u8g_t u8g;

#ifdef USB_DEBUG
char usboutbuf[256];

va_list args;
int nlen;

int VCP_write(const void *pBuffer, int size)
{
    if (size > CDC_DATA_HS_OUT_PACKET_SIZE)
    {
        int offset;
        for (offset = 0; offset < size; offset++)
        {
            int todo = MIN(CDC_DATA_HS_OUT_PACKET_SIZE,
                           size - offset);
            int done = VCP_write(((char *)pBuffer) + offset, todo);
            if (done != todo)
                return offset + done;
        }

        return size;
    }

    USBD_CDC_HandleTypeDef *pCDC =
            (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    // XXX - set timeout?
    while(pCDC->TxState) {  HAL_Delay(10); } //Wait for previous transfer

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)pBuffer, size);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
        return 0;
    return size;
}

void debug(char *fmt, ...) {
	   // Check if we're really connected
	   if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
		   return;
	   va_start(args, fmt);
	   nlen = vsnprintf(usboutbuf, sizeof(usboutbuf), fmt, args);
	   va_end(args);
	   // CDC_Transmit_FS((uint8_t *) usboutbuf, nlen);
	   VCP_write(usboutbuf, nlen);
}
#else
void debug(char *fmt, ...) {
}
#endif

void Init_Screen() {
#if defined(SSD1306_128_64)
	u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);
#endif
}

// Coil resistance
float rcoil;

// Coil resistance at 30C
float rcoilzero = 999;

// Locked coil resistance at 25C (don't use if zero)
float rcoillock = 0.0;

// CONF - Minimum and maximum coil resistances
float rmincoil = 0.2;
float rmaxcoil = 6.0;

// Battery voltage, updated in measure_int()
float vmainv;
// Voltage on drain of test FET while testing, updated in measure_int()
float vcheck;
// Voltage on coil while testing, updated in measure_int()
float coilv;
int16_t vmainv_readout = 0;
int16_t vcheck_readout = 0;
int16_t coilv_readout = 0;

// ADC DMA array
int16_t adcdata[5];

// CONF - Temperature limit (in celsius)
float tcut = 230.0;

// CONF - Power limit (in watts)
float pcut = 1.0;

// CONF - Coil regulator mode, 0 - temperature control (tcut), 1 - power limiting (pcut)
uint32_t coilmode = 0;

// Chip temprature (averaged from few readings)
float tchip;

// Chip temperature at wake-up (treating like it is outside temperature)
float tair = -100;

// Coil temperature
float tcoil;

// CONF - coil material type, 0 - Titan
uint32_t coiltype = 0;

// Temperature/resistance koefficient (Titan, SS316L, SS304, NiFe30, Nickel, Kanthal D)
float rtchange[] = {0.003525, 0.000879, 0.001016, 0.0032, 0.006, 0.0001};

// VDDA calculated voltage
float vdda;

// VBAT voltage
float vbat;

// USB voltage
float vusb;

// First cell voltage (2S balance pin)
float vbalance;

// PWM output
float output;

// Alert condition (shouldn't fire coil)
uint8_t alert;

// Coil heat time/flag (in milliseconds), if 0 - should be turned off
uint32_t coilheat;

// CONF - Slow start time (rising output power from 1/10 to maximum during this time), in milliseconds
uint32_t slowheat = 600;

// CONF - lock for plus/minus keys (will able to update through menu only)
uint32_t changekeylock = 0;

// Light is on flag
uint8_t lighton;

// CONF - Sleep timer (if COIL button wasn't pressed)
uint32_t sleeptime = 60;

// CONF - slowcharge mode (less than 100 mA for small batteries)
uint32_t slowcharge = 1;

// CONF - Maximum charge voltage (lower is recommended if no secondary battery checker)
float maxcharge = 8.2;

// While in recharging - boost PWM output
uint8_t charging = 0;

// CONF - Voltage divider ratio (43K/11K - 0.2037, 44K/11K - 0.2)
float dividerratio = 0.2;

// CONF - Coil PID regulator variables
float pidp = 10.0;
float pidi = 0.2;
float pidd = 2.0;

// Main loop cycle count
uint32_t loopcycle = 0;

// HAL_GetTick value after last wake up
uint32_t lastawake;

// We're checking TSC buttons (have to delay SDADC measure as this make crosstalk of some sort)
uint8_t doingtsc = 0;

// Buttons indexes
#define I_BCOIL 0
#define I_BPLUS 1
#define I_BMINUS 2
#define I_BMENU 3

#if NBUTTONS == 5
#define I_BLIGHT 4
#endif

// Buttons calibration (searching for minimal sensor value for specified millseconds)
int buttonscalib;

// TSC Spread Spectrum (for less crosstalks to SDADC;
int tscspread = 1;

// Touch sensor buttons stuff
uint8_t sw_check[NBUTTONS];
uint8_t sbuttons[NBUTTONS];
uint16_t tmpbuttons[NBUTTONS];
uint16_t zerobuttons[NBUTTONS];
uint32_t idlebuttons[NBUTTONS];

// Processed buttons flags
int pbuttons[NBUTTONS];

// UI Control flags
// We're in menu (+/- - navigate, coil - change, menu - exit)
uint8_t inmenu;
// We're in menu, updating a value (+/- - change, coil - save, menu - exit without save)
uint8_t inupdate;
// We're in sleep mode (to unlock - press +, then -, then menu in one second)
uint8_t insleep;
// Need flash memory update (milliseconds from last configuration change)
uint32_t needupdate = 0;
// Have to wait until the  button is released (after wakeup, for example)
uint8_t lockuntilrelease = 255;

// Number of measures made
uint32_t measures = 0;

#ifdef USB_DEBUG
void Stop_USB() {
	USBD_Stop(&hUsbDeviceFS);
	USBD_DeInit(&hUsbDeviceFS);
} // Stop_USB

void Start_USB() {
	MX_USB_DEVICE_Init();
}
#endif

// Set up duty cycle on a timer
void PWM_Timer_Set(TIM_HandleTypeDef *phtim, uint32_t channel, uint32_t pulse) {
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
#ifdef INVERTED_PWM_COIL
	if (phtim == &htim2)
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
#endif
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(phtim, &sConfigOC, channel);
	HAL_TIM_PWM_Start(phtim, channel);
} // PWM_Timer_Set

void Stop_PWM() {
	charging = 0;
	HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_DeInit(&htim2);
	HAL_TIM_Base_Stop(&htim3);
	HAL_TIM_Base_DeInit(&htim3);
} // Stop_PWM

// Set-up coil and secondary PWMs
void Start_PWM() {
	MX_TIM2_Init();
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = (PWM_COIL_PERIOD - 1);
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);
	PWM_Timer_Set(&htim2, TIM_CHANNEL_1, 0);

	MX_TIM3_Init();
	PWM_Timer_Set(&htim3, TIM_CHANNEL_1, 0);
	charging = 0;
	lighton = 0;
	PWM_Timer_Set(&htim3, TIM_CHANNEL_3, 0);
} // Start_PWM

// Stop PWM output on coil temporarily
void Stop_Coil() {
	TIM2->CR1 &= ~ TIM_CR1_ARPE;
	TIM2->CCR1 = 0;
} // Stop_Coil

// Restart PWM output on coil FET
void Start_Coil() {
	uint32_t timout;

	if (alert) {
		Stop_Coil();
		return;
	}
	timout = trunc(output);
	if (timout >= PWM_COIL_PERIOD)
		timout = PWM_COIL_PERIOD - 1;
	TIM2->CCR1 = timout;
	TIM2->CR1 |= TIM_CR1_ARPE;
} // Start_Coil

// Reset coil variables/timers
void Reset_Coil() {
	coilheat = 0;
	output = 0.0;
	Stop_Coil();
	if (rcoillock > 0.01) {
		rcoilzero = rcoillock;
		tair = 25;
	}
} // Reset_Coil

// Lock coil resetistance
void Lock_Coil() {
	rcoillock = rcoil;
	rcoilzero = rcoil;
	tair = 25;
} // Lock_Coil

// Close test FET (non-inductor)
void Stop_FET() {
#ifndef INDUCTOR_BASED
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
#endif
} // Stop_FET

// Stops GPIO, I2C, TSC stuff
void Stop_Periph() {
	HAL_I2C_DeInit(&hi2c1);
	HAL_TSC_DeInit(&htsc);
} // Stop_GPIO

// Configures GPIO stuff
void Start_Periph() {
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TSC_Init();
	MX_RTC_Init();
} // Start_GPIO

#ifdef BURST_DEBUG_DATA
// Collecting data to print out through usb after run
float datacollect[1024];
unsigned int pos = 0;
uint32_t lastcollect = 0;

void Collect_Data(float data) {
	lastcollect = HAL_GetTick();

	datacollect[pos] = data;
	pos++;
	if (pos >= (sizeof(datacollect) / sizeof(datacollect[0])))
		pos = 0;
} // Collect_Data

void Check_Dump_Data(unsigned int linesize) {
	unsigned int i, j;

	if ((pos > 0) && (HAL_GetTick() - lastcollect) > 10000) {
		debug("pos: %f\n", pos);
		j = 0;

		for (i = 0; i < pos; i++) {
			debug("%f ", datacollect[i]);
			HAL_Delay(10);
			j++;
			if (j >= linesize) {
				debug("\n");
				j = 0;
			}
		}
		pos = 0;
	}
}
#endif

float errsum, lasterr;
float outputlast;

// PID controller for temperature control
void PID_Compute(int maxduty) {
	float error = tcut - tcoil;
    float derr = error - lasterr;

    errsum = errsum + error;
    if (errsum > PWM_COIL_PERIOD)
    	errsum = PWM_COIL_PERIOD;
    if (errsum < PWM_COIL_PERIOD)
    	errsum = PWM_COIL_PERIOD;
    output = pidp * error + pidi * errsum + pidd * derr;
    if (output > maxduty)
    	output = maxduty;
    if (output < 0)
    	output = 0;
    lasterr = error;
} // PID_Compute

// Reset PID controller
void Start_PID() {
	errsum = lasterr = outputlast = 0.0;
	output = 0.0;
	if (!coilmode)
		PID_Compute(slowheat ? (PWM_COIL_PERIOD - 1) / 10 : (PWM_COIL_PERIOD - 1));
} // Reset_PID

// Check failsafes
int Check_Failsafes() {
	alert = 0;
	// USB is connected
#ifndef DISABLE_CHARGING
	if (vusb > 1.0)
		alert = 1;
#endif
	// Coil resistance is weird
	if ((rcoil < rmincoil) || (rcoil > rmaxcoil))
		alert = 1;
	// Too high temperature (wrong coil type set perhaps)
	if (!coilmode && (tcoil > 300))
		alert = 1;
	// Device overheat
	if (tchip > 70.0)
		alert = 1;
	// battery voltage is overlimits
	if ((vmainv < MIN_VOLTAGE) || (vmainv > (MIN_VOLTAGE * 1.5)))
		alert = 1;
	if (vbat < 2.3)
		alert = 1;
	return alert;
} // Check_Failsafes

// Update coil PWM output
void Update_PWM() {
	int dividelimit = 1;

	// Slow start'ing to prevent "ooze shooting" and coil burn-up
	if (coilheat < slowheat)
		dividelimit = slowheat / (coilheat + slowheat / 10);
	if (coilmode) {
		// Power limiting
		float curpower = (vmainv * vmainv) / rcoil;

		if (curpower > pcut) {
			output = ((PWM_COIL_PERIOD - 1) * pcut) / curpower;
			// Correct last two in 10 cycles for precise regulation if PWM period is too crude
			if ((PWM_COIL_PERIOD < 1000) && ((coilheat % 10 == 8) || (coilheat % 10 == 9))) {
				int total = trunc(output) * 8;

				output = ((pcut * (PWM_COIL_PERIOD - 1) * 10) / curpower - total) / 2;
			}
		} else
			output = PWM_COIL_PERIOD - 1;
		output = output / dividelimit;
	} else {
		// Temperature control
		PID_Compute((PWM_COIL_PERIOD - 1) / dividelimit);
		// Skip cycle if the PID controller did overheat
		if ((tcoil - tcut) > 2)
			output = 0;
	}
#ifdef BURST_DEBUG_DATA
	Collect_Data(vmainv);
	Collect_Data(coilv);
	Collect_Data(rcoil);
	Collect_Data(coilheat);
	Collect_Data(output);
#endif
	Start_Coil();
} // Update_PWM

// Make measures and update output PWM
void measure_int() {
#ifdef INDUCTOR_BASED
	// We use non-stop DMA conversion when have inductor+shunt
#else
	// We need to turn off coil to take measure
	uint32_t chan;

#ifndef CALIBRATE_SPREAD
	// Skip cycles while TSC is running (too much interference)
	if (doingtsc)
		return;
#endif
	// Turn off coil and open test FET
	Stop_Coil();
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
#if FET_SWITCH_DELAY > 0
	for (int i = 0; i < FET_SWITCH_DELAY; i++);
#endif
    // Start SDADCs conversion
	HAL_SDADC_InjectedStart(&hsdadc2);
	HAL_SDADC_InjectedStart(&hsdadc3);
	HAL_SDADC_InjectedStart(&hsdadc1);
	// Wait for all SDADCs to finish
	while ((HAL_SDADC_PollForInjectedConversion(&hsdadc1, 0) == HAL_TIMEOUT)
			|| (HAL_SDADC_PollForInjectedConversion(&hsdadc2, 0) == HAL_TIMEOUT)
			|| (HAL_SDADC_PollForInjectedConversion(&hsdadc3, 0) == HAL_TIMEOUT));
	// Close test FET
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    // Restart coil
	if (coilheat)
		Start_Coil();

    // Weird HAL functions returns uint32_t while it is signed int16 actually
	coilv_readout =  (int16_t) HAL_SDADC_InjectedGetValue(&hsdadc1, &chan);
	vcheck_readout = (int16_t) HAL_SDADC_InjectedGetValue(&hsdadc2, &chan);
	vmainv_readout = (int16_t) HAL_SDADC_InjectedGetValue(&hsdadc3 , &chan);
#endif
    coilv = ((VREF_SDADC / 65536.0) / dividerratio) * (coilv_readout + 32767);
    vcheck = ((VREF_SDADC / 65536.0) / dividerratio) * (vcheck_readout + 32767);
    vmainv = ((VREF_SDADC / 65536.0) / dividerratio) * (vmainv_readout + 32767);
    // Calculate coil resistance, substracting test FET forward voltage
	rcoil = RTEST * ((vmainv - vcheck) / (coilv - vcheck) - 1.0);
	if (((rcoilzero / 2) > rcoil) && (rcoillock < 0.01)) {
		rcoilzero = rcoil;
		tcoil = tair;
	} else
		tcoil = (rcoil / rcoilzero - 1) / rtchange[coiltype] + tair;
	if (coilheat && !Check_Failsafes()) {
		Update_PWM();
		coilheat++;
//		debug("coilv %f, vcheck %f, vmainv %f, rcoil %f, tcoil %f, output %f\n", coilv, vcheck, vmainv, rcoil, tcoil, output);
	}
	measures++;
} // measure_int

// Cyclic array for averaging temperature data
float tdata[16];
unsigned int p_tdata = 0;

void Stop_ADC() {
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop_DMA(&hadc1);
} // Stop_ADC

// Start 12-bit ADC (VDUSB pin, temperature, vrefint, vbat)
void Start_ADC() {
	unsigned int i;

	for (i = 0; i < (sizeof(tdata) / sizeof(tdata[0])); i++)
		tdata[i] = -100;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcdata, 10);
	HAL_ADC_Start(&hadc1);
} // Start_ADC


// Calculate ADC stuff using internal Vrefint and calibration values
void Calculate_ADC() {
	int32_t t;
	unsigned int i;
	float tmp;

	// Calculate voltages based on calibration values and internal Vref
	vdda = (3.3 * VREFINT_CAL) / adcdata[2];
	vbat = (vdda * adcdata[3]) / 2048;
	vusb = ((adcdata[0] * vdda) / 4096) / dividerratio;
	vbalance = ((adcdata[4] * vdda) / 4096) / dividerratio;
	if (vusb < 1.0)
		vusb = 0.0;
	// Calculate temperature based on calibration values (average last 16 readouts)
	t = (adcdata[1] * VREFINT_CAL) / adcdata[2] - TS_CAL1;
    t =  t * ((110000 - 30000) / (TS_CAL2 - TS_CAL1)) + 30000;
    tmp = t / 1000.0;
    tdata[p_tdata] = tmp;
    p_tdata++;
    if (p_tdata >= (sizeof(tdata) / sizeof(tdata[0])))
    	p_tdata = 0;
    // Fill the array with same value at first run
    if (tdata[p_tdata] < -50)
    	for (i = 0; i < (sizeof(tdata) / sizeof(tdata[0])); i++)
    		tdata[i] = tmp;
    // Calculate average temperature
    tmp = 0.0;
    for (i = 0; i < (sizeof(tdata) / sizeof(tdata[0])); i++)
    	tmp += tdata[i];
    tchip = tmp / (sizeof(tdata) / sizeof(tdata[0]));
    // Set air temperature as first reading of the chip temperature
    // (while it did not heat itself yet)
    if (tair < -50)
    	tair = (tchip - 5);
} // Calculate_ADC

// Set PWM boost timer
void Set_Recharge(unsigned int duty) {
	PWM_Timer_Set(&htim3, TIM_CHANNEL_3, duty);
} // Set_Recharge

// Update PWM boost timer accordingly vmainv to keep current in limits
void Update_Recharge() {
	// We don't have current sensor for the battery, so are using pre-calculated values
	// These values are for 2S LiPo 500 mAh battery (150..250 mA charge current)
	switch ((int) round(vmainv * 10.0)) {
	case 60:
	case 61:
	case 62:
		charging = 12;
		break;
	case 63:
	case 64:
		charging = 13;
		break;
	case 65:
		charging = 14;
		break;
	case 66:
		charging = 15;
		break;
	case 67:
	case 68:
		charging = 16;
		break;
	case 69:
	case 70:
		charging = 17;
		break;
	case 71:
	case 72:
		charging = 18;
		break;
	case 73:
	case 74:
	case 75:
		charging = 19;
		break;
	case 76:
	case 77:
		charging = 20;
		break;
	case 78:
	case 79:
		charging = 21;
		break;
	case 80:
	case 81:
		charging = 22;
		break;
	case 82:
	case 83:
		charging = 23;
	case 84:
	case 85:
	case 86:
		charging = 24;
		break;
	default:
		if (vmainv < 6.0)
			charging = 10;
		else
			charging = 0;
	}
	if (slowcharge && (charging > 0))
		charging = charging - 2;
	Set_Recharge(charging);
} // Update_Recharge

// Recharge battery from USB if possible
void Check_Recharge() {
#ifndef DISABLE_CHARGING
	if (charging) {
#ifdef DISABLE_BALANCE
		if ((vusb < 1.0) || (vmainv >= maxcharge) || (tchip > 70)) {
#else
		if ((vusb < 1.0) || (vmainv >= maxcharge) || (tchip > 70) || ((vbalance > 1.0) && (vbalance < 5.0) && (vbalance >= (maxcharge / 2.0)))) {
#endif
			// Stop charging process
			charging = 0;
			Set_Recharge(0);
			return;
		}
		Update_Recharge();
	} else {
#ifdef DISABLE_BALANCE
		if ((vusb > 1.0) && (vmainv < (maxcharge - 0.2))) {
#else
		if ((vusb > 1.0) && (vmainv < (maxcharge - 0.2)) && ((vbalance < 1.0) || (vbalance > 5.0) || (vbalance < ((maxcharge - 0.5) / 2.0)))) {
#endif
			// Start charging process
			Update_Recharge();
		}
	}
#endif
} // Recharge

int8_t buttons_phase = 0;

// Calibrate touch sensors
void Calibrate_Touch() {
	unsigned int i;

	// Calibrate buttons for 200 milliseconds
	buttonscalib = 200;
	for (i = 0; i < NBUTTONS; i++)
		zerobuttons[i] = 65535;
} //

// Reset buttons state (init/wakeup)
void Reset_Buttons() {
	unsigned int i;

	buttons_phase = 0;
	lockuntilrelease = 255;
	inmenu = 0;
	insleep = 0;
	inupdate = 0;
	for (i = 0; i < NBUTTONS; i++) {
		sbuttons[i] = 0;
		idlebuttons[i] = 0;
		sw_check[i] = 0;
		tmpbuttons[i] = 65535;
		pbuttons[i] = 0;
	}
} // Reset_Buttons

// Clear buttons pressed status (after coil fire)
void Clear_Buttons() {
	unsigned int i;

	for (i = 0; i < NBUTTONS; i++) {
		idlebuttons[i] = 0;
		sbuttons[i] = 0;
		pbuttons[i] = 0;
		sw_check[i] = 0;
	}
} // Clear buttons

// Check if button is "pushed"
int Push_Check(unsigned int button) {
 return tmpbuttons[button] < zerobuttons[button];
} // Push_Check

uint32_t lasttick;

// Debounce buttons
void debounce_buttons() {
	unsigned int i;
	int press;
	uint32_t curtick;
	int ticksdiff;

	curtick = HAL_GetTick();
	ticksdiff = curtick - lasttick;
	lasttick = curtick;

	for (i = 0; i < NBUTTONS; i++) {
		press = Push_Check(i);
		if (sbuttons[i] != press) {
			if (sw_check[i] > BUTTONS_DEBOUNCE) {
				sw_check[i] = 0;
				sbuttons[i] = press;
				idlebuttons[i] = 0;
			} else {
				sw_check[i] += ticksdiff;
				idlebuttons[i] += ticksdiff;
			}

		} else {
			sw_check[i] = 0;
			idlebuttons[i] += ticksdiff;
		}
	}
} // debounce_buttons

// This function should be called every 1 msec, will check all TSC groups every third call
void Check_TSC_Buttons() {
	unsigned int i;

	switch (buttons_phase) {
	case 0:
		HAL_TSC_IODischarge(&htsc, ENABLE);
		break;
	case 1:
		HAL_TSC_Start(&htsc);
		doingtsc = 1;
		HAL_TSC_PollForAcquisition(&htsc);
		doingtsc = 0;
		break;
	case 2:
		if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP2_IDX) == TSC_GROUP_COMPLETED)
			tmpbuttons[0] = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP2_IDX);
		else
			tmpbuttons[0] = 65535;
		if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP3_IDX) == TSC_GROUP_COMPLETED)
			tmpbuttons[1] = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP3_IDX);
		else
			tmpbuttons[1] = 65535;
		if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP4_IDX) == TSC_GROUP_COMPLETED)
			tmpbuttons[2] = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP4_IDX);
		else
			tmpbuttons[2] = 65535;
#ifndef SWAP_MENU_LIGHT
		if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED)
			tmpbuttons[3] = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP5_IDX);
		else
			tmpbuttons[3] = 65535;
#else
		if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP6_IDX) == TSC_GROUP_COMPLETED)
			tmpbuttons[3] = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP6_IDX);
		else
			tmpbuttons[3] = 65535;
#endif
#ifndef DISABLE_LIGHT
#if NBUTTONS == 5
#ifndef SWAP_MENU_LIGHT
		if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP6_IDX) == TSC_GROUP_COMPLETED)
			tmpbuttons[4] = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP6_IDX);
		else
			tmpbuttons[4] = 65535;
#else
		if (HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED)
			tmpbuttons[4] = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP5_IDX);
		else
			tmpbuttons[4] = 65535;
#endif
#endif
#endif
		if (buttonscalib) {
			buttonscalib--;
			// Collect minimal values for all buttons
			for (i = 0; i < NBUTTONS; i++)
				if (zerobuttons[i] > tmpbuttons[i])
					zerobuttons[i] = tmpbuttons[i];
			// Calculating "pushed" value for all buttons
			if (buttonscalib <= 0) {
				buttonscalib = 0;
				for (i = 0; i < NBUTTONS; i++) {
					zerobuttons[i] = (uint16_t) trunc(zerobuttons[i] * BUTTONS_THRESHOLD);
					debug("button %d zero value - %d\n", i, zerobuttons[i]);
				}
			}
		} else
			debounce_buttons();
		break;
	case 3:
		HAL_TSC_Stop(&htsc);
		break;
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		break;
	default:
		buttons_phase = -1;
	}
	buttons_phase++;
} // Check_TSC_Buttons

void Flash_Update();
void Flash_Read();

// Check if need to update flash memory
void Check_Update() {
	if (needupdate) {
		needupdate++;
		if (needupdate > UPDATE_FLASH_AFTER) {
			needupdate = 0;
			Flash_Update();
		}
	}
} // Check_Update

// Additional checks, buttons, etc (period is 1 ms, may be bigger if slow code is running)
void timer_int() {
	Check_TSC_Buttons();
	// Don't do anything else in sleep mode
	if (insleep)
		return;
	Calculate_ADC();
	Check_Update();
	Check_Recharge();
} // timer_int

unsigned int wakeupphase;
unsigned int wakeupcount = 0;

// Wake-up check (few times per second when sleeping)
int Wakeup_Check() {
	unsigned int should;
	unsigned int i;

	switch (wakeupphase) {
	case 0:
		should = I_BPLUS;
		break;
	case 1:
		should = I_BMINUS;
		break;
	case 2:
#ifdef WAKEUP_ONLY_TWO
		wakeupphase = 0;
		return 1;
#endif
		should = I_BMENU;
		break;
	case 3:
		// Phase 3 is reached! Wake up!
		wakeupphase = 0;
		return 1;
	default:
		wakeupcount = should = 255;
	}

	wakeupcount++;
	if (wakeupcount > 50) {
		wakeupcount = 0;
		wakeupphase = 0;
		return 0;
	}

	// We check if pressed just one button, reset to start if else
	if (Push_Check(should)) {
		for (i = 0; i < NBUTTONS; i++)
//			if ((i != should) && Push_Check(i)) {
			if (((should == I_BPLUS) && Push_Check(I_BMINUS))
					|| ((should == I_BMINUS) && Push_Check(I_BPLUS))) {
				wakeupcount = 0;
				wakeupphase = 0;
				return 0;
			}
		lockuntilrelease = should;
		wakeupphase++;
		wakeupcount = 0;
	}
	return 0;
} // wakeup_check

void Stop_Timer() {
	HAL_TIM_Base_Stop_IT(&htim7);
	HAL_TIM_Base_DeInit(&htim7);
} // Stop_Timer

void Start_Timer() {
	HAL_TIM_Base_Init(&htim7);
	HAL_TIM_Base_Start_IT(&htim7);
} // Start_Timer

void Stop_Measure() {
	HAL_TIM_Base_Stop_IT(&htim6);
	HAL_TIM_Base_DeInit(&htim6);
} // Stop_Measure

// Set TIM6 interrupt (measure/update pwm) frequency
void Start_Measure() {
	htim6.Instance = TIM6;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
#if defined(MEASURE_200HZ)
	htim6.Init.Prescaler = 24;
	htim6.Init.Period = 9599;
#elif defined(MEASURE_500HZ)
	htim6.Init.Prescaler = 9;
	htim6.Init.Period = 9599;
#elif defined(MEASURE_1KHZ)
	htim6.Init.Prescaler = 4;
	htim6.Init.Period = 9599;
#elif defined(MEASURE_5KHZ)
	htim6.Init.Prescaler = 0;
	htim6.Init.Period = 9599;
#elif defined(MEASURE_20KHZ)
	htim6.Init.Prescaler = 1;
	htim6.Init.Period = 2399;
#endif
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
} // Start_Measure

#ifdef CALIBRATE_SPREAD
float range[128];

// Find best spread spectrum value of TSC (by lowest crosstalk to SDADC
void Calibrate_Spread() {
	unsigned int i, j;
	float min, max;

	for (i = 0; i < 128; i++) {
		tscspread = i;
		MX_TSC_Init();
		min = 9999999.9;
		max = 0.0;
		for (j = 0; j < 10; j++) {
			Check_TSC_Buttons();
			HAL_Delay(1);
			if (vmainv > max)
				max = vmainv;
			if (vmainv < min)
				min = vmainv;
		}
		range[i] = max - min;
	}
	min = 9999999.9;
	for (i = 0; i < 128; i++)
		if (range[i] < min) {
			tscspread = i;
			min = range[i];
		}
	debug("TSC Spread %d with %fV\n", tscspread, min);
	MX_TSC_Init();
} // Calibrate_Spread
#endif


// Start SDADC
void Start_SDADC() {

} // Start_SDADC

// Stop SDADC to enter sleep mode
void Stop_SDADC() {

} // Stop_SDADC

void Calibrate_SDADC() {
	HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_3);
	HAL_SDADC_PollForCalibEvent(&hsdadc1, HAL_MAX_DELAY);
	HAL_SDADC_CalibrationStart(&hsdadc2, SDADC_CALIBRATION_SEQ_3);
	HAL_SDADC_PollForCalibEvent(&hsdadc2, HAL_MAX_DELAY);
	HAL_SDADC_CalibrationStart(&hsdadc3, SDADC_CALIBRATION_SEQ_3);
	HAL_SDADC_PollForCalibEvent(&hsdadc3, HAL_MAX_DELAY);
} // Calibrate_SDADC

// Turn on LED flashlight
void Start_Light() {
	lighton = 1;
#ifdef LED_TIM3CH1
	PWM_Timer_Set(&htim3, TIM_CHANNEL_1, LED_PWM_DUTY);
#endif
} // Start_Light

// Turn off LED flashlight
void Stop_Light() {
	lighton = 0;
#ifdef LED_TIM3CH1
	PWM_Timer_Set(&htim3, TIM_CHANNEL_1, 0);
#endif
} // Stop_Light

// Reset light
void Reset_Light() {
	Stop_Light();
} // Reset_Light


// Initialize stuff
void Setup() {
#ifdef EXTENDED_DEBUG
	// So, you will have time to connect USB
	HAL_Delay(5000);
#endif
	lastawake = HAL_GetTick();
	Calibrate_Touch();
#ifdef CALIBRATE_SPREAD
	Calibrate_Spread();
#endif
	Calibrate_SDADC();
	Start_ADC();
	Reset_Buttons();
	Start_Timer();
	Flash_Read();
	Start_PWM();
	Reset_Coil();
	Reset_Light();
	// Delay to be sure we've calculated the chip temperature
	HAL_Delay(3);
	Start_Measure();
	Init_Screen();
	debug("Setup finished\n");
} // Setup


// UI Stuff

// Translate coiltype to string for UI
char *Coil_Type_Str(void *p) {
	switch (*((uint32_t *) p)) {
	case 0:
		return "Ti";
	case 1:
		return "316L";
	case 2:
		return "304";
	case 3:
		return "NiFe";
	case 4:
		return "Ni";
	case 5:
		return "Kant";
	default:
		coiltype = 0;
		return "Ti";
	}
} // Coil_Type_Str

// Translate coilmode to string for UI
char *Coil_Mode_Str(void *p) {
	if (*((uint32_t *) p))
		return "VariWatt";
	else
		return "TC";
} // Coil_Mode_Str

// Translate on/off to string for UI
char *OnOff_Str(void *p) {
	if (*((uint32_t *) p))
		return "On";
	else
		return "Off";
} // OnOff_Str

// Menu item types, TENUM - as int, calls *f_show to gets string to show, TPROCESS - run *f_on function and *f_off when finish
typedef enum { TFLOAT, TINT, TENUM, TPROCESS } t_menuitem;

// Menu item definition
typedef struct {
	char *name;
	char *suffix;
	void *pvalue;
	float min;
	float max;
	float step;
	t_menuitem mtype;
	void (*f_on)();
	void (*f_off)();
	char *(*f_show)(void *p);
} menuitem;

menuitem configmenu[] = {
		{"Flashlight", NULL,  NULL, 0, 0, 0, TPROCESS, &Start_Light, &Stop_Light, NULL},
		{"Mode", NULL, &coilmode, 0, 1, 1, TENUM, NULL, NULL, &Coil_Mode_Str},
		{"Coil type", NULL, &coiltype, 0, sizeof(rtchange)/sizeof(rtchange[0]), 1, TENUM, NULL, NULL, &Coil_Type_Str},
		{"Power limit", "W", &pcut, 0.5, MAX_COIL_POWER, 0.5, TFLOAT, NULL, NULL, NULL},
		{"Temperature", "C", &tcut, 40, 300, 5, TFLOAT, NULL, NULL, NULL},
		{"Lock Resist", "Ohm", &rcoillock, 0.0, 10.0, 0.01, TFLOAT, &Lock_Coil, NULL, NULL},
		{"Change lock", NULL, &changekeylock, 1, 1, 1, TENUM, NULL, NULL, &OnOff_Str},
		{"Slow start", "ms", &slowheat, 0, 3000, 300, TINT, NULL, NULL, NULL},
		{"Min Rcoil", "Ohm", &rmincoil, 0.1, 1.0, 0.1, TFLOAT, NULL, NULL, NULL},
		{"Max Rcoil", "Ohm", &rmaxcoil, 0.5, 10.0, 0.1, TFLOAT, NULL, NULL, NULL},
		{"Sleep timer", "s", &sleeptime, 30, 120, 10, TINT, NULL, NULL, NULL},
		{"Slow charge", NULL, &slowcharge, 0, 1, 1, TENUM, NULL, NULL, &OnOff_Str},
		{"Max charge", "V", &maxcharge, 7.8, 8.6, 0.1, TFLOAT, NULL, NULL, NULL},
		{"Calibration", NULL, &dividerratio, 0.1, 0.3, 0.0001, TFLOAT, NULL, NULL, NULL},
		{"PID P", NULL, &pidp, 0.1, 30, 0.1, TFLOAT, NULL, NULL, NULL},
		{"PID I", NULL, &pidi, 0.1, 30, 0.1, TFLOAT, NULL, NULL, NULL},
		{"PID D", NULL, &pidd, 0.1, 30, 0.1, TFLOAT, NULL, NULL, NULL},
};

#define MENU_SIZE (sizeof(configmenu) / sizeof(configmenu[0]))

void Flash_Set() {
	FlashData_Set_Size(MENU_SIZE * 4);
} // Flash_Set

uint32_t flashbuf[MENU_SIZE];

// Write configuration to the flash
void Flash_Update() {
	unsigned int i;

	Flash_Set();
	for (i = 0; i < MENU_SIZE; i++) {
		if (configmenu[i].pvalue != NULL)
			flashbuf[i] = *((uint32_t *) configmenu[i].pvalue);
	}

	FlashData_Write((void *) flashbuf);
} // Flash_Update

// Read configuration from the flash
void Flash_Read() {
	unsigned int i;

	Flash_Set();
	if (!FlashData_Read(flashbuf))
		return;
	for (i = 0; i < MENU_SIZE; i++) {
		if (configmenu[i].pvalue != NULL)
			*((uint32_t *) configmenu[i].pvalue) = flashbuf[i];
	}
	// Be sure we didn't broke our configuration
	if (sleeptime < 30)
		sleeptime = 30;
} // Flash_Read

float vmainv_show;
float rcoil_show;
float tcoil_show;
float tchip_show;
float output_show;

// Menu variables
int startitem;
int selectitem;

#if defined(SSD1306_128_64)
// Coordinates for helvR12 font
#define CX(x) ((x) * 9)
#define TEXT_BOTTOM_OFFSET 3
#define CY(y) (((y) + 1) * 16 - TEXT_BOTTOM_OFFSET)
#define MAX_WIDTH 128
#define MAX_HEIGHT 64
#define LINE_HEIGHT 16
#define MENU_LINES 3

// Sets inverted/noninverted colors
void Set_Inverted(unsigned int flag) {
	u8g_SetColorIndex(&u8g, flag ? 0 : 1);
} // Set_Inverted

// u8glib draw function, update item screen
void Draw_Item() {
	char buf[20];
	char *unit = "";

	u8g_SetFont(&u8g, u8g_font_9x15);
	u8g_DrawStr(&u8g, CX(0), CY(0), "Change:");
	u8g_DrawStr(&u8g, CX(0), CY(2), configmenu[selectitem].name);
	if (configmenu[selectitem].suffix)
		unit = configmenu[selectitem].suffix;
	switch (configmenu[selectitem].mtype) {
	case TFLOAT:
		if (configmenu[selectitem].step < 0.1)
			snprintf(buf, sizeof(buf), "%6.4f%s", *((float *) configmenu[selectitem].pvalue), unit);
		else
			snprintf(buf, sizeof(buf), "%6.1f%s", *((float *) configmenu[selectitem].pvalue), unit);
		u8g_DrawStr(&u8g, CX(0), CY(3), buf);
		break;
	case TINT:
		snprintf(buf, sizeof(buf), "%ld%s", *((uint32_t *) configmenu[selectitem].pvalue), unit);
		u8g_DrawStr(&u8g, CX(0), CY(3), buf);
		break;
	case TENUM:
		snprintf(buf, sizeof(buf), "%s", configmenu[selectitem].f_show(configmenu[selectitem].pvalue));
		u8g_DrawStr(&u8g, CX(0), CY(3), buf);
		break;
	case TPROCESS:
		u8g_DrawStr(&u8g, CX(0), CY(3), "MENU TO EXIT");
	}
} // Draw_Update

// u8glib draw function, menu screen
void Draw_Menu() {
	unsigned int i;

	u8g_SetFont(&u8g, u8g_font_9x15);
	u8g_DrawStr(&u8g, CX(0), CY(0), "Menu:");

	for (i = 0; i < MENU_LINES; i++) {
		if ((i + startitem) >= MENU_SIZE)
			break;
		// Draw selected item in inverted mode
		if (((int) i + startitem) == selectitem) {
			u8g_DrawBox(&u8g, CX(0), CY(i) + TEXT_BOTTOM_OFFSET, MAX_WIDTH, LINE_HEIGHT);
			Set_Inverted(1);
		} else
			Set_Inverted(0);
		u8g_DrawStr(&u8g, CX(0), CY(i + 1), configmenu[i + startitem].name);
	}
	Set_Inverted(0);
} // Draw_Menu

// u8glib draw function, main screen
void Draw_Main() {
  char tmpbuf[20];
  char *flag;

  u8g_SetFont(&u8g, u8g_font_9x15);

  // Battery voltage, flashing if too low
  if (vmainv >= WARN_VOLTAGE)
	  flag = "OK";
  else {
	if ((HAL_GetTick() % 500) < 250) {
		u8g_DrawBox(&u8g, CX(0), CY(2) + TEXT_BOTTOM_OFFSET, MAX_WIDTH, LINE_HEIGHT);
		Set_Inverted(1);
	}
    flag = "LOW!";
  }
  snprintf(tmpbuf, sizeof(tmpbuf), "Vbatt %3.1fV %s", vmainv_show, flag);
  u8g_DrawStr(&u8g, CX(0), CY(3), tmpbuf);
  Set_Inverted(0);

  if (charging || (sbuttons[I_BCOIL] && alert)) {
	  // User pressed COIL but we have a problem, print it on the first line
	  u8g_DrawBox(&u8g, CX(0), 0, MAX_WIDTH, LINE_HEIGHT);
	  Set_Inverted(1);
	  if (!charging) {
		  if (vusb > 1.0)
			  u8g_DrawStr(&u8g, CX(0), CY(0), "DISCONNECT USB");
		  if (vmainv < MIN_VOLTAGE)
			  u8g_DrawStr(&u8g, CX(0), CY(0), " BATTERY LOW");
	  } else {
		  u8g_DrawStr(&u8g, CX(0), CY(0), "   CHARGING");
		  // Show current output level
		  u8g_DrawBox(&u8g, 0, 0, (MAX_WIDTH * charging) / PWM_PERIOD_1MHZ, 1);
	  }
	  if (tchip > 70)
		  u8g_DrawStr(&u8g, CX(0), CY(0), " MCU OVERHEAT");
	  Set_Inverted(0);
  } else {
	  // Buttons states (right top corner)
	  if (sbuttons[I_BCOIL])
		  u8g_DrawStr(&u8g, CX(9), CY(0), "C");
	  else
		  u8g_DrawStr(&u8g, CX(9), CY(0), ".");

	  if (sbuttons[I_BPLUS])
		  u8g_DrawStr(&u8g, CX(10), CY(0), "+");
	  else
		  u8g_DrawStr(&u8g, CX(10), CY(0), ".");

	  if (sbuttons[I_BMINUS])
		  u8g_DrawStr(&u8g, CX(11), CY(0), "-");
	  else
		  u8g_DrawStr(&u8g, CX(11), CY(0), ".");

	  if (sbuttons[I_BMENU])
		  u8g_DrawStr(&u8g, CX(12), CY(0), "M");
	  else
		  u8g_DrawStr(&u8g, CX(12), CY(0), ".");

#ifndef DISABLE_LIGHT
#if NBUTTONS == 5
	  if (sbuttons[I_BLIGHT])
		  u8g_DrawStr(&u8g, CX(13), CY(0), "L");
	  else
	  	  u8g_DrawStr(&u8g, CX(13), CY(0), ".");
#endif
#endif

	  // Show current temperature or power, coil state
	  if ((rcoil_show < rmincoil) || (rcoil_show > rmaxcoil))
		  snprintf(tmpbuf, sizeof(tmpbuf), "BAD");
	  else {
		  if (coilmode)
			  snprintf(tmpbuf, sizeof(tmpbuf), "% 3.0fW", (vmainv * vmainv) / rcoil);
		  else
			  snprintf(tmpbuf, sizeof(tmpbuf), "% 3.0fC", tcoil_show);
	  }
	  u8g_DrawStr(&u8g, CX(2), CY(0), tmpbuf);

	  if (coilheat && !alert)
		  u8g_DrawStr(&u8g, CX(0), CY(0), "*");
	  else
		  u8g_DrawStr(&u8g, CX(0), CY(0), " ");

	  // Show current output level
	  u8g_DrawBox(&u8g, 0, 0, (MAX_WIDTH * output) / PWM_COIL_PERIOD, 1);
  }


  // Show coil temperature/power limit
  if (coilmode)
	  snprintf(tmpbuf, sizeof(tmpbuf), "= %4.1fW", pcut);
  else
	  snprintf(tmpbuf, sizeof(tmpbuf), "= %4.0fC", tcut);
  u8g_DrawStr(&u8g, CX(0), CY(2), tmpbuf);

  // Show coil resistance
  if ((rcoil_show < rmincoil) || (rcoil_show > rmaxcoil))
	  snprintf(tmpbuf, sizeof(tmpbuf), "R BAD %s", Coil_Type_Str(&coiltype));
  else
	  snprintf(tmpbuf, sizeof(tmpbuf), "R%5.2f %s", rcoil_show, Coil_Type_Str(&coiltype));
  u8g_DrawStr(&u8g, CX(0), CY(1), tmpbuf);

  // Show internal temperature
  snprintf(tmpbuf, sizeof(tmpbuf), "%3.0fC", tchip_show);
  u8g_DrawStr(&u8g, CX(8), CY(2), tmpbuf);
} // Draw_Main
#endif

// Returns 1 if button is just pushed, 2 if keep pushed for more than 0.5 second
int Just_Pushed(uint8_t button) {
	if (!sbuttons[button]) {
		pbuttons[button] = 0;
		return 0;
	}
	if (!pbuttons[button]) {
		pbuttons[button] = 1;
		return 1;
	}
	if ((idlebuttons[button] - pbuttons[button]) > BUTTONS_REPEAT_DELAY) {
		pbuttons[button] = idlebuttons[button];
		return 2;
	}
	// User holds button less than 0.5 second from last action, ignore
	return 0;
} // Just_Pushed

// Round pcut variable to 0.5
void Round_PCUT() {
	pcut = round(pcut * 2.0) / 2.0;
} // Round_PCUT

// Update item (increase if 1 or decrease if -1)
void Update_Item(menuitem *item, int increase) {
	float ftmp;
	int32_t itmp;
	int holdtime;
	float stepratio = 1.0;

	needupdate = 1;
	if ((item->mtype == TFLOAT) || (item->mtype == TINT)) {
		// Increase step if holding button more than 3s (10 times) or 7s (100 times)
		if (((item->max - item->min) / item->step) > 100) {
			if (sbuttons[I_BPLUS])
				holdtime = idlebuttons[I_BPLUS];
			else
				holdtime = idlebuttons[I_BMINUS];
			if (holdtime > 3000)
				stepratio = 10.0;
			if (holdtime > 7000)
				stepratio = 100.0;
		}
	}
	switch (item->mtype) {
	case TFLOAT:
		ftmp = *((float *) item->pvalue) + (float) increase * item->step * stepratio;
		if ((ftmp <= item->max) && (ftmp >= item->min))
			*((float *) item->pvalue) = ftmp;
		break;
	case TINT:
		itmp = *((int32_t *) item->pvalue) + increase * round(item->step);
		if ((itmp <= round(item->max)) && (itmp >= round(item->min)))
			*((int32_t *) item->pvalue) = itmp;
		break;
	case TENUM:
		itmp = *((int32_t *) item->pvalue) + increase;
		if (itmp > round(item->max))
			itmp = round(item->min);
		if (itmp < round(item->min))
			itmp = round(item->max);
		*((int32_t *) item->pvalue) = itmp;
		break;
	default:
		return;
	}
} // Update_Item

// Navigate menu (down if 1 or up if -1)
void Navigate_Menu(int direction) {
	 selectitem += direction;
	 if (selectitem < 0)
		 selectitem = MENU_SIZE - 1;
	 if (selectitem >= (int) MENU_SIZE)
		 selectitem = 0;
	 if ((selectitem - startitem) >= MENU_LINES)
		 startitem = abs(selectitem - MENU_LINES + 1);
	 if (startitem > selectitem)
		 startitem = selectitem;
} // Navigate_Menu

void Process_Buttons() {
	if (inmenu) {
		// We're in the menu
		// Navigate with +/-
		if (Just_Pushed(I_BPLUS))
			Navigate_Menu(1);
		if (Just_Pushed(I_BMINUS))
			Navigate_Menu(-1);

		if (Just_Pushed(I_BCOIL)) {
			inmenu = 0;
			inupdate = 1;
			if ((configmenu[selectitem].mtype != TPROCESS) && configmenu[selectitem].f_on)
				configmenu[selectitem].f_on();
		} else
			if (Just_Pushed(I_BMENU) == 1) {
				inmenu = 0;
				debug("Exiting menu\n");
			}
	} else if (inupdate) {
		if (Just_Pushed(I_BPLUS)) {
			Update_Item(&configmenu[selectitem], 1);
		}
		if (Just_Pushed(I_BMINUS)) {
			Update_Item(&configmenu[selectitem], -1);
		}
		if (configmenu[selectitem].mtype == TPROCESS) {
			if (Just_Pushed(I_BCOIL))
				configmenu[selectitem].f_on();
			if (Just_Pushed(I_BMENU)) {
				configmenu[selectitem].f_off();
				inupdate = 0;
				inmenu = 1;
			}
		} else {
			if (Just_Pushed(I_BCOIL) || Just_Pushed(I_BMENU)) {
				inupdate = 0;
				inmenu = 1;
			}
		}
	} else {
		// Normal mode
		if (sbuttons[I_BCOIL] && (idlebuttons[I_BCOIL] < MAX_COIL_TIME)) {
			if (coilheat == 0) {
				Start_PID();
				coilheat = 1;
			}
			return;
		} else {
			// Ignore other buttons while COIL pressed
			if (coilheat) {
				coilheat = 0;
				Reset_Coil();
				Clear_Buttons();
			}
			// Changing temperature/power with plus/minus buttons
			if (Just_Pushed(I_BPLUS) && !changekeylock) {
				debug("Plus!\n");
				if (coilmode && pcut < MAX_COIL_POWER) {
					pcut = pcut + 0.5;
					Round_PCUT();
				}
				if (!coilmode && tcut < 300)
					tcut = round(tcut + 1);
				needupdate = 1;
			}
			if (Just_Pushed(I_BMINUS) && !changekeylock) {
				debug("Minus!\n");
				if (coilmode && pcut > 0.5) {
					pcut = pcut - 0.5;
					Round_PCUT();
				}
				if (!coilmode && tcut > 30)
					tcut = round(tcut - 1);
				needupdate = 1;
			}

			// Entering menu with menu button
			if (Just_Pushed(I_BMENU) == 1) {
				// ignore MENU button right after wake up
				if (lockuntilrelease == I_BMENU)
					lockuntilrelease = 255;
				else {
					startitem = 0;
					selectitem = 0;
					inmenu = 1;
					debug("Entering menu\n");
				}
			}
#ifndef DISABLE_LIGHT
#if NBUTTONS == 5
			// Check LIGHT button, same function as menu item "Flashlight"
			if (!charging && !lighton && sbuttons[I_BLIGHT])
				Start_Light();
			if (lighton && !sbuttons[I_BLIGHT])
				Stop_Light();
#endif
#endif
		}
	}
} // Process_Buttons

// Prepare sleep mode
void Prepare_Sleep() {
	if (needupdate) {
		needupdate = 0;
		Flash_Update();
	}
	HAL_Delay(1);
	Stop_Light();
	u8g_SleepOn(&u8g);
#ifdef USB_DEBUG
	Stop_USB();
#endif
	Stop_PWM();
	Stop_Timer();
	Stop_Measure();
	HAL_Delay(1);
	Stop_ADC();
	Stop_SDADC();
	Stop_FET();
	rcoilzero = 999;
} // Prepare_Sleep

// Sleep after buttons key checkup
void Minimal_Sleep() {
	Stop_Timer();
	Stop_Periph();
	HAL_Delay(1);
	HAL_SuspendTick();
} // Minimal_Sleep

// Exiting sleep mode (just to be able to check buttons)
void Minimal_Unsleep() {
	HAL_ResumeTick();
	SystemClock_Config();
	Start_Periph();
	Stop_FET();
	Reset_Buttons();
	Start_Timer();
	HAL_Delay(4);
} // Minimal_Unsleep

// Back to normal function
void Finish_Unsleep() {
	loopcycle = 0;
	lastawake = HAL_GetTick();
	u8g_SleepOff(&u8g);
	Start_SDADC();
	Start_ADC();
	Start_PWM();
	Reset_Coil();
#ifdef CALIBRATE_SPREAD
	Calibrate_Spread();
#endif
	Calibrate_SDADC();
	Start_Measure();
#ifdef USB_DEBUG
	Start_USB();
#endif
} // Full_Unsleep

// main loop
void loop() {
	loopcycle++;

#ifndef DISABLE_SLEEP
	if (idlebuttons[I_BCOIL] > (sleeptime * 1000)) {
		// Can't go into sleep while charging
		if (!charging) {
			debug("Entering sleep\n");
			insleep = 1;
			Prepare_Sleep();
			do {
				Minimal_Sleep();

				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
				// HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

				Minimal_Unsleep();
			} while (!Wakeup_Check());
			Finish_Unsleep();
			insleep = 0;
		} else {
			// Just turn off screen if charging, turn on back when MENU is pressed
			debug("Turning off screen (%ld %ld)\n", idlebuttons[I_BCOIL], sleeptime);
			Reset_Coil();
			Stop_FET();
			u8g_SleepOn(&u8g);
			while(Just_Pushed(I_BMENU) == 0)
				HAL_Delay(10);
			u8g_SleepOff(&u8g);
			Reset_Buttons();
			lockuntilrelease = I_BMENU;
		}
	}
#endif

#ifdef EXTENDED_DEBUG
	debug("coilv %f (%d), vcheck %f (%d), vmainv %f (%d), output %f\n rcoil %f (%f:%f), vbat %f, vbalance %f, vusb %f, measures %u\n",
			coilv, coilv_readout, vcheck, vcheck_readout, vmainv, vmainv_readout, output,
			rcoil, tcoil, rcoilzero, vbat, vbalance, vusb, measures);
	debug("buttons: %d %d %d %d (%d)\n", tmpbuttons[0], tmpbuttons[1], tmpbuttons[2], tmpbuttons[3], coilheat);
//	debug("adc: %d %d %d %d %d %d %d\n", adcdata[0], adcdata[1], adcdata[2], adcdata[3],
//			VREFINT_CAL, TS_CAL1, TS_CAL2);
#endif

#ifdef BURST_DEBUG_DATA
	Check_Dump_Data(5);
#endif

	Process_Buttons();

	tcoil_show = tcoil;
	vmainv_show = vmainv;
	rcoil_show = rcoil;
	output_show = output;
	tchip_show = tchip;
	if (loopcycle > 1) {
		u8g_FirstPage(&u8g);
		do {
			if (inmenu)
				Draw_Menu();
			else if (inupdate)
				Draw_Item();
			else
				Draw_Main();
		} while (u8g_NextPage(&u8g));
	}

    HAL_Delay(50);
}
