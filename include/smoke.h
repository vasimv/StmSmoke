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

// Dump debug information through USB virtual com-port (may create weird effects, DON'T USE if you don't need)
// #define USB_DEBUG

#ifdef USB_DEBUG
// Disable USB charging (for test purposes)
#define DISABLE_CHARGING

// Disable Sleep mode (for test purposes)
#define DISABLE_SLEEP

// Extended debug (very noisy and may hung up)
#define EXTENDED_DEBUG

// Collecting coil data into internal memory to dump it after 10 seconds (doesn't work actually)
// #define BURST_DEBUG_DATA
#endif

// 48 MHz clocking
#define PWM_PERIOD_1MHZ 48
#define PWM_PERIOD_100KHZ 480
#define PWM_PERIOD_10KHZ 4800

// Use inductor-based schematics (double check or it'll fry your board!)
// Inductor - TIM2_CH1 - Coil FET with gate driver, TIM3_CH3 - USB boost FET, PC13 - LED/with resistor
//  ADC1_IN0 - USB, ADC1_IN2 - 2S balance pin, SDADC1_AIN7P - Current sense, SDADC3_AIN6P - Battery
//  Only 4 sensor buttons
//
// Without inductor - TIM2_CH1 - Coil FET without driver, TIM3_CH1 - LED/with inductor, TIM3_CH3 - USB boost FET, PE8 - Resistance test FET,
//  ADC1_IN0 - USB, ADC1_In2 - 2S balance pin, SDADC1_AIN7P - Resistance test, SDADC2_AIN6P - Resistance test FET drain, SDADC3_AIN6P - Battery
//  5 sensor buttons

// #define INDUCTOR_BASED

#ifdef INDUCTOR_BASED
#define NBUTTONS 4
#define PWM_COIL_PERIOD PWM_PERIOD_1MHZ // 1 MHz pwm output, with fast-switching gate driver
#define LED_PC13
#define DISABLE_LIGHT
#else
#define NBUTTONS 5
#define PWM_COIL_PERIOD PWM_PERIOD_10KHZ // 10 KHz pwm output, without gate driver
#define LED_TIM3CH1
// duty for LED buck regulator (70 mA current with two white LEDs in series)
#define LED_PWM_DUTY 36
#endif

// Swap LIGHT and MENU buttons
#if NBUTTONS == 5
#define SWAP_MENU_LIGHT
#endif

// Disable light button
#define DISABLE_LIGHT

// Maximum coil heat time (in milliseconds)
#define MAX_COIL_TIME 60000

// Maximum power in variwatt mode, in watts
#define MAX_COIL_POWER 140

// test FET switching delay
#define FET_SWITCH_DELAY 36

// buttons debounce delay (in milliseconds)
#define BUTTONS_DEBOUNCE 100

// touch sensors threshold koefficient (from maximum at calibration)
#define BUTTONS_THRESHOLD 0.90

// buttons repeat delay (in milliseconds)
#define BUTTONS_REPEAT_DELAY 500

// Write configuration to flash after last change (in milliseconds)
#define UPDATE_FLASH_AFTER 30000

// Frequency for measure/pid routine call (without inductor - only 500Hz/1kHz is supported
// as it will heat coil a bit). 200Hz is for debugging
// #define MEASURE_200HZ
#define MEASURE_500HZ
// #define MEASURE_1KHZ
// #define MEASURE_5KHZ
// #define MEASURE_10KHZ

// VREF used for SDADCs (1.8 volts internal)
#define VREF_SDADC 1.8

// Test resistor resistance, in Ohms
#define RTEST 15.0

// Warning (flashing on screen) battery voltage level
#define WARN_VOLTAGE 6.6

// Minimum battery voltage (won't heat coil if less)
#define MIN_VOLTAGE 6.2

// Screen type
// not supported yet - #define SSD1306_96_32
#define SSD1306_128_64

// Defined in main.c
extern SDADC_HandleTypeDef hsdadc1;
extern SDADC_HandleTypeDef hsdadc2;
extern SDADC_HandleTypeDef hsdadc3;
extern TSC_HandleTypeDef htsc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern TSC_HandleTypeDef htsc;

extern void SystemClock_Config();
extern void MX_GPIO_Init(void);
extern void MX_I2C1_Init(void);
extern void MX_TIM2_Init(void);
extern void MX_TIM3_Init(void);
extern void MX_TSC_Init(void);
extern void MX_RTC_Init(void);

#ifdef USB_DEBUG
#include "usbd_def.h"
// Defined in usb_device.c
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

// STM32F37x datasheet addresses
//Temperature sensor raw value at 30 degrees C, VDDA=3.3V
#define TS_CAL1 (*((uint16_t *) 0x1FFFF7B8))
//Temperature sensor raw value at 110 degrees C, VDDA=3.3V
#define TS_CAL2 (*((uint16_t *) 0x1FFFF7C2))
//Internal voltage reference raw value at 30 degrees C, VDDA=3.3V
#define VREFINT_CAL (*((uint16_t *) 0x1FFFF7BA))

// Debug output routine (over USB if USB_DEBUG defined)
void debug(char *fmt, ...);
// Initialization (after HAL inits)
void Setup();
// Main loop
void loop();
// Measure/PID interrupt routine
void measure_int();
// 1 kHz low priority interrupt routine (buttons debounce, some checks, etc)
void timer_int();

// How many PWM periods between measures (for the PID controller)
#if defined(MEASURE_200HZ)
#define PID_TIMECHANGE ((double) (240000 / PWM_COIL_PERIOD))
#elif defined(MEASURE_500HZ)
#define PID_TIMECHANGE ((double) (96000 / PWM_COIL_PERIOD))
#elif defined(MEASURE_1KHZ)
#define PID_TIMECHANGE ((double) (48000 / PWM_COIL_PERIOD))
#elif defined(MEASURE_5KHZ)
#define PID_TIMECHANGE ((double) (9600 / PWM_COIL_PERIOD))
#elif defined(MEASURE_10KHZ)
#define PID_TIMECHANGE ((double) (4800 / PWM_COIL_PERIOD))
#endif
