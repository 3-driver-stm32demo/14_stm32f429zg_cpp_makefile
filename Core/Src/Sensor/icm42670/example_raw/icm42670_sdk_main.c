/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "example_raw.h"

// /* InvenSense utils */
// #include "Invn/EmbUtils/Message.h"
// #include "Invn/EmbUtils/ErrorHelper.h"
// #include "Invn/EmbUtils/RingBuffer.h"

// /* board driver */
// #include "common.h"
// #include "uart_mngr.h"
// #include "delay.h"
// #include "gpio.h"
// #include "timer.h"

// #include "system_interface.h"

#include "Sensor/icm42670/system_interface/system_interface.h"

/* std */
#include <stdio.h>

/*
 * Select UART port on which INV_MSG() will be printed.
 */
#define LOG_UART_ID INV_UART_SENSOR_CTRL

/* 
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

/* 
 * Set of timers used throughout standalone applications 
 */
#define TIMEBASE_TIMER INV_TIMER1
#define DELAY_TIMER    INV_TIMER2

/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */

#if !USE_FIFO
/* 
	 * Buffer to keep track of the timestamp when IMU data ready interrupt fires.
	 * The buffer can contain up to 64 items in order to store one timestamp for each packet in FIFO.
	 */
RINGBUFFER(timestamp_buffer, 64, uint64_t);
#endif

/* --------------------------------------------------------------------------------------
 *  Static variables
 * -------------------------------------------------------------------------------------- */

/* Flag set from IMU device irq handler */
static volatile int irq_from_device;

/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */
static int  setup_mcu(struct inv_imu_serif *icm_serif);
static void ext_interrupt_cb(void *context, unsigned int int_num);
static void check_rc(int rc, const char *msg_context);
//void        msg_printer(int level, const char *str, va_list ap);

/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

int icm42670_sdk_main_init(void)
{
	int                  rc = 0;
	struct inv_imu_serif icm_serif;

	rc |= setup_mcu(&icm_serif);
	rc |= setup_imu_device(&icm_serif);
	rc |= configure_imu_device();
	check_rc(rc, "error during initialization");
	printf("IMU device successfully initialized");

	// do {
	// 	/* Poll device for data */
	// 	if (irq_from_device & TO_MASK(INV_GPIO_INT1)) {
	// 		inv_disable_irq();
	// 		irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
	// 		inv_enable_irq();

	// 		rc = get_imu_data();
	// 		check_rc(rc, "error while getting data");
	// 	}

	// } while (1);
}

int icm42670_sdk_main_data_read(void)
{
	int rc = 0;
	rc = get_imu_data();
	return rc;
}	


/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group and GPIO so that MCU can receive interrupts from IMU 
 *   - a microsecond timer requested by IMU driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a serial link to communicate from MCU to IMU 
 */
static int setup_mcu(struct inv_imu_serif *icm_serif)
{
	int rc = 0;
	inv_io_hal_board_init();
	/* Initialize serial interface between MCU and IMU */
	icm_serif->context    = 0; /* no need */
	icm_serif->read_reg   = inv_io_hal_read_reg;
	icm_serif->write_reg  = inv_io_hal_write_reg;
	icm_serif->max_read   = 1024 * 32; /* maximum number of bytes allowed per serial read */
	icm_serif->max_write  = 1024 * 32; /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;

	return rc;
}

/*
 * IMU interrupt handler.
 * Function is executed when an IMU interrupt rises on MCU.
 * This function get a timestamp and store it in the timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * are implemented for shared variable timestamp_buffer.
 */
// static void ext_interrupt_cb(void *context, unsigned int int_num)
// {
// 	(void)context;

// #if !USE_FIFO
// 	/* 
// 	 * Read timestamp from the timer dedicated to timestamping 
// 	 */
// 	uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);

// 	if (int_num == INV_GPIO_INT1) {
// 		if (!RINGBUFFER_FULL(&timestamp_buffer))
// 			RINGBUFFER_PUSH(&timestamp_buffer, &timestamp);
// 	}
// #endif

// 	irq_from_device |= TO_MASK(int_num);
// }

const char * inv_error_str(int error)
{
	switch(error) {
	case INV_ERROR_SUCCESS:      return "Success";
	case INV_ERROR:              return "Unspecified error";
	case INV_ERROR_NIMPL:        return "Not implemented";
	case INV_ERROR_TRANSPORT:    return "Transport error";
	case INV_ERROR_TIMEOUT:      return "Timeout, action did not complete in time";
	case INV_ERROR_SIZE:         return "Wrong size error";
	case INV_ERROR_OS:           return "Operating system failure";
	case INV_ERROR_IO:           return "Input/Output error";
	case INV_ERROR_MEM: 		 return "Bad allocation";
	case INV_ERROR_HW:           return "Hardware error";
	case INV_ERROR_BAD_ARG:      return "Invalid arguments";
	case INV_ERROR_UNEXPECTED:   return "Unexpected error";
	case INV_ERROR_FILE:         return "Invalid file format";
	case INV_ERROR_PATH:         return "Invalid file path";
	case INV_ERROR_IMAGE_TYPE:   return "Unknown image type";
	case INV_ERROR_WATCHDOG:     return "Watchdog error";
	default:                     return "Unknown error";
	}
}

/*
 * Helper function to check RC value and block program exectution
 */
static void check_rc(int rc, const char *msg_context)
{
	if (rc < 0) {
		printf("%s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
		while (1)
			;
	}
}

// /*
//  * Printer function for message facility
//  */
// void msg_printer(int level, const char *str, va_list ap)
// {
// 	static char out_str[256]; /* static to limit stack usage */
// 	unsigned    idx                  = 0;
// 	const char *s[INV_MSG_LEVEL_MAX] = {
// 		"", // INV_MSG_LEVEL_OFF
// 		"[E] ", // INV_MSG_LEVEL_ERROR
// 		"[W] ", // INV_MSG_LEVEL_WARNING
// 		"[I] ", // INV_MSG_LEVEL_INFO
// 		"[V] ", // INV_MSG_LEVEL_VERBOSE
// 		"[D] ", // INV_MSG_LEVEL_DEBUG
// 	};
// 	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
// 	if (idx >= (sizeof(out_str)))
// 		return;
// 	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
// 	if (idx >= (sizeof(out_str)))
// 		return;
// 	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
// 	if (idx >= (sizeof(out_str)))
// 		return;

// 	inv_uart_mngr_puts(LOG_UART_ID, out_str, idx);
// }

/* --------------------------------------------------------------------------------------
 *  Extern functions definition
 * -------------------------------------------------------------------------------------- */


#include "stm32f4xx_hal.h"

__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}
//?????????us
uint32_t micros(void)
{
  return getCurrentMicros();
}

/* Get time implementation */
uint64_t inv_imu_get_time_us(void)
{
	return micros();
}

void inv_imu_sleep_us(uint32_t us)
{
	uint32_t time_us_now =0 ;
	time_us_now = inv_imu_get_time_us();
	uint32_t wait_time_us = us + time_us_now;
	while (wait_time_us>inv_imu_get_time_us())
	{
		/* code */
	}	
}
