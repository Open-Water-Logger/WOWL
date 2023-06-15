/*
	wowlInit.c
	
	Initialization of hardware and software modules for the WOWL.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#include <fstorage.h>
#include <nrf52.h>
#include <nrf_clock.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include <system_nrf52.h>

#include <gpiote/nrf_drv_gpiote.h>
#include <twi_master/nrf_drv_twi.h>

#include <timer/app_timer.h>


extern int __vector_table[];

//********************************  Constants  *******************************//
#ifndef CONFIG_NFCT_PINS_AS_GPIOS
#	error CONFIG_NFCT_PINS_AS_GPIOS must be defined in project settings to \
		access NFC pins.
#endif

//***********************  Local Function Declarations  **********************//
static void initAppTimer(void);
static void initGpio(void);
static void initI2C(void);

static void i2cEventHandler(const nrf_drv_twi_evt_t *pEvent, void *pCtx);
static void nResetCallback(uint32_t pin, nrf_gpiote_polarity_t action);


//*******************************  Module Data  ******************************//
// We use instance 1, to avoid interrupt conflicts with SPIM0, which shares
//  the same vector (instantiation ID; see Table 11 in the PS).
static const nrf_drv_twi_t i2cInstance = NRF_DRV_TWI_INSTANCE(1);

static const nrf_drv_twi_config_t i2cConfig = {                                                                              \
	.frequency          = NRF_TWI_FREQ_400K,
	.scl                = PIN_SCL,
	.sda                = PIN_SDA,
	.interrupt_priority = WOWL_I2C_PRIO,                  
	.clear_bus_init     = true,    // attempts to clear stuck busses
	// if true, leave pullups in after unnint; we have pullups on the board
	.hold_bus_uninit    = false
};

static wowl_i2c_callback_t i2cSubCallback;

//****************************  Global Functions  ****************************//
void wowlInit(void)
{
	// Start the 32-MHz external crystal.  This is more accurate and 
	//  stable than the internal oscillator.
	nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);	

	// Initialize global GPIO.  GPIO controlled by specific modules is
	//  initialized there.
	initGpio();
	
	// Initialize the shared I2C module.
	initI2C();
	
	// Initialize the shared low-power RTC timer:
	initAppTimer();
					
	// Initialize the storage module:
	fs_ret_t fsResult = fs_init();	
	assert(fsResult == FS_SUCCESS);
		
	// Initialize the BLE software stack.  This activates the low frequency
	//  clock, and enables the soft-device.  Note this also initializes the
	//  wowlService module.	
	wowlBleInit();
	
	// Enable the internal DC-DC converter.
	(void) sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

	// Optionally perform hardware unit tests:
	wowlTests();	

	// Initialize the remaining modules in alphabetical order.	
	wowlAdcInit();
	wowlBattInit();
	wowlFlashInit();
	wowlLedInit();	
	wowlPmInit();
	// Note this initializes the digital sensors:
	wowlSampleInit();
	
	// Setup the watchdog timer:
	// TODO
	
	// Notify the state machine that initialization is complete:
	wowlSmSetEvent(EVENT_INIT_COMPLETE);
}

void wowlInitSetupGpioInterrupt(
		uint8_t pin, 
		fwd_gpiote_evt_handler_t pFun,
		nrf_gpiote_polarity_t polarity,
		nrf_gpio_pin_pull_t pullDir)
{	
	const nrf_drv_gpiote_in_config_t config = {
		.sense = polarity,
		.pull = pullDir,
		.is_watcher = false,
		.hi_accuracy = false
	};
	
	// Setup the GPIOTE on this pin.
	int result = nrf_drv_gpiote_in_init(
			pin,
			&config,
			(nrf_drv_gpiote_evt_handler_t) pFun
	);
	assert(result == NRF_SUCCESS);
	// Enable the interrupt.  Void return for this function.
	nrf_drv_gpiote_in_event_enable(pin, true);
}

bool wowlInitI2cXfer(uint8_t addr,
		const void* pTx, uint8_t nTx, void* pRx, uint8_t nRx,
		wowl_i2c_callback_t callback)
{
	i2cSubCallback = callback;
	
	// This is copied by the driver so may remain on the stack.
	nrf_drv_twi_xfer_desc_t xferDesc =     {                                                                  \
        .type = nRx ? NRF_DRV_TWI_XFER_TXRX // TX followed by RX
        		: NRF_DRV_TWI_XFER_TX,      // just TX
        .address = addr,          // 7-bit addr, copied directly to ADDRESS reg
        .primary_length   = nTx,
        .secondary_length = nRx,
        .p_primary_buf    = (uint8_t*) pTx,
        .p_secondary_buf  = pRx
    };
	
	const ret_code_t err = nrf_drv_twi_xfer(
			&i2cInstance,
			&xferDesc,
			0u             // no flags
	);
	
	assert(err == NRF_SUCCESS);
	return err == NRF_SUCCESS;
}


//***********************  Local Function Definitions  ***********************//

static void initAppTimer(void)
{
	DISABLE_INTR();
	// Initialize timer module, making it use the scheduler
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	ENABLE_INTR();
}


static void initGpio(void)
{
	///// Configure Outputs  /////
	static const int8_t outputPins[] = {
		PIN_TP21,
		PIN_TP20,
		PIN_TP19,
		PIN_TP27,
	};
	
	for (uint32_t iPin = 0u; iPin < _countof(outputPins); ++iPin) {
		// Latch each pin low, and then set as an output pin:
		nrf_gpio_pin_clear(outputPins[iPin]);
		nrf_gpio_cfg_output(outputPins[iPin]);
	}
	
	// Initialize the GPIO interrupt driver, if necessary.
	//  Return value indicates if already done,
	//  which we discard as uninteresting.
	(void) nrf_drv_gpiote_init();
	
	/////  Configure Inputs  /////
	// Set the BLE_nRESET pin to be an input with a pull-up.  This is necessary
	//  because there is no pullup on the board.  It will trigger an interrupt.
	wowlInitSetupGpioInterrupt(
			PIN_BLE_nRESET,
			nResetCallback,
			NRF_GPIOTE_POLARITY_HITOLO,
			NRF_GPIO_PIN_PULLUP
	);
}

static void initI2C(void)
{
	ret_code_t err;
	
	// Initialize the I2C instance:
	err = nrf_drv_twi_init(
			&i2cInstance,
			&i2cConfig,
			i2cEventHandler,
			NULL  // no user context
	);
	assert(err == NRF_SUCCESS);
	
	// Enable it.  TODO: may only want to do when needed.
	nrf_drv_twi_enable(&i2cInstance);	
}

// Callback from driver after I2C transfer.
static void i2cEventHandler(const nrf_drv_twi_evt_t *pEvent, void *pCtx)
{
	UNUSED(pCtx);
	
	if (i2cSubCallback) {
		i2cSubCallback(pEvent);
	} else {
		// This should not occur.
		assert(false);
		wowlMainSetStatus(WOWL_STATUS_SPURIOUS_I2C_EVENT);
	}	
}

// Callback when nReset is pulled low.
static void nResetCallback(uint32_t pin, nrf_gpiote_polarity_t action)
{
#ifdef NDEBUG
	// Reset the processor.
	sd_nvic_SystemReset();
#else
	// Halt in the debugger.
	assert(false);
#endif
}

