/*
	wowlAssert.c
	
	Handler for assertions.
	
	2020-09-22  WHF  Created.
*/

#include "wowl.h"

#if 1

#include <nrf_assert.h>


//********************************  Constants  *******************************//

// Value used as error code on stack dump, can be used to identify 
//  stack location on stack unwind.
#define DEAD_BEEF						0xDEADBEEF									


//****************************  Global Functions  ****************************//
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *			how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num	  Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#endif

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
#ifndef NDEBUG
	printf("FAULT id=%08x, pc=%08x, inf=%08x\n", id, pc, info);
	DEBUG_BREAK();
#endif
}

void app_error_handler_bare(ret_code_t error_code)
{
#ifndef NDEBUG
	printf("ERROR code %08x\n", error_code);
	DEBUG_BREAK();
#endif
}



