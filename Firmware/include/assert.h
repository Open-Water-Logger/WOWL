/*
	assert.h
	
	Simple implementation of run-time assertions, that merely causes
	a debugger breakpoint.
	
	2016-11-29  WHF  Created.
*/

#ifndef __ASSERT_H__
#define __ASSERT_H__

#include <cmsis_compiler.h>

// Suppress MISRA-C rule 20.1 (std lib not redefined)
#pragma diag_suppress=Pm098

// Suppress MISRA-C rule 20.2 (std lib macros not reused)
#pragma diag_suppress=Pm150

#ifdef NDEBUG
// Suppress MISRA-C rules against code without side effects.
#	define assert(x)                      \
		_Pragma("diag_suppress=Pm049")    \
		_Pragma("diag_suppress=Pm136")    \
		((void) (x))                      \
		_Pragma("diag_default=Pm049")     \
		_Pragma("diag_default=Pm136")

#else
#	define assert(x)     do { if (!(x)) { __BKPT(0x99); } else { } } while(0)
#endif

#pragma diag_default=Pm098
#pragma diag_default=Pm150

// Static assertions should always be active:
#define static_assert  _Static_assert

#endif /* __ASSERT_H__ */


