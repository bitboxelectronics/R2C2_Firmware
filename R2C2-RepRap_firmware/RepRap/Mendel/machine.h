/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef	_MACHINE_H
#define	_MACHINE_H

/*
	firmware build options
*/

#define REPRAP_HOST_COMPATIBILITY

// This overrides ACCELERATION_REPRAP and ACCELERATION_RAMPING
#define USE_GRBL

/*
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up
*/
#define	MOVEBUFFER_SIZE	8

//#define STEPPER_ENABLE_PIN

#define F_CPU 100000000 /* 100MHz */


/* Stepper LED Flashing options */
//#define STEP_LED_NONE             // No LED control, easier to see stepper output
//#define STEP_LED_ON_WHEN_ACTIVE   // LED on when stepper output is active
//#define STEP_LED_FLASH_FIXED      // LED Flash at fixed rate
#define STEP_LED_FLASH_VARIABLE   // LED Flash at variable rate according to stepper speed

//#define USE_BOOT_BUTTON


#endif	/* _MACHINE_H */  
