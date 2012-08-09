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

#include "lpc_types.h"
#include "lpc17xx_gpio.h"

#include "ios.h"


/* Initialize all the IO pins */
/* Example of usage: pin_mode(PORT_0, X_STEP_PIN, OUTPUT); */
void pin_mode(uint8_t portNum, uint32_t bitMask, uint8_t dir)
{
    FIO_SetDir(portNum, bitMask, dir);
}

/* Example of usage: digital_write(PORT_0, X_STEP_PIN, HIGH); */
void digital_write(uint8_t portNum, uint32_t bitMask, uint8_t state)
{
    if (state)
        FIO_SetValue(portNum, bitMask);

    else
        FIO_ClearValue(portNum, bitMask);
}

/* Example of usage: value = digital_read(PORT_0, PIN); */
uint32_t digital_read(uint8_t portNum, uint32_t bitMask)
{
  return ((FIO_ReadValue(portNum) & bitMask) ? 1 : 0);
}

// 
// Using tPinDef
//

void      set_pin_mode (tPinDef pin, uint8_t dir)
{
  if (pin.port != UNDEFINED_PORT)
    FIO_SetDir(pin.port, _BV(pin.pin_number), dir);
}

uint32_t  read_pin (tPinDef pin)
{
  if (pin.port == UNDEFINED_PORT)
    return 0;
  else
    return ((FIO_ReadValue(pin.port) & _BV(pin.pin_number)) ? 1 : 0 ) ^ pin.active_low;
}

void  write_pin (tPinDef pin, uint8_t state)
{
  if (pin.port != UNDEFINED_PORT)
  {
    if (state ^ pin.active_low)
      FIO_SetValue (pin.port, _BV(pin.pin_number));
    else
      FIO_ClearValue (pin.port, _BV(pin.pin_number));
  }
}


