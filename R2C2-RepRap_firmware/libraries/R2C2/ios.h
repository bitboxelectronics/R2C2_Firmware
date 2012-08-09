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

#ifndef IOS_H
#define IOS_H

#include <stdint.h>

#define INPUT   0
#define OUTPUT  1

#define LOW     0
#define HIGH    1

#define DISABLE     0
#define ENABLE      1

#define INACTIVE     0
#define ACTIVE       1

#define ACTIVE_HIGH 0
#define ACTIVE_LOW  1

// create a struct initialiser
#define PIN_DEF(port,pin,polarity) {(port),(pin),(polarity),0}

// special pin defs
#define UNDEFINED_PORT         0xFF
#define UNDEFINED_PIN_NUMBER   0xFF

#define UNDEFINED_PIN_DEF     PIN_DEF(UNDEFINED_PORT, UNDEFINED_PIN_NUMBER, 0)

// convert a bit number (0-31) to a 32 bit mask
#define _BV(bit) (1 << (bit))

// ----
// encode port and bit number in a byte
// port 0-7
// bit 0-31
#define ENCODE_PORT_BIT(port,bit) ((port)<<5)|(bit)

// decode to port number and bitmask
//#define DECODE_PORT_BITMASK(port_bit) ((port_bit)>>5),1<<((port_bit) & 0x1F)

// decode to port number
#define DECODE_PORTNUM(port_bit) ((port_bit)>>5)
// decode to bit number
#define DECODE_BITNUM(port_bit) ((port_bit) & 0x1F)
// decode to bit mask
#define DECODE_BITMASK(port_bit) (_BV((port_bit) & 0x1F))

#define PACKED_PORT_BIT(pindef) ((pindef.port)<<5)|(pindef.pin_number)


#define pinMode(pin,mode)       pin_mode (DECODE_PORTNUM(pin), DECODE_BITMASK(pin), mode)
#define digitalWrite(pin,value) digital_write (DECODE_PORTNUM(pin), DECODE_BITMASK(pin), value)
#define digitalRead(pin)        digital_read (DECODE_PORTNUM(pin), DECODE_BITMASK(pin))

// ----

typedef struct 
{
  uint8_t port;
  uint8_t pin_number;
  uint8_t active_low;
  uint8_t reserved;
} tPinDef;



void      pin_mode (uint8_t portNum, uint32_t bitMask, uint8_t dir);
uint32_t  digital_read (uint8_t portNum, uint32_t bitMask);
void      digital_write (uint8_t portNum, uint32_t bitMask, uint8_t state);


void      set_pin_mode (tPinDef pin, uint8_t dir);
uint32_t  read_pin (tPinDef pin);
void      write_pin (tPinDef pin, uint8_t state);

#endif
