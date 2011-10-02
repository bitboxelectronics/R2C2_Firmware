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

#include        "sersendf.h"
#include        <stdarg.h>
#include        "serial.h"
#include        "sermsg.h"

char   str_ox[] = "0x";

void sersendf(char *format, ...) {
        va_list args;
        va_start(args, format);

        unsigned int i = 0;
        unsigned char c, j = 0;
        while ((c = format[i++])) {
                if (j) {
                        switch(c) {
                                case 'l':
                                        j = 4;
                                        break;
                                case 'u':
                                        if (j == 4)
                                                serwrite_uint32(va_arg(args, unsigned int));
                                        else
                                                serwrite_uint16(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 'd':
                                        if (j == 4)
                                                serwrite_int32(va_arg(args, int));
                                        else
                                                serwrite_int16(va_arg(args, int));
                                        j = 0;
                                        break;

                                /* print a double in normal notation */
                                case 'g':
                                serwrite_double(va_arg(args, double));
                                j = 0;
                                break;

                                case 'p':
                                case 'x':
                                        serial_writestr(str_ox);
                                        if (j == 4)
                                                serwrite_hex32(va_arg(args, unsigned int));
                                        else
                                                serwrite_hex16(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 'c':
                                        serial_writechar(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 's':
                                        serial_writestr(va_arg(args, char *));
                                        j = 0;
                                        break;
                                default:
                                        j = 0;
                                        break;
                        }
                }
                else {
                        if (c == '%') {
                                j = 2;
                        }
                        else {
                                serial_writechar(c);
                        }
                }
        }
        va_end(args);
}
