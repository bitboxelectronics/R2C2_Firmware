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

#ifndef	_UART_H
#define	_UART_H

typedef enum {
  parity_none,
  parity_even,
  parity_odd,
  parity_always1,
  parity_always0} tParity;

typedef struct {
  uint32_t  baud_rate;
  uint8_t   data_bits;
  tParity   parity;
  uint8_t   stop_bits;
} tPortSettings;

void uart_init(int uart_num);
bool uart_configure(int uart_num, tPortSettings *port_settings_p);
int  uart_data_available(int uart_num);
char uart_receive(int uart_num);
void uart_send(int uart_num, char byte);

#if 0
void uart_writestr(char *data);
#endif

void uart0_init(void);
int  uart0_data_available(void);
char uart0_receive(void);
void uart0_send(char byte);

//void uart0_writestr(char *data);

void uart1_init(void);
int  uart1_data_available(void);
char uart1_receive(void);
void uart1_send(char byte);

void uart2_init(void);
int  uart2_data_available(void);
char uart2_receive(void);
void uart2_send(char byte);

void uart3_init(void);
int  uart3_data_available(void);
char uart3_receive(void);
void uart3_send(char byte);

void uart3_writestr(char *data);


// #define serial_writechar(x) uart_send(x)

#endif	/* _UART_H */
