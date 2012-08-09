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

#ifndef CONFIG_H
#define CONFIG_H

#include "stdint.h"
#include "stdbool.h"

#include "ios.h"
#include "ff.h"

#define TYPE_INT      0
#define TYPE_DOUBLE   1
#define TYPE_PIN_DEF  2

typedef struct {
  char      *name;
  void      *pValue;
  uint8_t   type;
  union {
    int32_t   val_i;
    double    val_d;
    tPinDef   val_pin_def;
    };
} tConfigItem;

typedef struct {
  char *key;
  uint32_t hash;
  } tKeyHash ;

void set_defaults (const tConfigItem lookup[], int num_tokens);

FRESULT read_config_file (char *filename, const tConfigItem lookup[], int num_tokens, tKeyHash hashes[]);

void print_config_table (const tConfigItem lookup[], int num_token);

void create_key_hash_table (int num_tokens,  const tConfigItem lookup[], tKeyHash hashes[]);

#endif /* CONFIG_H */
