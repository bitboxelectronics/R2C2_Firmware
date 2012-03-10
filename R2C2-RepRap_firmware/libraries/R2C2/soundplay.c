/* Copyright (c) 2012 Rui Ribeiro - racribeiro@gmail.com       */
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

#include   <string.h>
#include "lpc_types.h"
#include "buzzer.h"

#include   "soundplay.h"

static int whole_note_time = WHOLE_NOTE_TIME;
 
void set_whole_note_time(uint32_t new_note_time)
{
  whole_note_time = new_note_time;
} 
 
void play_music_string(char *music) {

  int noteCount = strlen(music);
  float freq = 0;
  uint32_t duration = 0;
  
  for (int i=0;i<noteCount;i+=2) {
    switch(music[i]){
      case 'D' :
        freq=FREQ_D4;
        break;
      case 'e' :
        freq=FREQ_E4;
        break;
      case 'f' :
        freq=FREQ_F4;
        break;
      case 'g' :
        freq=FREQ_G4;
        break;
      case 'a' :
        freq=FREQ_A4;
        break;
      case 'b' :
        freq=FREQ_B4;
        break;
      case 'c' :
        freq=FREQ_C5;
        break;
      case 'd' :
        freq=FREQ_D5;
        break;
    }
    //Note Timing
    switch(music[i+1]){
      case '1' :
        duration=whole_note_time;
        break;
      case '2' :
        duration=whole_note_time/2;
        break;
      case '4' :
        duration=whole_note_time/4;
        break;
      case '8' :
        duration=whole_note_time/8;
        break;
    }
		
    buzzer_play(freq, duration);		
	buzzer_wait();
  }
}

void play_jingle_bell()
{
  //Jingle Bells translated from sheet music
  char jingle_bells[] = 
  "D4"         "D4b4a4g4"   "D2 4D4"   "D4b4a4g4"
  "e2 4e4"     "e4c4b4a4"   "f2 4d4"   "d4d4c4a4"
  "b2g4D4"     "D4b4a4g4"   "D2 4D4"   "D4b4a4g4"
  "e2 4e4"     "e4c4b4a4"   "d4d4d4d4" "e4d4c4a4"
  "g2g4 4"     "b4b4b2"     "b4b4b2"   "b4d4g4a8" "b2"
  "c4c4c4c8"   "c4b4b4b8b8" "b4a4a4b4" "a2d2"
  "b4b4b2"     "b4b4b2"     "b4d4g4a8" "b2 4" "c4c4"
  "c4b4b4b8b8" "d4d4c4a4"   "g2";

  play_music_string(jingle_bells);
}