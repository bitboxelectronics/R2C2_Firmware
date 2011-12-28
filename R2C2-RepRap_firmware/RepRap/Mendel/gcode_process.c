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

#include   <string.h>
#include	"gcode_process.h"
#include	"gcode_parse.h"

#include	"serial.h"
#include	"sermsg.h"
#include	"sersendf.h"

#include	"temp.h"
#include "timer.h"
#include "pinout.h"
#include "config.h"
#include "ff.h"
#include "buzzer.h"
//#include "debug.h"

#include "planner.h"
#include "stepper.h"
#include "geometry.h"

FIL       file;
uint32_t  filesize = 0;
uint32_t  sd_pos = 0;
bool      sd_printing = false;      // printing from SD file
bool      sd_active = false;        // SD card active
bool      sd_writing_file = false;  // writing to SD file

#define EXTRUDER_NUM_1  1
#define EXTRUDER_NUM_2  2
#define EXTRUDER_NUM_3  4

uint8_t   extruders_on;
double    extruder_1_speed;         // in RPM

uint32_t  auto_prime_steps = 0;
uint32_t  auto_reverse_steps = 0;
const double auto_prime_feed_rate = 18000;
const double auto_reverse_feed_rate = 18000;
double auto_prime_factor = 640;
double auto_reverse_factor = 640;

#if 0
static void enqueue_move (TARGET *pTarget)
{
#ifndef USE_GRBL
  enqueue(pTarget);
#else      
  //grbl
  tActionRequest request;
  
  if (pTarget->options.g28)
    request.ActionType = AT_MOVE_ENDSTOP;
  else
    request.ActionType = AT_MOVE;
  request.target.x = (double)pTarget->X / config.steps_per_mm_x;
  request.target.y = (double)pTarget->Y / config.steps_per_mm_y; 
  request.target.z = (double)pTarget->Z / config.steps_per_mm_z; 
  request.target.e = (double)pTarget->E / config.steps_per_mm_e; 
  request.target.feed_rate = (double)pTarget->F; 
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
  //startpoint = *pTarget;
#endif
}
#endif

static void enqueue_moved (tTarget *pTarget)
{
  // grbl
  tActionRequest request;
  
  if (pTarget->x != startpoint.x || pTarget->y != startpoint.y ||
      pTarget->z != startpoint.z || pTarget->e != startpoint.e
     )
  {  
    request.ActionType = AT_MOVE;
    request.target= *pTarget;
    request.target.invert_feed_rate =  false;
    
    if (config.enable_extruder_1 == 0)
      request.target.e = startpoint.e;

    plan_buffer_action (&request);
  }
  else
  {
    // no move, just set feed rate
    plan_set_feed_rate (pTarget);
  }
}

static void enqueue_wait_temp (void)
{
#ifndef USE_GRBL
  enqueue(NULL);
#else      
#if 1
  //grbl
  tActionRequest request;
  
  request.ActionType = AT_WAIT_TEMPS;
  plan_buffer_action (&request);
#else
  while (!temp_achieved(EXTRUDER_0))
  {
    //temp_tick();
  }
  serial_writestr("Temp achieved\r\n");
#endif
#endif
}

// wait for move queue to be empty
static void synch_queue (void)
{
#ifndef USE_GRBL
  // wait for queue to complete
  for (;queue_empty() == 0;) {}
#else
  st_synchronize();
#endif
}

static void SpecialMoveXY(double x, double y, double f) 
{
  tActionRequest request;
  
  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = x;
  request.target.y = y;
  request.target.z = startpoint.z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f; 
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

static void SpecialMoveZ(double z, double f) 
{
  tActionRequest request;
  
  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = startpoint.x;
  request.target.y = startpoint.y;
  request.target.z = z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f; 
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

#if 0
// this function is not used
static void SpecialMoveE(int32_t e, uint32_t f) {
	TARGET t = startpoint;
#ifdef ACCELERATION_REPRAP
	// no accel for ACCEL_REPRAP
	startpoint.F = f;
#endif
	t.E = e;
	t.F = f;
	//t.options.g28 = 1; /* signal a G28 command */
	enqueue(&t);
}
#endif

static void SpecialMoveE (double e, double feed_rate) 
{
  tTarget next_targetd;
  
  if (config.enable_extruder_1)
  {
    next_targetd = startpoint;
    next_targetd.e = startpoint.e + e;
    next_targetd.feed_rate = feed_rate;
    enqueue_moved(&next_targetd);
  }
}

static void zero_x(void)
{
  //TODO: move distance needs to be a least as big as print area
  
  // hit endstops, no acceleration- we don't care about skipped steps
  SpecialMoveXY(startpoint.x - 300, startpoint.y, config.homing_feedrate_x);
  
  synch_queue();
  
  // move forward a bit
  SpecialMoveXY(startpoint.x + 3, startpoint.y, config.search_feedrate_x);

  // move back in to endstops slowly
  SpecialMoveXY(startpoint.x - 6, startpoint.y, config.search_feedrate_x);
  
  synch_queue();

  // this is our home point
  //startpoint.X = current_position.X = config.home_pos_x * config.steps_per_mm_x;
  tTarget new_pos = startpoint;
  new_pos.x = config.home_pos_x;
  plan_set_current_position (&new_pos);
}

static void zero_y(void)
{
  int dir;

  // hit endstops, no acceleration- we don't care about skipped steps
  if (config.home_direction_y < 0)
    dir = -1;
  else
    dir = 1;
    
  SpecialMoveXY(startpoint.x, startpoint.y + dir * 250, config.homing_feedrate_y);
  
  synch_queue();
  
  // move forward a bit
  SpecialMoveXY(startpoint.x, startpoint.y - dir * 3, config.search_feedrate_y);
  // move back in to endstops slowly
  SpecialMoveXY(startpoint.x, startpoint.y + dir * 6, config.search_feedrate_y);

  synch_queue();

  // this is our home point
  //startpoint.Y = current_position.Y = config.home_pos_y * config.steps_per_mm_y;
  tTarget new_pos = startpoint;
  new_pos.y = config.home_pos_y;
  plan_set_current_position (&new_pos);
}

static void zero_z(void)
{
  // hit endstops, no acceleration- we don't care about skipped steps
  SpecialMoveZ(startpoint.z - 250, config.homing_feedrate_z);  
  synch_queue();
  
  // move forward a bit
  SpecialMoveZ(startpoint.z + 1, config.search_feedrate_z);
  synch_queue();

  // move back in to endstops slowly
  SpecialMoveZ(startpoint.z - 6, config.search_feedrate_z);
  synch_queue();

  // this is our home point
  //startpoint.Z = current_position.Z = config.home_pos_z * config.steps_per_mm_z;
  tTarget new_pos = startpoint;
  new_pos.z = config.home_pos_z;
  plan_set_current_position (&new_pos);
}

static void zero_e(void)
{
  // extruder only runs one way and we have no "endstop", just set this point as home
  //startpoint.E = current_position.E = 0;
  tTarget new_pos = startpoint;
  new_pos.e = 0;
  plan_set_current_position (&new_pos);
}

void sd_initialise(void)
{
  sd_active = true;
}

FRESULT sd_list_dir_sub (char *path)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;
#if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

    res = f_opendir(&dir, path);
    if (res == FR_OK) 
    {
      i = strlen(path);
      for (;;) 
      {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            if (fno.fname[0] == '.') continue;
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) 
            {
                sersendf("%s/%s/\r\n", path, fn);
                
                strcat (path, "/");
                strcat (path, fn);
                // sprintf(&path[i], "/%s", fn);
                res = sd_list_dir_sub(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else 
            {
                sersendf("%s/%s\r\n", path, fn);
            }
        }
    }

    return res;    
}

void sd_list_dir (void)
{
  char path[120];
    
  strcpy (path, "");
    
  sd_list_dir_sub(path);
}

unsigned sd_open(FIL *pFile, char *path, uint8_t flags)
{
  FRESULT res;

  res = f_open (pFile, path, flags);

  if (res == FR_OK)
  {
    return 1;
  }
  else
  {
    //debug
    sersendf ("sd_open:%d", res);
    return 0;
  }
}

void sd_close(FIL *pFile)
{
  f_close (pFile);
}

bool sd_read_file(tLineBuffer *pLine)
{
  char *ptr;

  ptr = f_gets(pLine->data, MAX_LINE, &file);

  if (ptr != NULL)
  {
    pLine->len = strlen(ptr);
    sd_pos += pLine->len;
    return true;
  }
  else
  {
    return false;
   }
}
 
bool sd_write_to_file(char *pStr, unsigned bytes_to_write)
{
  UINT bytes_written;
  FRESULT result;

  result = f_write (&file, pStr, bytes_to_write, &bytes_written);

  return result == FR_OK;
}

unsigned sd_filesize (FIL *pFile)
{
  return f_size(pFile);
}

void sd_seek(FIL *pFile, unsigned pos)
{
  f_lseek (pFile, pos);
}

/****************************************************************************
*                                                                           *
* Command Received - process it                                             *
*                                                                           *
****************************************************************************/

eParseResult process_gcode_command()
{
  double backup_f;
  uint8_t axisSelected = 0;
  eParseResult result = PR_OK;
  bool reply_sent = false;
  
  tTarget next_targetd = startpoint;

  // convert relative to absolute
  if (next_target.option_relative)
  {  
    next_targetd.x = startpoint.x + next_target.target.x;
    next_targetd.y = startpoint.y + next_target.target.y;
    next_targetd.z = startpoint.z + next_target.target.z;
    next_targetd.e = startpoint.e + next_target.target.e;
    if (next_target.seen_F)
      next_targetd.feed_rate = next_target.target.feed_rate;
  }
  else
  {
    // absolute
    if (next_target.seen_X)
      next_targetd.x = next_target.target.x;
    if (next_target.seen_Y)
      next_targetd.y = next_target.target.y;
    if (next_target.seen_Z)
      next_targetd.z = next_target.target.z;
    if (next_target.seen_E)
      next_targetd.e = next_target.target.e;  
    if (next_target.seen_F)
      next_targetd.feed_rate = next_target.target.feed_rate;
  }

//  sersendf(" X:%ld Y:%ld Z:%ld E:%ld F:%ld\r\n", (int32_t)next_target.target.X, (int32_t)next_target.target.Y, (int32_t)next_target.target.Z, (int32_t)next_target.target.E, (uint32_t)next_target.target.F);
//  sersendf(" X:%g Y:%g Z:%g E:%g F:%g\r\n", next_targetd.x, next_targetd.y, next_targetd.z, next_targetd.e, next_targetd.feed_rate);
    
  // E ALWAYS absolute 
  // host should periodically reset E with "G92 E0", otherwise we overflow our registers after only a few layers
	
  if (next_target.seen_G)
  {
    switch (next_target.G)
    {
      // G0 - rapid, unsynchronised motion
      // since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
      case 0:
      backup_f = next_targetd.feed_rate;
      next_targetd.feed_rate = config.maximum_feedrate_x * 2;
      enqueue_moved (&next_targetd);
      next_targetd.feed_rate = backup_f;
      break;

      // G1 - synchronised motion
      case 1:
      if ( (extruders_on == EXTRUDER_NUM_1) && !next_target.seen_E)
      {
        // approximate translation for 3D code. distance to extrude is move distance times extruder speed factor
        //TODO: extrude distance for Z moves
        double d = calc_distance (ABS(next_targetd.x - startpoint.x), ABS(next_targetd.y - startpoint.y));
        
        next_targetd.e = startpoint.e + d * extruder_1_speed / next_targetd.feed_rate * 24.0;
      }
      enqueue_moved(&next_targetd);
      break;

      //	G2 - Arc Clockwise
      // unimplemented

      //	G3 - Arc Counter-clockwise
      // unimplemented

      //	G4 - Dwell
      case 4:
      // wait for all moves to complete
      synch_queue();
      
      // delay
      delay_ms(next_target.P);
      break;

      //	G20 - inches as units
      case 20:
      next_target.option_inches = 1;
      break;

      //	G21 - mm as units
      case 21:
      next_target.option_inches = 0;
      break;

      //	G30 - go home via point
      case 30:
      enqueue_moved(&next_targetd);
      // no break here, G30 is move and then go home

      //	G28 - go home
      case 28:
      
      if (next_target.seen_X)
      {
        zero_x();
        axisSelected = 1;
      }

      if (next_target.seen_Y)
      {
        zero_y();
        axisSelected = 1;
      }

      if (next_target.seen_Z)
      {
        zero_z();
        axisSelected = 1;
      }

      if (next_target.seen_E)
      {
        zero_e();
        axisSelected = 1;
      }

      if(!axisSelected)
      {
        // move down to clear Z endstop
        // Rapman only?
//        SpecialMoveZ (startpoint.Z + 3 * config.steps_per_mm_z, config.homing_feedrate_x);
          next_targetd = startpoint;
          next_targetd.z += 3;
          next_targetd.feed_rate = config.homing_feedrate_z;
          enqueue_moved(&next_targetd);
        
        zero_x();
        zero_y();
        zero_z();
        zero_e();
      }

//!      startpoint.F = config.homing_feedrate_x;  //?
      
#ifdef USE_GRBLX
      plan_set_current_position (
        (double)startpoint.X / config.steps_per_mm_x, 
        (double)startpoint.Y / config.steps_per_mm_y, 
        (double)startpoint.Z / config.steps_per_mm_z);
#endif
      break;

      // G90 - absolute positioning
      case 90:
      next_target.option_relative = 0;
      break;

      // G91 - relative positioning
      case 91:
      next_target.option_relative = 1;
      break;

      //	G92 - set home
      case 92:
      {
        tTarget new_pos;
        
      // must have no moves pending if changing position
      synch_queue();

      new_pos = startpoint;
      
      if (next_target.seen_X)
      {
        new_pos.x = 0;
        axisSelected = 1;
      }

      if (next_target.seen_Y)
      {
        new_pos.y = 0;
        axisSelected = 1;
      }

      if (next_target.seen_Z)
      {
        new_pos.z = 0;
        axisSelected = 1;
      }

      if (next_target.seen_E)
      {
        new_pos.e = 0;
        axisSelected = 1;
      }

      if(!axisSelected)
      {
          new_pos.x = 0;
          new_pos.y = 0;
          new_pos.z = 0;
          new_pos.e = 0;
      }
      
      plan_set_current_position (&new_pos);
      }
      break;

      // unknown gcode: spit an error
      default:
              serial_writestr("E: Bad G-code ");
              serwrite_uint8(next_target.G);
              serial_writestr("\r\n");
    }
  }
  else if (next_target.seen_M)
  {
    switch (next_target.M)
    {

      // SD File functions
      case 20: // M20 - list SD Card files
      serial_writestr("Begin file list\r\n");
      // list files in root folder
      sd_list_dir();
      serial_writestr("End file list\r\n");
      break;

      case 21: // M21 - init SD card
      sd_printing = false;
      sd_initialise();
      // NB : assume that the disk has been mounted in config.c
      // TODO: mount volume here and change config.c
      break;

      case 22: // M22 - release SD card
      sd_printing = false;
      sd_active = false;
      // TODO: should unmount volume
      break;
    
      case 23: // M23 <filename> - Select file
      if (!sd_active)
        sd_initialise();
      if(sd_active)
      {
        sd_printing = false;
        sd_close(&file);
        if (sd_open(&file, next_target.filename, FA_READ)) 
        {
          filesize = sd_filesize(&file);
          sersendf("File opened: %s Size: %d\r\n", next_target.filename, filesize);
          sd_pos = 0;
          sersendf("File selected\r\n");
        }
        else
        {
          sersendf("file.open failed\r\n");
        }
      }
      break;

      case 24: //M24 - Start SD print
      if(sd_active)
      {
        sd_printing = true;
      }
      break;

      case 25: //M25 - Pause SD print
      if(sd_printing)
      {
        sd_printing = false;
      }
      break;
    
      case 26: //M26 - Set SD file pos
      if(sd_active && next_target.seen_S)
      {
        sd_pos = next_target.S;  // 16 bit
        sd_seek(&file, sd_pos);
      }
      break;
    
      case 27: //M27 - Get SD status
      if(sd_active)
      {
        sersendf("SD printing byte %d/%d\r\n", sd_pos, filesize);
      }
      else
      {
    	  serial_writestr("Not SD printing\r\n");
      }
      break;
    
      case 28: //M28 <filename> - Start SD write
      if (!sd_active)
        sd_initialise();
      if(sd_active)
      {
        sd_close(&file);
        sd_printing = false;

        if (!sd_open(&file, next_target.filename, FA_CREATE_ALWAYS | FA_WRITE))
        {
          sersendf("open failed, File: %s.\r\n", next_target.filename);
        }
        else
        {
          sd_writing_file = true;
          sersendf("Writing to file: %s\r\n", next_target.filename);
        }
      }
      break;
    
      case 29: //M29 - Stop SD write
      // processed in gcode_parse_char()
      break;
    
      case 82: // M82 - use absolute distance for extrusion
      // no-op, we always do absolute
      break;
      
      // M101- extruder on
      case 101:
      extruders_on = EXTRUDER_NUM_1;
      if (auto_prime_steps != 0)
      {      
        SpecialMoveE ((double)auto_prime_steps / auto_prime_factor, auto_prime_feed_rate);
      }

      break;

      // M102- extruder reverse

      // M103- extruder off
      case 103:
      extruders_on = 0;
      if (auto_reverse_steps != 0)
      {      
        SpecialMoveE (-(double)auto_reverse_steps / auto_reverse_factor, auto_reverse_feed_rate);
      }
      break;

      // M104- set temperature
      case 104:
      if (config.enable_extruder_1)
      {
        temp_set(next_target.S, EXTRUDER_0);
      
        if (config.wait_on_temp)
          enqueue_wait_temp();
      }

      break;

      // M105- get temperature
      case 105:
      temp_print();
      reply_sent = true;
      break;

      // M106- fan on
      case 106:
      extruder_fan_on();
      break;
      
      // M107- fan off
      case 107:
      extruder_fan_off();
      break;

      // M108 - set extruder speed
      // S = RPM * 10
      case 108:
      if (next_target.seen_S)
      {
        extruder_1_speed = (double)next_target.S / 10.0;
      }
      break;

      // M109- set temp and wait
      case 109:
      if (config.enable_extruder_1)
      {
        temp_set(next_target.S, EXTRUDER_0);
        enqueue_wait_temp();
      }
      break;

      // M110- set line number
      case 110:
      next_target.N_expected = next_target.S - 1;
      break;

      // M111- set debug level
      case 111:
      //debug_flags = next_target.S;
      break;

      // M112- immediate stop
      case 112:
      disableHwTimer(0);
#ifndef USE_GRBL
      queue_flush();
#endif
      power_off();
      break;

      // M113- extruder PWM
      case 113:
      break;

      /* M114- report XYZE to host */
      case 114:
      // wait for queue to complete ???
      //for (;queue_empty() == 0;) {}

      if (next_target.option_inches)
      {

      }
      else
      {
        sersendf("ok C: X:%g Y:%g Z:%g E:%g\r\n", startpoint.x, startpoint.y, startpoint.z, startpoint.e);
      }
      reply_sent = true;
      break;
      
      // M115- report firmware version
      case 115:
        sersendf("FIRMWARE_NAME:Teacup_R2C2 FIRMWARE_URL:http%%3A//github.com/bitboxelectronics/R2C2 PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel\r\n");
      break;

      case 119:
      // M119 - Get Endstop Status
      #if (X_MIN_PIN > -1)
        serial_writestr ("x_min:");
        serial_writestr ( x_min() ? "H ":"L ");
      #endif
      #if (Y_MIN_PIN > -1)
        serial_writestr ("y_min:");
        serial_writestr ( y_min() ? "H ":"L ");
      #endif
      #if (Z_MIN_PIN > -1)
        serial_writestr ("z_min:");
        serial_writestr ( z_min() ? "H ":"L ");
      #endif
      serial_writestr ("\r\n");
      break;

      // M130- heater P factor
      case 130:
      //if (next_target.seen_S)
        //p_factor = next_target.S;
      break;
              // M131- heater I factor
      case 131:
        //if (next_target.seen_S)
              //i_factor = next_target.S;
      break;

      // M132- heater D factor
      case 132:
      //if (next_target.seen_S)
              //d_factor = next_target.S;
      break;

      // M133- heater I limit
      case 133:
      //if (next_target.seen_S)
              //i_limit = next_target.S;
      break;

      // M134- save PID settings to eeprom
      case 134:
      //heater_save_settings();
      break;

      /* M140 - Bed Temperature (Fast) */
      case 140:
      temp_set(next_target.S, HEATED_BED_0);
      break;

      /* M141 - Chamber Temperature (Fast) */
      case 141:
      break;

      /* M142 - Bed Holding Pressure */
      case 142:
      break;

      // M190- power on
      case 190:
      power_on();
      x_enable();
      y_enable();
      z_enable();
      e_enable();
      steptimeout = 0;
      break;

      // M191- power off
      case 191:
      x_disable();
      y_disable();
      z_disable();
      e_disable();
      power_off();
      break;

      // M200 - set steps per mm
      case 200:
      if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0)
      {
        reply_sent = true;
        sersendf ("ok X%g Y%g Z%g E%g\r\n", 
          config.steps_per_mm_x,
          config.steps_per_mm_y,
          config.steps_per_mm_z,
          config.steps_per_mm_e
          );
      }
      else
      {
        if (next_target.seen_X)
          config.steps_per_mm_x = next_target.target.x;
        if (next_target.seen_Y)
          config.steps_per_mm_y = next_target.target.y;
        if (next_target.seen_Z)
          config.steps_per_mm_z = next_target.target.z;
        if (next_target.seen_E)
          config.steps_per_mm_e = next_target.target.e;
          
        gcode_parse_init();  
#ifndef USE_GRBL        
        dda_init();
#endif      
      }
      break;
      
      // M202 - set max speed in mm/min
      case 202:
      if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0)
      {
        reply_sent = true;
        sersendf ("ok X%d Y%d Z%d E%d\r\n", 
          config.maximum_feedrate_x,
          config.maximum_feedrate_y,
          config.maximum_feedrate_z,
          config.maximum_feedrate_e
          );
      }
      else
      {
        if (next_target.seen_X)
          config.maximum_feedrate_x = next_target.target.x;
        if (next_target.seen_Y)
          config.maximum_feedrate_y = next_target.target.y;
        if (next_target.seen_Z)
          config.maximum_feedrate_z = next_target.target.z;
        if (next_target.seen_E)
          config.maximum_feedrate_e = next_target.target.e;
      }
      break;

      // M206 - set accel in mm/sec^2
      case 206:
      if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0)
      {
        reply_sent = true;
        sersendf ("ok X%g\r\n", 
          config.acceleration
          );
      }
      else
      {
        if (next_target.seen_X)
          config.acceleration = next_target.target.x;
      }
      break;
      
      // M227 - Enable Auto-prime/reverse (steps)
      // P: prime on start (steps)
      // S: reverse on stop (steps)
      case 227:
      if (next_target.seen_S && next_target.seen_P)
      {
        auto_prime_steps = next_target.S;
        auto_reverse_steps = next_target.P;
      }
      break;
      
      // M228 - Disable Auto-prime/reverse
      case 228:
      auto_prime_steps = 0;
      auto_reverse_steps = 0;
      break;
      
      // M229 - Enable Auto-prime/reverse
      // P: prime on start (rotations)
      // S: reverse on stop (rotations)
      case 229:
      if (next_target.seen_S && next_target.seen_P)
      {
        auto_prime_steps = next_target.S * config.steps_per_revolution_e;
        auto_reverse_steps = next_target.P * config.steps_per_revolution_e;
      }
      break;
      
      // M300 - beep
      // S: frequency
      // P: duration
      case 300:
      {
        uint16_t frequency = 1000;  // 1kHz
        uint16_t duration = 1000; // 1 second
        
        if (next_target.seen_S)
          frequency = next_target.S;
        if (next_target.seen_P)
          duration = next_target.P;

        buzzer_wait ();          
        buzzer_play (frequency, duration);
      }  
      break;
      
      // M500 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 500:
      if (next_target.seen_S && next_target.seen_P)
        temp_set_table_entry (EXTRUDER_0, next_target.S, next_target.P);
      else if (next_target.seen_S)
      {
        reply_sent = true;
        sersendf ("ok [%d] = %d\r\n", next_target.S, temp_get_table_entry (EXTRUDER_0, next_target.S));
      }
      else
        serial_writestr ("E: bad param\r\n");
      break;

      // M501 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 501:
      if (next_target.seen_S && next_target.seen_P)
        temp_set_table_entry (HEATED_BED_0, next_target.S, next_target.P);
      else if (next_target.seen_S)
      {
        reply_sent = true;
        sersendf ("ok [%d] = %d\r\n", next_target.S, temp_get_table_entry (HEATED_BED_0, next_target.S));
      }
      else
        serial_writestr ("E: bad param\r\n");
      break;

      // M542 - nozzle wipe/move to rest location
      case 542:
      // TODO: this depends on current origin being same as home position
      /* order should be:
        move to nozzle wipe entry
        move to rest
        [move to dump
        */
      if (config.have_rest_pos)
      {
        // move above bed if ncessary
        if (startpoint.z < 2)
        {
          // SpecialMoveZ (2 * config.steps_per_mm_z, config.maximum_feedrate_x);
          next_targetd = startpoint;
          next_targetd.z = 2;
          next_targetd.feed_rate = config.maximum_feedrate_z;
          enqueue_moved(&next_targetd);
        }
        
        // move to rest position
        next_targetd.x = config.rest_pos_x;
        next_targetd.y = config.rest_pos_y;
        next_targetd.z = startpoint.z;
        next_targetd.feed_rate = config.maximum_feedrate_x;
        
        enqueue_moved(&next_targetd);
      }
      break;

      // M543 - exit nozzle wipe/no op
      case 543:
        if (config.have_wipe_pos)
        {
          //TODO
        }
      break;

      // M551 - Prime extruder 1
      // P : number of steps
      // S : RPM * 10
      case 551:
      if (next_target.seen_S && next_target.seen_P)
      {
        // calc E distance, use approximate conversion to get distance, not critical
        // TODO: how to derive magic number
        // S is RPM*10, but happens to give about the right speed in mm/min        
#if 0
        next_targetd = startpoint;
        next_targetd.e = startpoint.e + (double)next_target.P / 256;
        next_targetd.feed_rate = next_target.S;
        enqueue_moved(&next_targetd);
#endif        
        SpecialMoveE ((double)next_target.P / 256.0, next_target.S);
      }
      break;
                
      // M600 print the values read from the config file                  
      case 600:
      {
        print_config();
      }
      break;
      
      // unknown mcode: spit an error
      default:
        serial_writestr("E: Bad M-code ");
        serwrite_uint8(next_target.M);
        serial_writestr("\r\n");
    }
  }

  if (!reply_sent)
  {
    serial_writestr("ok\r\n");
  }
  
  return result;
}
