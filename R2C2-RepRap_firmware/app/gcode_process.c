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

/*
  References:
  [1]  http://reprap.org/wiki/G-code
  [2]  "The NIST RS274NGC Interpreter - Version 3"

*/

#include   <string.h>
#include   <ctype.h>  // for tolower

#include "gcode_defs.h"
#include "gcode_process.h"
//#include "gcode_parse.h"
#include "lw_io.h"
#include "temp.h"
#include "timer.h"
#include "pin_control.h"
#include "app_config.h"
#include "ff.h"
#include "buzzer.h"
#include "soundplay.h"
//#include "debug.h"
#include "ios.h"
#include "planner.h"
#include "stepper.h"
#include "geometry.h"


extern tGcodeInputMsg file_input_msg;

extern void reboot (void);


bool      sd_active = false;        // SD card active
bool      sd_printing = false;      // printing from SD file
bool      sd_writing_file = false;  // writing to SD file
FIL       file;
uint8_t   file_mode;
uint32_t  filesize = 0;
uint32_t  sd_pos = 0;
char      sd_file_name [21];

#define EXTRUDER_NUM_0  1
#define EXTRUDER_NUM_1  2
#define EXTRUDER_NUM_2  4

uint8_t   extruders_on;
double    extruder_0_speed;         // in RPM

uint32_t  auto_prime_steps = 0;
uint32_t  auto_reverse_steps = 0;
const double auto_prime_feed_rate = 18000;
const double auto_reverse_feed_rate = 18000;
double auto_prime_factor = 640;
double auto_reverse_factor = 640;

#if 0
static char tolower (char c)
{
  if ((c >= 'A') && (c <= 'Z'))
    return 'a' + c-'A';
  else
    return c;
}

static char toupper (char c)
{
  if ((c >= 'a') && (c <= 'z'))
    return 'A' + c-'a';
  else
    return c;
}
#endif

static eParseResult enqueue_move (tTarget *pTarget)
{
  tActionRequest request;
  
  if (pTarget->x != startpoint.x || pTarget->y != startpoint.y ||
      pTarget->z != startpoint.z || pTarget->e != startpoint.e
     )
  {  
    request.ActionType = AT_MOVE;
    request.target = *pTarget;
    request.target.invert_feed_rate = false;
    
    if (config.enable_extruder_0 == 0)
      request.target.e = startpoint.e;

    if (plan_queue_full ())
      return PR_BUSY;
    else
    {
      plan_buffer_action (&request);
      return PR_OK;
    }
  }
  else
  {
    // no move, just set feed rate
    plan_set_feed_rate (pTarget);
    return PR_OK;
  }
}

static eParseResult enqueue_wait_for_temperatures (uint16_t param)
{
  tActionRequest request;
  
  request.ActionType = AT_WAIT_TEMPERATURES;
  
  //request.wait_param = _BV(WE_WAIT_TEMP_EXTRUDER_0) | _BV(WE_WAIT_TEMP_HEATED_BED);
  request.wait_param = param;
  
  if (plan_queue_full ())
    return PR_BUSY;
  else
  {
    plan_buffer_action (&request);
    return PR_OK;
  }
}

// wait time in milliseconds
static eParseResult enqueue_wait_time (uint16_t param)
{
  tActionRequest request;
  
  request.ActionType = AT_WAIT_TIME;
  request.wait_param = param;
  
  if (plan_queue_full ())
    return PR_BUSY;
  else
  {
    plan_buffer_action (&request);
    return PR_OK;
  }
}

// wait for move queue to be empty
static void synch_queue (void)
{
  st_synchronize();
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

// CAN BLOCK
static eParseResult RelativeMoveE (double e, double feed_rate) 
{
  tTarget next_target;
  
  if (config.enable_extruder_0)
  {
    next_target = startpoint;
    next_target.e = startpoint.e + e;
    next_target.feed_rate = feed_rate;
    return enqueue_move(&next_target);
  }
  else
    return PR_OK;
}

static void home_x(void)
{
  int dir;
  int max_travel;

  if (config.axis[X_AXIS].home_direction < 0)
    dir = -1;
  else
    dir = 1;
  max_travel = max (300, config.axis[X_AXIS].printing_vol);

  // move to endstop
  SpecialMoveXY(startpoint.x + dir * max_travel, startpoint.y, config.axis[X_AXIS].homing_feedrate);
//  synch_queue();
  
  // move forward a bit
  SpecialMoveXY(startpoint.x - dir * 3, startpoint.y, config.axis[X_AXIS].search_feedrate);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x + dir * 6, startpoint.y, config.axis[X_AXIS].search_feedrate);

//  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.x = config.axis[X_AXIS].home_pos;
  plan_set_current_position (&new_pos);
}

static void home_y(void)
{
  int dir;
  int max_travel;

  if (config.axis[Y_AXIS].home_direction < 0)
    dir = -1;
  else
    dir = 1;
  max_travel = max (300, config.axis[Y_AXIS].printing_vol);
    
  // move to endstop
  SpecialMoveXY(startpoint.x, startpoint.y + dir * max_travel, config.axis[Y_AXIS].homing_feedrate);
//  synch_queue();
  
  // move forward a bit
  SpecialMoveXY(startpoint.x, startpoint.y - dir * 3, config.axis[Y_AXIS].search_feedrate);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x, startpoint.y + dir * 6, config.axis[Y_AXIS].search_feedrate);

//  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.y = config.axis[Y_AXIS].home_pos;
  plan_set_current_position (&new_pos);
}

static void home_z(void)
{
  int dir;
  int max_travel;

  if (config.axis[Z_AXIS].home_direction < 0)
    dir = -1;
  else
    dir = 1;
  max_travel = max (300, config.axis[Z_AXIS].printing_vol);

  // move to endstop
  SpecialMoveZ(startpoint.z + dir * max_travel, config.axis[Z_AXIS].homing_feedrate);  
//  synch_queue();
  
  // move forward a bit
  SpecialMoveZ(startpoint.z - dir * 1, config.axis[Z_AXIS].search_feedrate);
//  synch_queue();

  // move back in to endstop slowly
  SpecialMoveZ(startpoint.z + dir * 6, config.axis[Z_AXIS].search_feedrate);
//  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.z = config.axis[Z_AXIS].home_pos;
  plan_set_current_position (&new_pos);
}

// --------------------------------------------------------------------------
// SD Card functions
// --------------------------------------------------------------------------

void sd_initialise(void)
{
  sd_active = true;
}

FRESULT sd_list_dir_sub (char *path, LW_FILE *out_file)
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
                lw_fprintf(out_file, "%s/%s/\r\n", path, fn);
                
                strcat (path, "/");
                strcat (path, fn);
                // sprintf(&path[i], "/%s", fn);
                res = sd_list_dir_sub(path, out_file);
                if (res != FR_OK) break;
                path[i] = 0;
            } else 
            {
                lw_fprintf(out_file, "%s/%s\r\n", path, fn);
            }
        }
    }

    return res;    
}

void sd_list_dir (LW_FILE *out_file)
{
  char path[120];
    
  strcpy (path, "");
    
  sd_list_dir_sub(path, out_file);
}


// --------------------------------------------------------------------------
// SD file functions
// --------------------------------------------------------------------------

unsigned sd_open(FIL *pFile, char *path, uint8_t flags, LW_FILE *out_file)
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
    lw_fprintf (out_file, "sd_open:%d\r\n", res);
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
// --------------------------------------------------------------------------
// 
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief       Command Received - process it
//! @param[in]
//! @param[out]
//! @return      PR_OK, PR_ERROR or PR_BUSY
// --------------------------------------------------------------------------

eParseResult process_gcode_command (tGcodeInputMsg *pGcodeInputMsg, tGcodeInterpreterState *pInterpreterState)
{
  double backup_f;
  int moves_required = 0;
  bool reply_sent = false;
  uint8_t axisSelected = 0;
  eParseResult result = PR_OK;
  
  tTarget next_target = startpoint;

  // convert relative to absolute
  if (pInterpreterState->option_relative)
  { 
    if (gcode_command.seen_X)
      next_target.x = startpoint.x + gcode_command.target.x;
    if (gcode_command.seen_Y)
      next_target.y = startpoint.y + gcode_command.target.y;
    if (gcode_command.seen_Z)
      next_target.z = startpoint.z + gcode_command.target.z;
    if (gcode_command.seen_E)
      next_target.e = startpoint.e + gcode_command.target.e;
    if (gcode_command.seen_F)
      next_target.feed_rate = gcode_command.target.feed_rate;
  }
  else
  {
    // absolute
    if (gcode_command.seen_X)
      next_target.x = gcode_command.target.x;
    if (gcode_command.seen_Y)
      next_target.y = gcode_command.target.y;
    if (gcode_command.seen_Z)
      next_target.z = gcode_command.target.z;
    if (gcode_command.seen_E)
      next_target.e = gcode_command.target.e;  
    if (gcode_command.seen_F)
      next_target.feed_rate = gcode_command.target.feed_rate;
  }

//  lw_fprintf(pGcodeInputMsg->out_file, " X:%ld Y:%ld Z:%ld E:%ld F:%ld\r\n", (int32_t)gcode_command.target.X, (int32_t)gcode_command.target.Y, (int32_t)gcode_command.target.Z, (int32_t)gcode_command.target.E, (uint32_t)gcode_command.target.F);
//  lw_fprintf(pGcodeInputMsg->out_file, " X:%g Y:%g Z:%g E:%g F:%g\r\n", next_target.x, next_target.y, next_target.z, next_target.e, next_target.feed_rate);
    
  // E ALWAYS absolute 
	
  if (gcode_command.seen_G)
  {
    switch (gcode_command.G)
    {
      // G0 - rapid, unsynchronised motion
      // since it would be a major hassle to force the stepper to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
      // CAN BLOCK
      case 0:
        backup_f = next_target.feed_rate;
        next_target.feed_rate = config.axis[X_AXIS].maximum_feedrate * 2; // actual feedrate will be limited by planner
        result = enqueue_move (&next_target);
        next_target.feed_rate = backup_f;
      break;

      // G1 - synchronised motion
      // CAN BLOCK
      case 1:
        if ( (extruders_on == EXTRUDER_NUM_0) && !gcode_command.seen_E)
        {
          // approximate translation for 3D code. distance to extrude is move distance times extruder speed factor
          //TODO: extrude distance for Z moves
          double d = calc_distance (ABS(next_target.x - startpoint.x), ABS(next_target.y - startpoint.y));
        
          next_target.e = startpoint.e + d * extruder_0_speed / next_target.feed_rate * 24.0;
        }
        result = enqueue_move(&next_target);
      break;

      //	G2 - Arc Clockwise
      // unimplemented

      //	G3 - Arc Counter-clockwise
      // unimplemented

      //	G4 - Dwell, P = milliseconds to pause
      // CAN BLOCK
      case 4:
        result = enqueue_wait_time (gcode_command.P);
      break;

      //	G20 - inches as units
      case 20:
        pInterpreterState->option_inches = 1;
      break;

      //	G21 - mm as units
      case 21:
        pInterpreterState->option_inches = 0;
      break;

      //	G30 - go home via point
      // CAN BLOCK
      case 30:
        //TODO: this and following command generates several moves which could block

//        result = enqueue_move(&next_target);
        moves_required = 1;
        // no break here, G30 is move and then go home
        // *** fall through ***

      //	G28 - go home
      // CAN BLOCK
      case 28:
      {
      
        // work out number of moves required
        if (gcode_command.seen_X)
          moves_required += 3;
        if (gcode_command.seen_Y)
          moves_required += 3;
        if (gcode_command.seen_Z)
          moves_required += 3;

        if (moves_required <= 1)
        {
          moves_required += 9;
          if (config.machine_model == MM_RAPMAN)
            moves_required ++;
        }

        if (plan_num_free_slots() < moves_required)
        {
          result = PR_BUSY;
          break;
        }

        // now perform moves
        if (gcode_command.M == 30)
        {
          result = enqueue_move(&next_target);
        }

        if (gcode_command.seen_X)
        {
          home_x();
          axisSelected = 1;
        }

        if (gcode_command.seen_Y)
        {
          home_y();
          axisSelected = 1;
        }

        if (gcode_command.seen_Z)
        {
          home_z();
          axisSelected = 1;
        }

        if(!axisSelected)
        {
          if (config.machine_model == MM_RAPMAN)
          {
            // move stage down to clear Z endstop
            // Rapman only?
            next_target = startpoint;
            next_target.z += 3;
            next_target.feed_rate = config.axis[Z_AXIS].homing_feedrate;
            enqueue_move(&next_target);
          }
                
          home_x();
          home_y();
          home_z();
        }

      }
      break;

      // G90 - absolute positioning
      case 90:
        pInterpreterState->option_relative = 0;
      break;

      // G91 - relative positioning
      case 91:
        pInterpreterState->option_relative = 1;
      break;

      //	G92 - set home
      // !CAN BLOCK
      case 92:
      {
        tTarget new_pos;
        
        // must have no moves pending if changing position??
        // synch_queue();

        new_pos = startpoint;
      
        if (gcode_command.seen_X)
        {
          new_pos.x = gcode_command.target.x;
          axisSelected = 1;
        }

        if (gcode_command.seen_Y)
        {
          new_pos.y = gcode_command.target.y;
          axisSelected = 1;
        }

        if (gcode_command.seen_Z)
        {
          new_pos.z = gcode_command.target.z;
          axisSelected = 1;
        }

        if (gcode_command.seen_E)
        {
          new_pos.e = gcode_command.target.e;
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
        lw_fprintf (pGcodeInputMsg->out_file, "E: Bad G-code %d\r\n", gcode_command.G);
        result = PR_ERROR;
    }
  }
  else if (gcode_command.seen_M)
  {
    switch (gcode_command.M)
    {
      // M0 - Stop? / pause-feedhold 
      case 0: 
      break;

      // M1 - SLeep  / pause-feedhold optional
      case 1: 
      break;

      // M2 - Program end
      case 2: 
      break;

      // M17 - Enable/power on steppers motors
      case 17: 
        enable_all_axes();
      break;

      // M18 - Disable all stepper motors
      case 18: 
        disable_all_axes();
      break;

      // SD File functions
      case 20: // M20 - list SD Card files
      lw_fputs("Begin file list\r\n", pGcodeInputMsg->out_file);
      // list files in root folder
      sd_list_dir(pGcodeInputMsg->out_file);
      lw_fputs("End file list\r\n", pGcodeInputMsg->out_file);
      break;

      case 21: // M21 - init SD card
      // NB : assume that the disk has been mounted by app init
      sd_printing = false;
      sd_initialise();
      break;

      case 22: // M22 - release SD card
      sd_printing = false;
      sd_active = false;
      // TODO: should unmount volume
      break;
    
      case 23: // M23 <filename> - Select and open file
      if (!sd_active)
      {
        sd_initialise();
      }

      if (sd_active)
      {
        sd_printing = false;
        sd_close(&file);
        if (sd_open(&file, gcode_command.str_param, FA_READ, pGcodeInputMsg->out_file)) 
        {
          file_input_msg.out_file = pGcodeInputMsg->out_file;
          filesize = sd_filesize(&file);
          sd_pos = 0;
          strncpy (sd_file_name, gcode_command.str_param, sizeof(sd_file_name)-1);
          lw_fprintf(pGcodeInputMsg->out_file, "File opened: %s Size: %d\r\n", gcode_command.str_param, filesize);
          lw_fprintf(pGcodeInputMsg->out_file, "File selected\r\n");
        }
        else
        {
          lw_fprintf(pGcodeInputMsg->out_file, "E: file.open failed\r\n");
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
      if(sd_active && gcode_command.seen_S)
      {
        sd_pos = gcode_command.S;  // 16 bit
        sd_seek(&file, sd_pos);
      }
      break;
    
      case 27: //M27 - Get SD status
      if(sd_active)
      {
        lw_fprintf(pGcodeInputMsg->out_file, "SD printing byte %d/%d\r\n", sd_pos, filesize);
      }
      else
      {
    	  lw_fputs("Not SD printing\r\n", pGcodeInputMsg->out_file);
      }
      break;
    
      case 28: //M28 <filename> - Start SD write
      if (!sd_active)
        sd_initialise();

      if(sd_active)
      {
        sd_close(&file);
        sd_printing = false;

        if (!sd_open(&file, gcode_command.str_param, FA_CREATE_ALWAYS | FA_WRITE, pGcodeInputMsg->out_file))
        {
          lw_fprintf(pGcodeInputMsg->out_file, "E: open failed, File: %s.\r\n", gcode_command.str_param);
        }
        else
        {
          if (gcode_command.seen_S && (gcode_command.S == 1))
            file_mode = FM_HEX_BIN;
          else
            file_mode = FM_GCODE;

          sd_writing_file = true;
          lw_fprintf(pGcodeInputMsg->out_file, "Writing to file: %s\r\n", gcode_command.str_param);
        }
      }
      break;
    
      case 29: //M29 - Stop SD write
      // processed in gcode_parse_line()
      break;
    
      case 30: //M30 <data> - write raw data to SD file
      // processed in gcode_parse_line()
      break;

      case 80: // M80 - ATX Power On 
        // Bring ATX PSU out of standby
        // Enable motor/heater PSU (e.g. 12V/24V supply)
        atx_power_on();
      break;

      case 81: // M81 - ATX Power Off 
        // Put ATX PSU into standby, board runs off +5V standby
        // Disable motor/heater PSU (e.g. 12V/24V supply)
        atx_power_off();
      break;

      case 82: // M82 - use absolute distance for extrusion
      // no-op, we always do absolute
      break;

      case 83: // M83 - use relative distance for extrusion
      // no-op, we always do absolute
      break;

      case 84: // M84 - disable idle hold / enable idle timeout
      // when printer is idle, disable stepper motors to reduce nuisance noises
      // default: idle hold is on (no idle timeout)
      
      break;

      // M101- extruder on
      // CAN BLOCK
      case 101:
      extruders_on = EXTRUDER_NUM_0;
      if (auto_prime_steps != 0)
      {      
        result = RelativeMoveE ((double)auto_prime_steps / auto_prime_factor, auto_prime_feed_rate);
      }

      break;

      // M102- extruder reverse

      // M103- extruder off
      // CAN BLOCK
      case 103:
      extruders_on = 0;
      if (auto_reverse_steps != 0)
      {      
        result = RelativeMoveE (-(double)auto_reverse_steps / auto_reverse_factor, auto_reverse_feed_rate);
      }
      break;

      // M104- set temperature (fast)
      // CAN BLOCK
      case 104:
      if (config.enable_extruder_0)
      {
        if (config.wait_on_temp)
        {
          if (plan_queue_full())
          {
            result = PR_BUSY;
          }
          else
          {
            temp_set(gcode_command.S, EXTRUDER_0);
            result = enqueue_wait_for_temperatures( _BV(WE_WAIT_TEMP_EXTRUDER_0) );
          }
        }
        else
        {
          temp_set(gcode_command.S, EXTRUDER_0);
        }
      }
      break;

      // M105- get temperature
      case 105:
      {        
        uint16_t extruder0_temp;
        uint16_t heated_bed_temp;

        extruder0_temp = temp_get (EXTRUDER_0);
        heated_bed_temp = temp_get (HEATED_BED_0);

        lw_fprintf (pGcodeInputMsg->out_file, "ok T:%u.0 B:%u.0\r\n", extruder0_temp, heated_bed_temp); /* for RepRap software */
        reply_sent = true;
      }
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
      if (gcode_command.seen_S)
      {
        extruder_0_speed = (double)gcode_command.S / 10.0;
      }
      break;

      // M109- set temp and wait
      // CAN BLOCK
      case 109:
      if (config.enable_extruder_0)
      {
        temp_set(gcode_command.S, EXTRUDER_0);
        result = enqueue_wait_for_temperatures(_BV(WE_WAIT_TEMP_EXTRUDER_0));
      }
      break;

      // M110- set line number
      // G-Code spec [1] has parameter N, but that would be silly
      case 110:
        // set to S-1, 1 is added later to get correct value
        // Note: if S not seen then N_expected = -1 (underflows unsigned)
        pInterpreterState->N_expected = gcode_command.S - 1;
      break;

      // M111- set debug level
      case 111:
        if (gcode_command.seen_S)
          config.debug_flags = gcode_command.S;
      break;

      // M112- emergency/immediate stop
      case 112:
        disableHwTimer(0); // disable stepper ?
        //TODO: queue_flush();
        atx_power_off();
      break;

      // M113- extruder PWM
      case 113:
      break;

      /* M114- report XYZE to host */
      case 114:
      // wait for queue to complete ???

      if (pInterpreterState->option_inches)
      {

      }
      else
      {
        lw_fprintf(pGcodeInputMsg->out_file, "ok C: X:%g Y:%g Z:%g E:%g\r\n", startpoint.x, startpoint.y, startpoint.z, startpoint.e);
      }
      reply_sent = true;
      break;
      
      // M115- report firmware version
      case 115:
        lw_fprintf(pGcodeInputMsg->out_file, "FIRMWARE_NAME:Teacup_R2C2 FIRMWARE_URL:http%%3A//github.com/bitboxelectronics/R2C2_Firmware " 
                "PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel FEATURES:0/R2C2_BOOTLOAD\r\n");
      break;

      // M116 - Wait for all temperatures and other slowly-changing variables to arrive at their set values
      case 116:
          result = enqueue_wait_for_temperatures( _BV(WE_WAIT_TEMP_EXTRUDER_0) | _BV(WE_WAIT_TEMP_EXTRUDER_1) | _BV(WE_WAIT_TEMP_HEATED_BED));

      break;

      // M118 - select features
      case 118:

      break;

      // M119 - Get Endstop Status
      case 119:
      {
        int axis;
        char buf [10];
        
        for (axis = 0; axis < config.num_axes; axis++)
        {
          if (config.axis [axis].is_configured)
          {
            // min limit
            if (config.axis[axis].pin_min_limit.port != 0xFF)
            {
              strcpy (buf, "?_min: ? ");
              buf [0] = tolower (config.axis[axis].letter_code);
              
              if (read_pin (config.axis[axis].pin_min_limit))
                buf [7] = 'H';
              else
                buf [7] = 'L';
              
              lw_fputs (buf, pGcodeInputMsg->out_file);
            }
            // max limit?
            if (config.axis[axis].pin_max_limit.port != 0xFF)
            {
              strcpy (buf, "?_max: ? ");
              buf [0] = tolower (config.axis[axis].letter_code);
              
              if (read_pin (config.axis[axis].pin_max_limit))
                buf [7] = 'H';
              else
                buf [7] = 'L';
              
              lw_fputs (buf, pGcodeInputMsg->out_file);
            }
          }
        }
            
        lw_fputs ("\r\n", pGcodeInputMsg->out_file);
      }
      break;

      // M130- heater P factor
      case 130:
      //if (gcode_command.seen_S)
        //p_factor = gcode_command.S;
      break;
      
      // M131- heater I factor
      case 131:
        //if (gcode_command.seen_S)
              //i_factor = gcode_command.S;
      break;

      // M132- heater D factor
      case 132:
      //if (gcode_command.seen_S)
              //d_factor = gcode_command.S;
      break;

      // M133- heater I limit
      case 133:
      //if (gcode_command.seen_S)
              //i_limit = gcode_command.S;
      break;

      // M134- save PID settings to eeprom
      case 134:
      //heater_save_settings();
      break;

      /* M140 - Bed Temperature (Fast) */
      case 140:
        temp_set(gcode_command.S, HEATED_BED_0);
      break;

      /* M141 - Chamber Temperature (Fast) */
      case 141:
      break;

      /* M142 - Bed Holding Pressure */
      case 142:
      break;

      // M190: Wait for bed temperature to reach target temp 
      case 190:
        temp_set(gcode_command.S, HEATED_BED_0);
        result = enqueue_wait_for_temperatures(_BV(WE_WAIT_TEMP_HEATED_BED));
      break;


      // very similar to M80, could be automatic
      // M190- power on
//      case 190:
//      {
//        atx_power_on();
//        enable_all_axes();
//        steptimeout = 0;
//      }
//      break;

      // M191- power off
      // same as M2?
      case 191:
      {
        disable_all_axes();
        atx_power_off();
      }
      break;

      // M200 - set steps per mm
      case 200:
      if ((gcode_command.seen_X | gcode_command.seen_Y | gcode_command.seen_Z | gcode_command.seen_E) == 0)
      {
        reply_sent = true;
        lw_fprintf (pGcodeInputMsg->out_file, "ok X%g Y%g Z%g E%g\r\n", 
          config.axis[X_AXIS].steps_per_mm,
          config.axis[Y_AXIS].steps_per_mm,
          config.axis[Z_AXIS].steps_per_mm,
          config.axis[E_AXIS].steps_per_mm
          );
      }
      else
      {
        if (gcode_command.seen_X)
          config.axis[X_AXIS].steps_per_mm = gcode_command.target.x;
        if (gcode_command.seen_Y)
          config.axis[Y_AXIS].steps_per_mm = gcode_command.target.y;
        if (gcode_command.seen_Z)
          config.axis[Z_AXIS].steps_per_mm = gcode_command.target.z;
        if (gcode_command.seen_E)
          config.axis[E_AXIS].steps_per_mm = gcode_command.target.e;
          
        gcode_parse_init();  
      }
      break;
      
      // M202 - set max speed in mm/min
      case 202:
      if ((gcode_command.seen_X | gcode_command.seen_Y | gcode_command.seen_Z | gcode_command.seen_E) == 0)
      {
        reply_sent = true;
        lw_fprintf (pGcodeInputMsg->out_file, "ok X%d Y%d Z%d E%d\r\n", 
          config.axis[X_AXIS].maximum_feedrate,
          config.axis[Y_AXIS].maximum_feedrate,
          config.axis[Z_AXIS].maximum_feedrate,
          config.axis[E_AXIS].maximum_feedrate
          );
      }
      else
      {
        if (gcode_command.seen_X)
          config.axis[X_AXIS].maximum_feedrate = gcode_command.target.x;
        if (gcode_command.seen_Y)
          config.axis[Y_AXIS].maximum_feedrate = gcode_command.target.y;
        if (gcode_command.seen_Z)
          config.axis[Z_AXIS].maximum_feedrate = gcode_command.target.z;
        if (gcode_command.seen_E)
          config.axis[E_AXIS].maximum_feedrate = gcode_command.target.e;
      }
      break;

      // M206 - set accel in mm/sec^2
      case 206:
      if ((gcode_command.seen_X | gcode_command.seen_Y | gcode_command.seen_Z | gcode_command.seen_E) == 0)
      {
        // TODO: per axis accel
        reply_sent = true;
        lw_fprintf (pGcodeInputMsg->out_file, "ok X%g\r\n", 
          config.acceleration
          );
      }
      else
      {
        // not sure this is the right way to used axis words
        if (gcode_command.seen_X)
          config.axis[X_AXIS].acceleration = gcode_command.target.x;
          
        if (gcode_command.seen_Y)
          config.axis[Y_AXIS].acceleration = gcode_command.target.y;
          
        if (gcode_command.seen_Z)
          config.axis[Z_AXIS].acceleration = gcode_command.target.z;
          
        if (gcode_command.seen_E)
          config.axis[E_AXIS].acceleration = gcode_command.target.e;
      }
      break;
      
      // M227 - Enable Auto-prime/reverse (steps)
      // P: prime on start (steps)
      // S: reverse on stop (steps)
      case 227:
      if (gcode_command.seen_S && gcode_command.seen_P)
      {
        auto_prime_steps = gcode_command.S;
        auto_reverse_steps = gcode_command.P;
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
      if (gcode_command.seen_S && gcode_command.seen_P)
      {
        auto_prime_steps = gcode_command.S * config.steps_per_revolution_e;
        auto_reverse_steps = gcode_command.P * config.steps_per_revolution_e;
      }
      break;
      
      // M230: Disable / Enable Wait for Temperature Change
      // S0 Disable wait for temperature change, S1 Enable wait for temperature change 
      case 230:
      {
        if (gcode_command.seen_S)
        {
          if (gcode_command.S == 0)
            config.wait_on_temp = 0;
          else if (gcode_command.S == 1)
            config.wait_on_temp = 1;
        }
      }
      break;

      // M300 - beep
      // S: frequency
      // P: duration
      // CAN BLOCK
      case 300:
      {
        float frequency = 1000;  // 1kHz
        float duration = 1000; // 1 second
        
        if (gcode_command.seen_S)
          frequency = gcode_command.S;
        if (gcode_command.seen_P)
          duration = gcode_command.P;

        buzzer_wait ();          
        buzzer_play (frequency, duration);		
      }  
      break;
      
      // Plays Jingle Bell from Static Library
      case 301:
      {
         play_jingle_bell();
      }
      break;

      // Plays Music from command line:
      // Usage:
      //   M302 "music" P"tempo"
      // Example:
      //   M302 D4b3a3g3 P600
      //
      case 302:
      {
        if (gcode_command.seen_P) {
          set_whole_note_time(gcode_command.P);
        }
        play_music_string(gcode_command.str_param);    
      }
      break;
    
      case 401:
      // reset
      reboot();
      break;

      // M500 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 500:
      if (gcode_command.seen_S && gcode_command.seen_P)
        temp_set_table_entry (EXTRUDER_0, gcode_command.S, gcode_command.P);
      else if (gcode_command.seen_S)
      {
        reply_sent = true;
        lw_fprintf (pGcodeInputMsg->out_file, "ok [%d] = %d\r\n", gcode_command.S, temp_get_table_entry (EXTRUDER_0, gcode_command.S));
      }
      else
        lw_fputs ("E: bad param\r\n", pGcodeInputMsg->out_file);
      break;

      // M501 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 501:
        if (gcode_command.seen_S && gcode_command.seen_P)
          temp_set_table_entry (HEATED_BED_0, gcode_command.S, gcode_command.P);
        else if (gcode_command.seen_S)
        {
          reply_sent = true;
          lw_fprintf (pGcodeInputMsg->out_file, "ok [%d] = %d\r\n", gcode_command.S, temp_get_table_entry (HEATED_BED_0, gcode_command.S));
        }
        else
          lw_fputs ("E: bad param\r\n", pGcodeInputMsg->out_file);
      break;

      // M542 - nozzle wipe/move to rest location
      // CAN BLOCK
      case 542:
      // TODO: this depends on current origin being same as home position
      if (config.have_rest_pos || config.have_wipe_pos)
      {
        moves_required = 2;

        if (plan_num_free_slots() < moves_required)
        {
          result = PR_BUSY;
        }
        else
        {
          // move above bed if necessary
          if (startpoint.z < 2)
          {
            next_target = startpoint;
            next_target.z = 2;
            next_target.feed_rate = config.axis[Z_AXIS].maximum_feedrate;
            enqueue_move(&next_target);
          }
        
          if (config.have_wipe_pos)
          {
            // move to start of wipe area
            next_target.x = config.wipe_entry_pos_x;
            next_target.y = config.wipe_entry_pos_y;
            next_target.z = startpoint.z;
            next_target.feed_rate = config.axis[X_AXIS].maximum_feedrate;
          }
          else
          {
            // move to rest position
            next_target.x = config.rest_pos_x;
            next_target.y = config.rest_pos_y;
            next_target.z = startpoint.z;
            next_target.feed_rate = config.axis[X_AXIS].maximum_feedrate;
          }
        
          enqueue_move(&next_target);
        }
      }
      break;

      // M543 - exit nozzle wipe/no op
      // CAN BLOCK
      case 543:
        if (config.have_wipe_pos)
        {
          if (plan_num_free_slots() < 2)
          {
            result = PR_BUSY;
            break;
          }

          // move out of wipe area
          next_target.x = config.wipe_exit_pos_x;
          next_target.y = config.wipe_exit_pos_y;
          next_target.z = startpoint.z;
          next_target.feed_rate = config.axis[X_AXIS].maximum_feedrate;
          enqueue_move(&next_target);
          
          next_target.x = config.wipe_entry_pos_x;
          next_target.y = config.wipe_entry_pos_y;
          next_target.z = startpoint.z;
          next_target.feed_rate = config.axis[X_AXIS].maximum_feedrate;
          enqueue_move(&next_target);
          
        }
      break;

      // M551 - Prime extruder 1
      // P : number of steps
      // S : RPM * 10
      // CAN BLOCK
      case 551:
      if (gcode_command.seen_S && gcode_command.seen_P)
      {
        // calc E distance, use approximate conversion to get distance, not critical
        // TODO: how to derive magic number
        // S is RPM*10, but happens to give about the right speed in mm/min        
        result = RelativeMoveE ((double)gcode_command.P / 256.0, gcode_command.S);
      }
      break;
                
      // M600 print the values read from the config file                  
      // CAN BLOCK?
      case 600:
      {
        app_config_print();
      }
      break;
      
      // unknown mcode: spit an error
      default:
        lw_fprintf (pGcodeInputMsg->out_file, "E: Bad M-code %d\r\n", gcode_command.M);
        result = PR_ERROR;
    }
  }

  if ( (result == PR_OK) || (result == PR_ERROR) )
  {
    if (!reply_sent)
      lw_fputs("ok\r\n", pGcodeInputMsg->out_file);

    //lw_fprintf(pGcodeInputMsg->out_file, "ok Q:%d\r\n", plan_queue_size());
  }
  
  return result;
}
