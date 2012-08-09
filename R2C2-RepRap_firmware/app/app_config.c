/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
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

#include <string.h>

#include "spi.h"
#include "ff.h"
#include "ios.h"
#include "gcode_parse.h"
#include "debug.h"    // may not be initialised yet?
#include "pinout.h"
#include "stepper.h"
#include "config.h"
#include "app_config.h"

/* structure reflecting the configuration of the system
 */
tApplicationConfiguration config;



#define NUM_TOKENS(table) (sizeof(table)/sizeof(tConfigItem))

// lookup table for general/application level configuration
// the lookup table associates keywords with config items in RAM
static const tConfigItem config_lookup [] = 
{
  //  
  // general config
  //
  
  { "machine_model",        &config.machine_model, TYPE_INT, {.val_i=0}},
  { "acceleration",         &config.acceleration, TYPE_DOUBLE, {.val_d=100.0}},         /* 100mm / second^2 */
  { "junction_deviation",   &config.junction_deviation, TYPE_DOUBLE, {.val_d=0.05}},  
  
  { "auto_power_off_time",  &config.auto_power_off_time, TYPE_INT, {.val_i = 0}},
  
  //  
  // interfaces
  //
  { "debug_flags",          &config.debug_flags, TYPE_INT, {.val_i=0}},
  { "step_led_flash_method", &config.step_led_flash_method, TYPE_INT, {.val_i = STEP_LED_FLASH_VARIABLE}},
  { "beep_on_events",       &config.beep_on_events, TYPE_INT, {.val_i=0x0000000F}},

  { "control_panel",        &config.interface_control_panel_enabled, TYPE_INT, {.val_i = 0}},
  
  { "cp_lcd_type",        &config.interface_cp_lcd_type, TYPE_INT, {.val_i = 1}},
  { "cp_lcd_rows",        &config.interface_cp_lcd_rows, TYPE_INT, {.val_i = 4}},
  { "cp_lcd_cols",        &config.interface_cp_lcd_cols, TYPE_INT, {.val_i = 20}},

  { "tcp_ip_enabled",       &config.interface_tcp_ip_enabled, TYPE_INT, {.val_i = 0}},
  { "network_interface",    &config.interface_tcp_ip_phy_type, TYPE_INT, {.val_i = 0}},

  //  
  // axis config
  //
    
  { "steps_per_mm_x", &config.axis[X_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=80}},
  { "steps_per_mm_y", &config.axis[Y_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=80}},
  { "steps_per_mm_z", &config.axis[Z_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=6400}},
  { "steps_per_mm_e", &config.axis[E_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d=36}},    /* Wades extruder, NEMA 17 geared extruder (1/39 * 6.5mm) */

  /* used for G0 rapid moves and as a cap for all other feedrates */
  { "maximum_feedrate_x", &config.axis[X_AXIS].maximum_feedrate, TYPE_INT, {.val_i=3000}}, /* 50mm / second */
  { "maximum_feedrate_y", &config.axis[Y_AXIS].maximum_feedrate, TYPE_INT, {.val_i=3000}},
  { "maximum_feedrate_z", &config.axis[Z_AXIS].maximum_feedrate, TYPE_INT, {.val_i=60}},   /* 1mm / second */
  { "maximum_feedrate_e", &config.axis[E_AXIS].maximum_feedrate, TYPE_INT, {.val_i=3000}}, /* 50mm / second */

  // if axis acceleration is 0, general default acceleration will be used
  { "x.acceleration", &config.axis[X_AXIS].acceleration, TYPE_INT, {.val_i=0}},
  { "y.acceleration", &config.axis[Y_AXIS].acceleration, TYPE_INT, {.val_i=0}},
  { "z.acceleration", &config.axis[Z_AXIS].acceleration, TYPE_INT, {.val_i=0}},
  { "e.acceleration", &config.axis[E_AXIS].acceleration, TYPE_INT, {.val_i=0}},

  { "x.dir.invert", &config.axis[X_AXIS].dir_invert, TYPE_INT, {.val_i=0}},
  { "y.dir.invert", &config.axis[Y_AXIS].dir_invert, TYPE_INT, {.val_i=0}},
  { "z.dir.invert", &config.axis[Z_AXIS].dir_invert, TYPE_INT, {.val_i=0}},

  /* used when searching endstops and similar */
  { "search_feedrate_x", &config.axis[X_AXIS].search_feedrate, TYPE_INT, {.val_i=120}},
  { "search_feedrate_y", &config.axis[Y_AXIS].search_feedrate, TYPE_INT, {.val_i=120}},
  { "search_feedrate_z", &config.axis[Z_AXIS].search_feedrate, TYPE_INT, {.val_i=60}},
  { "search_feedrate_e", &config.axis[E_AXIS].search_feedrate, TYPE_INT, {.val_i=1600}},  // does E have endstop??
  
  { "homing_feedrate_x", &config.axis[X_AXIS].homing_feedrate, TYPE_INT, {.val_i=3000}},
  { "homing_feedrate_y", &config.axis[Y_AXIS].homing_feedrate, TYPE_INT, {.val_i=3000}},
  { "homing_feedrate_z", &config.axis[Z_AXIS].homing_feedrate, TYPE_INT, {.val_i=60}},
  
  // home pos is left front
  { "home_direction_x", &config.axis[X_AXIS].home_direction, TYPE_INT, {.val_i=-1}}, 
  { "home_direction_y", &config.axis[Y_AXIS].home_direction, TYPE_INT, {.val_i=-1}},
  { "home_direction_z", &config.axis[Z_AXIS].home_direction, TYPE_INT, {.val_i=-1}},
  
  { "home_pos_x", &config.axis[X_AXIS].home_pos, TYPE_INT, {.val_i=0}},
  { "home_pos_y", &config.axis[Y_AXIS].home_pos, TYPE_INT, {.val_i=0}},
  { "home_pos_z", &config.axis[Z_AXIS].home_pos, TYPE_INT, {.val_i=0}},

  { "printing_vol_x", &config.axis[X_AXIS].printing_vol , TYPE_INT, {.val_i=0}},
  { "printing_vol_y", &config.axis[Y_AXIS].printing_vol , TYPE_INT, {.val_i=0}},
  { "printing_vol_z", &config.axis[Z_AXIS].printing_vol , TYPE_INT, {.val_i=0}},
  
  //  
  // config for printers
  //

  // dump pos
  { "have_dump_pos", &config.have_dump_pos , TYPE_INT, {.val_i=0}},
  { "dump_pos_x", &config.dump_pos_x , TYPE_INT, {.val_i=0}},
  { "dump_pos_y", &config.dump_pos_x , TYPE_INT, {.val_i=0}},
  
  // rest pos
  { "have_rest_pos", &config.have_rest_pos , TYPE_INT, {.val_i=0}},
  { "rest_pos_x", &config.rest_pos_x , TYPE_INT, {.val_i=0}},
  { "rest_pos_y", &config.rest_pos_y , TYPE_INT, {.val_i=0}},

  // wipe pos
  { "have_wipe_pos",   &config.have_wipe_pos , TYPE_INT, {.val_i=0}},
  { "wipe_entry_pos_x", &config.wipe_entry_pos_x , TYPE_INT, {.val_i=0}},
  { "wipe_entry_pos_y", &config.wipe_entry_pos_y , TYPE_INT, {.val_i=0}},
  { "wipe_pos_x", &config.wipe_entry_pos_x , TYPE_INT, {.val_i=0}},     // DEPRECATED
  { "wipe_pos_y", &config.wipe_entry_pos_y , TYPE_INT, {.val_i=0}},     // DEPRECATED
  { "wipe_exit_pos_x", &config.wipe_exit_pos_x , TYPE_INT, {.val_i=0}},
  { "wipe_exit_pos_y", &config.wipe_exit_pos_y , TYPE_INT, {.val_i=0}},

  { "steps_per_revolution_e", &config.steps_per_revolution_e, TYPE_INT, {.val_i=3200}},  // 200 * 16
  
  { "wait_on_temp", &config.wait_on_temp, TYPE_INT, {.val_i=0}},
    
  { "num_extruders", &config.num_extruders, TYPE_INT, {.val_i=1}},

  { "enable_extruder_0", &config.enable_extruder_0, TYPE_INT, {.val_i=1}},
};


// lookup table for pin mapping
// this allows some portability to different setups without affecting application config
// This has default values for the R2C2 v1.2 electronics board
static const tConfigItem config_lookup_pindef [] = 
{

  // X Axis (axis 0)
  { "x.pin_step",   &config.axis[X_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = X_STEP_PIN  }},
  { "x.pin_dir",    &config.axis[X_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = X_DIR_PIN   }},
  { "x.pin_enable", &config.axis[X_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = X_ENABLE_PIN}},
  { "x.pin_reset",  &config.axis[X_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "x.pin_min_limit", &config.axis[X_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = X_MIN_PIN}},
  { "x.pin_max_limit", &config.axis[X_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  // Y Axis (axis 1)
  { "y.pin_step",   &config.axis[Y_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = Y_STEP_PIN   }},
  { "y.pin_dir",    &config.axis[Y_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = Y_DIR_PIN    }},
  { "y.pin_enable", &config.axis[Y_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = Y_ENABLE_PIN }},
  { "y.pin_reset",  &config.axis[Y_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "y.pin_min_limit", &config.axis[Y_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = Y_MIN_PIN    }},
  { "y.pin_max_limit", &config.axis[Y_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // Z Axis (axis 2)
  { "z.pin_step",   &config.axis[Z_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = Z_STEP_PIN   }},
  { "z.pin_dir",    &config.axis[Z_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = Z_DIR_PIN    }},
  { "z.pin_enable", &config.axis[Z_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = Z_ENABLE_PIN }},
  { "z.pin_reset",  &config.axis[Z_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "z.pin_min_limit", &config.axis[Z_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = Z_MIN_PIN        }},
  { "z.pin_max_limit", &config.axis[Z_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // E Axis (axis 3)
  { "e.pin_step",   &config.axis[E_AXIS].pin_step,   TYPE_PIN_DEF, {.val_pin_def = E_STEP_PIN   }},
  { "e.pin_dir",    &config.axis[E_AXIS].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = E_DIR_PIN    }},
  { "e.pin_enable", &config.axis[E_AXIS].pin_enable, TYPE_PIN_DEF, {.val_pin_def = E_ENABLE_PIN }},
  { "e.pin_reset",  &config.axis[E_AXIS].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  { "e.pin_min_limit", &config.axis[E_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},
  { "e.pin_max_limit", &config.axis[E_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},

  { "all_steppers.reset",  &config.pin_all_steppers_reset,  TYPE_PIN_DEF, {.val_pin_def = STEPPERS_RESET_PIN}},

  // CTC #1 / Extruder 0
  { "extruder_0.heater",              &config.extruder_ctc[0].pin_heater,  TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_0_HEATER_PIN}},
  { "extruder_0.temp_sensor",         &config.extruder_ctc[0].pin_temp_sensor,  TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_0_SENSOR_ADC_PIN}},
  { "extruder_0.cooler",              &config.extruder_ctc[0].pin_cooler,  TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_0_FAN_PIN}},
  { "extruder_0.sensor_adc_channel",  &config.extruder_ctc[0].sensor_adc_channel,  TYPE_INT, {.val_i = EXTRUDER_0_SENSOR_ADC_CHANNEL}},

  // CTC #2 / Heated Bed
  { "heated_bed.heater",              &config.heated_bed_ctc.pin_heater,  TYPE_PIN_DEF, {.val_pin_def = HEATED_BED_0_HEATER_PIN}},
  { "heated_bed.temp_sensor",         &config.heated_bed_ctc.pin_temp_sensor,  TYPE_PIN_DEF, {.val_pin_def = HEATED_BED_0_ADC_PIN}},
  { "heated_bed.cooler",              &config.heated_bed_ctc.pin_cooler,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  { "heated_bed.sensor_adc_channel",  &config.heated_bed_ctc.sensor_adc_channel,  TYPE_INT, {.val_i = HEATED_BED_0_SENSOR_ADC_CHANNEL}},

  // Extruder 1 (not supported yet)
  { "extruder_1.heater",              &config.extruder_ctc[1].pin_heater,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  { "extruder_1.temp_sensor",         &config.extruder_ctc[1].pin_temp_sensor,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  { "extruder_1.cooler",              &config.extruder_ctc[1].pin_cooler,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  { "extruder_1.sensor_adc_channel",  &config.extruder_ctc[1].sensor_adc_channel,  TYPE_INT, {.val_i = 0}},


  { "cp_lcd_data_0",   &config.interface_cp_lcd_pin_data[0],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_1",   &config.interface_cp_lcd_pin_data[1],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_2",   &config.interface_cp_lcd_pin_data[2],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_3",   &config.interface_cp_lcd_pin_data[3],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_4",   &config.interface_cp_lcd_pin_data[4],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_5",   &config.interface_cp_lcd_pin_data[5],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_6",   &config.interface_cp_lcd_pin_data[6],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_7",   &config.interface_cp_lcd_pin_data[7],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_rw",   &config.interface_cp_lcd_pin_rw,   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_rs",   &config.interface_cp_lcd_pin_rs,   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_en",   &config.interface_cp_lcd_pin_en,   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},

  { "cp_led_0",   &config.interface_cp_led_pin[0],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_led_1",   &config.interface_cp_led_pin[1],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_led_2",   &config.interface_cp_led_pin[2],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},

  { "cp_btn_0",   &config.interface_cp_btn_pin[0],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,22,1)   }},
  { "cp_btn_1",   &config.interface_cp_btn_pin[1],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,4,1)   }},
  { "cp_btn_2",   &config.interface_cp_btn_pin[2],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,8,1)   }},
  { "cp_btn_3",   &config.interface_cp_btn_pin[3],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(0,26,1)   }},
  { "cp_btn_4",   &config.interface_cp_btn_pin[4],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(2,1,1)   }},
  { "cp_btn_5",   &config.interface_cp_btn_pin[5],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(2,0,1)   }},
  { "cp_btn_6",   &config.interface_cp_btn_pin[6],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,1,1)   }},
  { "cp_btn_7",   &config.interface_cp_btn_pin[7],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,10,1)    }},
  { "cp_btn_8",   &config.interface_cp_btn_pin[8],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(0,9,1)    }},
  { "cp_btn_9",   &config.interface_cp_btn_pin[9],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},


};


static tKeyHash config_keys [NUM_TOKENS(config_lookup)];
static tKeyHash config_pindef_keys [NUM_TOKENS(config_lookup_pindef)];




void app_config_set_defaults(void)
{
  set_defaults (config_lookup, NUM_TOKENS(config_lookup));
  
  set_defaults (config_lookup_pindef, NUM_TOKENS(config_lookup_pindef));

    // set default axis map for R2C2 - printer
  config.num_axes = NUM_AXES;
  config.axis[0].letter_code = 'X';
  config.axis[0].is_configured = true;
  config.axis[1].letter_code = 'Y';
  config.axis[1].is_configured = true;
  config.axis[2].letter_code = 'Z';
  config.axis[2].is_configured = true;
  config.axis[3].letter_code = 'E';
  config.axis[3].is_configured = true;

}


// read the config files from SD Card
void app_config_read (void)
{
  FRESULT res;    /* FatFs function common result code */
      
  create_key_hash_table (NUM_TOKENS(config_lookup_pindef), config_lookup_pindef, config_pindef_keys);
  create_key_hash_table (NUM_TOKENS(config_lookup), config_lookup, config_keys);

  res = read_config_file ("conf_pin.txt", config_lookup_pindef, NUM_TOKENS(config_lookup_pindef), config_pindef_keys);

  res = read_config_file ("config.txt", config_lookup, NUM_TOKENS(config_lookup), config_keys);
}

// print the config tables
void app_config_print()
{
  print_config_table (config_lookup, NUM_TOKENS(config_lookup) );
}

static tLineBuffer line_buf;

// read a file and execute GCode commands
void exec_gcode_file (char *filename)
{
  char *pLine;
  FIL file;
  FRESULT res;
  tGcodeInputMsg GcodeInputMsg;

  res = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
  if (res == FR_OK)
  {
    GcodeInputMsg.pLineBuf = &line_buf;
    GcodeInputMsg.out_file = NULL;
    
    pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file); /* read one line */
    while (pLine)
    {
      line_buf.len = strlen(pLine);
      gcode_parse_line (&GcodeInputMsg);
      pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file); /* read next line */
    }

    /* Close file */
    res = f_close(&file);
    if (res)
      debug("Error closing %s\n", filename);
  }  

}