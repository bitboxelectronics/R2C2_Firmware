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

#ifndef _PINOUT_H
#define _PINOUT_H

#include "ios.h"
#include "machine.h"

/*
        Machine Pin Definitions
*/

/*
        user defined pins
        adjust to suit your electronics,
        or adjust your electronics to suit this
*/

//x axis pins
#define X_STEP_PORT     1         /* P1.20 */
#define X_STEP_PIN      (1 << 20) /* P1.20 */

#define X_DIR_PORT      1         /* P1.23 */
#define X_DIR_PIN       (1 << 23) /* P1.23 */

#define X_ENABLE_PORT   1         /* P1.24 */
#define X_ENABLE_PIN    (1 << 24) /* P1.24 */

#define X_MIN_PORT      2         /* P2. 6 */
#define X_MIN_PIN       (1 <<  6) /* P2. 6 */

//y axis pins
#define Y_STEP_PORT     1         /* P1.25 */
#define Y_STEP_PIN      (1 << 25) /* P1.25 */

#define Y_DIR_PORT      1         /* P1.26 */
#define Y_DIR_PIN       (1 << 26) /* P1.26 */

#define Y_ENABLE_PORT   1         /* P1.28 */
#define Y_ENABLE_PIN    (1 << 28) /* P1.28 */

#define Y_MIN_PORT      2         /* P2. 7 */
#define Y_MIN_PIN       (1 <<  7) /* P2. 7 */

//z axis pins
#define Z_STEP_PORT     1         /* P1.29 */
#define Z_STEP_PIN      (1 << 29) /* P1.29 */

#define Z_DIR_PORT      0         /* P0. 0 */
#define Z_DIR_PIN       (1 <<  0) /* P0. 0 */

#define Z_ENABLE_PORT   0         /* P0. 1 */
#define Z_ENABLE_PIN    (1 <<  1) /* P0. 1 */

#define Z_MIN_PORT      2         /* P2. 8 */
#define Z_MIN_PIN       (1 <<  8) /* P2. 8 */

//e axis pins
#define E_STEP_PORT     0         /* P0.10 */
#define E_STEP_PIN      (1 << 10) /* P0.10 */

#define E_DIR_PORT      0         /* P0.11 */
#define E_DIR_PIN       (1 << 11) /* P0.11 */

#define E_ENABLE_PORT   2         /* P2.10 */
#define E_ENABLE_PIN    (1 << 10) /* P2.10 */

#define STEPPERS_RESET_PORT     0         /* P0.22 */
#define STEPPERS_RESET_PIN      (1 << 22) /* P0.22 */

#define EXTRUDER_0_HEATER_PORT          2        /* P2.4 */
#define EXTRUDER_0_HEATER_PIN           (1 << 4) /* P2.4 */
#define EXTRUDER_0_SENSOR_ADC_PORT      0        /* P0.2 */
#define EXTRUDER_0_SENSOR_ADC_PIN       2        /* P0.2 */
#define EXTRUDER_0_SENSOR_ADC_CHANNEL   7        /* P0.2 */

#define EXTRUDER_0_FAN_PORT             2         /* P2.3 */
#define EXTRUDER_0_FAN_PIN              (1<<3)

#define HEATED_BED_0_HEATER_PORT        2        /* P2.5 */
#define HEATED_BED_0_HEATER_PIN         (1 << 5) /* P2.5 */
#define HEATED_BED_0_ADC_PORT           0        /* P0.3 */
#define HEATED_BED_0_ADC_PIN            3        /* P0.3 */
#define HEATED_BED_0_SENSOR_ADC_CHANNEL 6        /* P0.3 */

#define BUZZER_PORT     2         /* P2.2 PWM1[3] */
#define BUZZER_PIN      (1 << 22) /* P2.2 PWM1[3] */

/*
        X Stepper
*/
#define x_enable() digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 0)
#define x_disable() digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 1)
#define x_step() digital_write(X_STEP_PORT, X_STEP_PIN, 1)
#define x_unstep() digital_write(X_STEP_PORT, X_STEP_PIN, 0)
#define x_direction(dir) digital_write(X_DIR_PORT, X_DIR_PIN, dir)
#define x_min() ((digital_read(X_MIN_PORT, X_MIN_PIN))?0:1)

/*
        Y Stepper
*/
#define y_enable() digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 0)
#define y_disable() digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 1)
#define y_step() digital_write(Y_STEP_PORT, Y_STEP_PIN, 1)
#define y_unstep() digital_write(Y_STEP_PORT, Y_STEP_PIN, 0)
#define y_direction(dir) digital_write(Y_DIR_PORT, Y_DIR_PIN, dir)
#define y_min() ((digital_read(Y_MIN_PORT, Y_MIN_PIN))?0:1)

/*
        Z Stepper
*/
#define z_enable() digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 0)
#define z_disable() digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 1)
#define z_step() digital_write(Z_STEP_PORT, Z_STEP_PIN, 1)
#define z_unstep() digital_write(Z_STEP_PORT, Z_STEP_PIN, 0)
#define z_direction(dir) digital_write(Z_DIR_PORT, Z_DIR_PIN, dir)
#define z_min() ((digital_read(Z_MIN_PORT, Z_MIN_PIN))?0:1)

/*
        Extruder
*/
#define e_enable() digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 0)
#define e_disable() digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 1)
#define e_step() digital_write(E_STEP_PORT, E_STEP_PIN, 1)
#define e_unstep() digital_write(E_STEP_PORT, E_STEP_PIN, 0)
#define e_direction(dir) digital_write(E_DIR_PORT, E_DIR_PIN, dir)

#define extruder_heater_on() digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, HIGH);
#define extruder_heater_off() digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, LOW);

#define extruder_fan_on() digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, HIGH);
#define extruder_fan_off() digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, LOW);

/*
        Heated Bed
*/
#define heated_bed_on() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, HIGH);
#define heated_bed_off() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, LOW);

#define power_on()      if (0) {}
#define power_off()     if (0) {}


/*
        End Step - All Steppers
        (so we don't have to delay in interrupt context)
*/

#define unstep()                                                        do { x_unstep(); y_unstep(); z_unstep(); e_unstep(); } while (0)

#endif  /* _PINOUT_H */
