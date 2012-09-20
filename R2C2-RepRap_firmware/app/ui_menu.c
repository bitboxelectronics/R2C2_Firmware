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

#include "app_config.h"
#include "lw_io.h"
#include "gcode_parse.h"
#include "gcode_task.h"
#include "gcode_process.h"
#include "packed_gcode.h"
#include "sermsg.h"
#include "temp.h"
#include "LiquidCrystal.h"

#include "keypad_gen4_mb.h"

#include "ui_menu.h"

extern void reboot (void);

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define CH_CURSOR       '>'

/* Display config */
#define LCD_NUM_ROWS     4
#define LCD_NUM_COLS     20

#define NUM_LIST_LINES 3
#define TOP_LIST_ROW   1

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

eMenu menu;

eHostState host_state;
eOperatorAttention attention_status;

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef enum {
  DISTANCE_TINY,
  DISTANCE_SHORT,
  DISTANCE_MEDIUM,
  DISTANCE_LONG
  } eJogDistance;


// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

static tLineBuffer LineBuf;
static tGcodeInputMsg GcodeInputMsg;

static int32_t timer_ticks;
static LW_FILE *lcdf;

// monitor menu

// jog menu
static eJogDistance jog_distance = DISTANCE_SHORT;

// SD menu

static uint8_t      cursor_pos;     // 0..2
static uint16_t     list_top;
static uint16_t     list_count;

//static uint16_t     cur_index;      // index into current list, 0 based

//static int          dir_count;      // no of entries

static char cur_path [MAX_LINE] = "/";

/*
  array of char *
  array of struct {byte info; char data[]}

*/

typedef struct {
  uint8_t attrib;
  char data [LCD_NUM_COLS+1];
} item_entry_t;

#define NUM_LIST_ENTRIES(list) (sizeof(list)/sizeof(item_entry_t))

static char * *item_list; // memory block containing index + data, pointer to start of index
//static char * item_data;  // 

const item_entry_t menu_home_list [] = 
{ 
  {0, "Home All"},
  {0, "Home X"},
  {0, "Home Y"}, 
  {0, "Home Z"} 
 };

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

static void create_list (const item_entry_t items [], int num_items)
{
  int data_size;
  int index_size;
  char *item_data;
  int j;
 
  list_count = num_items;

  index_size = sizeof(char *) * list_count;
  data_size =  index_size + sizeof (item_entry_t) * list_count;

  if (item_list != NULL)
    vPortFree (item_list);

  item_list = pvPortMalloc (data_size);
  item_data = (char *) item_list + index_size;

  memcpy (item_data, items, sizeof (item_entry_t) * list_count);
  for (j=0; j < list_count; j++)
  {
    item_list [j] = item_data;
    item_data += sizeof (item_entry_t);
  }
}



static void jog (uint8_t key_pressed) 
{
  float dist;
  uint8_t axis;
  tTarget target;

  if (GcodeInputMsg.in_use)
    return;

  GcodeInputMsg.type = GC_PACKED;

  target.feed_rate = 1200;
  target.invert_feed_rate = false;

  switch(jog_distance) 
  {
    case DISTANCE_TINY:
      dist = 0.1;
    break;
    case DISTANCE_SHORT:
      dist = 1.0;
    break;
    case DISTANCE_MEDIUM:
      dist = 10.0;
    break;
    case DISTANCE_LONG:
      dist = 50.0;
    break;
  }

  switch(key_pressed) 
  {
    case KEY_X_MINUS:
      axis = 'X';
      dist = -dist;
    break;

    case KEY_X_PLUS:
      axis = 'X';
    break;

    case KEY_Y_MINUS:
      axis = 'Y';
      dist = -dist;
    break;

    case KEY_Y_PLUS:
      axis = 'Y';
    break;

    case KEY_Z_MINUS:
      axis = 'Z';
      dist = -dist;
    break;

    case KEY_Z_PLUS:
      axis = 'Z';
    break;
  }

  plan_set_feed_rate (&target);

  LineBuf.len = 0;
  gcode_add_word_int (&LineBuf, 'G', 91);
  gcode_add_code (&LineBuf, CODE_END_COMMAND);

  gcode_add_word_int (&LineBuf, 'G', 1);
  gcode_add_word_float (&LineBuf, axis, dist);
  gcode_add_code (&LineBuf, CODE_END_COMMAND);

  gcode_add_word_int (&LineBuf, 'G', 90);
  gcode_add_code (&LineBuf, CODE_END_COMMAND);

  GcodeInputMsg.in_use = 1;
  tGcodeInputMsg *p_message = &GcodeInputMsg; 
  xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);

}

//
static void home (uint8_t axis) 
{

  if (GcodeInputMsg.in_use)
    return;

  GcodeInputMsg.type = GC_PACKED;

  switch(axis) 
  {
    case 1:
      axis = 'X';
    break;

    case 2:
      axis = 'Y';
    break;

    case 3:
      axis = 'Z';
    break;

    default:
      axis = 0;
  }

  LineBuf.len = 0;
  gcode_add_word_int (&LineBuf, 'G', 28);

  if (axis > 0)
  {
    gcode_add_word_int (&LineBuf, axis, 0);
  }
  gcode_add_code (&LineBuf, CODE_END_COMMAND);

  GcodeInputMsg.in_use = 1;
  tGcodeInputMsg *p_message = &GcodeInputMsg; 
  xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);

}

static void start_sd_print  (item_entry_t *pItem)
{
  
  if (GcodeInputMsg.in_use)
    return;

  GcodeInputMsg.type = GC_PACKED;

// select file, start print
  LineBuf.len = 0;
  gcode_add_word_int (&LineBuf, 'M', 23);
  gcode_add_word_str (&LineBuf, CODE_STR_PARAM, pItem->data);
  gcode_add_code (&LineBuf, CODE_END_COMMAND);

  gcode_add_word_int (&LineBuf, 'M', 24);
  gcode_add_code (&LineBuf, CODE_END_COMMAND);

  GcodeInputMsg.in_use = 1;
  tGcodeInputMsg *p_message = &GcodeInputMsg; 
  xQueueSend (GcodeRxQueue, &p_message, portMAX_DELAY);

  // check result?
}

// --------------------------------------------------------------------------
// 
// --------------------------------------------------------------------------


#if 0
typedef struct list_entry_s {
    char				*name;
    struct list_entry_s	*next;
} list_entry_t, *list_entry_p;

typedef struct dir_entry_s {
    char				*name;
    struct dir_entry_s	*next;
    //uint32_t			first_cluster;
    uint8_t				attribs;        // ATTR_xxx
} dir_entry_t, *dir_entry_p;


static dir_entry_t  *cur_dir;       // first entry not ".."    

list_entry_t *list_get_entry (list_entry_t *first, uint16_t index)
{
}

#endif


void lcd_clear_line (uint8_t y)
{
    uint8_t j;

    lcd_goto_xy (0,y);
    for (j=0; j< LCD_NUM_COLS; j++)
        lcd_write (' ');
    lcd_goto_xy (0,y);
}

void show_cursor (uint8_t ch)
{
    if (cursor_pos < NUM_LIST_LINES )  //0..2
    {
        lcd_goto_xy (0, cursor_pos + TOP_LIST_ROW);
        lcd_write (ch);
    }
}

//void dir_display_entry (dir_entry_t *dir_entry, uint8_t row, uint8_t col)
void dir_display_entry (uint16_t index, uint8_t row, uint8_t col)
{
  char *info = item_list[index];

  {
    lcd_setCursor (col, row);
    if (info[0] & AM_DIR) 
      lcd_write ('<');
    lcd_print (info+1);
    if (info[0] & AM_DIR) 
      lcd_write ('>');
  }
}


void show_list (bool init)
{
    uint8_t row;

    if (init)
    {
        cursor_pos = 0;
        list_top   = 0;
//        cur_index  = 0;

//        list_count = dir_count;
    }

    for (row=0; row < NUM_LIST_LINES; row++ ) //0..2
    {
        lcd_clear_line (row+TOP_LIST_ROW);
        if (list_top + row < list_count)
        {
            //dir_display_entry ( (dir_entry_t *)list_get_entry((list_entry_p)cur_dir, list_top + row), row+TOP_LIST_ROW, 2);    
            dir_display_entry ( list_top + row, row+TOP_LIST_ROW, 1);    
        }
    }
    show_cursor (CH_CURSOR);
}

void cursor_up (void)
{
    if (cursor_pos > 0)
    {
        show_cursor (' ');
        cursor_pos--;
        show_cursor (CH_CURSOR);
    }
    else
    {
        if (list_top > 0)
        {
            list_top--;
            show_list(false);
        }
    }
}

void cursor_down (void)
{
    if (cursor_pos < NUM_LIST_LINES-1)
    {
        show_cursor (' ');
        cursor_pos++;
        show_cursor (CH_CURSOR);
    }
    else
    {
        if (list_top + NUM_LIST_LINES < list_count)
        {
            list_top++;
            show_list(false);
        }
    }
}

//
bool match_file (FILINFO *fno)
{
  char *dot_p;

  if (fno->fname[0] == '.') return false;

  if (fno->fattrib & AM_DIR)
    return true;

  dot_p = strstr (fno->fname, ".");
  if (dot_p)
  {
    dot_p++;
    if ( (strcasecmp (dot_p, "g")==0) || (strcasecmp (dot_p, "gco")==0) )
    {
      return true;
    }
  }
  return false;
}

//
FRESULT sd_dir_get_entries (char *path, char *item_index[], char *strbuf_p)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
#if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif
    int cur_index;

    cur_index = 0;

    res = f_opendir(&dir, path);
    if (res == FR_OK) 
    {
      for (;;) 
      {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        if (match_file (&fno))
        {
            item_index[cur_index] = strbuf_p;

            *strbuf_p = fno.fattrib;
            strcpy (strbuf_p + 1, fno.fname);
            strbuf_p += strlen (fno.fname)+2;

            cur_index ++;
        }
      } // for
    }

    return res;    
}

//
FRESULT sd_dir_count (char *path, uint16_t *num_entries_p, int *data_size_p)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
#if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

    *num_entries_p = 0;
    *data_size_p = 0;

    res = f_opendir(&dir, path);
    if (res == FR_OK) 
    {
      for (;;) 
      {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        if (match_file (&fno))
        {
          (*num_entries_p) ++;
          (*data_size_p) +=  strlen(fno.fname) + 2;
        }
      }
    }

    return res;    
}


void get_directory_list (void)
{
  int data_size;
  int index_size;
  char * item_data;

  if (sd_dir_count (cur_path, &list_count, &data_size) == FR_OK)
  {
    if (item_list != NULL)
      vPortFree (item_list);

    index_size = sizeof(char *) * list_count;
    item_list = pvPortMalloc (data_size + index_size);

    item_data = (char *) item_list + index_size;

    sd_dir_get_entries (cur_path, item_list, item_data);
  }

}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief enter a new menu/screen
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------
void menu_enter (eMenu new_menu)
{
  menu = new_menu;

  switch (menu)
  {
    case menu_splash:
      // splash screen
      lcd_begin (20, 4);
      
      lcd_print ("RepRap Control Panel");
      lcd_setCursor (0,2); lcd_print ("Initialising");

      timer_ticks = 0;

    break;

    case menu_monitor:
      lcd_clear();

      lcd_setCursor (0,2);
      lcd_print ("Tool: ---/---\xdf");
      lcd_setCursor (0,3);
      lcd_print ("Bed : ---/---\xdf");
      
    break;


    case menu_jog:
      lcd_clear();
      lcd_print ("Jog mode: ");

      switch (jog_distance)
      {
        case DISTANCE_TINY:
          lcd_print ("0.1 mm ");
          break;
        case DISTANCE_SHORT:
          lcd_print ("1.0 mm ");
          break;
        case DISTANCE_MEDIUM:
          lcd_print ("10 mm  ");
          break;
        case DISTANCE_LONG:
          lcd_print ("50 mm  ");
          break;
      }

      lcd_setCursor (0,1);
      lcd_print ("  Y+          Z+");

      lcd_setCursor (0,2);
      lcd_print ("X-  X+    (mode)");

      lcd_setCursor (0,3);
      lcd_print ("  Y-          Z-");

    break;

    case menu_homing:
      lcd_clear();
      lcd_print ("Home axis: ");

      create_list (menu_home_list, NUM_LIST_ENTRIES(menu_home_list));
      show_list (true);
    break;

    case menu_sd_select:
      // display entries, pointer
      lcd_clear();
      lcd_print ("Select file: ");

      get_directory_list();
      show_list (true);
    break;

  }
}

// --------------------------------------------------------------------------
//! @brief update current menu/display
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------

void menu_poll (void)
{
  timer_ticks++;
  
  switch (menu)
  {
    case menu_splash:

      if (timer_ticks == 10)
      {
        menu_enter (menu_monitor);
      }
    break;

    case menu_monitor:
    {
        uint16_t val;

        lcd_setCursor (0,0);
        switch (host_state)
        {
          case hs_ready:
            if (sd_printing)
            {
              lcd_print ("Printing");
              lcd_setCursor (0,1);
              lcd_print (sd_file_name);
              //todo: percent done etc
            }
            else
            {
              lcd_print ("Ready");
              lcd_clear_line (1);
            }
            break;

          case hs_running:
            lcd_print ("Printing");
            
            lcd_setCursor (0,1);
            lcd_print (sd_file_name);
            break;

          case hs_error:
            lcd_print ("Fault");
            break;
        }

        val = temp_get (EXTRUDER_0);
        lcd_setCursor (6,2);
        fserwrite_int32_wz (lcdf, val, 3, 0);

        val = temp_get_target (EXTRUDER_0);
        lcd_setCursor (10,2);
        fserwrite_int32_wz (lcdf, val, 3, 0);

        val = temp_get (HEATED_BED_0);
        lcd_setCursor (6,3);
        fserwrite_int32_wz (lcdf, val, 3, 0);

        val = temp_get_target (HEATED_BED_0);
        lcd_setCursor (10,3);
        fserwrite_int32_wz (lcdf, val, 3, 0);
    }
    break;
  

    case menu_jog:
      lcd_setCursor (10, 0);

      switch (jog_distance)
      {
        case DISTANCE_TINY:
          lcd_print ("0.1 mm ");
          break;
        case DISTANCE_SHORT:
          lcd_print ("1.0 mm ");
          break;
        case DISTANCE_MEDIUM:
          lcd_print ("10 mm  ");
          break;
        case DISTANCE_LONG:
          lcd_print ("50 mm  ");
          break;
      }
    break;

    case menu_homing:
    break;

    case menu_sd_select:
    break;
  }
}

void menu_keyhandler (uint8_t key_pressed)
{
  
  if (key_pressed == KEY_CANCEL)
    reboot();

  switch (menu)
  {
    case menu_splash:
      // no-op
      break;

    case menu_monitor:
      if (key_pressed == KEY_ZERO)
        menu_enter (menu_jog);
      break;

    case menu_jog:
      switch (key_pressed)
      {
        case KEY_Y_MINUS:
        case KEY_Z_MINUS:
        case KEY_Y_PLUS:
        case KEY_Z_PLUS:
        case KEY_X_MINUS:
        case KEY_X_PLUS:
          jog(key_pressed);
        break;

        case KEY_OK:
          jog_distance++;
          if (jog_distance > DISTANCE_LONG)
            jog_distance = 0;
        break;

        case KEY_ZERO:
//        case KEY_CANCEL:
          menu_enter (menu_homing);
          break;
      }
    break;

    case menu_homing:
      switch (key_pressed)
      {
        case KEY_ZERO:
//        case KEY_CANCEL:
          menu_enter (menu_sd_select);
          break;
        case KEY_Z_PLUS:
          cursor_up();
        break;

        case KEY_Z_MINUS:
          cursor_down();
        break;

        case KEY_OK:
          home (list_top + cursor_pos);
          break;
      }
    break;

    case menu_sd_select:
      switch (key_pressed)
      {
        case KEY_ZERO:
          menu_enter (menu_monitor);
        break;

        case KEY_Z_PLUS:
          cursor_up();
        break;

        case KEY_Z_MINUS:
          cursor_down();
        break;

        case KEY_OK:
          // confirm?
          // print current entry : cur_path + cur_name
          start_sd_print (item_list [list_top + cursor_pos]);
          break;
      }
    break;
  }
}

void menu_init()
{
    LiquidCrystal_4bit (PACKED_PORT_BIT(config.interface_cp_lcd_pin_rs), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_en), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[4]), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[5]), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[6]), 
      PACKED_PORT_BIT(config.interface_cp_lcd_pin_data[7]) ); 

    GcodeInputMsg.pLineBuf = &LineBuf;
    GcodeInputMsg.out_file = NULL;
    GcodeInputMsg.result = PR_OK;
    GcodeInputMsg.in_use = 0;

    LineBuf.len = 0;

    timer_ticks = 0;
    lcdf = lw_fopen ("lcd", "w");

    menu_enter (menu_splash);
}

