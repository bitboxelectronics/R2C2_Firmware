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

#ifndef _LW_IO_H
#define _LW_IO_H

#include <stdarg.h>
#include <stddef.h>

#include "stdint.h"
#include "stdbool.h"

// some conflicts with stdio.h: stdin, stdout, stderr

#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

#define LW_FILENAME_MAX 256
#define LW_FOPEN_MAX   16
#define LW_TMP_MAX     256
#define LW_L_tmpnam    256


#define LW_O_RDONLY 0     // Open for reading only.
#define LW_O_WRONLY 1     // Open for writing only. 
#define LW_O_RDWR   2     // Open for reading and writing.

#ifndef EOF
#define EOF (-1)
#endif

// configuration
#define MAX_FILES LW_FOPEN_MAX

//typedef void FILE;
typedef struct 
{
  int handle;
  int dev_major;
  int dev_minor;

  int flags;
  int mode;

  int _errno;

  uint8_t in_use :1;
} LW_FILE;

typedef long fpos_t;


bool lw_initialise (void);

// Directed to stdout
int lw_putchar (int c);
int lw_puts (const char *s);
int lw_printf (const char *format, ...);

// Functions with LW_FILE * parameter
LW_FILE *lw_fopen(char *filename, char *mode);

int lw_putc (int c, LW_FILE *f);
int lw_fputc(int c, LW_FILE *f);
int lw_fputs(const char *s, LW_FILE *f);

int lw_fprintf(LW_FILE *, const char *, ...);
int lw_vfprintf(LW_FILE *, const char *, va_list);

int lw_fgetc(LW_FILE *);

//int lw_frxready (LW_FILE *f);


// POSIX compatible from fcntlh.h

#define LW_F_GETFL  F_GETFL
#define LW_F_SETFL  F_SETFL

#define LW_O_NONBLOCK   O_NONBLOCK

int lw_fcntl(LW_FILE *f, int cmd, ...);

int lw_ferror(LW_FILE *f);

// extra
int lw_get_errno(LW_FILE *f);

// ---------------------------

extern LW_FILE *stdin;
extern LW_FILE *stdout;
extern LW_FILE *stderr;

/***
#ifdef NOT_YET
int getchar(void);
char *gets(char *__s);
int sprintf(char *__s, const char *__format, ...);
int snprintf(char *__s, size_t __n, const char *__format, ...);
int vsnprintf(char *__s, size_t __n, const char *__format, __va_list __arg);

int vprintf(const char *__format, __va_list __arg);
int vsprintf(char *__s, const char *__format, __va_list __arg);

int scanf(const char *__format, ...);
int sscanf(const char *__s, const char *__format, ...);
int vscanf(const char *__format, __va_list __arg);
int vsscanf(const char *__s, const char *__format, __va_list __arg);

// Macro version of putchar.
#define putchar(x) __putchar(x)

void clearerr(FILE *);
int fclose(FILE *);
int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
int fgetpos(FILE *, fpos_t *);
char *fgets(char *, int, FILE *);
FILE *fopen(const char *, const char *);
size_t fread(void *, size_t, size_t, FILE *);
FILE *freopen(const char *, const char *, FILE *);
int fscanf(FILE *, const char *, ...);
int fseek(FILE *, long, int);
int fsetpos(FILE *, const fpos_t *); 
long ftell(FILE *);
size_t fwrite(const void *, size_t, size_t, FILE *);
int getc(FILE *);
void perror(const char *);
int remove(const char *);
int rename(const char *, const char *);
void rewind(FILE *);
void setbuf(FILE *, char *);
int setvbuf(FILE *, char *, int, size_t);
FILE *tmpfile(void);
char * tmpnam(char *);
int ungetc(int, FILE *);
int vfscanf(FILE *, const char *, __va_list);

#endif
*/

#endif