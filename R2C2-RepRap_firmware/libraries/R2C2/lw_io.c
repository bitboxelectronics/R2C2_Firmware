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

#include "lw_io.h"

#include "lw_syscalls.h"


// Global FILE table

FILE file_table [MAX_FILES] = 
{
    {.handle=0, .dev_major = DEV_STDIN,  .dev_minor = 0},
    {.handle=1, .dev_major = DEV_STDOUT, .dev_minor = 0},
    {.handle=2, .dev_major = DEV_STDERR, .dev_minor = 0},
    {.handle=3, .dev_major = DEV_DBGOUT, .dev_minor = 0},
};


// predefined file handles
FILE *stdin  = &file_table[0];
FILE *stdout = &file_table[1];
FILE *stderr = &file_table[2];

FILE *dbgout = &file_table[3];

// ----------------------------------------------------------------
// functions taking FILE parameter
// ----------------------------------------------------------------

int lw_fputs(const char *s, FILE *f)
{
  return _write (f->handle, s, strlen (s));
}

int lw_fputc(int c, FILE *f)
{
  return _write (f->handle, (char *)&c, 1);
}

int lw_putc(int c, FILE *f)
{
  lw_fputc (c, f);
}

// ----------------------------------------------------------------
// functions working on predefined streams: STDIN, STDOUT, STDERR
// ----------------------------------------------------------------


int  lw_puts(const char *s)
{
  return _write (stdout->handle, s, strlen (s));
}

int  lw_putchar(int c)
{
  return _write (stdout->handle, (char *) &c, 1);
}


// ----------------------------------------------------------------
// not yet implemented
// ----------------------------------------------------------------

#define __va_list va_list

int  getchar(void);
char *gets(char *__s);

int sprintf(char *__s, const char *__format, ...);
int snprintf(char *__s, size_t __n, const char *__format, ...);
int vsnprintf(char *__s, size_t __n, const char *__format, __va_list __arg);

int printf(const char *__format, ...);
int vprintf(const char *__format, __va_list __arg);
int vsprintf(char *__s, const char *__format, __va_list __arg);

int scanf(const char *__format, ...);
int sscanf(const char *__s, const char *__format, ...);
int vscanf(const char *__format, __va_list __arg);
int vsscanf(const char *__s, const char *__format, __va_list __arg);

void clearerr(FILE *);
int fclose(FILE *);
int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
int fgetc(FILE *);
int fgetpos(FILE *, fpos_t *);
char *fgets(char *, int, FILE *);
FILE *fopen(const char *, const char *);
int fprintf(FILE *, const char *, ...);
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
int vfprintf(FILE *, const char *, __va_list);
int vfscanf(FILE *, const char *, __va_list);

// ----------------------------------------------------------------
