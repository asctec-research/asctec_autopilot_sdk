/*
 * Copyright (C) 2016 Ascending Technologies GmbH, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <reent.h>
#include <sys/stat.h>
#include <string.h>
#include "errno.h"
#include "hal/uart0.h"
#include "uart1.h"

#undef errno
extern int errno;

void _exit(int status)
{
  (void)status;

  while(1)
    asm volatile("nop");
}

int _close(int file)
{
  (void)file;

  return -1;
}

int _execve(char *name, char **argv, char **env)
{
  (void)name;
  (void)argv;
  (void)env;

  errno = ENOMEM;
  return -1;
}

int _fork()
{
  errno = EAGAIN;
  return -1;
}

int _fstat(int file, struct stat *st)
{
  (void)file;

  st->st_mode = S_IFCHR;
  return 0;
}

int _getpid(void)
{
  return 1;
}

int _isatty(int file)
{
  (void)file;

  return 1;
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;

  errno = EINVAL;
  return (-1);
}

int _link(char *old, char *new)
{
  (void)old;
  (void)new;

  errno = EMLINK;
  return -1;
}

int _lseek(int file, int ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;

  return 0;
}

extern char _end[]; /*  end is set in the linker command  */
static char *heap_ptr; /* Points to current end of the heap.  */
caddr_t _sbrk(int incr)
{
  char *base;

  if(incr == 0)
    return (caddr_t)0;

  /*  Initialize if first time through.   */
  if(!heap_ptr)
  {
    heap_ptr = _end;
  }

  base = heap_ptr; /*  Point to end of heap.      */
  heap_ptr += incr; /*  Increase heap.        */

  return (caddr_t)base; /*  Return pointer to start of new heap area.  */
}

int _read(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  (void)len;

  return -1;
}

int _stat(const char *filepath, struct stat *st)
{
  (void) filepath;

  memset(st, 0, sizeof(*st));
  st->st_mode = S_IFCHR;
  return 0;
}

clock_t _times(struct tms *buf)
{
  (void)buf;

  return -1;
}

int _unlink(char *name)
{
  (void)name;

  errno = ENOENT;
  return -1;
}

int _wait(int *status)
{
  (void)status;

  errno = ECHILD;
  return -1;
}

int _write(int file, char *ptr, int len)
{
  (void)file;

  int i;
  const unsigned char *p;

  p = (const unsigned char*)ptr;

  for(i = 0; i < len; i++)
  {
    if(*p == '\n')
      UART0WriteChar('\r');
    UART0WriteChar(*p++);
  }

  return len;
}
