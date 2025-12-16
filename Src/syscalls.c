// Copyright (c) Acconeer AB, 2019-2023
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>

#include "main.h"

extern UART_HandleTypeDef DEBUG_UART_HANDLE;

int _fstat(int file, struct stat *st)
{
	(void)file;

	st->st_mode = S_IFCHR;
	return 0;
}

int _close(int file)
{
	(void)file;

	return -1;
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

int _lseek(int file, int ptr, int dir)
{
	(void)file;
	(void)ptr;
	(void)dir;

	return 0;
}

int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig;

	errno = EINVAL;
	return -1;
}

int _read(int file, char *ptr, int len)
{
	(void)file;
	(void)ptr;
	(void)len;

	return -1;
}

int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == 1 || fd == 2) {
    hstatus = HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return -1;
  }
  return -1;
}
