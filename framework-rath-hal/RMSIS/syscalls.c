#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "gd32vf103xb.h"


int _kill(int pid, int sig) {
	errno = EINVAL;
	return -1;
}

void _exit(int status) {
	_kill(status, -1);
	while (1) {}		/* Make sure we hang here */
}

int _close(int fd) {
  return -1;
}

int _fstat(int fd, struct stat* st) {
  if (isatty(fd)) {
    st->st_mode = S_IFCHR;
    return 0;
  }
  return -1;
}

int _isatty(int fd) {
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    return 1;
  }
  return 0;
}

off_t _lseek(int fd, off_t ptr, int dir) {
  if (isatty(fd))
    return 0;
  return -1;
}

int16_t _read(int fd, void* ptr, uint16_t size) {
  return -1;
}

void *_sbrk(ptrdiff_t incr) {
  extern char _end[];
  extern char _heap_end[];
  static char *curbrk = _end;

  if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

  curbrk += incr;
  return curbrk - incr;
}


extern int _put_char(int ch) __attribute__((weak));

int16_t _write(int fd, const void *ptr, uint16_t size) {
	const uint8_t *buf = (const uint8_t *)ptr;

  for (uint16_t i=0; i<size; i+=1) {
    _put_char(buf[i]);
  }

  return size;
}

int puts(const char *string) {
	return _write(0, (const void *)string, strlen(string));
}
