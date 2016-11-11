#include "os.h"
#include <sys/stat.h>
#include <stm32f4xx_hal.h>

extern UART_HandleTypeDef UART_stdio;


int _close(int file) { return -1; }

int _fstat(int file, struct stat *st) {
 st->st_mode = S_IFCHR;
 return 0;
}

int _isatty(int file) { return 1; }

int _lseek(int file, int ptr, int dir) { return 0; }

int _open(const char *name, int flags, int mode) { return -1; }

int _read(int file, char *ptr, int len) {
    return 0;
}

char *heap_end = 0;
caddr_t _sbrk(int incr) {
 extern char heap_low; /* Defined by the linker */
 extern char heap_top; /* Defined by the linker */
 char *prev_heap_end;

 if (heap_end == 0) {
  heap_end = &heap_low;
 }
 prev_heap_end = heap_end;

 if (heap_end + incr > &heap_top) {
  /* Heap and stack collision */
     for(;;);
 }

 heap_end += incr;
 return (caddr_t) prev_heap_end;
 }

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&UART_stdio,(uint8_t *)ptr,len,1000);
  return len;
}

static int errno; 

int *__errno(void) { return &errno; } 
