/* IR remote controller for Raspberry pi */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/mman.h>
#include "piirremcon.h"

#define SUB_CARRIER 38000 /* Hz */
#define DT_US (1000000 / SUB_CARRIER)
#define IR_LED 14   /* GPIO # */
#define PI2_PERI_BASE 0x3f000000  /* PI2, 3 */
#define PI1_PERI_BASE 0x20000000  /* PI0, 1 */

// set mode
#define INP_GPIO(g) *(gState.gpio + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gState.gpio + ((g)/10)) |= (1<<(((g)%10)*3))
// do I/O
#define GPIO_SET *(gState.gpio +  7)
#define GPIO_CLR *(gState.gpio + 10)
#define GPIO_GET *(gState.gpio + 13)

#define PAGE_SIZE  (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

/* 起点時刻から us [us] 後まで待つ */
/* @return 現在の us 値 */
inline static int waitUs(struct timeval tv_zero, int until_us) {
  struct timeval tv;
  int us;
  while (1) {
    gettimeofday(&tv, NULL);
    us = (tv.tv_sec - tv_zero.tv_sec) * 1000000 + (tv.tv_usec - tv_zero.tv_usec);
    if (us >= until_us) break;
  }
  return us;
}


struct PIIRState {
  int led;
  unsigned int gpio_base;
  char *gpio_map;
  char *gpio_mem;
  int mem_fd;
  volatile unsigned int *gpio;
} gState;

/** 
 *  @fn
 *  transmit raw bit pattern on sub carrier. (AEHA format)
 *  @param (pat) bit pattern (MSB is the first bit to be transmitted.)
 *  @param (n_bits) number of bits
 */
void piir_transmitPatternAEHA(const unsigned char *pat, const int n_bits)
{
  int t_us = 425;
  int us;
  int us_next;
  int us_next2;
  int i, j, b;
  const unsigned char *p;
  struct timeval tv0;

  gettimeofday(&tv0, NULL);
  // leader
  us_next = DT_US;
  do {
    GPIO_SET = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US/2;
    GPIO_CLR = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US/2*3;
  } while (us_next - DT_US < t_us * 8);
  us_next = us_next + t_us * 4 - DT_US;
  waitUs(tv0, us_next);
  us_next = us_next + DT_US;
  // data
  for (i = 0, p = pat; i < (n_bits + 7) / 8; i++, p++) {
    for (b = 1, j = 0; b <= 128 && i * 8 + j < n_bits; b = b << 1, j = j + 1) {
      us_next2 = us_next + t_us;       // 0 =  T
      do {
	GPIO_SET = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US/2;
	GPIO_CLR = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US/2*3;
      } while (us_next - DT_US < us_next2);
      GPIO_CLR = 1 << gState.led;
      if (*p & b) {
	us_next = us_next + t_us * 3;   // 1 = 3T
      } else {
	us_next = us_next + t_us;       // 0 =  T
      }
      waitUs(tv0, us_next);
      us_next = us_next + DT_US;
    }
  }
}

void piir_transmitPatternSIRC(const unsigned char *pat, const int n_bits)
{
  int t_us = 600;
  int us;
  int us_next;
  int us_next2;
  int i, j, b;
  const unsigned char *p;
  struct timeval tv0;
  
  gettimeofday(&tv0, NULL);
  // leader
  us_next = DT_US;
  do {
    GPIO_SET = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US / 3;
    GPIO_CLR = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US / 3 * 2;
  } while (us_next - DT_US < t_us * 4);
  // data
  for (i = 0, p = pat; i < (n_bits + 7) / 8; i++, p++) {
    for (b = 1, j = 0; b <= 128 && i * 8 + j < n_bits; b = b << 1, j = j + 1) {
      us_next = us_next - DT_US + t_us;  // 共通の消灯部分 (T)
      GPIO_CLR = 1 << gState.led;
      waitUs(tv0, us_next);
      if (*p & b) {
	us_next2 = us_next + t_us * 2;   // 1 = 2T
      } else {
	us_next2 = us_next + t_us;       // 0 =  T
      }
      us_next = us_next + DT_US;
      do {
	GPIO_SET = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US / 3;
	GPIO_CLR = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US /3 * 2;
      } while (us_next - DT_US < us_next2);
    }
  }
}

void piir_transmitPatternSIRC12(const unsigned char *pat, const int n_bits)
{
  unsigned char pat2[2];
  int p = pat[0] | (pat[1] << 7);
  pat2[0] = p & 0xff;
  pat2[1] = p >> 8;
  piir_transmitPatternSIRC(pat2, 12);
}

void piir_transmit(const struct IRCode code)
{
  switch (code.codetype) {
  case AEHA:
    piir_transmitPatternAEHA(code.data, code.length);
    break;
  case SIRC:
    if (code.length == 12) {
      piir_transmitPatternSIRC12(code.data, code.length);
    }
    break;
  }
}
      
int piir_initialize(int gpio_led, int pi_type)
{
  gState.led = gpio_led;
  switch (pi_type) {
  case PIIR_TYPE_PI1:
  case PIIR_TYPE_PI0:
    gState.gpio_base = PI1_PERI_BASE + 0x200000;
    break;
  case PIIR_TYPE_PI3:
  case PIIR_TYPE_PI2:
  default:
    gState.gpio_base = PI2_PERI_BASE + 0x200000;
  }
  if ((gState.mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) return -1;
  if ((gState.gpio_mem = (char*) malloc(BLOCK_SIZE + (PAGE_SIZE - 1))) == NULL)
    return -1;
  if ((unsigned long) gState.gpio_mem % PAGE_SIZE)
    gState.gpio_mem += PAGE_SIZE - ((unsigned long) gState.gpio_mem % PAGE_SIZE);
  gState.gpio_map = (char*) mmap(
				 (void*) gState.gpio_mem,
				 BLOCK_SIZE,
				 PROT_READ | PROT_WRITE,
				 MAP_SHARED | MAP_FIXED,
				 gState.mem_fd,
				 gState.gpio_base
			  );
  if ((long) gState.gpio_map < 0) return -1;
  gState.gpio = (volatile unsigned int *) gState.gpio_map;
  INP_GPIO(gpio_led);
  OUT_GPIO(gpio_led);
  GPIO_CLR = 1 << gpio_led;
  return 0;
}

int main(int argc, char *argv[])
{
  int i, j;
  if (argc < 3) {
    fprintf(stderr,
	    "usage: piirremcon device command\n"
	    "example: piirremcon sharptv power\n"
	    );
    return -1;
  }

  if (piir_initialize(IR_LED, PIIR_TYPE_PI0)) printf("initialization error.\n");


  for (i = 0; i < sizeof(PIIR_codedb) / sizeof(struct IRCode); i++) {
    if (strcmp(PIIR_codedb[i].device, argv[1]) == 0 &&
	strcmp(PIIR_codedb[i].command, argv[2]) == 0) {
      piir_transmit(PIIR_codedb[i]);
      usleep(40000);
      piir_transmit(PIIR_codedb[i]);
      usleep(40000);
      piir_transmit(PIIR_codedb[i]);
      usleep(40000);
      piir_transmit(PIIR_codedb[i]);
      usleep(40000);
      piir_transmit(PIIR_codedb[i]);
      usleep(40000);
    }
  }
  return 0;
}
