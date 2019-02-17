/* IR remote controller for Raspberry pi */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/mman.h>
#include "piirremcon.h"

#define SUB_CARRIER_AEHA 38000 /* Hz */
#define SUB_CARRIER_NEC  38000 /* Hz */
#define SUB_CARRIER_SIRC 40000 /* Hz */
#define DT_US_AEHA (1000000 / SUB_CARRIER_AEHA)
#define DT_US_NEC (1000000 / SUB_CARRIER_NEC)
#define DT_US_SIRC (1000000 / SUB_CARRIER_SIRC)
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
  int mem_fd;
  volatile unsigned int *gpio;
} gState;

/** 
 *  @fn
 *  transmit raw bit pattern on sub carrier. (AEHA format)
 *  @param (pat) bit pattern (LSB is the first bit to be transmitted.)
 *  @param (n_bits) number of bits
 */
void piir_transmitPatternAEHA(const unsigned char *pat, const int n_bits)
{
  int t_us = 425;
  int us_next;
  int us_next2;
  int i, j, b;
  const unsigned char *p;
  struct timeval tv0;

  gettimeofday(&tv0, NULL);
  // leader
  us_next = DT_US_AEHA;
  do {
    GPIO_SET = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US_AEHA/2;
    GPIO_CLR = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US_AEHA/2*3;
  } while (us_next - DT_US_AEHA < t_us * 8);
  us_next = us_next + t_us * 4 - DT_US_AEHA;
  waitUs(tv0, us_next);
  us_next = us_next + DT_US_AEHA;
  // data
  for (i = 0, p = pat; i < (n_bits + 7) / 8; i++, p++) {
    for (b = 1, j = 0; b <= 128 && i * 8 + j < n_bits; b = b << 1, j = j + 1) {
      us_next2 = us_next + t_us;       // 0 =  T
      do {
	GPIO_SET = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US_AEHA/2;
	GPIO_CLR = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US_AEHA/2*3;
      } while (us_next - DT_US_AEHA < us_next2);
      GPIO_CLR = 1 << gState.led;
      if (*p & b) {
	us_next = us_next + t_us * 3;   // 1 = 3T
      } else {
	us_next = us_next + t_us;       // 0 =  T
      }
      waitUs(tv0, us_next);
      us_next = us_next + DT_US_AEHA;
    }
  }
}

/** 
 *  @fn
 *  transmit raw bit pattern on sub carrier. (NEC format)
 *  @param (pat) bit pattern (LSB is the first bit to be transmitted.)
 *  @param (n_bits) number of bits (normally 33 bit fixed)
 */
void piir_transmitPatternNEC(const unsigned char *pat, const int n_bits)
{
  int t_us = 562;
  int us_next;
  int us_next2;
  int i, j, b;
  const unsigned char *p;
  struct timeval tv0;

  gettimeofday(&tv0, NULL);
  // leader
  us_next = DT_US_NEC;
  do {
    GPIO_SET = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US_NEC/2;
    GPIO_CLR = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US_NEC/2*3;
  } while (us_next - DT_US_NEC < t_us * 16);
  us_next = us_next + t_us * 8 - DT_US_NEC;
  waitUs(tv0, us_next);
  us_next = us_next + DT_US_NEC;
  // data
  for (i = 0, p = pat; i < (n_bits + 7) / 8; i++, p++) {
    for (b = 1, j = 0; b <= 128 && i * 8 + j < n_bits; b = b << 1, j = j + 1) {
      us_next2 = us_next + t_us;       // 0 =  T
      do {
	GPIO_SET = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US_NEC/2;
	GPIO_CLR = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US_NEC/2*3;
      } while (us_next - DT_US_NEC < us_next2);
      GPIO_CLR = 1 << gState.led;
      if (*p & b) {
	us_next = us_next + t_us * 3;   // 1 = 3T
      } else {
	us_next = us_next + t_us;       // 0 =  T
      }
      waitUs(tv0, us_next);
      us_next = us_next + DT_US_NEC;
    }
  }
}

/** 
 *  @fn
 *  transmit raw bit pattern on sub carrier. (SIRC format)
 *  @param (pat) bit pattern (LSB is the first bit to be transmitted.)
 *  @param (n_bits) number of bits
 */
void piir_transmitPatternSIRC(const unsigned char *pat, const int n_bits)
{
  int t_us = 600;
  int us_next;
  int us_next2;
  int i, j, b;
  const unsigned char *p;
  struct timeval tv0;
  
  gettimeofday(&tv0, NULL);
  // leader
  us_next = DT_US_SIRC;
  do {
    GPIO_SET = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US_SIRC / 3;
    GPIO_CLR = 1 << gState.led;
    waitUs(tv0, us_next);
    us_next = us_next + DT_US_SIRC / 3 * 2;
  } while (us_next - DT_US_SIRC < t_us * 4);
  // data
  for (i = 0, p = pat; i < (n_bits + 7) / 8; i++, p++) {
    for (b = 1, j = 0; b <= 128 && i * 8 + j < n_bits; b = b << 1, j = j + 1) {
      us_next = us_next - DT_US_SIRC + t_us;  // 共通の消灯部分 (T)
      GPIO_CLR = 1 << gState.led;
      waitUs(tv0, us_next);
      if (*p & b) {
	us_next2 = us_next + t_us * 2;   // 1 = 2T
      } else {
	us_next2 = us_next + t_us;       // 0 =  T
      }
      us_next = us_next + DT_US_SIRC;
      do {
	GPIO_SET = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US_SIRC / 3;
	GPIO_CLR = 1 << gState.led;
	waitUs(tv0, us_next);
	us_next = us_next + DT_US_SIRC /3 * 2;
      } while (us_next - DT_US_SIRC < us_next2);
    }
  }
}


/** 
 *  @fn
 *  transmit remote command (common for AEHA, SIRC, and NEC)
 *  @param (code) IR code (struct IRcode). See piirremcon.h
 */
void piir_transmit(const struct IRCode code)
{
  switch (code.codetype) {
  case AEHA:
    piir_transmitPatternAEHA(code.data, code.length);
    break;
  case SIRC:
    piir_transmitPatternSIRC(code.data, code.length);
    break;
  case NEC:
    piir_transmitPatternNEC(code.data, code.length);
    break;
  }
}
      
/** 
 *  @fn
 *  hardware initialization. need to be called at first.
 *  @param (gpio_led) GPIO (BCM number) that is connected to LED
 *  @param (pi_type)  Raspberry Pi type such as 'PIIR_TYPE_PI3'.
 */
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
  if ((gState.mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) return -1;
  gState.gpio_map = (char*) mmap(
				 NULL,
				 BLOCK_SIZE,
				 PROT_READ | PROT_WRITE,
				 MAP_SHARED,// | MAP_FIXED,
				 gState.mem_fd,
				 gState.gpio_base
			  );
  if (gState.gpio_map == MAP_FAILED) return -1;
  gState.gpio = (volatile unsigned int *) gState.gpio_map;
  INP_GPIO(gpio_led);
  OUT_GPIO(gpio_led);
  GPIO_CLR = 1 << gpio_led;
  return 0;
}

int hexval(unsigned char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return 0;
}

// parse hex string
void parse_hex(unsigned char data[], const char *str, int buf_length) {
  const char *p;
  int i;
  for (p = str, i = 0; i < buf_length && *p && *(p + 1); i++, p += 2) {
    data[i] = hexval(*p) * 16 + hexval(*(p + 1));
  }
}


int main(int argc, char *argv[])
{
  int i, j;
  unsigned char data[9];
  for (i = 1; i < argc; i++) {
    if (strcmp("--list", argv[i]) == 0) {
      for (j = 0; j < sizeof(PIIR_codedb) / sizeof(struct IRCode); j++) {
	printf("%s %s\n", PIIR_codedb[j].device, PIIR_codedb[j].command);
      }
      return 0;
    }
  }
  if (argc < 3) {
    fprintf(stderr,
	    "usage: piirremcon device command\n"
	    "       piirremcon --list\n"
	    "example: piirremcon sharptv power\n"
	    "note: [device] can also be 'AEHA' or 'SIRC' which is followed by length and hex codes.\n"
	    );
    return -1;
  }

  if (piir_initialize(IR_LED, PIIR_TYPE_PI0)) printf("initialization error.\n");

  
  if (strcmp("AEHA", argv[1]) == 0 && argc > 3) {  // direct command
      parse_hex(data, argv[3], 9);
      piir_transmitPatternAEHA((const unsigned char *)data, (const int) atoi(argv[2]));
      usleep(40000);
      piir_transmitPatternAEHA((const unsigned char *)data, (const int) atoi(argv[2]));
      usleep(40000);
      piir_transmitPatternAEHA((const unsigned char *)data, (const int) atoi(argv[2]));
      usleep(40000);
      piir_transmitPatternAEHA((const unsigned char *)data, (const int) atoi(argv[2]));
      return 0;
  }
  if (strcmp("SIRC", argv[1]) == 0 && argc > 3) { // direct command
    parse_hex(data, argv[3], 9);
    piir_transmitPatternSIRC((const unsigned char *)data, (const int) atoi(argv[2]));
    usleep(40000);
    piir_transmitPatternSIRC((const unsigned char *)data, (const int) atoi(argv[2]));
    usleep(40000);
    piir_transmitPatternSIRC((const unsigned char *)data, (const int) atoi(argv[2]));
    return 0;
  }
  for (i = 0; i < sizeof(PIIR_codedb) / sizeof(struct IRCode); i++) {
    if (strcmp(PIIR_codedb[i].device, argv[1]) == 0 &&
	strcmp(PIIR_codedb[i].command, argv[2]) == 0) {
      piir_transmit(PIIR_codedb[i]);
      usleep(30000);
      piir_transmit(PIIR_codedb[i]);
      usleep(30000);
      piir_transmit(PIIR_codedb[i]);
      usleep(30000);
      piir_transmit(PIIR_codedb[i]);
      usleep(30000);
      piir_transmit(PIIR_codedb[i]);
      usleep(30000);
      piir_transmit(PIIR_codedb[i]);
      usleep(30000);
    }
  }
  return 0;
}
