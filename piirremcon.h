/* IR remote controller for Raspberry Pi */

#define PIIR_TYPE_PI3 3
#define PIIR_TYPE_PI2 2
#define PIIR_TYPE_PI1 1
#define PIIR_TYPE_PI0 0

enum CODETYPE {AEHA, SIRC};
	       
struct IRCode {
  const enum CODETYPE codetype;
  const int  length;   // bit length
  const unsigned char data[9];
  const char *device;
  const char *command;
};

const struct IRCode PIIR_codedb[] =
  {
   {SIRC, 12, {0x12, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}, "sonytv", "volup"},
   {SIRC, 12, {0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}, "sonytv", "voldown"},
   {SIRC, 12, {0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}, "sonytv", "power"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x16, 0xd1, 0x01}, "sharptv", "power"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x13, 0x81, 0x01}, "sharptv", "source"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x4e, 0x32, 0x01}, "sharptv", "ch1"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x4f, 0x22, 0x01}, "sharptv", "ch2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x50, 0xc2, 0x01}, "sharptv", "ch3"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x51, 0xd2, 0x01}, "sharptv", "ch4"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x52, 0xe2, 0x01}, "sharptv", "ch5"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x53, 0xf2, 0x01}, "sharptv", "ch6"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x54, 0x82, 0x01}, "sharptv", "ch7"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x55, 0x92, 0x01}, "sharptv", "ch8"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x56, 0xa2, 0x01}, "sharptv", "ch9"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x57, 0xb2, 0x01}, "sharptv", "ch10"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x58, 0x42, 0x01}, "sharptv", "ch11"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x59, 0x52, 0x01}, "sharptv", "ch12"},
   {AEHA, 49, {0xaa, 0x5a, 0xf1, 0x48, 0x68, 0x8b, 0x01}, "sharptv", "power2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x15, 0xe1, 0x01}, "sharptv", "voldown"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x14, 0xf1, 0x01}, "sharptv", "volup"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xc4, 0x21, 0x01}, "sharptv", "menu"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x80, 0x12, 0x01}, "sharptv", "blue"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x81, 0x02, 0x01}, "sharptv", "red"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x82, 0x32, 0x01}, "sharptv", "green"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x83, 0x22, 0x01}, "sharptv", "yellow"},
  };

/**
 *  @fn
 *  initialize
 *  @param (gpio) GPIO number for IR LED
 *  @return -1 if error, 0 otherwise
 */
int piir_initialize(int gpio, int pi_type);

/**
 *  @fn
 *  transmit IR remote commander
 *  @param (cmd) 
 */
//int piir_transmit(char *cmd, char *manufacturer, int n_repeat);