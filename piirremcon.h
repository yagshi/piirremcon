/* IR remote controller for Raspberry Pi */

#define PIIR_TYPE_PI3 3
#define PIIR_TYPE_PI2 2
#define PIIR_TYPE_PI1 1
#define PIIR_TYPE_PI0 0

enum CODETYPE {AEHA, SIRC, NEC};
	       
struct IRCode {
  const enum CODETYPE codetype;
  const int  length;   // bit length
  const unsigned char data[9];
  const char *device;
  const char *command;
};


const struct IRCode PIIR_codedb[] =
  {
   {SIRC, 12, {0x12, 0x01}, "sonytv", "volup"},
   {SIRC, 12, {0x13, 0x01}, "sonytv", "voldown"},
   {SIRC, 12, {0x15, 0x01}, "sonytv", "power"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x01, 0xb1, 0x01}, "sharptv", "ch1"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x02, 0x81, 0x01}, "sharptv", "ch2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x03, 0x91, 0x01}, "sharptv", "ch3"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x04, 0xe1, 0x01}, "sharptv", "ch4"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x05, 0xf1, 0x01}, "sharptv", "ch5"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x06, 0xc1, 0x01}, "sharptv", "ch6"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x07, 0xd1, 0x01}, "sharptv", "ch7"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x08, 0x21, 0x01}, "sharptv", "ch8"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x09, 0x31, 0x01}, "sharptv", "ch9"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x0a, 0x01, 0x01}, "sharptv", "ch10"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x0b, 0x11, 0x01}, "sharptv", "ch11"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x0c, 0x61, 0x01}, "sharptv", "ch11"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x0f, 0x62, 0x01}, "sharptv", "web"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x11, 0xa1, 0x01}, "sharptv", "chnext"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x12, 0x91, 0x01}, "sharptv", "chprev"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x13, 0x81, 0x01}, "sharptv", "source"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x14, 0xf1, 0x01}, "sharptv", "volup"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x15, 0xe1, 0x01}, "sharptv", "voldown"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x16, 0xd1, 0x01}, "sharptv", "power"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x17, 0xc1, 0x01}, "sharptv", "mute"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x18, 0x31, 0x01}, "sharptv", "audioselect"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x19, 0x21, 0x01}, "sharptv", "wakeuptimer"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x1a, 0x11, 0x01}, "sharptv", "sleeptimer"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x1b, 0x01, 0x01}, "sharptv", "info"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x1e, 0x62, 0x01}, "sharptv", "end"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x20, 0x81, 0x01}, "sharptv", "down"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x32, 0xb1, 0x01}, "sharptv", "hdmi"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x37, 0xd2, 0x01}, "sharptv", "savemode"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x39, 0x01, 0x01}, "sharptv", "avposition"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x3d, 0x41, 0x01}, "sharptv", "ambientlightsensor"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x3e, 0x42, 0x01}, "sharptv", "jogccw"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x3f, 0x52, 0x01}, "sharptv", "jogcw"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x40, 0xe1, 0x01}, "sharptv", "service"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x40, 0xd2, 0x01}, "sharptv", "jogcw2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x41, 0xc2, 0x01}, "sharptv", "jogcw3"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x42, 0xf2, 0x01}, "sharptv", "jogcw4"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x43, 0xe2, 0x01}, "sharptv", "jogccw2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x44, 0x92, 0x01}, "sharptv", "jogccw3"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x45, 0x82, 0x01}, "sharptv", "jogccw4"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x46, 0xb2, 0x01}, "sharptv", "jogccw5"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x47, 0xa2, 0x01}, "sharptv", "jogccw6"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x4a, 0x72, 0x01}, "sharptv", "poweron"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x4b, 0x62, 0x01}, "sharptv", "poweroff"},
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
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x5a, 0x62, 0x01}, "sharptv", "chsel"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x5c, 0x02, 0x01}, "sharptv", "ch?"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x5d, 0x12, 0x01}, "sharptv", "chdata?"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x5e, 0x22, 0x01}, "sharptv", "data"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x5f, 0x32, 0x01}, "sharptv", "programinfo"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x60, 0xf2, 0x01}, "sharptv", "epg"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x61, 0xe2, 0x01}, "sharptv", "blue"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x62, 0xd2, 0x01}, "sharptv", "red"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x63, 0xc2, 0x01}, "sharptv", "green"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x64, 0xb2, 0x01}, "sharptv", "yellow"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x66, 0x92, 0x01}, "sharptv", "info3"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x67, 0x82, 0x01}, "sharptv", "caption"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x68, 0x72, 0x01}, "sharptv", "register"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x6b, 0x42, 0x01}, "sharptv", "toggletvdata"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x75, 0xb2, 0x01}, "sharptv", "epg2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x80, 0x12, 0x01}, "sharptv", "blue"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x81, 0x02, 0x01}, "sharptv", "red"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x82, 0x32, 0x01}, "sharptv", "green"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x83, 0x22, 0x01}, "sharptv", "yellow"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x89, 0x82, 0x01}, "sharptv", "tv"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x8a, 0xb2, 0x01}, "sharptv", "bs"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x8b, 0xa2, 0x01}, "sharptv", "cs"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xab, 0x82, 0x01}, "sharptv", "??"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xaf, 0xc2, 0x01}, "sharptv", "10keypad"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xb5, 0x72, 0x01}, "sharptv", "iptv"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xbc, 0xe2, 0x01}, "sharptv", "tool"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xc4, 0x21, 0x01}, "sharptv", "menu"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xc7, 0x11, 0x01}, "sharptv", "caption2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xce, 0x81, 0x01}, "sharptv", "dnr"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xd5, 0x21, 0x01}, "sharptv", "size"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xdb, 0xf2, 0x01}, "sharptv", "c?"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xe3, 0x71, 0x01}, "sharptv", "videomute"},
   {AEHA, 49, {0xaa, 0x5a, 0xf1, 0x48, 0x68, 0x8b, 0x01}, "sharptv2", "power"},
   {NEC, 33, {0x83, 0x55, 0x9a, 0x65, 0x01}, "epsonprojector", "menu"},
   {NEC, 33, {0x83, 0x55, 0x90, 0x6f, 0x01}, "epsonprojector", "power"},
   {NEC, 33, {0x83, 0x55, 0x84, 0x7b, 0x01}, "epsonprojector", "back"},
   {NEC, 33, {0x83, 0x55, 0x85, 0x7a, 0x01}, "epsonprojector", "ok"},
   {NEC, 33, {0x83, 0x55, 0xb0, 0x4f, 0x01}, "epsonprojector", "up"},
   {NEC, 33, {0x83, 0x55, 0xb1, 0x4e, 0x01}, "epsonprojector", "right"},
   {NEC, 33, {0x83, 0x55, 0xb2, 0x4d, 0x01}, "epsonprojector", "down"},
   {NEC, 33, {0x83, 0x55, 0xb3, 0x4c, 0x01}, "epsonprojector", "left"},
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
