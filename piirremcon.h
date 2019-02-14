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
   {SIRC, 12, {0x80, 0x00}, "sonytv", "ch1"}, 
   {SIRC, 12, {0x81, 0x00}, "sonytv", "ch2"}, 
   {SIRC, 12, {0x82, 0x00}, "sonytv", "ch3"}, 
   {SIRC, 12, {0x83, 0x00}, "sonytv", "ch4"}, 
   {SIRC, 12, {0x84, 0x00}, "sonytv", "ch5"}, 
   {SIRC, 12, {0x85, 0x00}, "sonytv", "ch6"}, 
   {SIRC, 12, {0x86, 0x00}, "sonytv", "ch7"}, 
   {SIRC, 12, {0x87, 0x00}, "sonytv", "ch8"}, 
   {SIRC, 12, {0x88, 0x00}, "sonytv", "ch9"}, 
   {SIRC, 12, {0x89, 0x00}, "sonytv", "ch10"}, 
   {SIRC, 12, {0x8a, 0x00}, "sonytv", "ch11"}, 
   {SIRC, 12, {0x8b, 0x00}, "sonytv", "ch12"}, 
   {SIRC, 12, {0x8e, 0x00}, "sonytv", "epg"}, 
   {SIRC, 12, {0x90, 0x00}, "sonytv", "chnext"},  // cmd=10
   {SIRC, 12, {0x91, 0x00}, "sonytv", "chprev"},  // cmd=11
   {SIRC, 12, {0x92, 0x00}, "sonytv", "volup"},   // cmd=12
   {SIRC, 12, {0x93, 0x00}, "sonytv", "voldown"}, // cmd=13
   {SIRC, 12, {0x94, 0x00}, "sonytv", "mute"},    // cmd=14
   {SIRC, 12, {0x95, 0x00}, "sonytv", "power"},   // cmd=15
   {SIRC, 12, {0xa5, 0x00}, "sonytv", "source"}, 
   {SIRC, 12, {0xe5, 0x00}, "sonytv", "ok"},      // cmd=65
   {SIRC, 12, {0xf4, 0x00}, "sonytv", "up"},      // cmd=74
   {SIRC, 12, {0xf5, 0x00}, "sonytv", "down"},    // cmd=75
   {SIRC, 12, {0xb3, 0x00}, "sonytv", "right"},   // cmd=33
   {SIRC, 12, {0xb4, 0x00}, "sonytv", "left"},    // cmd=34
   // Blu-ray player (RMT-VB101J)
   {SIRC, 20, {0x15, 0x2d, 0x0e}, "sonybd", "power"}, // cmd=15
   {SIRC, 20, {0x1a, 0x2d, 0x0e}, "sonybd", "play"},
   {SIRC, 20, {0x1a, 0x16, 0x0e}, "sonybd", "eject"},
   {SIRC, 20, {0x1a, 0x18, 0x0e}, "sonybd", "stop"},
   {SIRC, 20, {0x1a, 0x19, 0x0e}, "sonybd", "pause"},
   {SIRC, 20, {0x39, 0x19, 0x0e}, "sonybd", "up"},    // cmd=13
   {SIRC, 20, {0x3a, 0x19, 0x0e}, "sonybd", "down"},
   {SIRC, 20, {0x3b, 0x19, 0x0e}, "sonybd", "left"},  // cmd=3b
   {SIRC, 20, {0x3c, 0x19, 0x0e}, "sonybd", "right"},
   {SIRC, 20, {0x3d, 0x19, 0x0e}, "sonybd", "ok"},    // cmd=3d
   {SIRC, 20, {0x43, 0x19, 0x0e}, "sonybd", "back"},
   {SIRC, 20, {0x56, 0x19, 0x0e}, "sonybd", "next"},
   {SIRC, 20, {0x57, 0x19, 0x0e}, "sonybd", "prev"},
   // AQUOS (LC-40F5)
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
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xa9, 0xa2, 0x01}, "sharptv", "mycircle"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xab, 0x82, 0x01}, "sharptv", "??"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xaf, 0xc2, 0x01}, "sharptv", "10keypad"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xb5, 0x72, 0x01}, "sharptv", "iptv"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xbb, 0x92, 0x01}, "sharptv", "home"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xbc, 0xe2, 0x01}, "sharptv", "tool"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xc4, 0x21, 0x01}, "sharptv", "menu"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xc7, 0x11, 0x01}, "sharptv", "caption2"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xce, 0x81, 0x01}, "sharptv", "dnr"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xd5, 0x21, 0x01}, "sharptv", "size"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xdb, 0xf2, 0x01}, "sharptv", "c?"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xe3, 0x71, 0x01}, "sharptv", "videomute"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xe4, 0x01, 0x01}, "sharptv", "back"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x52, 0xd1, 0x01}, "sharptv", "ok"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x57, 0x81, 0x01}, "sharptv", "up"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0x20, 0x81, 0x01}, "sharptv", "down"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xd7, 0x01, 0x01}, "sharptv", "left"},
   {AEHA, 49, {0xaa, 0x5a, 0x8f, 0x12, 0xd8, 0xf1, 0x01}, "sharptv", "right"},
   
   {AEHA, 49, {0xaa, 0x5a, 0xf1, 0x48, 0x68, 0x8b, 0x01}, "sharptv2", "power"},
   // REGZA (CT-90243)
   {NEC, 33, {0x40, 0xbf, 0x12, 0xed, 0x01}, "toshibatv", "power"},
   {NEC, 33, {0x40, 0xbf, 0x1a, 0xe5, 0x01}, "toshibatv", "volup"},
   {NEC, 33, {0x40, 0xbf, 0x1e, 0xe1, 0x01}, "toshibatv", "voldown"},
   {NEC, 33, {0x40, 0xbf, 0x1b, 0xe4, 0x01}, "toshibatv", "chprev"},
   {NEC, 33, {0x40, 0xbf, 0x1f, 0xe0, 0x01}, "toshibatv", "chnext"},
   {NEC, 33, {0x40, 0xbf, 0x3d, 0xc2, 0x01}, "toshibatv", "ok"},
   {NEC, 33, {0x40, 0xbf, 0x3c, 0xc3, 0x01}, "toshibatv", "end"},
   {NEC, 33, {0x40, 0xbf, 0x3b, 0xc4, 0x01}, "toshibatv", "back"},
   {NEC, 33, {0x40, 0xbf, 0x01, 0xfe, 0x01}, "toshibatv", "ch1"},
   {NEC, 33, {0x40, 0xbf, 0x02, 0xfd, 0x01}, "toshibatv", "ch2"},
   {NEC, 33, {0x40, 0xbf, 0x03, 0xfc, 0x01}, "toshibatv", "ch3"},
   {NEC, 33, {0x40, 0xbf, 0x04, 0xfb, 0x01}, "toshibatv", "ch4"},
   {NEC, 33, {0x40, 0xbf, 0x05, 0xfa, 0x01}, "toshibatv", "ch5"},
   {NEC, 33, {0x40, 0xbf, 0x06, 0xf9, 0x01}, "toshibatv", "ch6"},
   {NEC, 33, {0x40, 0xbf, 0x07, 0xf8, 0x01}, "toshibatv", "ch7"},
   {NEC, 33, {0x40, 0xbf, 0x08, 0xf7, 0x01}, "toshibatv", "ch8"},
   {NEC, 33, {0x40, 0xbf, 0x09, 0xf6, 0x01}, "toshibatv", "ch9"},
   {NEC, 33, {0x40, 0xbf, 0x0a, 0xf5, 0x01}, "toshibatv", "ch10"},
   {NEC, 33, {0x40, 0xbf, 0x0b, 0xf4, 0x01}, "toshibatv", "ch11"},
   {NEC, 33, {0x40, 0xbf, 0x0c, 0xf3, 0x01}, "toshibatv", "ch12"},
   {NEC, 33, {0x40, 0xbf, 0x3e, 0xc1, 0x01}, "toshibatv", "up"},
   {NEC, 33, {0x40, 0xbf, 0x3f, 0xc0, 0x01}, "toshibatv", "down"},
   {NEC, 33, {0x40, 0xbf, 0x5f, 0xa0, 0x01}, "toshibatv", "left"},
   {NEC, 33, {0x40, 0xbf, 0x5b, 0xa4, 0x01}, "toshibatv", "right"},
   //
   {NEC, 33, {0x83, 0x55, 0x9a, 0x65, 0x01}, "epsonprojector", "menu"},
   {NEC, 33, {0x83, 0x55, 0x90, 0x6f, 0x01}, "epsonprojector", "power"},
   {NEC, 33, {0x83, 0x55, 0x84, 0x7b, 0x01}, "epsonprojector", "back"},
   {NEC, 33, {0x83, 0x55, 0x85, 0x7a, 0x01}, "epsonprojector", "ok"},
   {NEC, 33, {0x83, 0x55, 0xb0, 0x4f, 0x01}, "epsonprojector", "up"},
   {NEC, 33, {0x83, 0x55, 0xb1, 0x4e, 0x01}, "epsonprojector", "right"},
   {NEC, 33, {0x83, 0x55, 0xb2, 0x4d, 0x01}, "epsonprojector", "down"},
   {NEC, 33, {0x83, 0x55, 0xb3, 0x4c, 0x01}, "epsonprojector", "left"},

   {NEC, 49, {0x18, 0xe9, 0x08, 0xf7, 0x00, 0xff, 0x01}, "necprojector", "power"},
   {NEC, 49, {0x18, 0xe9, 0x14, 0xeb, 0x00, 0xff, 0x01}, "necprojector", "standby"},
   {NEC, 49, {0x18, 0xe9, 0x17, 0xe8, 0x00, 0xff, 0x01}, "necprojector", "ok"},
   {NEC, 49, {0x18, 0xe9, 0x25, 0xd2, 0x00, 0xff, 0x01}, "necprojector", "back"},
   {NEC, 49, {0x18, 0xe9, 0x46, 0xb9, 0x00, 0xff, 0x01}, "necprojector", "menu"},
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
 *  transmit remote command (common for AEHA, SIRC, and NEC)
 *  @param (code) IR code (struct IRcode).
 */
void piir_transmit(const struct IRCode code);

