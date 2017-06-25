#ifndef DISPLAY_H
#define DISPLAY_H

#define DUFF_DEVICE_8(aCount, aAction) \
do { \
        int32_t count_ = (aCount); \
        int32_t times_ = (count_ + 15) >> 4; \
        switch (count_ & 7)             \
        { \
                case 0: do { aAction; \
                case 15: aAction; \
                case 14: aAction; \
                case 13: aAction; \
                case 12: aAction; \
                case 11: aAction; \
                case 10: aAction; \
                case 9: aAction; \
                case 8: aAction; \
                case 7: aAction; \
                case 6: aAction; \
                case 5: aAction; \
                case 4: aAction; \
                case 3: aAction; \
                case 2: aAction; \
                case 1: aAction; \
        } while (--times_ > 0); \
        } \
} while (0)

typedef struct
{
        unsigned char red;
        unsigned char green;
        unsigned char blue;
} COLOR;

struct display_device {
  unsigned char brightness_max;

  void          (*brightness)(unsigned char brightness);
  void          (*init)(void);
  unsigned int  (*status)(void);

  void          (*command)(unsigned char command);
  void          (*data)(unsigned int data);
  void          (*pixel)(void);
  void          (*set_xy)(int x, int y);
  void          (*set_window)(int x_start, int y_start, int x_end, int y_end);
  void          (*read)(COLOR *pixel);
  void          (*enable)(void);
  void          (*disable)(void);
  void          (*resume)(void);
  void          (*suspend)(void);
  void          (*fill)(unsigned long length);
  int			(*screen_rev)(void);

};

extern struct display_device *display;

#endif
