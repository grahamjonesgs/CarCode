#include "fontx.h"
#include <string>

#define RED             0xf800
#define GREEN           0x07e0
#define BLUE            0x001f
#define BLACK           0x0000
#define WHITE           0xffff
#define GRAY            0x8c51
#define YELLOW          0xFFE0
#define CYAN            0x07FF
#define PURPLE          0xF81F

#define DIRECTION0      0
#define DIRECTION90     1
#define DIRECTION180    2
#define DIRECTION270    3

struct LcdWindow {
        int offsetx;
        int offsety;
        int width;
        int height;
        bool boarder;
        uint16_t color;
};


void lcdWriteCommandByte(uint8_t c);
void lcdWriteDataByte(uint8_t c);
void lcdWriteDataWord(uint16_t w);
struct LcdWindow* lcdInit(int offsetx, int offsety, int width, int, bool vertical);
struct LcdWindow* lcdWindowInit(int offsetx, int offsety, int width, int height);
void lcdWindowRemove(struct LcdWindow* windowHandle);
void lcdReset(void);
void lcdSetup(void);
void lcdDrawPixel(struct LcdWindow* windowHandle, uint16_t x, uint16_t y, uint16_t color);
void lcdDrawFillRect(struct LcdWindow* windowHandle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDisplayOff(void);
void lcdDisplayOn(void);
void lcdInversionOn(void);
void lcdFillScreen(struct LcdWindow* windowHandle, uint16_t color);
void lcdDrawLine(struct LcdWindow* windowHandle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDrawRect(struct LcdWindow* windowHandle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDrawCircle(struct LcdWindow* windowHandle, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void lcdDrawFillCircle(struct LcdWindow* windowHandle, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void lcdDrawRoundRect(struct LcdWindow* windowHandle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color);
void lcdDrawArrow(struct LcdWindow* windowHandle, uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color);
void lcdDrawFillArrow(struct LcdWindow* windowHandle, uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color);
uint16_t rgb565_conv(uint16_t r,uint16_t g,uint16_t b);
int lcdDrawSJISChar(struct LcdWindow* windowHandle, FontxFile *fx, uint16_t x,uint16_t y,uint16_t sjis,uint16_t color);
int lcdDrawUTF8Char(struct LcdWindow* windowHandle, FontxFile *fx, uint16_t x,uint16_t y,uint8_t *utf8,uint16_t color);
int lcdDrawUTF8String(struct LcdWindow* windowHandle, FontxFile *fx, uint16_t x,uint16_t y,const char *utfs,uint16_t color);
//int lcdDrawUTF8Strings(struct LcdWindow* windowHandle, FontxFile *fx, uint16_t x,uint16_t y,String text,uint16_t color);
void lcdSetFontDirection(uint16_t);
void lcdSetFontFill(uint16_t color);
void lcdUnsetFontFill(void);
void lcdSetFontUnderLine(uint16_t color);
void lcdUnsetFontUnderLine(void);
