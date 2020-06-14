// ili9340.c
// Used by bcm2835 library.(Mike McCauley)
//
// Original is http://imagewriteriij.blogspot.jp/2014/01/raspberry-pi-9-lcd-1.html
// LCD test program 20130103 by ImageWriter
//
// [Pin connection]
// ILI9340-SPI     Rpi(pin)
// ------------------------
// MISO------------MISO(21)
// LED---220ohm----3.3V( 1)
// SCK-------------SCLK(23)
// MOSI------------MOSI(19)
// D/C-------------IO02( 3)  LOW = 0 = COMMAND
// RES-------------IO03( 5)  LOW = 0 = RESET
// CS--------------CS0 (24)  LOW = 0 = Chip Select
// GND-------------0V  ( 6)
// VCC-------------3.3V( 1)
// ------------------------
//
// [SPI settings]
// ORDER  : MSB First
// MODE   : Mode0
// CLOCK  : 31.25MHz on Rpi/Rpi2, 50MHz on RPI3
// CS     : CS0
// CS_POL : LOW
//
//
#define BCM
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <string>
#ifdef WPI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif
#ifdef BCM
#include <bcm2835.h>
#endif

#include "wili9340.h"

#define D_C  2  // GPIO2=Pin#3
#define RES  3  // GPIO3=Pin#5
#define C_S  8  // GPIO8=Pin#24

#define _DEBUG_   0


uint16_t _FONT_DIRECTION_;
uint16_t _FONT_FILL_;
uint16_t _FONT_FILL_COLOR_;
uint16_t _FONT_UNDER_LINE_;
uint16_t _FONT_UNDER_LINE_COLOR_;

int _width;
int _height;
int _offsetx;
int _offsety;
bool _vertical=false;

#ifdef WPI
// Write Command 8Bit
// D/C=LOW then,write command(8bit)
void lcdWriteCommandByte(uint8_t c){
        digitalWrite(D_C, LOW);
        wiringPiSPIDataRW(0, &c, 1);
}

// Write Data 8Bit
// D/C=HIGH then,write data(8bit)
void lcdWriteDataByte(uint8_t c){
        digitalWrite(D_C, HIGH);
        wiringPiSPIDataRW(0, &c, 1);
}

// Write Data 16Bit
void lcdWriteDataWord(uint16_t w){
        uint8_t hi,lo;
        hi = (w >> 8) & 0xFF;
        lo = w & 0xFF;
        lcdWriteDataByte(hi);
        lcdWriteDataByte(lo);
}

// Write Tow Data 8Bit
void lcdWriteAddr(uint8_t addr1, uint8_t addr2){
        uint8_t byte[4];
        byte[0] = (addr1 >> 8) & 0xFF;
        byte[1] = addr1 & 0xFF;
        byte[2] = (addr2 >> 8) & 0xFF;
        byte[3] = addr2 & 0xFF;
        digitalWrite(D_C, HIGH);
        wiringPiSPIDataRW(0, byte, 4);
}

// Write Data 16Bit
void lcdWriteColor(uint16_t color, uint16_t size) {
        uint8_t byte[1024];
        int index=0;
        int i;
        for(i=0; i<size; i++) {
                byte[index++] = (color >> 8) & 0xFF;
                byte[index++] = color & 0xFF;
        }
        digitalWrite(D_C, HIGH);
        wiringPiSPIDataRW(0, byte, size*2);
}
#endif

#ifdef BCM
// Write Command
// D/C=LOW then,write command(8bit)
void lcdWriteCommandByte(uint8_t c){
        bcm2835_gpio_write(D_C, LOW);
        bcm2835_spi_transfer(c);
}

// Write Data 8Bit
// D/C=HIGH then,write data(8bit)
void lcdWriteDataByte(uint8_t c){
        bcm2835_gpio_write(D_C, HIGH);
        bcm2835_spi_transfer(c);
}

// Write Data 16Bit
void lcdWriteDataWord(uint16_t w){
        bcm2835_gpio_write(D_C, HIGH);
        bcm2835_spi_write(w);
}

// Write Tow Data 8Bit
void lcdWriteAddr(uint8_t addr1, uint8_t addr2){
        bcm2835_gpio_write(D_C, HIGH);
        bcm2835_spi_write(addr1);
        bcm2835_spi_write(addr2);
}

// Write Data 16Bit
void lcdWriteColor(uint16_t color, uint16_t size) {
        bcm2835_gpio_write(D_C, HIGH);
        int i;
        for(i=0; i<size; i++) bcm2835_spi_write(color);
}
#endif


#ifdef WPI
// SPI interfase initialize
// MSB,mode0,clock=8,cs0=low
struct LcdWindow* lcdInit(int offsetx, int offsety, int width, int height, bool vertical){
        _width = width;
        _height = height;
        _offsetx = offsetx;
        _offsety = offsety;
        _vertical = vertical;

        if (wiringPiSetupGpio() == -1) {
                printf("wiringPiSetup Error\n");
                return 0;
        }
        wiringPiSPISetup(0, 16000000);
//  wiringPiSPISetup(0, 32000000);

        _FONT_DIRECTION_ = vertical ? DIRECTION0 : DIRECTION90;
        _FONT_FILL_ = false;
        _FONT_UNDER_LINE_ = false;

        struct LcdWindow *lcdWindow;
        lcdWindow=(LcdWindow*) malloc(sizeof(struct LcdWindow));
        lcdWindow->width=width;
        lcdWindow->width=height;
        lcdWindow->offsetx=offsetx;
        lcdWindow->offsety=offsety;
        return lcdWindow;

}
void lcdReset(void){
        pinMode(D_C, OUTPUT);
        pinMode(RES, OUTPUT);
        pinMode(C_S, OUTPUT);
        digitalWrite(D_C, HIGH);
        digitalWrite(C_S, LOW);

        digitalWrite(RES, LOW);
        delay(100);
        digitalWrite(RES, HIGH);
        delay(100);
}
#endif

#ifdef BCM
// SPI interfase initialize
// MSB,mode0,clock=8,cs0=low
struct LcdWindow* lcdInit(int offsetx, int offsety, int width, int height, bool vertical){
        _width = width;
        _height = height;
        _offsetx = offsetx;
        _offsety = offsety;
        _vertical = vertical;
        if (bcm2835_init() == -1) {
                printf("bmc2835_init Error\n");
                return 0;
        }

        bcm2835_spi_begin();
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8);
        bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
        // Send a byte to the slave and simultaneously read a byte back from the slave
        // If you tie MISO to MOSI, you should read back what was sent

        _FONT_DIRECTION_ = _vertical ? DIRECTION0 : DIRECTION90;
        _FONT_FILL_ = false;
        _FONT_UNDER_LINE_ = false;

        struct LcdWindow *lcdWindow;
        lcdWindow=(LcdWindow*)malloc(sizeof(struct LcdWindow));
        lcdWindow->width=width;
        lcdWindow->height=height;
        lcdWindow->offsetx=offsetx;
        lcdWindow->offsety=offsety;
        return lcdWindow;

}

// TFT Reset
void lcdReset(void){
        bcm2835_gpio_fsel(D_C,BCM2835_GPIO_FSEL_OUTP); // D/C
        bcm2835_gpio_fsel(RES,BCM2835_GPIO_FSEL_OUTP); // Reset
        //bcm2835_gpio_write(D_C, HIGH);

        bcm2835_gpio_write(RES, LOW);
        bcm2835_delay(100);
        bcm2835_gpio_write(RES, HIGH);
        bcm2835_delay(100);
}
#endif


struct LcdWindow* lcdWindowInit(int offsetx, int offsety, int width, int height){
        struct LcdWindow *lcdWindow;
        lcdWindow=(LcdWindow*)malloc(sizeof(struct LcdWindow));
        ((width+offsetx) > _width) ? (lcdWindow->width=_width) : (lcdWindow->width=width);
        ((height+offsety) > _height) ? (lcdWindow->height=_height) : (lcdWindow->height=height);
        (offsetx > _width) ? (lcdWindow->offsetx=_width) : (lcdWindow->offsetx=offsetx);
        (offsety > _height) ? (lcdWindow->offsety=_height) : (lcdWindow->offsety=offsety);
        return lcdWindow;
}

void lcdWindowRemove(struct LcdWindow* windowHandle){
        if (!windowHandle) return;
        free(windowHandle);
}

// TFT initialize
void lcdSetup(void){
        lcdWriteCommandByte(0xC0); //Power Control 1
        lcdWriteDataByte(0x23);

        lcdWriteCommandByte(0xC1); //Power Control 2
        lcdWriteDataByte(0x10);

        lcdWriteCommandByte(0xC5); //VCOM Control 1
        lcdWriteDataByte(0x3e);
        lcdWriteDataByte(0x28);

        lcdWriteCommandByte(0xC7); //VCOM Control 2
        lcdWriteDataByte(0x86);

        lcdWriteCommandByte(0x36); //Memory Access Control
        lcdWriteDataByte(0x48); //Left bottom start

        lcdWriteCommandByte(0x3A); //Pixel Format Set
        lcdWriteDataByte(0x55); //65K color: 16-bit/pixel

        lcdWriteCommandByte(0x20); //Display Inversion OFF

        lcdWriteCommandByte(0xB1); //Frame Rate Control
        lcdWriteDataByte(0x00);
        lcdWriteDataByte(0x18);

        lcdWriteCommandByte(0xB6); //Display Function Control
        lcdWriteDataByte(0x08);
        lcdWriteDataByte(0xA2);
        lcdWriteDataByte(0x27);
        lcdWriteDataByte(0x00);

        lcdWriteCommandByte(0xF2); //3Gamma Function Disable
        lcdWriteDataByte(0x00);

        lcdWriteCommandByte(0x26); //Gamma Set
        lcdWriteDataByte(0x01);

        lcdWriteCommandByte(0xE0); //Positive Gamma Correction
        lcdWriteDataByte(0x0F);
        lcdWriteDataByte(0x31);
        lcdWriteDataByte(0x2B);
        lcdWriteDataByte(0x0C);
        lcdWriteDataByte(0x0E);
        lcdWriteDataByte(0x08);
        lcdWriteDataByte(0x4E);
        lcdWriteDataByte(0xF1);
        lcdWriteDataByte(0x37);
        lcdWriteDataByte(0x07);
        lcdWriteDataByte(0x10);
        lcdWriteDataByte(0x03);
        lcdWriteDataByte(0x0E);
        lcdWriteDataByte(0x09);
        lcdWriteDataByte(0x00);

        lcdWriteCommandByte(0XE1); //Negative Gamma Correction
        lcdWriteDataByte(0x00);
        lcdWriteDataByte(0x0E);
        lcdWriteDataByte(0x14);
        lcdWriteDataByte(0x03);
        lcdWriteDataByte(0x11);
        lcdWriteDataByte(0x07);
        lcdWriteDataByte(0x31);
        lcdWriteDataByte(0xC1);
        lcdWriteDataByte(0x48);
        lcdWriteDataByte(0x08);
        lcdWriteDataByte(0x0F);
        lcdWriteDataByte(0x0C);
        lcdWriteDataByte(0x31);
        lcdWriteDataByte(0x36);
        lcdWriteDataByte(0x0F);

        lcdWriteCommandByte(0x11); //Sleep Out
#ifdef BCM
        bcm2835_delay(200);
#endif
#ifdef WPI
        delay(200);
#endif

        lcdWriteCommandByte(0x29); //Display ON
}

// Draw pixel
// x:X coordinate
// y:Y coordinate
// color:color
void lcdDrawPixel(struct LcdWindow* windowHandle,uint16_t x, uint16_t y, uint16_t color)
{
        if (!windowHandle) return;
        if (x >= windowHandle->width || x < 0 ) return;
        if (y >= windowHandle->height || y < 0 ) return;

        uint16_t _x = _vertical ? (x + windowHandle->offsetx) : (_width - (x + windowHandle->offsetx));
        uint16_t _y = _height - (y + windowHandle->offsety);
        //uint16_t _y = (y + windowHandle->offsety);
        lcdWriteCommandByte(0x2A); // set column(x) address
        lcdWriteDataWord(_vertical ? _x : _y);
        lcdWriteDataWord(_vertical ? _x : _y);
        lcdWriteCommandByte(0x2B); // set Page(y) address
        lcdWriteDataWord(_vertical ? _y : _x);
        lcdWriteDataWord(_vertical ? _y : _x);
        lcdWriteCommandByte(0x2C); // Memory Write
        lcdWriteDataWord(color);
}

// Draw rectangule of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
void lcdDrawFillRect(struct LcdWindow* windowHandle,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
        if (!windowHandle) return;
        int i,j;
        if (x1 >= windowHandle->width) return;
        if (x2 >= windowHandle->width) x2=windowHandle->width;
        if (y1 >= windowHandle->width) return;
        if (y2 >= windowHandle->height) y2=windowHandle->height;
        if(x2<=x1) return;
        if(y2<=y1) return;

        uint16_t _x1 = x1 + windowHandle->offsetx;
        uint16_t _x2 = x2 + windowHandle->offsetx;
        uint16_t _y1 = _height - (y2 + windowHandle->offsety);
        uint16_t _y2 = _height - (y1 + windowHandle->offsety);

        //for (int x = )
        //printf("xxxxxxx _x1 %i,_y1 %i, _x2 %i, _y2 %i\n", _x1,_y1,_x2,_y2);

        _x1 = _vertical ? _x1 : _width - _x1 -1;
        _x2 = _vertical ? _x2 : _width - _x2 -1;
        //printf("xxxxxxx _x1 %i,_y1 %i, _x2 %i, _y2 %i\n", _x1,_y1,_x2,_y2);

        lcdWriteCommandByte(0x2A); // set column(x) address
        lcdWriteDataWord(_vertical ? _x1 : _y1);
        lcdWriteDataWord(_vertical ? _x2 : _y2);
        //lcdWriteAddr(_x1, _x2); // Don't work
        lcdWriteCommandByte(0x2B); // set Page(y) address
        lcdWriteDataWord(_vertical ? _y1 : _x2);
        lcdWriteDataWord(_vertical ? _y2 : _x1);
        //lcdWriteAddr(_y1, _y2); // Don't work
        lcdWriteCommandByte(0x2C); // Memory Write

        for(i=_vertical ? _x1 : _x2; i<=(_vertical ? _x2 : _x1); i++) {
                uint16_t size = _vertical ? (y2-y1+1) : (_x1-_x2+1);
                //printf("xxxxxxx size is %i\n", size);
                lcdWriteColor(color, size);
        }

}

// Display Off
void lcdDisplayOff(void)
{
        lcdWriteCommandByte(0x28); //Display OFF
}

// Display On
void lcdDisplayOn(void)
{
        lcdWriteCommandByte(0x29); //Display ON
}

// Display Inversion On
void lcdInversionOn(void)
{
        lcdWriteCommandByte(0x21); //Display Inversion ON
}

// Fill screen
// color:color
void lcdFillScreen(struct LcdWindow* windowHandle,uint16_t color)
{
        if (!windowHandle) return;
        lcdDrawFillRect(windowHandle,0, 0, _width-1, _height-1, color);
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
void lcdDrawLine(struct LcdWindow* windowHandle,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
        if (!windowHandle) return;
        int i;
        int dx,dy;
        int sx,sy;
        int E;

        /* distance between two points */
        dx = ( x2 > x1 ) ? x2 - x1 : x1 - x2;
        dy = ( y2 > y1 ) ? y2 - y1 : y1 - y2;

        /* direction of two point */
        sx = ( x2 > x1 ) ? 1 : -1;
        sy = ( y2 > y1 ) ? 1 : -1;

        /* inclination < 1 */
        if ( dx > dy ) {
                E = -dx;
                for ( i = 0; i <= dx; i++ ) {
                        lcdDrawPixel(windowHandle,x1, y1, color );
                        x1 += sx;
                        E += 2 * dy;
                        if ( E >= 0 ) {
                                y1 += sy;
                                E -= 2 * dx;
                        }
                }
                /* inclination >= 1 */
        } else {
                E = -dy;
                for ( i = 0; i <= dy; i++ ) {
                        lcdDrawPixel(windowHandle,x1, y1, color );
                        y1 += sy;
                        E += 2 * dx;
                        if ( E >= 0 ) {
                                x1 += sx;
                                E -= 2 * dy;
                        }
                }
        }
}

// Draw rectangule
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color
void lcdDrawRect(struct LcdWindow* windowHandle,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
        if (!windowHandle) return;
        lcdDrawLine(windowHandle,x1,y1,x2,y1,color);
        lcdDrawLine(windowHandle,x2,y1,x2,y2,color);
        lcdDrawLine(windowHandle,x2,y2,x1,y2,color);
        lcdDrawLine(windowHandle,x1,y2,x1,y1,color);
}

// Draw round
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawCircle(struct LcdWindow* windowHandle, uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
        if (!windowHandle) return;
        int x;
        int y;
        int err;
        int old_err;

        x=0;
        y=-r;
        err=2-2*r;
        do {
                lcdDrawPixel(windowHandle,x0-x,y0+y,color);
                lcdDrawPixel(windowHandle,x0-y,y0-x,color);
                lcdDrawPixel(windowHandle,x0+x,y0-y,color);
                lcdDrawPixel(windowHandle,x0+y,y0+x,color);
                if ((old_err=err)<=x) err+=++x*2+1;
                if (old_err>y || err>x) err+=++y*2+1;
        } while(y<0);

}

// Draw round of filling
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawFillCircle(struct LcdWindow* windowHandle,uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
        if (!windowHandle) return;
        int x;
        int y;
        int err;
        int old_err;
        int ChangeX;

        x=0;
        y=-r;
        err=2-2*r;
        ChangeX=1;
        do {
                if(ChangeX) {
                        lcdDrawLine(windowHandle,x0-x,y0-y,x0-x,(y0>-y) ? (y0+y) : 0,color);
                        lcdDrawLine(windowHandle,x0+x,y0-y,x0+x,(y0>-y) ? (y0+y) : 0,color);
                } // if
                ChangeX=(old_err=err)<=x;
                if (ChangeX) err+=++x*2+1;
                if (old_err>y || err>x) err+=++y*2+1;

        } while(y<=0);

}

// Draw rectangule with round corner
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// r:radius
// color:color
void lcdDrawRoundRect(struct LcdWindow* windowHandle,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color)
{
        if (!windowHandle) return;
        int x;
        int y;
        int err;
        int old_err;
        unsigned char temp;

        if(x1>x2) {
                temp=x1;
                x1=x2;
                x2=temp;
        }

        if(y1>y2) {
                temp=y1;
                y1=y2;
                y2=temp;
        }
        if (x2 - x1 < r) return; // Add 20190517
        if (y2 - y1 < r) return; // Add 20190517

        x=0;
        y=-r;
        err=2-2*r;

        do {
                if(x) {
                        lcdDrawPixel(windowHandle,x1+r-x,y1+r+y,color);
                        lcdDrawPixel(windowHandle,x2-r+x,y1+r+y,color);
                        lcdDrawPixel(windowHandle,x1+r-x,y2-r-y,color);
                        lcdDrawPixel(windowHandle,x2-r+x,y2-r-y,color);
                } // if
                if ((old_err=err)<=x) err+=++x*2+1;
                if (old_err>y || err>x) err+=++y*2+1;
        } while(y<0);

        lcdDrawLine(windowHandle,x1+r,y1,x2-r,y1,color);
        lcdDrawLine(windowHandle,x1+r,y2,x2-r,y2,color);
        lcdDrawLine(windowHandle,x1,y1+r,x1,y2-r,color);
        lcdDrawLine(windowHandle,x2,y1+r,x2,y2-r,color);
}

// Draw arrow
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
// Thanks http://k-hiura.cocolog-nifty.com/blog/2010/11/post-2a62.html
void lcdDrawArrow(struct LcdWindow* windowHandle,uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color)
{
        if (!windowHandle) return;

        double Vx= x1 - x0;
        double Vy= y1 - y0;
        double v = sqrt(Vx*Vx+Vy*Vy);

        double Ux= Vx/v;
        double Uy= Vy/v;

        uint16_t L[2],R[2];
        L[0]= x1 - Uy*w - Ux*v;
        L[1]= y1 + Ux*w - Uy*v;
        R[0]= x1 + Uy*w - Ux*v;
        R[1]= y1 - Ux*w - Uy*v;


//   lcdDrawLine(x0,y0,x1,y1,color);
        lcdDrawLine(windowHandle,x1,y1,L[0],L[1],color);
        lcdDrawLine(windowHandle,x1,y1,R[0],R[1],color);
        lcdDrawLine(windowHandle,L[0],L[1],R[0],R[1],color);
}


// Draw arrow of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
void lcdDrawFillArrow(struct LcdWindow* windowHandle,uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color)
{
        if (!windowHandle) return;
        double Vx= x1 - x0;
        double Vy= y1 - y0;
        double v = sqrt(Vx*Vx+Vy*Vy);

        double Ux= Vx/v;
        double Uy= Vy/v;

        uint16_t L[2],R[2];
        L[0]= x1 - Uy*w - Ux*v;
        L[1]= y1 + Ux*w - Uy*v;
        R[0]= x1 + Uy*w - Ux*v;
        R[1]= y1 - Ux*w - Uy*v;

        lcdDrawLine(windowHandle,x0,y0,x1,y1,color);
        lcdDrawLine(windowHandle,x1,y1,L[0],L[1],color);
        lcdDrawLine(windowHandle,x1,y1,R[0],R[1],color);
        lcdDrawLine(windowHandle,L[0],L[1],R[0],R[1],color);

        int ww;
        for(ww=w-1; ww>0; ww--) {
                L[0]= x1 - Uy*ww - Ux*v;
                L[1]= y1 + Ux*ww - Uy*v;
                R[0]= x1 + Uy*ww - Ux*v;
                R[1]= y1 - Ux*ww - Uy*v;
                lcdDrawLine(windowHandle,x1,y1,L[0],L[1],color);
                lcdDrawLine(windowHandle,x1,y1,R[0],R[1],color);
        }
}


// RGB565 conversion
// RGB565 is R(5)+G(6)+B(5)=16bit color format.
// Bit image "RRRRRGGGGGGBBBBB"
uint16_t rgb565_conv(uint16_t r,uint16_t g,uint16_t b)
{
        return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}


// Draw SJIS character
// x:X coordinate
// y:Y coordinate
// sjis: SJIS code
// color:color
int lcdDrawSJISChar(struct LcdWindow* windowHandle,FontxFile *fx, uint16_t x,uint16_t y,uint16_t sjis,uint16_t color)
{
        if (!windowHandle) return 0;
        uint16_t xx,yy,bit,ofs;
        unsigned char fonts[128]; // font pattern
        unsigned char pw, ph;
        int h,w;
        uint16_t mask;
        bool rc;

        if(_DEBUG_) printf("_FONT_DIRECTION_=%d\n",_FONT_DIRECTION_);
//  sjis = UTF2SJIS(utf8);

        rc = GetFontx(fx, sjis, fonts, &pw, &ph); // SJIS -> Font pattern
        if(_DEBUG_) printf("GetFontx rc=%d pw=%d ph=%d\n",rc,pw,ph);
        if (!rc) return 0;

        uint16_t xd1, yd1;
        uint16_t xd2, yd2;
        uint16_t xss, yss;
        uint16_t xsd, ysd;
        int next;
        uint16_t x0 = 0;
        uint16_t x1 = 0;
        uint16_t y0 = 0;
        uint16_t y1 = 0;
        if (_FONT_DIRECTION_ == 0) {
                xd1 = 1;
                yd1 = 1; //Negated to fix co-ord system
                xd2 =  0;
                yd2 =  0;
                xss =  x;
                //yss =  y + (ph - 1);
                yss =  y;
                xsd =  1;
                ysd =  0;
                next = x + pw;

                x0 = x;
                y0 = y;
                x1 = x + (pw-1);
                y1 = y + (ph-1);
        } else if (_FONT_DIRECTION_ == 2) {
                xd1 = -1;
                yd1 = -1;
                xd2 =  0;
                yd2 =  0;
                xss =  x;
                yss =  y - (ph + 1);
                xsd =  1;
                ysd =  0;
                next = x - pw;

                x0 = x - (pw-1);
                y0 = y - (ph-1);
                x1 = x;
                y1 = y;
        } else if (_FONT_DIRECTION_ == 1) {
                xd1 =  0;
                yd1 =  0;
                xd2 = -1;
                yd2 = -1;
                xss =  x + (ph - 1); // Bug Fix
                yss =  y;
                xsd =  0;
                ysd =  1;
                next = y - pw;

                x0 = x;
                y0 = y - (pw-1);
                x1 = x + (ph-1);
                y1 = y;
        } else if (_FONT_DIRECTION_ == 3) {
                xd1 =  0;
                yd1 =  0;
                xd2 = +1;
                yd2 = +1;
                xss =  x - (ph - 1); // Bug Fix
                yss =  y;
                xsd =  0;
                ysd =  1;
                next = y + pw;

                x0 = x - (ph-1);
                y0 = y;
                x1 = x;
                y1 = y + (pw-1);
        }
        if (_FONT_FILL_) lcdDrawFillRect(windowHandle,x0, y0, x1, y1, _FONT_FILL_COLOR_);

        int bits;
        if(_DEBUG_) printf("xss=%d yss=%d\n",xss,yss);
        ofs = 0;
        yy = yss;
        xx = xss;
        for(h=0; h<ph; h++) {
                if(xsd) xx = xss;
                if(ysd) yy = yss;
//    for(w=0;w<(pw/8);w++) {
                bits = pw;
                for(w=0; w<((pw+4)/8); w++) {
                        mask = 0x80;
                        for(bit=0; bit<8; bit++) {
                                bits--;
                                if (bits < 0) continue;
//if(_DEBUG_)printf("xx=%d yy=%d mask=%02x fonts[%d]=%02x\n",xx,yy,mask,ofs,fonts[ofs]);
                                if (fonts[ofs] & mask) {
                                        lcdDrawPixel(windowHandle,xx,yy,color);
                                } else {
                                        //if (_FONT_FILL_) lcdDrawPixel(xx,yy,_FONT_FILL_COLOR_);
                                }
                                if (h == (ph-2) && _FONT_UNDER_LINE_)
                                        lcdDrawPixel(windowHandle,xx,yy,_FONT_UNDER_LINE_COLOR_);
                                if (h == (ph-1) && _FONT_UNDER_LINE_)
                                        lcdDrawPixel(windowHandle,xx,yy,_FONT_UNDER_LINE_COLOR_);

                                xx = xx + xd1;
                                yy = yy + yd2;
                                mask = mask >> 1;
                        }
                        ofs++;
                }
                yy = yy + yd1;
                xx = xx + xd2;

        }

        if (next < 0) next = 0;
        return next;
}

// Draw UTF8 character
// x:X coordinate
// y:Y coordinate
// utf8: UTF8 code
// color:color
int lcdDrawUTF8Char(struct LcdWindow* windowHandle,FontxFile *fx, uint16_t x,uint16_t y,uint8_t *utf8,uint16_t color)
{
        if (!windowHandle) return 0;
        uint16_t sjis[1];

        sjis[0] = UTF2SJIS(utf8);
        if(_DEBUG_) printf("sjis=%04x\n",sjis[0]);
        return lcdDrawSJISChar(windowHandle,fx,x,y,sjis[0],color);
}
/*
int lcdDrawUTF8Strings(struct LcdWindow* windowHandle,FontxFile *fx, uint16_t x,uint16_t y,String text,uint16_t color)
{
        unsigned char utf8[256];
        if (!windowHandle) return 0;
        strncpy((char *)utf8, text, sizeof(utf8));
        lcdDrawUTF8String(windowHandle,fx, x,y,utf8,color);

        return 0;
}*/

// Draw UTF8 string
// x:X coordinate
// y:Y coordinate
// utfs: UTF8 string
// color:color
int lcdDrawUTF8String(struct LcdWindow* windowHandle,FontxFile *fx, uint16_t x,uint16_t y,const char *text,uint16_t color)
{
        if (!windowHandle) return 0;

        if(_DEBUG_) printf("lcdDrawUTF8String start x=%d y=%d\n",x,y);
        int i;
        int spos;
        uint16_t sjis[64];
        unsigned char utfs[64];
        strncpy((char *)utfs, text, sizeof(utfs));
        spos = String2SJIS(utfs, strlen((char *)utfs), sjis, 64);
        if(_DEBUG_) printf("spos=%d\n",spos);
        for(i=0; i<spos; i++) {
                if(_DEBUG_) printf("sjis[%d]=%x y=%d\n",i,sjis[i],y);
                if (_FONT_DIRECTION_ == 0)
                        x=lcdDrawSJISChar(windowHandle,fx,x,y,sjis[i],color);
                if (_FONT_DIRECTION_ == 1)
                        y=lcdDrawSJISChar(windowHandle,fx,x,y,sjis[i],color);
                if (_FONT_DIRECTION_ == 2)
                        x=lcdDrawSJISChar(windowHandle,fx,x,y,sjis[i],color);
                if (_FONT_DIRECTION_ == 3)
                        y=lcdDrawSJISChar(windowHandle,fx,x,y,sjis[i],color);
        }
        if(_DEBUG_) printf("lcdDrawUTF8String end x=%d y=%d\n",x,y);
        if (_FONT_DIRECTION_ == 0) return x;
        if (_FONT_DIRECTION_ == 2) return x;
        if (_FONT_DIRECTION_ == 1) return y;
        if (_FONT_DIRECTION_ == 3) return y;
        return 0;
}

// Set font direction
// dir:Direction
void lcdSetFontDirection(uint16_t dir)
{
        _FONT_DIRECTION_ = dir;
}

// Set font filling
// color:fill color
void lcdSetFontFill(uint16_t color)
{
        _FONT_FILL_ = true;
        _FONT_FILL_COLOR_ = color;
}

// UnSet font filling
void lcdUnsetFontFill(void)
{
        _FONT_FILL_ = false;
}

// Set font underline
// color:frame color
void lcdSetFontUnderLine(uint16_t color)
{
        _FONT_UNDER_LINE_ = true;
        _FONT_UNDER_LINE_COLOR_ = color;
}

// UnSet font filling
void lcdUnsetFontUnderLine(void)
{
        _FONT_UNDER_LINE_ = false;
}
