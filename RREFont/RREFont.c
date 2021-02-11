// RRE Font support library
// (c) 2019 by Pawel A. Hernik

#include <stdarg.h>
#include <stdio.h>
#include "RREFont.h"

#define max(x,y) ((x)>(y)?(x):(y))
#define isdigit(x) ((x)>='0' && (x)<='9')

static bool (*isNumberFun)(unsigned char ch);
static void (*fillRectFun)(int x, int y, int w, int h, int c);
static RRE_Font *rFont;
static int xf,yf,wf,hf;
static int scrWd, scrHt;

static int spacing = 1;
static int spacingY = 0;
static int bold = 0;
static int sx = 1;
static int sy = 1;
static int cr = 0;
static int fg = 1;
static int bg = 1;
static unsigned char minCharWd;
static unsigned char minDigitWd;
static unsigned char dualChar;

void font_setScale2(unsigned char _sx, unsigned char _sy) { sx = _sx; sy = _sy; }
void font_setScale1(unsigned char s) { sx = sy = s; }
void font_setCR(unsigned char _cr) { cr = _cr; }
void font_setFg(int _fg) { fg = _fg; }
void font_setBg(int _bg) { bg = _bg; }
void font_setColor1(int c) { fg = bg = c; }
void font_setColor2(int c, int _bg) { fg = c; bg = _bg; }
void font_setBold(unsigned char _bold) { bold = _bold; }
void font_setSpacing(unsigned char sp) { spacing = sp; }
void font_setSpacingY(unsigned char sp) { spacingY = sp; }
void font_setFontMinWd(unsigned char wd) { minCharWd = wd; }
void font_setCharMinWd(unsigned char wd) { minCharWd = wd; }
void font_setDigitMinWd(unsigned char wd) { minDigitWd = wd; }
int font_getWidth() { return rFont ? rFont->wd : 0; }
int font_getHeight() { return rFont ? rFont->ht : 0; }
void font_setIsNumberFun(bool (*fun)(unsigned char)) { isNumberFun=fun; }
void font_setFillRectFun(void (*fun)(int x, int y, int w, int h, int c)) { fillRectFun=fun; }

// ----------------------------------------------------------------
// fillRect function callback and screen width and height is necessary to initialize the library
void font_init(void (*rectFun)(int x, int y, int w, int h, int c), int swd, int sht)
{
  fillRectFun = rectFun;
  scrWd = swd;
  scrHt = sht;
  cr = bold = dualChar = 0;
  isNumberFun = &font_isNumber;
  sx = sy = 1;
  fg = bg = 1;
}

// ---------------------------------
void font_setFont(RRE_Font *f)
{
  rFont = f;
  minCharWd = minDigitWd = 0;
  spacing = 1;
  spacingY = 0;
}

// ---------------------------------
int font_charWidthNoSort(unsigned char c, int *_xmin)
{
  if(c<rFont->firstCh || c>rFont->lastCh) return c==' ' ? rFont->wd/2 : 0;
  unsigned short recIdx = rFont->offs[c-rFont->firstCh];
  unsigned short recNum = rFont->offs[c-rFont->firstCh+1]-recIdx;
  if(!recNum) return rFont->wd/2;
  int i, xmin = rFont->wd, xmax = 0;
  switch(rFont->type & 7) {
    case RRE_16B:
      for(i=0; i<recNum; i++) {
        unsigned short *rects = (unsigned short*)rFont->rects;
        int v = rects[i+recIdx];
        xf = v & 0x000f;
        wf = 1+((v & 0x0f00)>>8);
        if(xf<xmin) xmin = xf;
        if(xf+wf>xmax) xmax = xf+wf;
      }
      break;
    case RRE_24B:
      for(i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*3;
        xf = rects[0] & 0x3f;
        wf = 1+(rects[2] & 0x3f);
        if(xf<xmin) xmin = xf;
        if(xf+wf>xmax) xmax = xf+wf;
      }
      break;
    case RRE_32B:
      for(i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*4;
        xf = rects[0];
        wf = rects[2]+1;
        if(xf<xmin) xmin = xf;
        if(xf+wf>xmax) xmax = xf+wf;
      }
      break;
    case RRE_V24B:
      for(i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*3;
        xf = rects[0];
        wf = 1;
        if(xf<xmin) xmin = xf;
        if(xf+wf>xmax) xmax = xf+wf;
      }
      break;
    case RRE_H24B:
      for(i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*3;
        xf = rects[0];
        wf = rects[2]+1;
        if(xf<xmin) xmin = xf;
        if(xf+wf>xmax) xmax = xf+wf;
      }
      break;
    default: break;
  }
  //Serial.println(String((char)c)+" xmin="+String(xmin)+" xmax="+xmax);
  if(!xmax) { xmax=rFont->wd; xmin=0; }
  if(_xmin) *_xmin=xmin;
  return xmax-xmin;
}

// ----------------------------------------------------------------
int font_charWidthOptim(unsigned char c, int *_xmin)
{
  if(c<rFont->firstCh || c>rFont->lastCh) return c==' '? rFont->wd/2 : 0;
  unsigned short recIdx = ((rFont->offs[c-rFont->firstCh]));
  unsigned short recNum = ((rFont->offs[c-rFont->firstCh+1]))-recIdx;
  if(!recNum) return rFont->wd/2;
  if(_xmin) *_xmin=0;
  int chWd = rFont->wd;
  switch(rFont->type & 7) {
#if ENABLE_RRE_16B==1
    case RRE_16B: {
      unsigned short *rects = (unsigned short*)rFont->rects;
      unsigned short v = (rects[recNum-1+recIdx]);
      chWd = (v & 0x000f)+1+((v & 0x0f00)>>8);
      }
      break;
#endif
#if ENABLE_RRE_24B==1
    case RRE_24B: {
      unsigned char *rects = (unsigned char*)rFont->rects + (recNum-1+recIdx)*3;
      chWd = ((rects[0]) & 0x3f)+((rects[2]) & 0x3f)+1;
      }
      break;
#endif
#if ENABLE_RRE_32B==1
    case RRE_32B: {
      unsigned char *rects = (unsigned char*)rFont->rects + (recNum-1+recIdx)*4;
      chWd = (rects[0])+(rects[2])+1;
      }
      break;
#endif
#if ENABLE_RRE_V16B==1
    case RRE_V16B: {
      unsigned short *rects = (unsigned short*)rFont->rects;
      unsigned short v = (rects[recNum-1+recIdx]);
      chWd = (v & 0x3f)+1;
      }
      break;
#endif
#if ENABLE_RRE_H16B==1
    case RRE_H16B: {
      unsigned short *rects = (unsigned short*)rFont->rects;
      unsigned short v = (rects[recNum-1+recIdx]);
      chWd = (v & 0x1f)+((v>>11) & 0x1f)+1;
      }
      break;
#endif
#if ENABLE_RRE_V24B==1
    case RRE_V24B: {
      unsigned char *rects = (unsigned char*)rFont->rects + (recNum-1+recIdx)*3;
      chWd = (rects[0])+1;
      }
      break;
#endif
#if ENABLE_RRE_H24B==1
    case RRE_H24B: {
      unsigned char *rects = (unsigned char*)rFont->rects + (recNum-1+recIdx)*3;
      chWd = (rects[0])+(rects[2])+1;
      }
      break;
#endif
    default: break;
  }
  return chWd;
}

// ----------------------------------------------------------------
int font_charWidth(unsigned char c)
{
#if CONVERT_PL_CHARS==1
  c = convertPolish(c);
#endif
#if ENABLE_NOSORT==1
  int wd = (rFont->type & RRE_NO_SORT) ? charWidthNoSort(c,_xmin) : charWidthOptim(c,_xmin);
#else
  int wd = font_charWidthOptim(c, 0);
#endif
  int wdL = 0, wdR = spacing; // default spacing before and behind char
  if((*isNumberFun)(c) && minDigitWd>0) {
    if(minDigitWd>wd) {
      wdL = (minDigitWd-wd)/2;
      wdR += (minDigitWd-wd-wdL);
    }
  } else if(minCharWd>wd) {
    wdL = (minCharWd-wd)/2;
    wdR += (minCharWd-wd-wdL);
  }
  return (wd+wdL+wdR+bold)*sx;
}

// ----------------------------------------------------------------
int font_drawChar(int x, int y, unsigned char c)
{
#if CONVERT_PL_CHARS==1
  c = convertPolish(c);
#endif
  if(x>=scrWd || y>=scrHt || x+rFont->wd*sx-1<0 || y+rFont->ht*sy-1<0) return 0;
  if(c<rFont->firstCh || c>rFont->lastCh) return font_charWidth(c);
  unsigned short recIdx = ((rFont->offs[c-rFont->firstCh]));
  unsigned short recNum = ((rFont->offs[c-rFont->firstCh+1]))-recIdx;
#if ENABLE_NOSORT==1
  int xmin, chWd = (rFont->type & RRE_NO_SORT) ? font_charWidthNoSort(c,&xmin) : font_charWidthOptim(c,&xmin);
#else
  int xmin, chWd = font_charWidthOptim(c,&xmin);
#endif
  int wd=chWd, wdL = 0, wdR = spacing;
  if((*isNumberFun)(c) && minDigitWd>0) {
    if(minDigitWd>wd) {
      wdL = (minDigitWd-wd)/2;
      wdR += (minDigitWd-wd-wdL);
    }
  } else if(minCharWd>wd) {
    wdL = (minCharWd-wd)/2;
    wdR += (minCharWd-wd-wdL);
  }
  if(x+wd+wdL+wdR>scrWd) wdR = max(scrWd-x-wdL-wd, 0);
  if(x+wd+wdL+wdR>scrWd) wd  = max(scrWd-x-wdL, 0);
  if(x+wd+wdL+wdR>scrWd) wdL = max(scrWd-x, 0);
  wd+=wdR+wdL;
  int type=rFont->type & 7;
  if(bg!=fg && (type==RRE_16B ||type==RRE_24B ||type==RRE_32B || !recNum)) (*fillRectFun)(x, y, (wd+bold)*sx, rFont->ht*sy, bg);
  x+=wdL*sx;
  if(!recNum) return (wd+bold)*sx;
  switch(type) {
#if ENABLE_RRE_16B==1
    case RRE_16B:
      for(int i=0; i<recNum; i++) {
        unsigned short *rects = (unsigned short*)rFont->rects;
        unsigned short v = (rects[i+recIdx]);
        xf = (v & 0x000f)-xmin;
        yf = (v & 0x00f0)>>4;
        wf = ((v & 0x0f00)>>8)+1;
        hf = ((v & 0xf000)>>12)+1;
        (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, hf*sy, fg);
        //Serial.println(String(i)+" "+xf+" "+yf+" "+wf+" "+hf);
      }
      break;
#endif
#if ENABLE_RRE_24B==1
    case RRE_24B:
      for(int i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*3;
        xf = ((rects[0]) & 0x3f)-xmin;
        yf = ((rects[1]) & 0x3f);
        wf = ((rects[2]) & 0x3f)+1;
        hf = 1+((((rects[0]) & 0xc0)>>6) | (((rects[1]) & 0xc0)>>4) | (((rects[2]) & 0xc0)>>2));
        (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, hf*sy, fg);
      }
      break;
#endif
#if ENABLE_RRE_32B==1
    case RRE_32B:
      for(int i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*4;
        xf = (rects[0])-xmin;
        yf = (rects[1]);
        wf = (rects[2])+1;
        hf = (rects[3])+1;
        (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, hf*sy, fg);
        //Serial.println(String(i)+" "+xf+" "+yf+" "+wf+" "+hf);
      }
      break;
#endif
#if ENABLE_RRE_V16B==1
    case RRE_V16B:
      if(bg!=fg) {
        if(wdL>0) (*fillRectFun)(x-wdL*sx, y, wdL*sx, rFont->ht*sy, bg);
        unsigned short *rects = (unsigned short*)rFont->rects+recIdx;
        int idx=0;
        while(idx<recNum) {
          unsigned short v = *(rects+idx);
          xf = (v & 0x3f)-xmin;
          int ybg=0,xfCur=xf;
          while(xf==xfCur && idx<recNum) {
            v = *(rects+idx);
            yf = (v>>6) & 0x1f;
            hf = ((v>>11) & 0x1f)+1;
            if(yf>ybg) (*fillRectFun)(x+xfCur*sx, y+ybg*sy, 1*sx, (yf-ybg)*sy, bg);
            ybg = yf+hf;
            (*fillRectFun)(x+xfCur*sx, y+yf*sy, bold+1*sx, hf*sy, fg);
            idx++;
            v = *(rects+idx);
            xf = (v & 0x3f)-xmin;
          }
          // last bg line
          if(ybg<rFont->ht) (*fillRectFun)(x+xfCur*sx, y+ybg*sy, bold+1*sx, (rFont->ht-ybg)*sy, bg);
        }
        if(wdR>0) (*fillRectFun)(x+chWd*sx, y, wdR*sx, rFont->ht*sy, bg);
      } else
      for(int i=0; i<recNum; i++) {
        unsigned short *rects = (unsigned short*)rFont->rects;
        unsigned short v = (rects[i+recIdx]);
        xf = (v & 0x3f)-xmin;
        yf = (v>>6) & 0x1f;
        hf = ((v>>11) & 0x1f)+1;
        wf = 1;
        (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, hf*sy, fg);
      }
      break;
#endif
#if ENABLE_RRE_H16B==1
    case RRE_H16B:
      if(bg!=fg) {
        if(wdL>0) (*fillRectFun)(x-wdL*sx, y, wdL*sx, rFont->ht*sy, bg);
        unsigned short *rects = (unsigned short*)rFont->rects;
        int idx=0,yfCur=0;
        while(idx<recNum) {
          unsigned short v = *(rects+idx+recIdx);
          yf = (v>>5) & 0x3f;
          if(yf-yfCur>1 /*|| (yf-yfCur>0 && yfCur==0)*/) (*fillRectFun)(x+0*sx, y+yfCur*sy, chWd*sx, (yf-yfCur)*sy, bg);
          //Serial.print(idx); Serial.print("  "); Serial.print(yf); Serial.print("  "); Serial.println(yfCur);
          yfCur=yf;
          int xbg=0;
          while(yf==yfCur && idx<recNum) {
            xf = v & 0x1f;
            wf = ((v>>11) & 0x1f)+1;
            if(xf>xbg) (*fillRectFun)(x+xbg*sx, y+yf*sy, (xf-xbg)*sx, 1*sy, bg);
            xbg = xf+wf;
            //if(idx==recNum-1) fg=0x07E0; else fg=0xffff;
            (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, 1*sy, fg);
            idx++;
            v = *(rects+idx+recIdx);
            yf = (v>>5) & 0x3f;
          }
          // last bg line
          if(xbg<chWd) (*fillRectFun)(x+xbg*sx, y+yfCur*sy, (chWd-xbg)*sx, 1*sy, bg);
          //Serial.print(idx); Serial.print(" ++ "); Serial.print(yf); Serial.print("  "); Serial.println(yfCur);
          if(idx==recNum-1 && yfCur<rFont->ht) (*fillRectFun)(x+0*sx, y+yfCur*sy, chWd*sx, (yfCur<rFont->ht-yfCur)*sy, bg);
        }
        if(wdR>0) (*fillRectFun)(x+chWd*sx, y, wdR*sx, rFont->ht*sy, bg);
      } else
      for(int i=0; i<recNum; i++) {
        unsigned short *rects = (unsigned short*)rFont->rects;
        unsigned short v = (rects[i+recIdx]);
        xf = (v & 0x1f)-xmin;
        yf = (v>>5) & 0x3f;
        wf = ((v>>11) & 0x1f)+1;
        hf = 1;
        (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, hf*sy, fg);
      }
      break;
#endif
#if ENABLE_RRE_V24B==1
    case RRE_V24B:
      if(bg!=fg) {
        if(wdL>0) (*fillRectFun)(x-wdL*sx, y, wdL*sx, rFont->ht*sy, bg);
        unsigned char *rects = (unsigned char*)rFont->rects + recIdx*3;
        int idx=0;
        while(idx<recNum*3) {
          xf = *(rects+idx+0);
          int ybg=0,xfCur=xf;
          while(xf==xfCur && idx<recNum*3) {
            yf = *(rects+idx+1);
            hf = *(rects+idx+2)+1;
            if(yf>ybg) (*fillRectFun)(x+xfCur*sx, y+ybg*sy, 1*sx, (yf-ybg)*sy, bg);
            ybg = yf+hf;
            (*fillRectFun)(x+xfCur*sx, y+yf*sy, bold+1*sx, hf*sy, fg);
            idx += 3;
            xf = *(rects+idx+0);
          }
          // last bg line
          if(ybg<rFont->ht) (*fillRectFun)(x+xfCur*sx, y+ybg*sy, bold+1*sx, (rFont->ht-ybg)*sy, bg);
        }
        if(wdR>0) (*fillRectFun)(x+chWd*sx, y, wdR*sx, rFont->ht*sy, bg);
      } else
      for(int i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*3;
        xf = (rects[0])-xmin;
        yf = (rects[1]);
        hf = (rects[2])+1;
        wf = 1;
        (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, hf*sy, fg);
      }
      break;
#endif
#if ENABLE_RRE_H24B==1
    case RRE_H24B:
      for(int i=0; i<recNum; i++) {
        unsigned char *rects = (unsigned char*)rFont->rects + (i+recIdx)*3;
        xf = (rects[0])-xmin;
        yf = (rects[1]);
        wf = (rects[2])+1;
        hf = 1;
        (*fillRectFun)(x+xf*sx, y+yf*sy, bold+wf*sx, hf*sy, fg);
      }
      break;
    default: break;
#endif
  }
  return (wd+bold)*sx;
}

// ----------------------------------------------------------------
int font_strWidth(const char *str)
{
  int wd = 0;
  while (*str) wd += font_charWidth(*str++);
  return wd;
}

static char printfbuffer[512];

int font_printf(int xpos, int ypos, const char *str, ...)
{
  va_list args;
  va_start(args, str);
  vsnprintf(printfbuffer, sizeof(printfbuffer), str, args);
  va_end(args);
  return font_printStr(xpos, ypos,printfbuffer);
}

// ----------------------------------------------------------------
int font_printStr(int xpos, int ypos, const char *str)
{
  unsigned char ch;
  int stl, row;
  int x = xpos;
  int y = ypos;
  int wd = font_strWidth(str);

  //fillRectFun(xpos, ypos, wd, font_getHeight(), 0);

  if(x < 0)
  {
    x = -x - wd;
  }

  if(x == 0)
    x = (scrWd - wd) / 2;

  if(x<0) x = 0; // left

  while(*str) {
    char ch = *str++;
    int wd = font_drawChar(x,y,ch);
    x+=wd;
    if((cr && x>=scrWd) || ch==10) { 
      x = cr ? 0 : xpos; 
      y += rFont->ht * sy + spacingY; 
    }
  }
  return wd;
}
// ----------------------------------------------------------------
// callbacks
bool font_isNumber(unsigned char ch)
{
  return isdigit(ch) || ch==' ' || ch=='-';
}

bool font_isNumberExt(unsigned char ch)
{
  return isdigit(ch) || ch=='-' || ch=='+' || ch=='.' || ch==' ';
}
// ----------------------------------------------------------------
unsigned char font_convertPolish(unsigned char _c)
{
  unsigned char pl, c = _c;
  if(c==196 || c==197 || c==195) {
    dualChar = c;
    return 0;
  }
  if(dualChar) { // UTF8 coding
    switch(_c) {
      case 133: pl = 1+9; break; // '�'
      case 135: pl = 2+9; break; // '�'
      case 153: pl = 3+9; break; // '�'
      case 130: pl = 4+9; break; // '�'
      case 132: pl = dualChar==197 ? 5+9 : 1; break; // '�' and '�'
      case 179: pl = 6+9; break; // '�'
      case 155: pl = 7+9; break; // '�'
      case 186: pl = 8+9; break; // '�'
      case 188: pl = 9+9; break; // '�'
      //case 132: pl = 1; break; // '�'
      case 134: pl = 2; break; // '�'
      case 152: pl = 3; break; // '�'
      case 129: pl = 4; break; // '�'
      case 131: pl = 5; break; // '�'
      case 147: pl = 6; break; // '�'
      case 154: pl = 7; break; // '�'
      case 185: pl = 8; break; // '�'
      case 187: pl = 9; break; // '�'
      default:  return c; break;
    }
    dualChar = 0;
  } else   
  switch(_c) {  // Windows coding
    case 165: pl = 1; break; // �
    case 198: pl = 2; break; // �
    case 202: pl = 3; break; // �
    case 163: pl = 4; break; // �
    case 209: pl = 5; break; // �
    case 211: pl = 6; break; // �
    case 140: pl = 7; break; // �
    case 143: pl = 8; break; // �
    case 175: pl = 9; break; // �
    case 185: pl = 10; break; // �
    case 230: pl = 11; break; // �
    case 234: pl = 12; break; // �
    case 179: pl = 13; break; // �
    case 241: pl = 14; break; // �
    case 243: pl = 15; break; // �
    case 156: pl = 16; break; // �
    case 159: pl = 17; break; // �
    case 191: pl = 18; break; // �
    default:  return c; break;
  }
  return pl+'~'+1;
}
// ---------------------------------
