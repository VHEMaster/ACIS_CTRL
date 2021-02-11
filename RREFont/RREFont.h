// RRE Font support library
// (c) 2019 by Pawel A. Hernik

#ifndef _RRE_FONT_H
#define _RRE_FONT_H 

// -------  CONFIG ---------
// saves 400 bytes
#define CONVERT_PL_CHARS 0
// saves 720 bytes
#define ENABLE_NOSORT    0
// disable unused types to save memory (up to 4100 bytes)
#define ENABLE_RRE_16B   1
#define ENABLE_RRE_24B   1
#define ENABLE_RRE_32B   1
#define ENABLE_RRE_V16B  1
#define ENABLE_RRE_H16B  1
#define ENABLE_RRE_V24B  1
#define ENABLE_RRE_H24B  1
// --------------------------


#define RRE_16B     0    // 16x16 pixels, rects (X4Y4W4H4)
#define RRE_V16B    1    // 64x32 pixels, lines (X6Y5W0H5)
#define RRE_H16B    2    // 32x64 pixels, lines (X5Y6W5H0)
#define RRE_24B     3    // 64x64 pixels, rects (X6Y6W6H6)
#define RRE_V24B    4    // 256x256 pixels, vertical lines (X8Y8W0H8)
#define RRE_H24B    5    // 256x256 pixels, horizontal lines (X8Y8W8H0)
#define RRE_32B     6    // 256x256 pixels, rects (X8Y8W8H8)
#define RRE_NO_SORT 0x80 // old fonts not optimized for fast width calculation

#define ALIGN_LEFT    0
#define ALIGN_RIGHT  -1 
#define ALIGN_CENTER -2

typedef struct  {
  unsigned char type;
  unsigned char wd;
  unsigned char ht;
  unsigned char firstCh;
  unsigned char lastCh;
  const unsigned char *rects;
  const unsigned short *offs;
}RRE_Font;

typedef int bool;

extern void font_init(void (*rectFun)(int x, int y, int w, int h, int c), int swd, int sht);
extern void font_setFont(RRE_Font *f);
extern unsigned char font_convertPolish(unsigned char _c);
extern int font_charWidthNoSort(unsigned char c, int *_xmin);
extern int font_charWidthOptim(unsigned char c, int *_xmin);
extern int font_charWidth(unsigned char c);
extern int font_drawChar(int x, int y, unsigned char c);
extern int font_strWidth(const char *str);
extern int font_printStr(int xpos, int ypos, const char *str);
extern int font_printf(int xpos, int ypos, const char *str, ...);


extern void font_setScale2(unsigned char _sx, unsigned char _sy);
extern void font_setScale1(unsigned char s);
extern void font_setCR(unsigned char _cr);
extern void font_setFg(int _fg);
extern void font_setBg(int _bg);
extern void font_setColor1(int c);
extern void font_setColor2(int c, int _bg);
extern void font_setBold(unsigned char _bold);
extern void font_setSpacing(unsigned char sp);
extern void font_setSpacingY(unsigned char sp);
extern void font_setFontMinWd(unsigned char wd);
extern void font_setCharMinWd(unsigned char wd);
extern void font_setDigitMinWd(unsigned char wd);
extern int font_getWidth();
extern int font_getHeight();
extern void font_setIsNumberFun(bool (*fun)(unsigned char));
extern void font_setFillRectFun(void (*fun)(int x, int y, int w, int h, int c));

extern bool font_isNumber(unsigned char ch);
extern bool font_isNumberExt(unsigned char ch);


extern RRE_Font rre_12x16;
extern RRE_Font rre_5x8;
extern RRE_Font rre_6x8;
extern RRE_Font rre_7x12;
extern RRE_Font rre_7x8;
extern RRE_Font rre_8x12;
extern RRE_Font rre_arial_62;
extern RRE_Font rre_arialb_16;
extern RRE_Font rre_ArialBlDig54b;
extern RRE_Font rre_ArialBlDig54bv;
extern RRE_Font rre_ArialDig150b;
extern RRE_Font rre_ArialDig47b;
extern RRE_Font rre_ArialDig47n;
extern RRE_Font rre_ArialDig47nh;
extern RRE_Font rre_ArialDig47nv;
extern RRE_Font rre_ArialDig62b;
extern RRE_Font rre_ArialDig62bv;
extern RRE_Font rre_ArialDig72n;
extern RRE_Font rre_ariali_16;
extern RRE_Font rre_arialn_16;
extern RRE_Font rre_bold_6x8;
extern RRE_Font rre_Bold13x20;
extern RRE_Font rre_Bold13x20h;
extern RRE_Font rre_Bold13x20no;
extern RRE_Font rre_Bold13x20v;
extern RRE_Font rre_c_8x16;
extern RRE_Font rre_c64_7x7;
extern RRE_Font rre_c64_7x8;
extern RRE_Font rre_chicago_20x24;
extern RRE_Font rre_comp_8x16;
extern RRE_Font rre_digi_19x15;
extern RRE_Font rre_fixed_8x16;
extern RRE_Font rre_fjg_8x16;
extern RRE_Font rre_grot_16x32;
extern RRE_Font rre_kappa_8x16;
extern RRE_Font rre_nadine_16x16;
extern RRE_Font rre_ncw_8x16;
extern RRE_Font rre_ocr_16x24;
extern RRE_Font rre_script_8x16;
extern RRE_Font rre_Tahoma110v;
extern RRE_Font rre_tahoma_65v;
extern RRE_Font rre_term_10x16;
extern RRE_Font rre_times_104;
extern RRE_Font rre_Times98v;
extern RRE_Font rre_ubuntu_32;
extern RRE_Font rre_ubuntub_32;
extern RRE_Font rre_vga_8x16;
extern RRE_Font rre_x_8x16;


#endif

