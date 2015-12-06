/*
 * s3cfb_70.c for 7 inch lcd
 *
 * (C) Copyright 2001-2002
 * Wolfgang Denk, DENX Software Engineering -- wd@denx.de
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/************************************************************************/
/* ** HEADER FILES							*/
/************************************************************************/

/* #define DEBUG */

#include <config.h>
#include <common.h>
#include <command.h>
#include <version.h>
#include <stdarg.h>
#include <linux/types.h>
#include <devices.h>


#include <lcd.h>
#include <watchdog.h>


#include <common.h>
#include <regs.h>
#include <regs-lcd.h>


#ifdef CONFIG_LCD

#define S3CFB_HFP		55	/* front porch */
#define S3CFB_HSW		119	/* hsync width */
#define S3CFB_HBP		73	/* back porch */

#define S3CFB_VFP		36	/* front porch */
#define S3CFB_VSW		5	/* vsync width */
#define S3CFB_VBP		32	/* back porch */

#define S3CFB_HRES              800
#define S3CFB_VRES              600

//#define S3CFB_VFRAME_FREQ       62
#define S3CFB_VFRAME_FREQ       90

#define S3CFB_IVCLK             0//CFG_LOW
#define S3CFB_IHSYNC            0//CFG_HIGH
#define S3CFB_IVSYNC            0//CFG_HIGH
#define S3CFB_IVDEN             0//CFG_LOW


#define S3CFB_PIXEL_CLOCK       (S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))

extern	uchar *osd_frame_buffer;

void lcd_ctrl_init_8_10(void *lcdbase)
{
	ulong freq_lcdclk;
	ulong freq_Hclk;
	ulong fb_size;
	unsigned char nn;
	unsigned short *pp;
	int i;

	GPICON_REG = 0xaaaaaaaa;
	GPIPUD_REG = 0xaaaaaaaa;
	GPJCON_REG = 0xaaaaaaaa;
	GPJPUD_REG = 0xaaaaaaaa;

	lcd_disable();
	S3C_WINCON0 &= (~(S3C_WINCONx_ENWIN_F_ENABLE));

	MIFPCON_REG &= (~SEL_BYPASS_MASK);
	SPCON_REG &= (~LCD_SEL_MASK);
	SPCON_REG |= (RGB_IF_STYLE_MASK);

	freq_lcdclk = S3CFB_PIXEL_CLOCK;
	freq_Hclk = get_HCLK();

	nn = (unsigned char)(freq_Hclk / freq_lcdclk) - 1;
	if(freq_lcdclk < freq_Hclk/2)
	{
		S3C_VIDCON0 = S3C_VIDCON0_INTERLACE_F_PROGRESSIVE + S3C_VIDCON0_VIDOUT_RGB_IF + \
		        S3C_VIDCON0_PNRMODE_RGB_P + S3C_VIDCON0_CLKVALUP_ST_FRM + S3C_VIDCON0_CLKVAL_F(nn) + \
		        S3C_VIDCON0_CLKDIR_DIVIDED + S3C_VIDCON0_CLKSEL_F_HCLK;
	}else
	{
		S3C_VIDCON0 = S3C_VIDCON0_INTERLACE_F_PROGRESSIVE + S3C_VIDCON0_VIDOUT_RGB_IF + \
		        S3C_VIDCON0_PNRMODE_RGB_P + S3C_VIDCON0_CLKVALUP_ST_FRM + S3C_VIDCON0_CLKVAL_F(0) + \
		        S3C_VIDCON0_CLKDIR_DIRECTED  + S3C_VIDCON0_CLKSEL_F_HCLK;
	}

	nn = 0;
	if(S3CFB_IVCLK)
	{
		nn += S3C_VIDCON1_IVCLK_RISE_EDGE;
	}
	if(S3CFB_IHSYNC)
	{
		nn += S3C_VIDCON1_IHSYNC_INVERT;
	}
	if(S3CFB_IVSYNC)
	{
		nn += S3C_VIDCON1_IVSYNC_INVERT;
	}
	if(S3CFB_IVDEN)
	{
		nn += S3C_VIDCON1_IVDEN_INVERT;
	}
	S3C_VIDCON1 = (unsigned int)nn;
	S3C_VIDCON2 = 0;

	S3C_VIDTCON0 = S3C_VIDTCON0_VBPD(S3CFB_VBP - 1) | S3C_VIDTCON0_VFPD(S3CFB_VFP - 1) | S3C_VIDTCON0_VSPW(S3CFB_VSW - 1);
	S3C_VIDTCON1 = S3C_VIDTCON1_HBPD(S3CFB_HBP - 1) | S3C_VIDTCON1_HFPD(S3CFB_HFP -1) | S3C_VIDTCON1_HSPW(S3CFB_HSW - 1);
	S3C_VIDTCON2 = S3C_VIDTCON2_LINEVAL(S3CFB_VRES - 1) | S3C_VIDTCON2_HOZVAL(S3CFB_HRES - 1);

#if LCD_BPP == LCD_COLOR32
	S3C_WINCON0 = S3C_WINCONx_BPPMODE_F_24BPP_888;
	S3C_WINCON1 = S3C_WINCONx_BPPMODE_F_24BPP_888 | S3C_WINCONx_BLD_PIX_PIXEL;
#else
	S3C_WINCON0 = S3C_WINCONx_BPPMODE_F_16BPP_565 | S3C_WINCONx_HAWSWP_ENABLE;
	S3C_WINCON1 = S3C_WINCONx_BPPMODE_F_16BPP_565 | S3C_WINCONx_HAWSWP_ENABLE | S3C_WINCONx_BLD_PIX_PIXEL;
#endif

	S3C_WINCON2 = 0;
	S3C_WINCON3 = 0;
	S3C_WINCON4 = 0;

	S3C_VIDOSD0A = S3C_VIDOSDxA_OSD_LTX_F(0) + S3C_VIDOSDxA_OSD_LTY_F(0);
	S3C_VIDOSD0B = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES - 1);
	S3C_VIDOSD0C = S3C_VIDOSD0C_OSDSIZE(S3CFB_HRES*S3CFB_VRES);

	S3C_VIDOSD1A = S3C_VIDOSDxA_OSD_LTX_F(0) + S3C_VIDOSDxA_OSD_LTY_F(0);
	S3C_VIDOSD1B = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES - 1);
	S3C_VIDOSD1C = 0xDDD000;/*alpha blending*/
	S3C_VIDOSD1D = S3C_VIDOSD0C_OSDSIZE(S3CFB_HRES*S3CFB_VRES);

	S3C_VIDOSD2A = 0;
	S3C_VIDOSD2B = 0;
	S3C_VIDOSD2C = 0;
	S3C_VIDOSD2D = 0;
	S3C_VIDOSD3A = 0;
	S3C_VIDOSD3B = 0;
	S3C_VIDOSD3C = 0;
	S3C_VIDOSD4A = 0;
	S3C_VIDOSD4B = 0;
	S3C_VIDOSD4C = 0;

	fb_size = calc_fbsize();

	S3C_VIDW00ADD0B0 = virt_to_phys(lcdbase);
	S3C_VIDW00ADD0B1 = 0;
	S3C_VIDW01ADD0B0 = virt_to_phys(osd_frame_buffer);
	S3C_VIDW01ADD0B1 = 0;
	S3C_VIDW02ADD0 = 0;
	S3C_VIDW03ADD0 = 0;
	S3C_VIDW04ADD0 = 0;

	S3C_VIDW00ADD1B0 = virt_to_phys((unsigned int)(lcdbase) + fb_size);
	S3C_VIDW00ADD1B1 = 0;
	S3C_VIDW01ADD1B0 = virt_to_phys(osd_frame_buffer) + fb_size;
	S3C_VIDW01ADD1B1 = 0;
	S3C_VIDW02ADD1 = 0;
	S3C_VIDW03ADD1 = 0;
	S3C_VIDW04ADD1 = 0;

	S3C_VIDW00ADD2 = 0;//S3C_VIDWxxADD2_OFFSIZE_F(0) | (S3C_VIDWxxADD2_PAGEWIDTH_F(panel_info.vl_col*panel_info.vl_bpix/8));
	S3C_VIDW01ADD2 = 0;//S3C_VIDWxxADD2_OFFSIZE_F(0) | (S3C_VIDWxxADD2_PAGEWIDTH_F(panel_info.vl_col*panel_info.vl_bpix/8));
	S3C_VIDW02ADD2 = 0;
	S3C_VIDW03ADD2 = 0;
	S3C_VIDW04ADD2 = 0;

	S3C_W1KEYCON0  = S3C_WxKEYCON0_KEYBLEN_ENABLE | S3C_WxKEYCON0_KEYEN_F_ENABLE | S3C_WxKEYCON0_COMPKEY(0xFFFF);
	S3C_W1KEYCON1  = 0x00000000;/*color key*/

#if 1
	memset(lcdbase,0x00,fb_size*2);
#else
	pp = lcdbase;
	for(i=0;i< S3CFB_HRES * S3CFB_VRES;i++)
	{
		*pp = 0xf100;
		pp++;
	}
#endif
	lcd_enable();

	S3C_WINCON0 |= S3C_WINCONx_ENWIN_F_ENABLE;
	S3C_WINCON1 |= S3C_WINCONx_ENWIN_F_ENABLE;
	return (0);
}


#endif /* CONFIG_LCD */
