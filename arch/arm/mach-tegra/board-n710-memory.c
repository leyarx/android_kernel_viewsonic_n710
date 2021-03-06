/*
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * Copyright (c) 2013, Yaroslav Levandovskiy <leyarx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/init.h>

#include "board.h"
#include "board-n710.h"
#include "tegra3_emc.h"
#include "fuse.h"


static const struct tegra_emc_table Micron_Elpida_1GB_ddr3_667Mhz_kh_sdmmc4_x8_12M_0704[] = {
	{
		0x32,       /* Rev 3.2 */
		25500,      /* SDRAM frequency */
		{
			0x0000001f, /* EMC_RC */
			0x000000ac, /* EMC_RFC */
			0x00000016, /* EMC_RAS */
			0x00000008, /* EMC_RP */
			0x00000005, /* EMC_R2W */
			0x0000000c, /* EMC_W2R */
			0x00000003, /* EMC_R2P */
			0x00000011, /* EMC_W2P */
			0x00000008, /* EMC_RD_RCD */
			0x00000008, /* EMC_WR_RCD */
			0x00000003, /* EMC_RRD */
			0x00000001, /* EMC_REXT */
			0x00000000, /* EMC_WEXT */
			0x00000007, /* EMC_WDV */
			0x0000000a, /* EMC_QUSE */
			0x00000009, /* EMC_QRST */
			0x0000000b, /* EMC_QSAFE */
			0x00000011, /* EMC_RDV */
			0x00001412, /* EMC_REFRESH */
			0x00000000, /* EMC_BURST_REFRESH_NUM */
			0x00000504, /* EMC_PRE_REFRESH_REQ_CNT */
			0x00000002, /* EMC_PDEX2WR */
			0x0000000e, /* EMC_PDEX2RD */
			0x00000001, /* EMC_PCHG2PDEN */
			0x00000000, /* EMC_ACT2PDEN */
			0x0000000c, /* EMC_AR2PDEN */
			0x00000016, /* EMC_RW2PDEN */
			0x000000b5, /* EMC_TXSR */
			0x00000200, /* EMC_TXSRDLL */
			0x00000005, /* EMC_TCKE */
			0x0000001b, /* EMC_TFAW */
			0x00000000, /* EMC_TRPAB */
			0x00000006, /* EMC_TCLKSTABLE */
			0x00000007, /* EMC_TCLKSTOP */
			0x00001453, /* EMC_TREFBW */
			0x0000000b, /* EMC_QUSE_EXTRA */
			0x00000006, /* EMC_FBIO_CFG6 */
			0x00000000, /* EMC_ODT_WRITE */
			0x00000000, /* EMC_ODT_READ */
			0x00005088, /* EMC_FBIO_CFG5 */
			0xf00b0191, /* EMC_CFG_DIG_DLL */
			0x00008000, /* EMC_CFG_DIG_DLL_PERIOD */
			0x0000000a, /* EMC_DLL_XFORM_DQS0 */
			0x0000000a, /* EMC_DLL_XFORM_DQS1 */
			0x0000000a, /* EMC_DLL_XFORM_DQS2 */
			0x0000000a, /* EMC_DLL_XFORM_DQS3 */
			0x0000000a, /* EMC_DLL_XFORM_DQS4 */
			0x0000000a, /* EMC_DLL_XFORM_DQS5 */
			0x0000000a, /* EMC_DLL_XFORM_DQS6 */
			0x0000000a, /* EMC_DLL_XFORM_DQS7 */
			0x00018000, /* EMC_DLL_XFORM_QUSE0 */
			0x00018000, /* EMC_DLL_XFORM_QUSE1 */
			0x00018000, /* EMC_DLL_XFORM_QUSE2 */
			0x00018000, /* EMC_DLL_XFORM_QUSE3 */
			0x00018000, /* EMC_DLL_XFORM_QUSE4 */
			0x00018000, /* EMC_DLL_XFORM_QUSE5 */
			0x00018000, /* EMC_DLL_XFORM_QUSE6 */
			0x00018000, /* EMC_DLL_XFORM_QUSE7 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS0 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS1 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS2 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS3 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS4 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS5 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS6 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS7 */
			0x0000000c, /* EMC_DLL_XFORM_DQ0 */
			0x0000000c, /* EMC_DLL_XFORM_DQ1 */
			0x0000000c, /* EMC_DLL_XFORM_DQ2 */
			0x0000000c, /* EMC_DLL_XFORM_DQ3 */
			0x000002a0, /* EMC_XM2CMDPADCTRL */
			0x0800013d, /* EMC_XM2DQSPADCTRL2 */
			0x22220000, /* EMC_XM2DQPADCTRL2 */
			0x77fff884, /* EMC_XM2CLKPADCTRL */
			0x01f1f501, /* EMC_XM2COMPPADCTRL */
			0x07077404, /* EMC_XM2VTTGENPADCTRL */
			0x54000000, /* EMC_XM2VTTGENPADCTRL2 */
			0x080001e8, /* EMC_XM2QUSEPADCTRL */
			0x08000021, /* EMC_XM2DQSPADCTRL3 */
			0x00000802, /* EMC_CTT_TERM_CTRL */
			0x00020000, /* EMC_ZCAL_INTERVAL */
			0x00000040, /* EMC_ZCAL_WAIT_CNT */
			0x0113000c, /* EMC_MRS_WAIT_CNT */
			0xa0f10000, /* EMC_AUTO_CAL_CONFIG */
			0x00000000, /* EMC_CTT */
			0x00000000, /* EMC_CTT_DURATION */
			0x800028a5, /* EMC_DYN_SELF_REF_CONTROL */
			0x00000014, /* MC_EMEM_ARB_CFG */
			0xc0000079, /* MC_EMEM_ARB_OUTSTANDING_REQ */
			0x00000003, /* MC_EMEM_ARB_TIMING_RCD */
			0x00000004, /* MC_EMEM_ARB_TIMING_RP */
			0x00000010, /* MC_EMEM_ARB_TIMING_RC */
			0x0000000a, /* MC_EMEM_ARB_TIMING_RAS */
			0x0000000d, /* MC_EMEM_ARB_TIMING_FAW */
			0x00000002, /* MC_EMEM_ARB_TIMING_RRD */
			0x00000003, /* MC_EMEM_ARB_TIMING_RAP2PRE */
			0x0000000b, /* MC_EMEM_ARB_TIMING_WAP2PRE */
			0x00000002, /* MC_EMEM_ARB_TIMING_R2R */
			0x00000002, /* MC_EMEM_ARB_TIMING_W2W */
			0x00000004, /* MC_EMEM_ARB_TIMING_R2W */
			0x00000008, /* MC_EMEM_ARB_TIMING_W2R */
			0x08040202, /* MC_EMEM_ARB_DA_TURNS */
			0x00140c10, /* MC_EMEM_ARB_DA_COVERS */
			0x70ea1f11, /* MC_EMEM_ARB_MISC0 */
			0x001f0000, /* MC_EMEM_ARB_RING1_THROTTLE */
			0xf8000000, /* EMC_FBIO_SPARE */
			0xff00ff49, /* EMC_CFG_RSV */
		},
		0x00000040, /* EMC_ZCAL_WAIT_CNT after clock change !!!*/
		0x001fffff, /* EMC_AUTO_CAL_INTERVAL */
		0x00000001, /* EMC_CFG.PERIODIC_QRST !!!*/
		0x80001221, /* Mode Register 0 !!!*/
		0x80100003, /* Mode Register 1 !!!*/
		0x80200008, /* Mode Register 2 !!!*/
		0x00000001, /* EMC_CFG.DYN_SELF_REF !!!*/
	},
	{
		0x32,       /* Rev 3.2 */
		51000,      /* SDRAM frequency */
		{
			0x0000001f, /* EMC_RC */
			0x000000ac, /* EMC_RFC */
			0x00000016, /* EMC_RAS */
			0x00000008, /* EMC_RP */
			0x00000005, /* EMC_R2W */
			0x0000000c, /* EMC_W2R */
			0x00000003, /* EMC_R2P */
			0x00000011, /* EMC_W2P */
			0x00000008, /* EMC_RD_RCD */
			0x00000008, /* EMC_WR_RCD */
			0x00000003, /* EMC_RRD */
			0x00000001, /* EMC_REXT */
			0x00000000, /* EMC_WEXT */
			0x00000007, /* EMC_WDV */
			0x0000000a, /* EMC_QUSE */
			0x00000009, /* EMC_QRST */
			0x0000000b, /* EMC_QSAFE */
			0x00000011, /* EMC_RDV */
			0x00001412, /* EMC_REFRESH */
			0x00000000, /* EMC_BURST_REFRESH_NUM */
			0x00000504, /* EMC_PRE_REFRESH_REQ_CNT */
			0x00000002, /* EMC_PDEX2WR */
			0x0000000e, /* EMC_PDEX2RD */
			0x00000001, /* EMC_PCHG2PDEN */
			0x00000000, /* EMC_ACT2PDEN */
			0x0000000c, /* EMC_AR2PDEN */
			0x00000016, /* EMC_RW2PDEN */
			0x000000b5, /* EMC_TXSR */
			0x00000200, /* EMC_TXSRDLL */
			0x00000005, /* EMC_TCKE */
			0x0000001b, /* EMC_TFAW */
			0x00000000, /* EMC_TRPAB */
			0x00000006, /* EMC_TCLKSTABLE */
			0x00000007, /* EMC_TCLKSTOP */
			0x00001453, /* EMC_TREFBW */
			0x0000000b, /* EMC_QUSE_EXTRA */
			0x00000006, /* EMC_FBIO_CFG6 */
			0x00000000, /* EMC_ODT_WRITE */
			0x00000000, /* EMC_ODT_READ */
			0x00005088, /* EMC_FBIO_CFG5 */
			0xf00b0191, /* EMC_CFG_DIG_DLL */
			0x00008000, /* EMC_CFG_DIG_DLL_PERIOD */
			0x0000000a, /* EMC_DLL_XFORM_DQS0 */
			0x0000000a, /* EMC_DLL_XFORM_DQS1 */
			0x0000000a, /* EMC_DLL_XFORM_DQS2 */
			0x0000000a, /* EMC_DLL_XFORM_DQS3 */
			0x0000000a, /* EMC_DLL_XFORM_DQS4 */
			0x0000000a, /* EMC_DLL_XFORM_DQS5 */
			0x0000000a, /* EMC_DLL_XFORM_DQS6 */
			0x0000000a, /* EMC_DLL_XFORM_DQS7 */
			0x00018000, /* EMC_DLL_XFORM_QUSE0 */
			0x00018000, /* EMC_DLL_XFORM_QUSE1 */
			0x00018000, /* EMC_DLL_XFORM_QUSE2 */
			0x00018000, /* EMC_DLL_XFORM_QUSE3 */
			0x00018000, /* EMC_DLL_XFORM_QUSE4 */
			0x00018000, /* EMC_DLL_XFORM_QUSE5 */
			0x00018000, /* EMC_DLL_XFORM_QUSE6 */
			0x00018000, /* EMC_DLL_XFORM_QUSE7 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS0 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS1 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS2 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS3 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS4 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS5 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS6 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS7 */
			0x0000000c, /* EMC_DLL_XFORM_DQ0 */
			0x0000000c, /* EMC_DLL_XFORM_DQ1 */
			0x0000000c, /* EMC_DLL_XFORM_DQ2 */
			0x0000000c, /* EMC_DLL_XFORM_DQ3 */
			0x000002a0, /* EMC_XM2CMDPADCTRL */
			0x0800013d, /* EMC_XM2DQSPADCTRL2 */
			0x22220000, /* EMC_XM2DQPADCTRL2 */
			0x77fff884, /* EMC_XM2CLKPADCTRL */
			0x01f1f501, /* EMC_XM2COMPPADCTRL */
			0x07077404, /* EMC_XM2VTTGENPADCTRL */
			0x54000000, /* EMC_XM2VTTGENPADCTRL2 */
			0x080001e8, /* EMC_XM2QUSEPADCTRL */
			0x08000021, /* EMC_XM2DQSPADCTRL3 */
			0x00000802, /* EMC_CTT_TERM_CTRL */
			0x00020000, /* EMC_ZCAL_INTERVAL */
			0x00000040, /* EMC_ZCAL_WAIT_CNT */
			0x0113000c, /* EMC_MRS_WAIT_CNT */
			0xa0f10000, /* EMC_AUTO_CAL_CONFIG */
			0x00000000, /* EMC_CTT */
			0x00000000, /* EMC_CTT_DURATION */
			0x800028a5, /* EMC_DYN_SELF_REF_CONTROL */
			0x00000014, /* MC_EMEM_ARB_CFG */
			0xc0000079, /* MC_EMEM_ARB_OUTSTANDING_REQ */
			0x00000003, /* MC_EMEM_ARB_TIMING_RCD */
			0x00000004, /* MC_EMEM_ARB_TIMING_RP */
			0x00000010, /* MC_EMEM_ARB_TIMING_RC */
			0x0000000a, /* MC_EMEM_ARB_TIMING_RAS */
			0x0000000d, /* MC_EMEM_ARB_TIMING_FAW */
			0x00000002, /* MC_EMEM_ARB_TIMING_RRD */
			0x00000003, /* MC_EMEM_ARB_TIMING_RAP2PRE */
			0x0000000b, /* MC_EMEM_ARB_TIMING_WAP2PRE */
			0x00000002, /* MC_EMEM_ARB_TIMING_R2R */
			0x00000002, /* MC_EMEM_ARB_TIMING_W2W */
			0x00000004, /* MC_EMEM_ARB_TIMING_R2W */
			0x00000008, /* MC_EMEM_ARB_TIMING_W2R */
			0x08040202, /* MC_EMEM_ARB_DA_TURNS */
			0x00130b10, /* MC_EMEM_ARB_DA_COVERS */
			0x70ea1f11, /* MC_EMEM_ARB_MISC0 */
			0x001f0000, /* MC_EMEM_ARB_RING1_THROTTLE */
			0xf8000000, /* EMC_FBIO_SPARE */
			0xff00ff49, /* EMC_CFG_RSV */
		},
		0x00000040, /* EMC_ZCAL_WAIT_CNT after clock change */
		0x001fffff, /* EMC_AUTO_CAL_INTERVAL */
		0x00000001, /* EMC_CFG.PERIODIC_QRST */
		0x80001221, /* Mode Register 0 */
		0x80100003, /* Mode Register 1 */
		0x80200008, /* Mode Register 2 */
		0x00000001, /* EMC_CFG.DYN_SELF_REF */
	},
	{
		0x32,       /* Rev 3.2 */
		102000,     /* SDRAM frequency */
		{
			0x0000001f, /* EMC_RC */
			0x000000ac, /* EMC_RFC */
			0x00000016, /* EMC_RAS */
			0x00000008, /* EMC_RP */
			0x00000005, /* EMC_R2W */
			0x0000000c, /* EMC_W2R */
			0x00000003, /* EMC_R2P */
			0x00000011, /* EMC_W2P */
			0x00000008, /* EMC_RD_RCD */
			0x00000008, /* EMC_WR_RCD */
			0x00000003, /* EMC_RRD */
			0x00000001, /* EMC_REXT */
			0x00000000, /* EMC_WEXT */
			0x00000007, /* EMC_WDV */
			0x0000000a, /* EMC_QUSE */
			0x00000009, /* EMC_QRST */
			0x0000000b, /* EMC_QSAFE */
			0x00000011, /* EMC_RDV */
			0x00001412, /* EMC_REFRESH */
			0x00000000, /* EMC_BURST_REFRESH_NUM */
			0x00000504, /* EMC_PRE_REFRESH_REQ_CNT */
			0x00000002, /* EMC_PDEX2WR */
			0x0000000e, /* EMC_PDEX2RD */
			0x00000001, /* EMC_PCHG2PDEN */
			0x00000000, /* EMC_ACT2PDEN */
			0x0000000c, /* EMC_AR2PDEN */
			0x00000016, /* EMC_RW2PDEN */
			0x000000b5, /* EMC_TXSR */
			0x00000200, /* EMC_TXSRDLL */
			0x00000005, /* EMC_TCKE */
			0x0000001b, /* EMC_TFAW */
			0x00000000, /* EMC_TRPAB */
			0x00000006, /* EMC_TCLKSTABLE */
			0x00000007, /* EMC_TCLKSTOP */
			0x00001453, /* EMC_TREFBW */
			0x0000000b, /* EMC_QUSE_EXTRA */
			0x00000006, /* EMC_FBIO_CFG6 */
			0x00000000, /* EMC_ODT_WRITE */
			0x00000000, /* EMC_ODT_READ */
			0x00005088, /* EMC_FBIO_CFG5 */
			0xf00b0191, /* EMC_CFG_DIG_DLL */
			0x00008000, /* EMC_CFG_DIG_DLL_PERIOD */
			0x0000000a, /* EMC_DLL_XFORM_DQS0 */
			0x0000000a, /* EMC_DLL_XFORM_DQS1 */
			0x0000000a, /* EMC_DLL_XFORM_DQS2 */
			0x0000000a, /* EMC_DLL_XFORM_DQS3 */
			0x0000000a, /* EMC_DLL_XFORM_DQS4 */
			0x0000000a, /* EMC_DLL_XFORM_DQS5 */
			0x0000000a, /* EMC_DLL_XFORM_DQS6 */
			0x0000000a, /* EMC_DLL_XFORM_DQS7 */
			0x00018000, /* EMC_DLL_XFORM_QUSE0 */
			0x00018000, /* EMC_DLL_XFORM_QUSE1 */
			0x00018000, /* EMC_DLL_XFORM_QUSE2 */
			0x00018000, /* EMC_DLL_XFORM_QUSE3 */
			0x00018000, /* EMC_DLL_XFORM_QUSE4 */
			0x00018000, /* EMC_DLL_XFORM_QUSE5 */
			0x00018000, /* EMC_DLL_XFORM_QUSE6 */
			0x00018000, /* EMC_DLL_XFORM_QUSE7 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS0 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS1 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS2 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS3 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS4 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS5 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS6 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS7 */
			0x0000000c, /* EMC_DLL_XFORM_DQ0 */
			0x0000000c, /* EMC_DLL_XFORM_DQ1 */
			0x0000000c, /* EMC_DLL_XFORM_DQ2 */
			0x0000000c, /* EMC_DLL_XFORM_DQ3 */
			0x000002a0, /* EMC_XM2CMDPADCTRL */
			0x0800013d, /* EMC_XM2DQSPADCTRL2 */
			0x22220000, /* EMC_XM2DQPADCTRL2 */
			0x77fff884, /* EMC_XM2CLKPADCTRL */
			0x01f1f501, /* EMC_XM2COMPPADCTRL */
			0x07077404, /* EMC_XM2VTTGENPADCTRL */
			0x54000000, /* EMC_XM2VTTGENPADCTRL2 */
			0x080001e8, /* EMC_XM2QUSEPADCTRL */
			0x08000021, /* EMC_XM2DQSPADCTRL3 */
			0x00000802, /* EMC_CTT_TERM_CTRL */
			0x00020000, /* EMC_ZCAL_INTERVAL */
			0x00000040, /* EMC_ZCAL_WAIT_CNT */
			0x0113000c, /* EMC_MRS_WAIT_CNT */
			0xa0f10000, /* EMC_AUTO_CAL_CONFIG */
			0x00000000, /* EMC_CTT */
			0x00000000, /* EMC_CTT_DURATION */
			0x800028a5, /* EMC_DYN_SELF_REF_CONTROL */
			0x00000014, /* MC_EMEM_ARB_CFG */
			0xc0000079, /* MC_EMEM_ARB_OUTSTANDING_REQ */
			0x00000003, /* MC_EMEM_ARB_TIMING_RCD */
			0x00000004, /* MC_EMEM_ARB_TIMING_RP */
			0x00000010, /* MC_EMEM_ARB_TIMING_RC */
			0x0000000a, /* MC_EMEM_ARB_TIMING_RAS */
			0x0000000d, /* MC_EMEM_ARB_TIMING_FAW */
			0x00000002, /* MC_EMEM_ARB_TIMING_RRD */
			0x00000003, /* MC_EMEM_ARB_TIMING_RAP2PRE */
			0x0000000b, /* MC_EMEM_ARB_TIMING_WAP2PRE */
			0x00000002, /* MC_EMEM_ARB_TIMING_R2R */
			0x00000002, /* MC_EMEM_ARB_TIMING_W2W */
			0x00000004, /* MC_EMEM_ARB_TIMING_R2W */
			0x00000008, /* MC_EMEM_ARB_TIMING_W2R */
			0x08040202, /* MC_EMEM_ARB_DA_TURNS */
			0x00130b10, /* MC_EMEM_ARB_DA_COVERS */
			0x70ea1f11, /* MC_EMEM_ARB_MISC0 */
			0x001f0000, /* MC_EMEM_ARB_RING1_THROTTLE */
			0xf8000000, /* EMC_FBIO_SPARE */
			0xff00ff49, /* EMC_CFG_RSV */
		},
		0x00000040, /* EMC_ZCAL_WAIT_CNT after clock change */
		0x001fffff, /* EMC_AUTO_CAL_INTERVAL */
		0x00000001, /* EMC_CFG.PERIODIC_QRST */
		0x80001221, /* Mode Register 0 */
		0x80100003, /* Mode Register 1 */
		0x80200008, /* Mode Register 2 */
		0x00000001, /* EMC_CFG.DYN_SELF_REF */
	},
	{
		0x32,       /* Rev 3.2 */
		204000,     /* SDRAM frequency */
		{
			0x0000001f, /* EMC_RC */
			0x000000ac, /* EMC_RFC */
			0x00000016, /* EMC_RAS */
			0x00000008, /* EMC_RP */
			0x00000005, /* EMC_R2W */
			0x0000000c, /* EMC_W2R */
			0x00000003, /* EMC_R2P */
			0x00000011, /* EMC_W2P */
			0x00000008, /* EMC_RD_RCD */
			0x00000008, /* EMC_WR_RCD */
			0x00000003, /* EMC_RRD */
			0x00000001, /* EMC_REXT */
			0x00000000, /* EMC_WEXT */
			0x00000007, /* EMC_WDV */
			0x0000000a, /* EMC_QUSE */
			0x00000009, /* EMC_QRST */
			0x0000000b, /* EMC_QSAFE */
			0x00000011, /* EMC_RDV */
			0x00001412, /* EMC_REFRESH */
			0x00000000, /* EMC_BURST_REFRESH_NUM */
			0x00000504, /* EMC_PRE_REFRESH_REQ_CNT */
			0x00000002, /* EMC_PDEX2WR */
			0x0000000e, /* EMC_PDEX2RD */
			0x00000001, /* EMC_PCHG2PDEN */
			0x00000000, /* EMC_ACT2PDEN */
			0x0000000c, /* EMC_AR2PDEN */
			0x00000016, /* EMC_RW2PDEN */
			0x000000b5, /* EMC_TXSR */
			0x00000200, /* EMC_TXSRDLL */
			0x00000005, /* EMC_TCKE */
			0x0000001b, /* EMC_TFAW */
			0x00000000, /* EMC_TRPAB */
			0x00000006, /* EMC_TCLKSTABLE */
			0x00000007, /* EMC_TCLKSTOP */
			0x00001453, /* EMC_TREFBW */
			0x0000000b, /* EMC_QUSE_EXTRA */
			0x00000006, /* EMC_FBIO_CFG6 */
			0x00000000, /* EMC_ODT_WRITE */
			0x00000000, /* EMC_ODT_READ */
			0x00005088, /* EMC_FBIO_CFG5 */
			0xf00b0191, /* EMC_CFG_DIG_DLL */
			0x00008000, /* EMC_CFG_DIG_DLL_PERIOD */
			0x0000000a, /* EMC_DLL_XFORM_DQS0 */
			0x0000000a, /* EMC_DLL_XFORM_DQS1 */
			0x0000000a, /* EMC_DLL_XFORM_DQS2 */
			0x0000000a, /* EMC_DLL_XFORM_DQS3 */
			0x0000000a, /* EMC_DLL_XFORM_DQS4 */
			0x0000000a, /* EMC_DLL_XFORM_DQS5 */
			0x0000000a, /* EMC_DLL_XFORM_DQS6 */
			0x0000000a, /* EMC_DLL_XFORM_DQS7 */
			0x00018000, /* EMC_DLL_XFORM_QUSE0 */
			0x00018000, /* EMC_DLL_XFORM_QUSE1 */
			0x00018000, /* EMC_DLL_XFORM_QUSE2 */
			0x00018000, /* EMC_DLL_XFORM_QUSE3 */
			0x00018000, /* EMC_DLL_XFORM_QUSE4 */
			0x00018000, /* EMC_DLL_XFORM_QUSE5 */
			0x00018000, /* EMC_DLL_XFORM_QUSE6 */
			0x00018000, /* EMC_DLL_XFORM_QUSE7 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS0 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS1 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS2 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS3 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS4 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS5 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS6 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS7 */
			0x0000000c, /* EMC_DLL_XFORM_DQ0 */
			0x0000000c, /* EMC_DLL_XFORM_DQ1 */
			0x0000000c, /* EMC_DLL_XFORM_DQ2 */
			0x0000000c, /* EMC_DLL_XFORM_DQ3 */
			0x000002a0, /* EMC_XM2CMDPADCTRL */
			0x0800013d, /* EMC_XM2DQSPADCTRL2 */
			0x22220000, /* EMC_XM2DQPADCTRL2 */
			0x77fff884, /* EMC_XM2CLKPADCTRL */
			0x01f1f501, /* EMC_XM2COMPPADCTRL */
			0x07077404, /* EMC_XM2VTTGENPADCTRL */
			0x54000000, /* EMC_XM2VTTGENPADCTRL2 */
			0x080001e8, /* EMC_XM2QUSEPADCTRL */
			0x08000021, /* EMC_XM2DQSPADCTRL3 */
			0x00000802, /* EMC_CTT_TERM_CTRL */
			0x00020000, /* EMC_ZCAL_INTERVAL */
			0x00000040, /* EMC_ZCAL_WAIT_CNT */
			0x0113000c, /* EMC_MRS_WAIT_CNT */
			0xa0f10000, /* EMC_AUTO_CAL_CONFIG */
			0x00000000, /* EMC_CTT */
			0x00000000, /* EMC_CTT_DURATION */
			0x800028a5, /* EMC_DYN_SELF_REF_CONTROL */
			0x00000014, /* MC_EMEM_ARB_CFG */
			0xc0000079, /* MC_EMEM_ARB_OUTSTANDING_REQ */
			0x00000003, /* MC_EMEM_ARB_TIMING_RCD */
			0x00000004, /* MC_EMEM_ARB_TIMING_RP */
			0x00000010, /* MC_EMEM_ARB_TIMING_RC */
			0x0000000a, /* MC_EMEM_ARB_TIMING_RAS */
			0x0000000d, /* MC_EMEM_ARB_TIMING_FAW */
			0x00000002, /* MC_EMEM_ARB_TIMING_RRD */
			0x00000003, /* MC_EMEM_ARB_TIMING_RAP2PRE */
			0x0000000b, /* MC_EMEM_ARB_TIMING_WAP2PRE */
			0x00000002, /* MC_EMEM_ARB_TIMING_R2R */
			0x00000002, /* MC_EMEM_ARB_TIMING_W2W */
			0x00000004, /* MC_EMEM_ARB_TIMING_R2W */
			0x00000008, /* MC_EMEM_ARB_TIMING_W2R */
			0x08040202, /* MC_EMEM_ARB_DA_TURNS */
			0x00130b10, /* MC_EMEM_ARB_DA_COVERS */
			0x70ea1f11, /* MC_EMEM_ARB_MISC0 */
			0x001f0000, /* MC_EMEM_ARB_RING1_THROTTLE */
			0xf8000000, /* EMC_FBIO_SPARE */
			0xff00ff49, /* EMC_CFG_RSV */
		},
		0x00000040, /* EMC_ZCAL_WAIT_CNT after clock change */
		0x001fffff, /* EMC_AUTO_CAL_INTERVAL */
		0x00000001, /* EMC_CFG.PERIODIC_QRST */
		0x80001221, /* Mode Register 0 */
		0x80100003, /* Mode Register 1 */
		0x80200008, /* Mode Register 2 */
		0x00000001, /* EMC_CFG.DYN_SELF_REF */
	},
	{
		0x32,       /* Rev 3.2 */
		333500,     /* SDRAM frequency */
		{
			0x0000001f, /* EMC_RC */
			0x000000ac, /* EMC_RFC */
			0x00000016, /* EMC_RAS */
			0x00000008, /* EMC_RP */
			0x00000005, /* EMC_R2W */
			0x0000000c, /* EMC_W2R */
			0x00000003, /* EMC_R2P */
			0x00000011, /* EMC_W2P */
			0x00000008, /* EMC_RD_RCD */
			0x00000008, /* EMC_WR_RCD */
			0x00000003, /* EMC_RRD */
			0x00000001, /* EMC_REXT */
			0x00000000, /* EMC_WEXT */
			0x00000007, /* EMC_WDV */
			0x0000000a, /* EMC_QUSE */
			0x00000009, /* EMC_QRST */
			0x0000000b, /* EMC_QSAFE */
			0x00000011, /* EMC_RDV */
			0x00001412, /* EMC_REFRESH */
			0x00000000, /* EMC_BURST_REFRESH_NUM */
			0x00000504, /* EMC_PRE_REFRESH_REQ_CNT */
			0x00000002, /* EMC_PDEX2WR */
			0x0000000e, /* EMC_PDEX2RD */
			0x00000001, /* EMC_PCHG2PDEN */
			0x00000000, /* EMC_ACT2PDEN */
			0x0000000c, /* EMC_AR2PDEN */
			0x00000016, /* EMC_RW2PDEN */
			0x000000b5, /* EMC_TXSR */
			0x00000200, /* EMC_TXSRDLL */
			0x00000005, /* EMC_TCKE */
			0x0000001b, /* EMC_TFAW */
			0x00000000, /* EMC_TRPAB */
			0x00000006, /* EMC_TCLKSTABLE */
			0x00000007, /* EMC_TCLKSTOP */
			0x00001453, /* EMC_TREFBW */
			0x0000000b, /* EMC_QUSE_EXTRA */
			0x00000006, /* EMC_FBIO_CFG6 */
			0x00000000, /* EMC_ODT_WRITE */
			0x00000000, /* EMC_ODT_READ */
			0x00005088, /* EMC_FBIO_CFG5 */
			0xf00b0191, /* EMC_CFG_DIG_DLL */
			0x00008000, /* EMC_CFG_DIG_DLL_PERIOD */
			0x0000000a, /* EMC_DLL_XFORM_DQS0 */
			0x0000000a, /* EMC_DLL_XFORM_DQS1 */
			0x0000000a, /* EMC_DLL_XFORM_DQS2 */
			0x0000000a, /* EMC_DLL_XFORM_DQS3 */
			0x0000000a, /* EMC_DLL_XFORM_DQS4 */
			0x0000000a, /* EMC_DLL_XFORM_DQS5 */
			0x0000000a, /* EMC_DLL_XFORM_DQS6 */
			0x0000000a, /* EMC_DLL_XFORM_DQS7 */
			0x00018000, /* EMC_DLL_XFORM_QUSE0 */
			0x00018000, /* EMC_DLL_XFORM_QUSE1 */
			0x00018000, /* EMC_DLL_XFORM_QUSE2 */
			0x00018000, /* EMC_DLL_XFORM_QUSE3 */
			0x00018000, /* EMC_DLL_XFORM_QUSE4 */
			0x00018000, /* EMC_DLL_XFORM_QUSE5 */
			0x00018000, /* EMC_DLL_XFORM_QUSE6 */
			0x00018000, /* EMC_DLL_XFORM_QUSE7 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS0 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS1 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS2 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS3 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS4 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS5 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS6 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS7 */
			0x0000000c, /* EMC_DLL_XFORM_DQ0 */
			0x0000000c, /* EMC_DLL_XFORM_DQ1 */
			0x0000000c, /* EMC_DLL_XFORM_DQ2 */
			0x0000000c, /* EMC_DLL_XFORM_DQ3 */
			0x000002a0, /* EMC_XM2CMDPADCTRL */
			0x0800013d, /* EMC_XM2DQSPADCTRL2 */
			0x22220000, /* EMC_XM2DQPADCTRL2 */
			0x77fff884, /* EMC_XM2CLKPADCTRL */
			0x01f1f501, /* EMC_XM2COMPPADCTRL */
			0x07077404, /* EMC_XM2VTTGENPADCTRL */
			0x54000000, /* EMC_XM2VTTGENPADCTRL2 */
			0x080001e8, /* EMC_XM2QUSEPADCTRL */
			0x08000021, /* EMC_XM2DQSPADCTRL3 */
			0x00000802, /* EMC_CTT_TERM_CTRL */
			0x00020000, /* EMC_ZCAL_INTERVAL */
			0x00000040, /* EMC_ZCAL_WAIT_CNT */
			0x0113000c, /* EMC_MRS_WAIT_CNT */
			0xa0f10000, /* EMC_AUTO_CAL_CONFIG */
			0x00000000, /* EMC_CTT */
			0x00000000, /* EMC_CTT_DURATION */
			0x800028a5, /* EMC_DYN_SELF_REF_CONTROL */
			0x00000014, /* MC_EMEM_ARB_CFG */
			0xc0000079, /* MC_EMEM_ARB_OUTSTANDING_REQ */
			0x00000003, /* MC_EMEM_ARB_TIMING_RCD */
			0x00000004, /* MC_EMEM_ARB_TIMING_RP */
			0x00000010, /* MC_EMEM_ARB_TIMING_RC */
			0x0000000a, /* MC_EMEM_ARB_TIMING_RAS */
			0x0000000d, /* MC_EMEM_ARB_TIMING_FAW */
			0x00000002, /* MC_EMEM_ARB_TIMING_RRD */
			0x00000003, /* MC_EMEM_ARB_TIMING_RAP2PRE */
			0x0000000b, /* MC_EMEM_ARB_TIMING_WAP2PRE */
			0x00000002, /* MC_EMEM_ARB_TIMING_R2R */
			0x00000002, /* MC_EMEM_ARB_TIMING_W2W */
			0x00000004, /* MC_EMEM_ARB_TIMING_R2W */
			0x00000008, /* MC_EMEM_ARB_TIMING_W2R */
			0x08040202, /* MC_EMEM_ARB_DA_TURNS */
			0x00130b10, /* MC_EMEM_ARB_DA_COVERS */
			0x70ea1f11, /* MC_EMEM_ARB_MISC0 */
			0x001f0000, /* MC_EMEM_ARB_RING1_THROTTLE */
			0xf8000000, /* EMC_FBIO_SPARE */
			0xff00ff49, /* EMC_CFG_RSV */
		},
		0x00000040, /* EMC_ZCAL_WAIT_CNT after clock change */
		0x001fffff, /* EMC_AUTO_CAL_INTERVAL */
		0x00000000, /* EMC_CFG.PERIODIC_QRST */
		0x80000321, /* Mode Register 0 */
		0x80100002, /* Mode Register 1 */
		0x80200000, /* Mode Register 2 */
		0x00000000, /* EMC_CFG.DYN_SELF_REF */
	},
	{
		0x32,       /* Rev 3.2 */
		667000,     /* SDRAM frequency */
		{
			0x0000001f, /* EMC_RC */
			0x000000ac, /* EMC_RFC */
			0x00000016, /* EMC_RAS */
			0x00000008, /* EMC_RP */
			0x00000005, /* EMC_R2W */
			0x0000000c, /* EMC_W2R */
			0x00000003, /* EMC_R2P */
			0x00000011, /* EMC_W2P */
			0x00000008, /* EMC_RD_RCD */
			0x00000008, /* EMC_WR_RCD */
			0x00000003, /* EMC_RRD */
			0x00000001, /* EMC_REXT */
			0x00000000, /* EMC_WEXT */
			0x00000007, /* EMC_WDV */
			0x0000000a, /* EMC_QUSE */
			0x00000009, /* EMC_QRST */
			0x0000000b, /* EMC_QSAFE */
			0x00000011, /* EMC_RDV */
			0x00001412, /* EMC_REFRESH */
			0x00000000, /* EMC_BURST_REFRESH_NUM */
			0x00000504, /* EMC_PRE_REFRESH_REQ_CNT */
			0x00000002, /* EMC_PDEX2WR */
			0x0000000e, /* EMC_PDEX2RD */
			0x00000001, /* EMC_PCHG2PDEN */
			0x00000000, /* EMC_ACT2PDEN */
			0x0000000c, /* EMC_AR2PDEN */
			0x00000016, /* EMC_RW2PDEN */
			0x000000b5, /* EMC_TXSR */
			0x00000200, /* EMC_TXSRDLL */
			0x00000005, /* EMC_TCKE */
			0x0000001b, /* EMC_TFAW */
			0x00000000, /* EMC_TRPAB */
			0x00000006, /* EMC_TCLKSTABLE */
			0x00000007, /* EMC_TCLKSTOP */
			0x00001453, /* EMC_TREFBW */
			0x0000000b, /* EMC_QUSE_EXTRA */
			0x00000006, /* EMC_FBIO_CFG6 */
			0x00000000, /* EMC_ODT_WRITE */
			0x00000000, /* EMC_ODT_READ */
			0x00005088, /* EMC_FBIO_CFG5 */
			0xf00b0191, /* EMC_CFG_DIG_DLL */
			0x00008000, /* EMC_CFG_DIG_DLL_PERIOD */
			0x0000000a, /* EMC_DLL_XFORM_DQS0 */
			0x0000000a, /* EMC_DLL_XFORM_DQS1 */
			0x0000000a, /* EMC_DLL_XFORM_DQS2 */
			0x0000000a, /* EMC_DLL_XFORM_DQS3 */
			0x0000000a, /* EMC_DLL_XFORM_DQS4 */
			0x0000000a, /* EMC_DLL_XFORM_DQS5 */
			0x0000000a, /* EMC_DLL_XFORM_DQS6 */
			0x0000000a, /* EMC_DLL_XFORM_DQS7 */
			0x00018000, /* EMC_DLL_XFORM_QUSE0 */
			0x00018000, /* EMC_DLL_XFORM_QUSE1 */
			0x00018000, /* EMC_DLL_XFORM_QUSE2 */
			0x00018000, /* EMC_DLL_XFORM_QUSE3 */
			0x00018000, /* EMC_DLL_XFORM_QUSE4 */
			0x00018000, /* EMC_DLL_XFORM_QUSE5 */
			0x00018000, /* EMC_DLL_XFORM_QUSE6 */
			0x00018000, /* EMC_DLL_XFORM_QUSE7 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS0 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS1 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS2 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS3 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS4 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS5 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS6 */
			0x00000000, /* EMC_DLI_TRIM_TXDQS7 */
			0x0000000c, /* EMC_DLL_XFORM_DQ0 */
			0x0000000c, /* EMC_DLL_XFORM_DQ1 */
			0x0000000c, /* EMC_DLL_XFORM_DQ2 */
			0x0000000c, /* EMC_DLL_XFORM_DQ3 */
			0x000002a0, /* EMC_XM2CMDPADCTRL */
			0x0800013d, /* EMC_XM2DQSPADCTRL2 */
			0x22220000, /* EMC_XM2DQPADCTRL2 */
			0x77fff884, /* EMC_XM2CLKPADCTRL */
			0x01f1f501, /* EMC_XM2COMPPADCTRL */
			0x07077404, /* EMC_XM2VTTGENPADCTRL */
			0x54000000, /* EMC_XM2VTTGENPADCTRL2 */
			0x080001e8, /* EMC_XM2QUSEPADCTRL */
			0x08000021, /* EMC_XM2DQSPADCTRL3 */
			0x00000802, /* EMC_CTT_TERM_CTRL */
			0x00020000, /* EMC_ZCAL_INTERVAL */
			0x00000040, /* EMC_ZCAL_WAIT_CNT */
			0x0113000c, /* EMC_MRS_WAIT_CNT */
			0xa0f10000, /* EMC_AUTO_CAL_CONFIG */
			0x00000000, /* EMC_CTT */
			0x00000000, /* EMC_CTT_DURATION */
			0x800028a5, /* EMC_DYN_SELF_REF_CONTROL */
			0x00000014, /* MC_EMEM_ARB_CFG */
			0xc0000079, /* MC_EMEM_ARB_OUTSTANDING_REQ */
			0x00000003, /* MC_EMEM_ARB_TIMING_RCD */
			0x00000004, /* MC_EMEM_ARB_TIMING_RP */
			0x00000010, /* MC_EMEM_ARB_TIMING_RC */
			0x0000000a, /* MC_EMEM_ARB_TIMING_RAS */
			0x0000000d, /* MC_EMEM_ARB_TIMING_FAW */
			0x00000002, /* MC_EMEM_ARB_TIMING_RRD */
			0x00000003, /* MC_EMEM_ARB_TIMING_RAP2PRE */
			0x0000000b, /* MC_EMEM_ARB_TIMING_WAP2PRE */
			0x00000002, /* MC_EMEM_ARB_TIMING_R2R */
			0x00000002, /* MC_EMEM_ARB_TIMING_W2W */
			0x00000004, /* MC_EMEM_ARB_TIMING_R2W */
			0x00000008, /* MC_EMEM_ARB_TIMING_W2R */
			0x08040202, /* MC_EMEM_ARB_DA_TURNS */
			0x00130b10, /* MC_EMEM_ARB_DA_COVERS */
			0x70ea1f11, /* MC_EMEM_ARB_MISC0 */
			0x001f0000, /* MC_EMEM_ARB_RING1_THROTTLE */
			0xf8000000, /* EMC_FBIO_SPARE */
			0xff00ff49, /* EMC_CFG_RSV */
		},
		0x00000040, /* EMC_ZCAL_WAIT_CNT after clock change */
		0x001fffff, /* EMC_AUTO_CAL_INTERVAL */
		0x00000001, /* EMC_CFG.PERIODIC_QRST */
		0x80000b71, /* Mode Register 0 */
		0x80100002, /* Mode Register 1 */
		0x80200018, /* Mode Register 2 */
		0x00000000, /* EMC_CFG.DYN_SELF_REF */
	},
};

#include "gpio-names.h"
int n710_emc_init(void)
{
	int ret=0;
	int mem_bootstrap_ad4=0,mem_bootstrap_ad5=0;
	#define MEMORY_BOOSTRAP_PIN_AD4 TEGRA_GPIO_PG4
	#define MEMORY_BOOSTRAP_PIN_AD5 TEGRA_GPIO_PG5

	tegra_gpio_enable( MEMORY_BOOSTRAP_PIN_AD4);
       ret = gpio_request( MEMORY_BOOSTRAP_PIN_AD4, "memory_bootstrap_ad4");
	if (ret < 0) {
		printk("keenhi_emc_init: request MEMORY_BOOSTRAP_PIN_AD4 failed\n");
		WARN_ON(1);
		goto err_handle;
	}

	ret= gpio_direction_input(MEMORY_BOOSTRAP_PIN_AD4);
	if (ret < 0) {
		printk("keenhi_emc_init: failed to configure MEMORY_BOOSTRAP_PIN_AD4\n");
		WARN_ON(1);
		goto err_handle;
	}
	mem_bootstrap_ad4=gpio_get_value(MEMORY_BOOSTRAP_PIN_AD4);

	tegra_gpio_enable( MEMORY_BOOSTRAP_PIN_AD5);
       ret = gpio_request( MEMORY_BOOSTRAP_PIN_AD5, "memory_bootstrap_ad5");
	if (ret < 0) {
		printk("keenhi_emc_init: request MEMORY_BOOSTRAP_PIN_AD5 failed\n");
		WARN_ON(1);
		goto err_handle;
	}

	ret= gpio_direction_input(MEMORY_BOOSTRAP_PIN_AD5);
	if (ret < 0) {
		printk("keenhi_emc_init: failed to configure MEMORY_BOOSTRAP_PIN_AD4\n");
		WARN_ON(1);
		goto err_handle;
	}
	mem_bootstrap_ad5=gpio_get_value(MEMORY_BOOSTRAP_PIN_AD5);

	printk("keenhi_emc_init:mem_bootstrap_ad4=%u mem_bootstrap_ad5=%u \n",mem_bootstrap_ad4,mem_bootstrap_ad5);
err_handle:
	tegra_init_emc(Micron_Elpida_1GB_ddr3_667Mhz_kh_sdmmc4_x8_12M_0704,
			ARRAY_SIZE(Micron_Elpida_1GB_ddr3_667Mhz_kh_sdmmc4_x8_12M_0704));
	printk("n710_emc_init:Micron_Elpida_1GB_ddr3_667Mhz_kh_sdmmc4_x8_12M_0704\n");
		
	return 0;
}
