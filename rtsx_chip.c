/* Driver for Realtek PCI-Express card reader
 *
 * Copyright(c) 2009 Realtek Semiconductor Corp. All rights reserved.  
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http:
 *
 * Author:
 *   wwang (wei_wang@realsil.com.cn)
 *   No. 450, Shenhu Road, Suzhou Industry Park, Suzhou, China
 */

#include <linux/blkdev.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/vmalloc.h>

#include "rtsx.h"
#include "rtsx_transport.h"
#include "rtsx_scsi.h"
#include "rtsx_card.h"
#include "rtsx_chip.h"
#include "rtsx_sys.h"
#include "general.h"

#include "sd.h"
#include "xd.h"
#include "ms.h"

void rtsx_disable_card_int(struct rtsx_chip *chip)
{
	u32 reg = rtsx_readl(chip, RTSX_BIER);
	
	reg &= ~(XD_INT_EN | SD_INT_EN | MS_INT_EN);
	rtsx_writel(chip, RTSX_BIER, reg);
}

void rtsx_enable_card_int(struct rtsx_chip *chip)
{
	u32 reg = rtsx_readl(chip, RTSX_BIER);
	int i;
	
	for (i = 0; i <= chip->max_lun; i++) {
		
		if (chip->lun2card[i] & XD_CARD) {
			reg |= XD_INT_EN;
		}
		if (chip->lun2card[i] & SD_CARD) {
			reg |= SD_INT_EN;
		}
		if (chip->lun2card[i] & MS_CARD) {
			reg |= MS_INT_EN;
		}
	}
	if (chip->hw_bypass_sd) {
		reg &= ~((u32)SD_INT_EN);
	}
	
	rtsx_writel(chip, RTSX_BIER, reg);
}

void rtsx_enable_bus_int(struct rtsx_chip *chip)
{
	u32 reg = 0;
#ifndef DISABLE_CARD_INT
	int i;
#endif

	reg = TRANS_OK_INT_EN | TRANS_FAIL_INT_EN;
	
#ifndef DISABLE_CARD_INT
	for (i = 0; i <= chip->max_lun; i++) {
		RTSX_DEBUGP(("lun2card[%d] = 0x%02x\n", i, chip->lun2card[i]));
		
		if (chip->lun2card[i] & XD_CARD) {
			reg |= XD_INT_EN;
		}
		if (chip->lun2card[i] & SD_CARD) {
			reg |= SD_INT_EN;
		}
		if (chip->lun2card[i] & MS_CARD) {
			reg |= MS_INT_EN;
		}
	}
	if (chip->hw_bypass_sd) {
		reg &= ~((u32)SD_INT_EN);
	}
#endif

#ifdef SUPPORT_OCP
	reg |= SD_OC_INT_EN;
#endif
	if (!chip->adma_mode) {
		reg |= DATA_DONE_INT_EN;
	}

	rtsx_writel(chip, RTSX_BIER, reg);
	
	RTSX_DEBUGP(("RTSX_BIER: 0x%08x\n", reg));
}

void rtsx_disable_bus_int(struct rtsx_chip *chip)
{
	rtsx_writel(chip, RTSX_BIER, 0);
}

#ifdef HW_AUTO_SWITCH_SD_BUS
static int rtsx_pre_handle_sdio(struct rtsx_chip *chip)
{
	u8 tmp;
	int sw_bypass_sd = 0;
	int retval;

	if (chip->driver_first_load) {
		RTSX_READ_REG(chip, SDIO_CFG, &tmp);
		if (tmp & SDIO_BUS_AUTO_SWITCH) {
			sw_bypass_sd = 1;
		}
	} else {
		if (chip->sdio_in_charge) {
			sw_bypass_sd = 1;
		}
	}
	RTSX_DEBUGP(("chip->sdio_in_charge = %d\n", chip->sdio_in_charge));
	RTSX_DEBUGP(("chip->driver_first_load = %d\n", chip->driver_first_load));
	RTSX_DEBUGP(("sw_bypass_sd = %d\n", sw_bypass_sd));

	if (sw_bypass_sd) {
		u8 cd_toggle_mask = 0;
		
		RTSX_READ_REG(chip, TLPTISTAT, &tmp);
		cd_toggle_mask = 0x10;
		if (tmp & cd_toggle_mask) {
			RTSX_WRITE_REG(chip, SDIO_CFG, SDIO_BUS_AUTO_SWITCH, 0);
			RTSX_WRITE_REG(chip, TLPTISTAT, 0xFF, tmp);
			
			chip->need_reset |= SD_CARD;
		} else {
			RTSX_DEBUGP(("Chip inserted with SDIO!\n"));
						
			if (chip->asic_code) {
				retval = sd_pull_ctl_enable(chip);
				if (retval != STATUS_SUCCESS) {
					TRACE_RET(chip, STATUS_FAIL);
				}
			} else {
				RTSX_WRITE_REG(chip, FPGA_PULL_CTL, 
					FPGA_SD_CD_PULL_CTL_MASK | FPGA_MS_CD_PULL_CTL_MASK | FPGA_SD_PULL_CTL_MASK,
					FPGA_SD_CD_PULL_CTL_EN | FPGA_MS_CD_PULL_CTL_DIS | FPGA_SD_PULL_CTL_EN);
			}
			retval = card_share_mode(chip, SD_CARD);
			if (retval != STATUS_SUCCESS) {
				TRACE_RET(chip, STATUS_FAIL);
			}
			
			RTSX_WRITE_REG(chip, SDIO_CFG, SDIO_BUS_AUTO_SWITCH, SDIO_BUS_AUTO_SWITCH);
			chip->chip_insert_with_sdio = 1;
			chip->sd_io = 1;
		}
	} else {
		RTSX_WRITE_REG(chip, TLPTISTAT, 0x10, 0x10);
		chip->need_reset |= SD_CARD;
	}
	
	return STATUS_SUCCESS;
}
#else
static int rtsx_pre_handle_sdio(struct rtsx_chip *chip)
{
	if (chip->ignore_sd && chip->sdio_func_exist) {
		if (chip->asic_code) {
			RTSX_WRITE_REG(chip, CARD_PULL_CTL5, 0xFF, 
				MS_INS_PU | SD_WP_PU | SD_CD_PU | SD_CMD_PU);
		} else {
			RTSX_WRITE_REG(chip, FPGA_PULL_CTL, 0xFF, FPGA_SD_PULL_CTL_EN);
		}
		RTSX_WRITE_REG(chip, CARD_SHARE_MODE, 0xFF, CARD_SHARE_48_SD);

		RTSX_WRITE_REG(chip, 0xFF2C, 0x01, 0x01);

		RTSX_WRITE_REG(chip, SDIO_CTRL, 0xFF, SDIO_BUS_CTRL | SDIO_CD_CTRL);

		chip->sd_int = 1;
		chip->sd_io = 1;			
	} else {
		chip->need_reset |= SD_CARD;
	}
	
	return STATUS_SUCCESS;
}
#endif

static int rtsx_init_host_aspm(struct rtsx_chip *chip)
{
	int retval;
	int save_aspm = 1;
	u8 val;

	retval = rtsx_cfg_try_lock(chip, 10000);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);

	retval = rtsx_set_cr_attach(chip, 1);
	if (retval != STATUS_SUCCESS)
		TRACE_GOTO(chip, Out);

	if (rtsx_chk_nic_attach(chip) > 0) {
		if (rtsx_chk_root_aspm_flag(chip) > 0)
			save_aspm = 0;
	}

	if (save_aspm) {
		rtsx_get_host_aspm(chip, &val);
		RTSX_DEBUGP(("Initial Host ASPM value: 0x%x\n", val));
		retval = rtsx_set_root_aspm_val(chip, val);
		if (retval != STATUS_SUCCESS)
			TRACE_GOTO(chip, Out);
		retval = rtsx_set_root_aspm_flag(chip, 1);
		if (retval != STATUS_SUCCESS)
			TRACE_GOTO(chip, Out);
	} else {
		retval = rtsx_get_root_aspm_val(chip, &val);
		if (retval != STATUS_SUCCESS)
			TRACE_GOTO(chip, Out);
		RTSX_DEBUGP(("Read root_aspm_val set by NIC: 0x%x\n", val));
	}

Out:
	(void)rtsx_cfg_unlock(chip);
	return retval;
}

static int rtsx_read_aspm_val_from_nic(struct rtsx_chip *chip)
{
	int retval;
	u8 aspm_val;

	RTSX_DEBUGP(("aspm_mod_flag set!\n"));

	retval = rtsx_get_aspm_val(chip, &aspm_val);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);
	RTSX_DEBUGP(("aspm_val = 0x%x\n", aspm_val));

	if (chip->host_aspm_val != (chip->aspm_l0s_l1_en & aspm_val)) {
		chip->host_aspm_enabled = 0;
		chip->host_aspm_val = chip->aspm_l0s_l1_en & aspm_val;
	}
	RTSX_DEBUGP(("chip->host_aspm_enabled = %d\n",
				chip->host_aspm_enabled));

	retval = rtsx_set_aspm_mod_flag(chip, 0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);
	
	return STATUS_SUCCESS;
}

static int rtsx_configure_host_aspm(struct rtsx_chip *chip)
{
	int retval;

	retval = rtsx_cfg_try_lock(chip, 10000);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);

	if (rtsx_chk_nic_attach(chip) > 0) {
		if (rtsx_chk_aspm_mod_flag(chip) > 0) {
			retval = rtsx_read_aspm_val_from_nic(chip);
			if (retval != STATUS_SUCCESS)
				TRACE_GOTO(chip, Out);
		}
	} else {
		if (chip->host_aspm_val != chip->aspm_l0s_l1_en) {
			RTSX_DEBUGP(("After NIC detached, " 
					"chip->host_aspm_enabled = %d\n",
					chip->host_aspm_enabled));
			chip->host_aspm_enabled = 0;
			chip->host_aspm_val = chip->aspm_l0s_l1_en;
		}
	}

Out:
	(void)rtsx_cfg_unlock(chip);
	return retval;
}

static int rtsx_restore_host_aspm_halt(struct rtsx_chip *chip)
{
	int retval;
	u8 val;

	retval = rtsx_cfg_try_lock(chip, 10000);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);

	if (rtsx_chk_nic_attach(chip) == 0) {
		retval = rtsx_get_root_aspm_val(chip, &val);
		if (retval != STATUS_SUCCESS)
			TRACE_GOTO(chip, Out);

		rtsx_set_host_aspm(chip, val);

		retval = rtsx_set_root_aspm_flag(chip, 0);
		if (retval != STATUS_SUCCESS)
			TRACE_RET(chip, retval);
	}

	retval = rtsx_set_cr_attach(chip, 0);
	if (retval != STATUS_SUCCESS)
		TRACE_GOTO(chip, Out);

Out:
	(void)rtsx_cfg_unlock(chip);
	return retval;
}

static int rtsx_restore_host_aspm_sleep(struct rtsx_chip *chip)
{
	int retval;
	u8 val;

	retval = rtsx_get_root_aspm_val(chip, &val);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);

	rtsx_set_host_aspm(chip, val);

	retval = rtsx_set_root_aspm_flag(chip, 0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);

	retval = rtsx_set_cr_attach(chip, 0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, retval);

	return STATUS_SUCCESS;
}

static int rtsx_lan_ctl(struct rtsx_chip *chip, u32 val)
{
	int retval;
	int i;
	u32 tmp = 0;

	retval = rtsx_write_cfg_dw(chip, 1, 0x820, 0xFFFFFFFF, val);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	for (i = 0; i < 3000; i++) {
		retval = rtsx_read_cfg_dw(chip, 1, 0x820, &tmp);
		if (retval != STATUS_SUCCESS)
			TRACE_RET(chip, STATUS_FAIL);

		if (!(tmp & 0xF0000000))
			break;

		mdelay(1);
	}

	if (tmp & 0xF0000000)
		TRACE_RET(chip, STATUS_FAIL);

	return STATUS_SUCCESS;
}

static inline int rtsx_disable_lan_setting_8411(struct rtsx_chip *chip)
{
	int retval;
	u32 val;

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, 0x8019fe25);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f0080);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, 0x801f0000);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f0060);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, 0x80151006);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f0060);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, 0x80001800);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f0060);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x800f00d0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_read_cfg_dw(chip, 1, 0x824, &val);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, val | 0x40);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f00d0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x800f00f0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_read_cfg_dw(chip, 1, 0x824, &val);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, val | 0x400000);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f00f0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	return STATUS_SUCCESS;
}

static inline int rtsx_disable_lan_setting_8402(struct rtsx_chip *chip)
{
	int retval;

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, 0x8019ff64);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f0080);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, 0x801f0000);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f0060);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_write_cfg_dw(chip, 1, 0x824, 0xFFFFFFFF, 0x80188310);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_lan_ctl(chip, 0x400f0060);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	return STATUS_SUCCESS;
}

static int rtsx_disable_lan_setting(struct rtsx_chip *chip)
{
	if (CHECK_PID(chip, RTL8411) || CHECK_PID(chip, RTL8411B))
		return rtsx_disable_lan_setting_8411(chip);

	return rtsx_disable_lan_setting_8402(chip);
}

int rtsx_reset_chip(struct rtsx_chip *chip)
{
	int retval;
	int i;
	u32 val;

	rtsx_writel(chip, RTSX_HCBAR, chip->host_cmds_addr);

	RTSX_WRITE_REG(chip, HOST_SLEEP_STATE, 0x03, 0x00);

	RTSX_WRITE_REG(chip, CARD_CLK_EN, 0x1E, 0);

#ifdef SUPPORT_OCP
	RTSX_WRITE_REG(chip, FPDCTL, OC_POWER_DOWN, 0);
	
	RTSX_WRITE_REG(chip, OCPPARA1, SD_OCP_TIME_MASK, SD_OCP_TIME_800);
	RTSX_WRITE_REG(chip, OCPPARA2, SD_OCP_THD_MASK, chip->sd_400mA_ocp_thd);
	RTSX_WRITE_REG(chip, OCPGLITCH, SD_OCP_GLITCH_MASK, SD_OCP_GLITCH_10000);
	RTSX_WRITE_REG(chip, OCPCTL, 0xFF, SD_OCP_INT_EN | SD_DETECT_EN);
#else
	RTSX_WRITE_REG(chip, FPDCTL, OC_POWER_DOWN, OC_POWER_DOWN);
#endif

	RTSX_WRITE_REG(chip, CARD_GPIO_DIR, 0xFF, 0x0F);

	RTSX_WRITE_REG(chip, CARD_GPIO, 0xFF, 0xFF);
	
	RTSX_WRITE_REG(chip, CHANGE_LINK_STATE, 0x0A, 0);
	
	RTSX_WRITE_REG(chip, CARD_DRIVE_SEL, 0xFF, chip->card_drive_sel);
	RTSX_WRITE_REG(chip, SD30_DRIVE_SEL, 0x07, chip->sd30_drive_sel_3v3);

#ifdef LED_AUTO_BLINK
	RTSX_WRITE_REG(chip, CARD_AUTO_BLINK, 0xFF, 
			LED_BLINK_SPEED | BLINK_EN | LED_GPIO0);
#endif

	if (chip->asic_code) {
		RTSX_WRITE_REG(chip, SSC_CTL1, 0xFF, SSC_8X_EN | SSC_SEL_4M);
		RTSX_WRITE_REG(chip, SSC_CTL2, 0xFF, 0x12);
	} else {
		RTSX_WRITE_REG(chip, FPGA_PULL_CTL, 
			       FPGA_SHARE_CD_PULL_CTL_MASK, FPGA_SHARE_CD_PULL_CTL_EN);
		
		if (CHECK_PID(chip, RTL8411) || CHECK_PID(chip, RTL8411B)) {
			RTSX_WRITE_REG(chip, LDO_CTL, 
				LDO_PWR_SEL_8411 | (REG_TUNED18 << TUNED18_SHIFT_8411), 
				LDO_FROM_DV33_8411 | (FPGA_3V3 << TUNED18_SHIFT_8411));
		} else {
			RTSX_WRITE_REG(chip, LDO_CTL, 
				LDO_PWR_SEL_8402 | (REG_TUNED18 << TUNED18_SHIFT_8402), 
				LDO_FROM_DV33_8402 | (FPGA_3V3 << TUNED18_SHIFT_8402));
		}
	}

#ifdef LDO_USING_CARD3V3
	RTSX_WRITE_REG(chip, CD_PAD_CTL, CD_DISABLE_MASK, CD_ENABLE);
#else
	RTSX_WRITE_REG(chip, CD_PAD_CTL, CD_DISABLE_MASK | CD_AUTO_DISABLE, CD_ENABLE);
#endif

	RTSX_WRITE_REG(chip, CHANGE_LINK_STATE, 0x16, 0x10);

	if (chip->aspm_l0s_l1_en) {
		if (chip->dynamic_aspm) {
			if (chip->sdio_func_exist) {
				retval = rtsx_write_cfg_dw(chip, 1, 0xC0, 0xFF, chip->aspm_l0s_l1_en);
				if (retval != STATUS_SUCCESS) {
					TRACE_RET(chip, STATUS_FAIL);
				}
			}
		} else {
			retval = rtsx_write_config_byte(chip, LCTLR, chip->aspm_l0s_l1_en);
			if (retval != STATUS_SUCCESS) {
				TRACE_RET(chip, STATUS_FAIL);
			}
			chip->aspm_level[0] = chip->aspm_l0s_l1_en;
			if (chip->sdio_func_exist) {
				chip->aspm_level[1] = chip->aspm_l0s_l1_en;
				retval = rtsx_write_cfg_dw(chip, 1, 0xC0, 0xFF, chip->aspm_l0s_l1_en);
				if (retval != STATUS_SUCCESS) {
					TRACE_RET(chip, STATUS_FAIL);
				}
			}

			chip->aspm_enabled = 1;
		}

		if (chip->config_host_aspm && chip->handshake_en) {
			retval = rtsx_init_host_aspm(chip);
			if (retval != STATUS_SUCCESS)
				TRACE_RET(chip, STATUS_FAIL);
		}
	} else {
		retval = rtsx_write_config_byte(chip, LCTLR, chip->aspm_l0s_l1_en);
		if (retval != STATUS_SUCCESS) {
			TRACE_RET(chip, STATUS_FAIL);
		}
	}	
	
	retval = rtsx_write_config_byte(chip, 0x81, 1);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}
	
	if (chip->sdio_func_exist) {
		retval = rtsx_write_cfg_dw(chip, 1, 0xC0, 0xFF00, 0x0100);
		if (retval != STATUS_SUCCESS) {
			TRACE_RET(chip, STATUS_FAIL);
		}
	}
	
	retval = rtsx_write_cfg_dw(chip, 0, 0x70C, 0xFF000000, 0x5B);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}
	
	RTSX_WRITE_REG(chip, IRQSTAT0, LINK_RDY_INT, LINK_RDY_INT);

	RTSX_WRITE_REG(chip, PERST_GLITCH_WIDTH, 0xFF, 0x80);
	
	rtsx_enable_bus_int(chip);
	
#ifdef HW_INT_WRITE_CLR
	RTSX_WRITE_REG(chip, NFTS_TX_CTRL, 0x02, 0);
#endif
	
	chip->need_reset = 0;

	for (i = 0; i < 10; i++) {
		chip->int_reg = rtsx_readl(chip, RTSX_BIPR);
		wait_timeout(5);
	}
#ifdef HW_INT_WRITE_CLR
	rtsx_writel(chip, RTSX_BIPR, chip->int_reg);
#endif
	if (chip->hw_bypass_sd) {
		goto NextCard;
	}
	RTSX_DEBUGP(("In rtsx_reset_chip, chip->int_reg = 0x%x\n", chip->int_reg));
	if (chip->int_reg & SD_EXIST) {
		retval = rtsx_pre_handle_sdio(chip);
		RTSX_DEBUGP(("chip->need_reset = 0x%x (rtsx_reset_chip)\n", (unsigned int)(chip->need_reset)));
		if (retval != STATUS_SUCCESS) {
			TRACE_RET(chip, STATUS_FAIL);
		}
	} else {
		chip->sd_io = 0;
		RTSX_WRITE_REG(chip, SDIO_CTRL, SDIO_BUS_CTRL | SDIO_CD_CTRL, 0);
	}

NextCard:
	if (chip->int_reg & XD_EXIST) {
		chip->need_reset |= XD_CARD;
	}
	if (chip->int_reg & MS_EXIST) {
		chip->need_reset |= MS_CARD;
	}
	if (chip->int_reg & CARD_EXIST) {
		RTSX_WRITE_REG(chip, SSC_CTL1, SSC_RSTB, SSC_RSTB);
	}

	RTSX_DEBUGP(("In rtsx_init_chip, chip->need_reset = 0x%x\n", (unsigned int)(chip->need_reset)));

	RTSX_WRITE_REG(chip, RCCTL, 0x01, 0x00);


	if (chip->remote_wakeup_en && !chip->auto_delink_en) {
		RTSX_WRITE_REG(chip, WAKE_SEL_CTL, 0x07, 0x07);
		if (chip->aux_pwr_exist) {
			RTSX_WRITE_REG(chip, PME_FORCE_CTL, 0xFF, 0x33);
		}
	} else {
		RTSX_WRITE_REG(chip, WAKE_SEL_CTL, 0x07, 0x04);
		RTSX_WRITE_REG(chip, PME_FORCE_CTL, 0xFF, 0x30);
	}

	if (chip->force_clkreq_0) {
		RTSX_WRITE_REG(chip, PETXCFG, 0x08, 0x08);
	} else {
		RTSX_WRITE_REG(chip, PETXCFG, 0x08, 0x00);
	}
	
	retval = rtsx_read_cfg_dw(chip, 0, 0x44, &val);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}
	val |= 0x0100;
	retval = rtsx_write_cfg_dw(chip, 0, 0x44, 0xFF00, val);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}
	retval = rtsx_read_cfg_dw(chip, 1, 0x44, &val);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}
	val |= 0x0100;
	retval = rtsx_write_cfg_dw(chip, 1, 0x44, 0xFF00, val);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}

	if (chip->ft2_fast_mode) {
		card_power_on(chip, 0);
		
		wait_timeout(200);
	}

	if (!chip->nic_func_exist && chip->handshake_en) {
		retval = rtsx_disable_lan_setting(chip);
		if (retval != STATUS_SUCCESS)
			TRACE_RET(chip, STATUS_FAIL);
	}

	rtsx_reset_detected_cards(chip, 0);

	chip->driver_first_load = 0;
	
	return STATUS_SUCCESS;
}

static inline int check_sd_speed_prior(u32 sd_speed_prior)
{
	int i, fake_para = 0;
	
	for (i = 0; i < 4; i++) {
		u8 tmp = (u8)(sd_speed_prior >> (i*8));
		if ((tmp < 0x01) || (tmp > 0x04)) {
			fake_para = 1;
			break;
		}
	}
	
	return !fake_para;
}

static inline int check_sd_current_prior(u32 sd_current_prior)
{
	int i, fake_para = 0;
	
	for (i = 0; i < 4; i++) {
		u8 tmp = (u8)(sd_current_prior >> (i*8));
		if (tmp > 0x03) {
			fake_para = 1;
			break;
		}
	}
	
	return !fake_para;
}

static int rtsx_init_from_hw(struct rtsx_chip *chip)
{
	int retval;
	u32 lval = 0;
	u8 val = 0;
	
	chip->aux_pwr_exist = 0;
	
	chip->ms_power_class_en = 0x03;
	
	retval = rtsx_read_cfg_dw(chip, 0, 0x724, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);
	RTSX_DEBUGP(("dw in 0x724: 0x%x\n", lval));
	val = (u8)lval;

	if (val & 0x02) {
		chip->hw_bypass_sd = 0;
	} else {
		chip->hw_bypass_sd = 1;
	}
	
	if (val & 0x04)
		chip->sdio_func_exist = 1;
	else
		chip->sdio_func_exist = 0;
	
	if (val & 0x08)
		chip->nic_func_exist = 1;
	else
		chip->nic_func_exist = 0;
		
	val = (u8)(lval >> 24);
	if (((val & 0x01) == 0) && chip->use_hw_setting) {
		u8 lun_mode[4] = {
			0xFF,
			MS_LUN,
			SD_LUN,
			DEFAULT_SINGLE,
		};
		u8 sd_drive[4] = {
			0x01,	
			0x02,	
			0x05,	
			0x03	
		};
		u8 ssc_depth[4] = {
			SSC_DEPTH_512K,
			SSC_DEPTH_1M,
			SSC_DEPTH_2M,
			SSC_DEPTH_4M,
		};

		chip->lun_mode = lun_mode[(val >> 6) & 0x03];	
		chip->aspm_l0s_l1_en = (val >> 4) & 0x03;
		chip->sd30_drive_sel_1v8 = sd_drive[(val >> 2) & 0x03];
		chip->card_drive_sel &= 0x3F;
		chip->card_drive_sel |= ((val >> 1) & 0x01) << 6;

		val = (u8)(lval >> 16);

		if ((val & 0xC0) != 0xC0) {
			chip->asic_sd_hs_clk = (49 - ((val >> 6) & 0x03) * 2) * 2;
			chip->asic_mmc_52m_clk = chip->asic_sd_hs_clk;
		}
		chip->sdr50_en = (val >> 5) & 0x01;
		chip->ddr50_en = (val >> 4) & 0x01;
		chip->sdr104_en = (val >> 3) & 0x01;
		if ((val & 0x07) != 0x07)
			chip->asic_ms_hg_clk = (59 - (val & 0x07)) * 2;

		val = (u8)(lval >> 8);

		if ((val & 0xE0) != 0xE0)	
			chip->asic_sd_sdr104_clk = 206 - ((val >> 5) & 0x07) * 3;
		if ((val & 0x1C) != 0x1C)
			chip->asic_sd_sdr50_clk = 98 - ((val >> 2) & 0x07) * 2;
		if ((val & 0x03) != 0x03)
			chip->asic_sd_ddr50_clk = (48 - (val & 0x03) * 2) * 2;

		val = (u8)lval;

		chip->ssc_depth_sd_sdr104 = ssc_depth[(val >> 6) & 0x03];
		chip->ssc_depth_sd_sdr50 = chip->ssc_depth_sd_sdr104;
		chip->ssc_depth_sd_ddr50 = ssc_depth[(val >> 4) & 0x03];
	}

	return STATUS_SUCCESS;
}

int rtsx_init_chip(struct rtsx_chip *chip)
{
	struct sd_info *sd_card = &(chip->sd_card);
	struct xd_info *xd_card = &(chip->xd_card);
	struct ms_info *ms_card = &(chip->ms_card);
	int retval;
	unsigned int i;
	u8 val;
	
	RTSX_DEBUGP(("Vendor ID: 0x%04x, Product ID: 0x%04x\n", 
		     chip->vendor_id, chip->product_id));
		     
	chip->ic_version = 0;
	
#ifdef _MSG_TRACE
	chip->msg_idx = 0;
#endif

	memset(xd_card, 0, sizeof(struct xd_info));
	memset(sd_card, 0, sizeof(struct sd_info));
	memset(ms_card, 0, sizeof(struct ms_info));
	
	chip->card_reset_cnt = 0;
	if (chip->driver_first_load)
		chip->cd_show_cnt = 0;
	else
		chip->cd_show_cnt = MAX_SHOW_CNT;
	
	chip->sd_io = 0;
	chip->auto_delink_cnt = 0;
	chip->auto_delink_allowed = 1;
	rtsx_set_stat(chip, RTSX_STAT_INIT);

	chip->sd20_mode = 0;
	chip->sd_sample_nodelay = 0;
	chip->aspm_enabled = 0;
	chip->host_aspm_enabled = 0;
	chip->chip_insert_with_sdio = 0;
	chip->sdio_aspm = 0;
	chip->sdio_idle = 0;
	chip->sdio_counter = 0;
	chip->cur_card = 0;
	chip->perf_adjust_nic = 0;
	memset(chip->sdio_raw_data, 0, 12);
	
	for (i = 0; i < MAX_ALLOWED_LUN_CNT; i++) {
		set_sense_type(chip, i, SENSE_TYPE_NO_SENSE);
		chip->rw_fail_cnt[i] = 0;
	}

	if (!check_sd_speed_prior(chip->sd_speed_prior)) {
		chip->sd_speed_prior = 0x01040203;
	}
	RTSX_DEBUGP(("sd_speed_prior = 0x%08x\n", chip->sd_speed_prior));
	
	if (!check_sd_current_prior(chip->sd_current_prior)) {
		chip->sd_current_prior = 0x00010203;
	}
	RTSX_DEBUGP(("sd_current_prior = 0x%08x\n", chip->sd_current_prior));

	if ((chip->sd_ddr_tx_phase > 31) || (chip->sd_ddr_tx_phase < 0)) {
		chip->sd_ddr_tx_phase = 0;
	}
	if ((chip->mmc_ddr_tx_phase > 31) || (chip->mmc_ddr_tx_phase < 0)) {
		chip->mmc_ddr_tx_phase = 0;
	}
	
	RTSX_WRITE_REG(chip, FPDCTL, SSC_POWER_DOWN, 0);
	udelay(200);

	RTSX_WRITE_REG(chip, CLK_DIV, 0x07, 0x07);
	
	RTSX_DEBUGP(("chip->use_hw_setting = %d\n", chip->use_hw_setting));

	retval = rtsx_init_from_hw(chip);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}

	chip->host_aspm_val = chip->aspm_l0s_l1_en;
	
	RTSX_READ_REG(chip, SYS_VER, &val);
	if (val & 0x80) {
		chip->asic_code = 0;
	} else {
		chip->asic_code = 1;
	}
	
	chip->ic_version = val & 0x0F;

	if (CHECK_PID(chip, RTL8411) || CHECK_PID(chip, RTL8411B)) {
		if (rtsx_chk_perf_prior_bios(chip) > 0) {
			rtsx_get_perf_prior(chip, &chip->perf_prior);
			chip->perf_prior_valid = 1;
		}
	} else {
		chip->perf_prior_valid = 1;
		chip->perf_prior = 0;
	}

	RTSX_DEBUGP(("chip->asic_code = %d\n", chip->asic_code));
	RTSX_DEBUGP(("chip->ic_version = 0x%x\n", chip->ic_version));
	RTSX_DEBUGP(("chip->aux_pwr_exist = %d\n", chip->aux_pwr_exist));
	RTSX_DEBUGP(("chip->sdio_func_exist = %d\n", chip->sdio_func_exist));
	RTSX_DEBUGP(("chip->nic_func_exist = %d\n", chip->nic_func_exist));
	RTSX_DEBUGP(("chip->aspm_l0s_l1_en = %d\n", chip->aspm_l0s_l1_en));
	RTSX_DEBUGP(("chip->dynamic_aspm = %d\n", chip->dynamic_aspm));
	RTSX_DEBUGP(("chip->host_aspm_val = %d\n", chip->host_aspm_val));
	RTSX_DEBUGP(("chip->lun_mode = %d\n", chip->lun_mode));
	RTSX_DEBUGP(("chip->auto_delink_en = %d\n", chip->auto_delink_en));
	RTSX_DEBUGP(("chip->baro_pkg = %d\n", chip->baro_pkg));
	RTSX_DEBUGP(("chip->perf_prior_valid = %d\n", chip->perf_prior_valid));
	RTSX_DEBUGP(("chip->perf_prior = %d\n", chip->perf_prior));

	chip->card2lun[XD_CARD] = 0;
	chip->card2lun[SD_CARD] = 0;
	chip->card2lun[MS_CARD] = 0;

	if (CHECK_LUN_MODE(chip, DEFAULT_SINGLE))
		chip->lun2card[0] = SD_CARD | MS_CARD | XD_CARD;
	else if (CHECK_LUN_MODE(chip, SD_LUN))
		chip->lun2card[0] = SD_CARD;
	else if (CHECK_LUN_MODE(chip, MS_LUN))
		chip->lun2card[0] = MS_CARD;
	else
		TRACE_RET(chip, STATUS_FAIL);

	chip->max_lun = 0;

	retval = rtsx_reset_chip(chip);
	if (retval != STATUS_SUCCESS) {
		TRACE_RET(chip, STATUS_FAIL);
	}
	
	return STATUS_SUCCESS;
}

void rtsx_release_chip(struct rtsx_chip *chip)
{
	xd_free_l2p_tbl(chip);
	ms_free_l2p_tbl(chip);
	chip->card_exist = 0;
	chip->card_ready = 0;
}

#if !defined(LED_AUTO_BLINK) && defined(REGULAR_BLINK)
static inline void rtsx_blink_led(struct rtsx_chip *chip)
{
	if (chip->card_exist && chip->blink_led) {
		if (chip->led_toggle_counter < LED_TOGGLE_INTERVAL) {
			chip->led_toggle_counter ++;
		} else {
			chip->led_toggle_counter = 0;
			toggle_gpio(chip, LED_GPIO);
		}
	}
}
#endif

void rtsx_polling_func(struct rtsx_chip *chip)
{
#ifdef SUPPORT_SD_LOCK
	struct sd_info *sd_card = &(chip->sd_card);
#endif
	
	if (rtsx_chk_stat(chip, RTSX_STAT_SUSPEND)) {
		return;
	}
	
	if (rtsx_chk_stat(chip, RTSX_STAT_DELINK)) {
		goto Delink_Stage;
	}
	
	if (chip->polling_config) {
		u8 val;
		rtsx_read_config_byte(chip, 0, &val);
	}

	if (chip->handshake_en) {
		if (!chip->perf_prior_valid) {
			if (rtsx_chk_nic_attach(chip) > 0) {
				rtsx_get_perf_prior(chip, &chip->perf_prior);
				chip->perf_prior_valid = 1;
				RTSX_DEBUGP(("chip->perf_prior = %d\n", chip->perf_prior));
			}
		}
	}

#ifdef SUPPORT_OCP
	if (chip->ocp_int) {
		rtsx_read_register(chip, OCPSTAT, &(chip->ocp_stat));
		
		if (chip->card_exist & SD_CARD) {
			sd_power_off_card3v3(chip);
		} else if (chip->card_exist & MS_CARD) {
			ms_power_off_card3v3(chip);
		} else if (chip->card_exist & XD_CARD) {
			xd_power_off_card3v3(chip);
		}
		
		chip->ocp_int = 0;
	}
#endif
		
#ifdef SUPPORT_SD_LOCK
	if (sd_card->sd_erase_status) {
		if (chip->card_exist & SD_CARD) {
			u8 val;
			rtsx_read_register(chip, SD_BUS_STAT, &val);
			if (val & SD_DAT0_STATUS) {
				sd_card->sd_erase_status = SD_NOT_ERASE;
				sd_card->sd_lock_notify = 1;
				chip->need_reinit |= SD_CARD;
			}
		} else {
			sd_card->sd_erase_status = SD_NOT_ERASE;
		}
	}
#endif

	rtsx_init_cards(chip);
	
	if (chip->idle_counter < IDLE_MAX_COUNT) {
		chip->idle_counter ++;
	} else {
		if (rtsx_get_stat(chip) != RTSX_STAT_IDLE) {
			RTSX_DEBUGP(("Idle state!\n"));
			rtsx_set_stat(chip, RTSX_STAT_IDLE);

			if (chip->handshake_en) {
				if (CHK_CR_FIRST(chip)) {
					if (chip->perf_adjust_nic) {
						rtsx_set_perf_adjust_nic(chip, 0);
						chip->perf_adjust_nic = 0;
					}
				}
			}

#if !defined(LED_AUTO_BLINK) && defined(REGULAR_BLINK)
			chip->led_toggle_counter = 0;
#endif
			rtsx_force_power_on(chip, SSC_PDCTL);
			
			turn_off_led(chip, LED_GPIO);
			
			if (chip->auto_power_down && !chip->card_ready && !chip->sd_io) {
				rtsx_force_power_down(chip, SSC_PDCTL | OC_PDCTL);
				rtsx_write_register(chip, FUNC1_FORCE_CTL,
					CR_SOFT_PFM_EN, CR_SOFT_PFM_EN);
			}
		}
	}

	switch (rtsx_get_stat(chip)) {
	case RTSX_STAT_RUN:
#if !defined(LED_AUTO_BLINK) && defined(REGULAR_BLINK)
		rtsx_blink_led(chip);
#endif 
		do_remaining_work(chip);
		break;

	case RTSX_STAT_IDLE:
		if (chip->sd_io && !chip->sd_int) {
			try_to_switch_sdio_ctrl(chip);
		}

		if (chip->handshake_en)
			(void)rtsx_configure_host_aspm(chip);
		rtsx_enable_aspm(chip);	
		break;

	default:
		break;
	}

	
#ifdef SUPPORT_OCP
	if (chip->ocp_stat & (SD_OC_NOW | SD_OC_EVER)) {
		RTSX_DEBUGP(("Over current, OCPSTAT is 0x%x\n", chip->ocp_stat));
		if (chip->card_exist & SD_CARD) {
			rtsx_write_register(chip, CARD_OE, SD_OUTPUT_EN, 0);
			chip->card_fail |= SD_CARD;
		} else if (chip->card_exist & MS_CARD) {
			rtsx_write_register(chip, CARD_OE, MS_OUTPUT_EN, 0);
			chip->card_fail |= MS_CARD;
		} else if (chip->card_exist & XD_CARD) {
			rtsx_write_register(chip, CARD_OE, XD_OUTPUT_EN, 0);
			chip->card_fail |= XD_CARD;
		}
		card_power_off(chip, SD_CARD);
	}
#endif

Delink_Stage:
	if (chip->auto_delink_en && chip->auto_delink_allowed && 
			!chip->card_ready && !chip->card_ejected && !chip->sd_io) {
		int delink_stage1_cnt = chip->delink_stage1_step;
		int delink_stage2_cnt = delink_stage1_cnt + chip->delink_stage2_step;
		int delink_stage3_cnt = delink_stage2_cnt + chip->delink_stage3_step;
		
		if (chip->auto_delink_cnt <= delink_stage3_cnt) {
			if (chip->auto_delink_cnt == delink_stage1_cnt) {
				rtsx_set_stat(chip, RTSX_STAT_DELINK);
				
				if (chip->card_exist) {
					RTSX_DEBUGP(("False card inserted, do force delink\n"));

					rtsx_write_register(chip, CHANGE_LINK_STATE, 0x0A, 0x0A);

					chip->auto_delink_cnt = delink_stage3_cnt + 1;
				} else {
					RTSX_DEBUGP(("No card inserted, do delink\n"));
                
#ifdef HW_INT_WRITE_CLR
					rtsx_writel(chip, RTSX_BIPR, 0xFFFFFFFF);
					RTSX_DEBUGP(("RTSX_BIPR: 0x%x\n", rtsx_readl(chip, RTSX_BIPR)));
#endif
					rtsx_write_register(chip, CHANGE_LINK_STATE, 0x02, 0x02);
				}
			}

			if (chip->auto_delink_cnt == delink_stage2_cnt) {
				RTSX_DEBUGP(("Try to do force delink\n"));
				rtsx_write_register(chip, CHANGE_LINK_STATE, 0x0A, 0x0A);
			}
			
			if (chip->auto_delink_cnt == delink_stage3_cnt) {
				RTSX_DEBUGP(("Notify user space to refresh driver\n"));
			}
			
			chip->auto_delink_cnt ++;
		}
	} else {
		chip->auto_delink_cnt = 0;
	}
}

void rtsx_undo_delink(struct rtsx_chip *chip)
{
	chip->auto_delink_allowed = 0;
	rtsx_write_register(chip, CHANGE_LINK_STATE, 0x0A, 0x00);
}

/**
 * rtsx_stop_cmd - stop command transfer and DMA transfer
 * @chip: Realtek's card reader chip
 * @card: flash card type
 *
 * Stop command transfer and DMA transfer.
 * This function is called in error handler. 
 */
void rtsx_stop_cmd(struct rtsx_chip *chip, int card)
{
#if DBG
	int i;

	for (i = 0; i <= 8; i++) {
		int addr = RTSX_HCBAR + i * 4;
		u32 reg = rtsx_readl(chip, addr);
		RTSX_DEBUGP(("BAR (0x%02x): 0x%08x\n", addr, reg));
	}
#endif
	rtsx_writel(chip, RTSX_HCBCTLR, STOP_CMD);
	rtsx_writel(chip, RTSX_HDBCTLR, STOP_DMA);

#if DBG
	for (i = 0; i < 16; i++) {
		u16 addr = 0xFE20 + (u16)i;
		u8 val;
		rtsx_read_register(chip, addr, &val);
		RTSX_DEBUGP(("0x%04X: 0x%02x\n", addr, val));
	}
#endif

	rtsx_write_register(chip, DMACTL, 0x80, 0x80);
	rtsx_write_register(chip, RBCTL, 0x80, 0x80);
}

#define MAX_RW_REG_CNT		1024

int rtsx_write_register(struct rtsx_chip *chip, u16 addr, u8 mask, u8 data)
{
	int i;
	u32 val = 3 << 30;

	val |= (u32)(addr & 0x3FFF) << 16;
	val |= (u32)mask << 8;
	val |= (u32)data;

	rtsx_writel(chip, RTSX_HAIMR, val);

	for (i = 0; i < MAX_RW_REG_CNT; i++) {
		val = rtsx_readl(chip, RTSX_HAIMR);
		if ((val & (1 << 31)) == 0) {
			if (data != (u8)val) {
				TRACE_RET(chip, STATUS_FAIL);
			}
			return STATUS_SUCCESS;
		}
	}

	TRACE_RET(chip, STATUS_TIMEDOUT);
}

int rtsx_read_register(struct rtsx_chip *chip, u16 addr, u8 *data)
{
	u32 val = 2 << 30;
	int i;

	if (data) {
		*data = 0;
	}

	val |= (u32)(addr & 0x3FFF) << 16;

	rtsx_writel(chip, RTSX_HAIMR, val);

	for (i = 0; i < MAX_RW_REG_CNT; i++) {
		val = rtsx_readl(chip, RTSX_HAIMR);
		if ((val & (1 << 31)) == 0) {
			break;
		}
	}

	if (i >= MAX_RW_REG_CNT) {
		TRACE_RET(chip, STATUS_TIMEDOUT);
	}

	if (data) {
		*data = (u8)(val & 0xFF);
	}

	return STATUS_SUCCESS;
}

int rtsx_write_cfg_dw(struct rtsx_chip *chip, u8 func_no, u16 addr, u32 mask, u32 val)
{
	u8 mode = 0, tmp;
	int i;
	

	for (i = 0; i < 4; i++) {
		if (mask & 0xFF) {
			RTSX_WRITE_REG(chip, CFGDATA0 + i, 0xFF, (u8)(val & mask & 0xFF));
			mode |= (1 << i);
		}
		mask >>= 8;
		val >>= 8;
	}

	if (mode) {
		RTSX_WRITE_REG(chip, CFGADDR0, 0xFF, (u8)addr);
		RTSX_WRITE_REG(chip, CFGADDR1, 0xFF, (u8)(addr >> 8));

		RTSX_WRITE_REG(chip, CFGRWCTL, 0xFF, 0x80 | mode | ((func_no & 0x03) << 4));

		for (i = 0; i < MAX_RW_REG_CNT; i++) {
			RTSX_READ_REG(chip, CFGRWCTL, &tmp);
			if ((tmp & 0x80) == 0) {
				break;
			}
		}
	}
	
	return STATUS_SUCCESS;
}

int rtsx_read_cfg_dw(struct rtsx_chip *chip, u8 func_no, u16 addr, u32 *val)
{
	int i;
	u8 tmp;
	u32 data = 0;
	

	RTSX_WRITE_REG(chip, CFGADDR0, 0xFF, (u8)addr);
	RTSX_WRITE_REG(chip, CFGADDR1, 0xFF, (u8)(addr >> 8));
	RTSX_WRITE_REG(chip, CFGRWCTL, 0xFF, 0x80 | ((func_no & 0x03) << 4));

	for (i = 0; i < MAX_RW_REG_CNT; i++) {
		RTSX_READ_REG(chip, CFGRWCTL, &tmp);
		if ((tmp & 0x80) == 0) {
			break;
		}
	}

	for (i = 0; i < 4; i++) {
		RTSX_READ_REG(chip, CFGDATA0 + i, &tmp);
		data |= (u32)tmp << (i * 8);
	}

	if (val) {
		*val = data;
	}
	
	return STATUS_SUCCESS;
}

int rtsx_write_cfg_seq(struct rtsx_chip *chip, u8 func, u16 addr, u8 *buf, int len)
{
	u32 *data, *mask;
	u16 offset = addr % 4;
	u16 aligned_addr = addr - offset;
	int dw_len, i, j;
	int retval;
	
	RTSX_DEBUGP(("%s\n", __FUNCTION__));
	
	if (!buf) {
		TRACE_RET(chip, STATUS_NOMEM);
	}
	
	if ((len + offset) % 4) {
		dw_len = (len + offset) / 4 + 1;
	} else {
		dw_len = (len + offset) / 4;
	}
	RTSX_DEBUGP(("dw_len = %d\n", dw_len));
	
	data = (u32 *)vmalloc(dw_len * 4);
	if (!data) {
		TRACE_RET(chip, STATUS_NOMEM);
	}
	memset(data, 0, dw_len * 4);
	
	mask = (u32 *)vmalloc(dw_len * 4);
	if (!mask) {
		vfree(data);
		TRACE_RET(chip, STATUS_NOMEM);
	}
	memset(mask, 0, dw_len * 4);
	
	j = 0;
	for (i = 0; i < len; i++) {
		mask[j] |= 0xFF << (offset * 8);
		data[j] |= buf[i] << (offset * 8);
		if (++offset == 4) {
			j++;
			offset = 0;
		}
	}
	
	RTSX_DUMP(mask, dw_len * 4);
	RTSX_DUMP(data, dw_len * 4);
	
	for (i = 0; i < dw_len; i++) {
		retval = rtsx_write_cfg_dw(chip, func, aligned_addr + i * 4, mask[i], data[i]);
		if (retval != STATUS_SUCCESS) {
			vfree(data);
			vfree(mask);
			TRACE_RET(chip, STATUS_FAIL);
		}
	}
	
	vfree(data);
	vfree(mask);
	
	return STATUS_SUCCESS;
}

int rtsx_read_cfg_seq(struct rtsx_chip *chip, u8 func, u16 addr, u8 *buf, int len)
{
	u32 *data;
	u16 offset = addr % 4;
	u16 aligned_addr = addr - offset;
	int dw_len, i, j;
	int retval;
	
	RTSX_DEBUGP(("%s\n", __FUNCTION__));
	
	if ((len + offset) % 4) {
		dw_len = (len + offset) / 4 + 1;
	} else {
		dw_len = (len + offset) / 4;
	}
	RTSX_DEBUGP(("dw_len = %d\n", dw_len));
	
	data = (u32 *)vmalloc(dw_len * 4);
	if (!data) {
		TRACE_RET(chip, STATUS_NOMEM);
	}
	
	for (i = 0; i < dw_len; i++) {
		retval = rtsx_read_cfg_dw(chip, func, aligned_addr + i * 4, data + i);
		if (retval != STATUS_SUCCESS) {
			vfree(data);
			TRACE_RET(chip, STATUS_FAIL);
		}
	}
	
	if (buf) {
		j = 0;
		
		for (i = 0; i < len; i++) {
			buf[i] = (u8)(data[j] >> (offset * 8));
			if (++offset == 4) {
				j++;
				offset = 0;
			}
		}
	}
	
	vfree(data);
	
	return STATUS_SUCCESS;
}

int rtsx_check_link_ready(struct rtsx_chip *chip)
{
	u8 val;

	RTSX_READ_REG(chip, IRQSTAT0, &val);
	
	RTSX_DEBUGP(("IRQSTAT0: 0x%x\n", val));
	if (val & LINK_RDY_INT) {
		RTSX_DEBUGP(("Delinked!\n"));

		rtsx_write_register(chip, IRQSTAT0, LINK_RDY_INT, LINK_RDY_INT);

		return STATUS_FAIL;
	}

	return STATUS_SUCCESS;
}


int rtsx_pre_handle_interrupt(struct rtsx_chip *chip)
{
	u32 status, int_enable;
	int exit_ss = 0;
#ifdef SUPPORT_OCP
	u32 ocp_int = 0;

	ocp_int = SD_OC_INT;
#endif
	
	int_enable = rtsx_readl(chip, RTSX_BIER);
	
	chip->int_reg = rtsx_readl(chip, RTSX_BIPR);

#ifdef HW_INT_WRITE_CLR
	rtsx_writel(chip, RTSX_BIPR, chip->int_reg);
#endif
	
	if (((chip->int_reg & int_enable) == 0) || (chip->int_reg == 0xFFFFFFFF)) {
		return STATUS_FAIL;
	}
	
	status = chip->int_reg &= (int_enable | 0x7FFFFF);
	
	if (status & CARD_INT) {
		chip->auto_delink_cnt = 0;

		if (status & SD_INT) {
			if (status & SD_EXIST) {
				RTSX_MSG_IN_INT(("Insert SD\n"));
				set_bit(SD_NR, &(chip->need_reset));
			} else {
				RTSX_MSG_IN_INT(("Remove SD\n"));
				set_bit(SD_NR, &(chip->need_release));
				clear_bit(SD_NR, &(chip->need_reset));
				chip->card_reset_cnt = 0;
				chip->cd_show_cnt = 0;
			}
		} else {
			if (exit_ss && (status & SD_EXIST)) {
				set_bit(SD_NR, &(chip->need_reinit));
			}
		}
		if (status & XD_INT) {
			if (status & XD_EXIST) {
				RTSX_MSG_IN_INT(("Insert xD\n"));
				set_bit(XD_NR, &(chip->need_reset));
			} else {
				RTSX_MSG_IN_INT(("Remove xD\n"));
				set_bit(XD_NR, &(chip->need_release));
				clear_bit(XD_NR, &(chip->need_reset));
				chip->card_reset_cnt = 0;
				chip->cd_show_cnt = 0;
			}
		} else {
			if (exit_ss && (status & XD_EXIST)) {
				set_bit(XD_NR, &(chip->need_reinit));
			}
		}
		if (status & MS_INT) {
			if (status & MS_EXIST) {
				RTSX_MSG_IN_INT(("Insert MS\n"));
				set_bit(MS_NR, &(chip->need_reset));
			} else {
				RTSX_MSG_IN_INT(("Remove MS\n"));
				set_bit(MS_NR, &(chip->need_release));
				clear_bit(MS_NR, &(chip->need_reset));
				chip->card_reset_cnt = 0;
				chip->cd_show_cnt = 0;
			}
		} else {
			if (exit_ss && (status & MS_EXIST)) {
				set_bit(MS_NR, &(chip->need_reinit));
			}
		}
	}
	
#ifdef SUPPORT_OCP
	chip->ocp_int = ocp_int & status;
#endif

	if (chip->sd_io) {
		if (chip->int_reg & DATA_DONE_INT) {
			chip->int_reg &= ~(u32)DATA_DONE_INT;
		}
	}
	
	return STATUS_SUCCESS;	
}

void rtsx_do_before_power_down(struct rtsx_chip *chip, int pm_stat, int driver_unload)
{
	int retval;
	
	RTSX_DEBUGP(("rtsx_do_before_power_down, pm_stat = %d, driver_unload = %d\n",
				pm_stat, driver_unload));
	
	rtsx_set_stat(chip, RTSX_STAT_SUSPEND);
	
	retval = rtsx_force_power_on(chip, SSC_PDCTL);
	if (retval != STATUS_SUCCESS)
		return;

	if (chip->handshake_en) {	
		if (driver_unload)
			(void)rtsx_restore_host_aspm_halt(chip);
		else
			(void)rtsx_restore_host_aspm_sleep(chip);
	}

#ifdef HW_AUTO_SWITCH_SD_BUS
	if (chip->sd_io) {
		chip->sdio_in_charge = 1;
		rtsx_write_register(chip, TLPTISTAT, 0x10, 0x10);
		rtsx_write_register(chip, SDIO_CFG, SDIO_BUS_AUTO_SWITCH, SDIO_BUS_AUTO_SWITCH);
	}
#endif

	rtsx_write_register(chip, PETXCFG, 0x08, 0x08);
	
	if (pm_stat == PM_S1) {
		RTSX_DEBUGP(("Host enter S1\n"));
		rtsx_write_register(chip, HOST_SLEEP_STATE, 0x03, HOST_ENTER_S1);
	} else if (pm_stat == PM_S3) {
		RTSX_DEBUGP(("Host enter S3\n"));
		rtsx_write_register(chip, HOST_SLEEP_STATE, 0x03, HOST_ENTER_S3);
	}
	
	if (chip->do_delink_before_power_down && chip->auto_delink_en) {
		rtsx_write_register(chip, CHANGE_LINK_STATE, 0x02, 2);
	}
	
	rtsx_release_cards(chip);
	rtsx_disable_bus_int(chip);
	turn_off_led(chip, LED_GPIO);
	rtsx_force_power_down(chip, SSC_PDCTL | OC_PDCTL);
	rtsx_write_register(chip, FUNC1_FORCE_CTL, CR_SOFT_PFM_EN, CR_SOFT_PFM_EN);
	
	chip->cur_clk = 0;
	chip->cur_card = 0;
	
	chip->card_exist = 0;
}

void rtsx_enable_aspm(struct rtsx_chip *chip)
{
	if (chip->aspm_l0s_l1_en && chip->dynamic_aspm) {
		if (!chip->aspm_enabled) {
			RTSX_DEBUGP(("Try to enable ASPM\n"));
			chip->aspm_enabled = 1;
			
			rtsx_write_config_byte(chip, LCTLR, chip->aspm_l0s_l1_en);
			
			if (chip->sdio_func_exist) {
				u16 val = chip->aspm_l0s_l1_en | 0x0100;
				rtsx_write_cfg_dw(chip, 1, 0xC0, 0xFFFF, val);
			}
		}

		if (chip->config_host_aspm) {
			if (!chip->host_aspm_enabled) {
				RTSX_DEBUGP(("Try to set Host ASPM to %d\n",
							chip->host_aspm_val));
				chip->host_aspm_enabled = 1;
				rtsx_set_host_aspm(chip, chip->host_aspm_val);
			}
		}
	}

	return;
}

void rtsx_disable_aspm(struct rtsx_chip *chip)
{
	if (chip->aspm_l0s_l1_en && chip->dynamic_aspm) {
		if (chip->config_host_aspm) {
			if (chip->host_aspm_enabled) {
				RTSX_DEBUGP(("Try to disable Host ASPM\n"));
				chip->host_aspm_enabled = 0;
				rtsx_disable_host_aspm(chip);
			}
		}
			
		if (chip->aspm_enabled) {
			RTSX_DEBUGP(("Try to disable ASPM\n"));
			chip->aspm_enabled = 0;
			
			rtsx_write_config_byte(chip, LCTLR, 0x00);
			wait_timeout(1);
		}
	}

	return;	
}

int rtsx_read_ppbuf(struct rtsx_chip *chip, u8 *buf, int buf_len)
{
	int retval;
	int i, j;
	u16 reg_addr;
	u8 *ptr;
	
	if (!buf) {
		TRACE_RET(chip, STATUS_ERROR);
	}
	
	ptr = buf;
	reg_addr = PPBUF_BASE2;
	for (i = 0; i < buf_len/256; i++) {
		rtsx_init_cmd(chip);
		
		for (j = 0; j < 256; j++) {
			rtsx_add_cmd(chip, READ_REG_CMD, reg_addr++, 0, 0);
		}
		
		retval = rtsx_send_cmd(chip, 0, 250);
		if (retval < 0) {
			TRACE_RET(chip, STATUS_FAIL);
		}
		
		memcpy(ptr, rtsx_get_cmd_data(chip), 256);
		ptr += 256;
	}
	
	if (buf_len%256) {
		rtsx_init_cmd(chip);
		
		for (j = 0; j < buf_len%256; j++) {
			rtsx_add_cmd(chip, READ_REG_CMD, reg_addr++, 0, 0);
		}
		
		retval = rtsx_send_cmd(chip, 0, 250);
		if (retval < 0) {
			TRACE_RET(chip, STATUS_FAIL);
		}
	}
	
	memcpy(ptr, rtsx_get_cmd_data(chip), buf_len%256);
	
	return STATUS_SUCCESS;
}

int rtsx_write_ppbuf(struct rtsx_chip *chip, u8 *buf, int buf_len)
{
	int retval;
	int i, j;
	u16 reg_addr;
	u8 *ptr;
	
	if (!buf) {
		TRACE_RET(chip, STATUS_ERROR);
	}
	
	ptr = buf;
	reg_addr = PPBUF_BASE2;
	for (i = 0; i < buf_len/256; i++) {
		rtsx_init_cmd(chip);
		
		for (j = 0; j < 256; j++) {
			rtsx_add_cmd(chip, WRITE_REG_CMD, reg_addr++, 0xFF, *ptr);
			ptr++;
		}
		
		retval = rtsx_send_cmd(chip, 0, 250);
		if (retval < 0) {
			TRACE_RET(chip, STATUS_FAIL);
		}
	}
	
	if (buf_len%256) {
		rtsx_init_cmd(chip);
		
		for (j = 0; j < buf_len%256; j++) {
			rtsx_add_cmd(chip, WRITE_REG_CMD, reg_addr++, 0xFF, *ptr);
			ptr++;
		}
		
		retval = rtsx_send_cmd(chip, 0, 250);
		if (retval < 0) {
			TRACE_RET(chip, STATUS_FAIL);
		}
	}
	
	return STATUS_SUCCESS;
}

int rtsx_check_chip_exist(struct rtsx_chip *chip)
{
	if (rtsx_readl(chip, 0) == 0xFFFFFFFF) {
		TRACE_RET(chip, STATUS_FAIL);
	}
	
	return STATUS_SUCCESS;
}

int rtsx_force_power_on(struct rtsx_chip *chip, u8 ctl)
{
	u8 mask = 0;
	
	if (ctl & SSC_PDCTL) {
		mask |= SSC_POWER_DOWN;
		RTSX_WRITE_REG(chip, LDO_SSC_CTL, LDO_EN, LDO_EN);
	}
	
#ifdef SUPPORT_OCP
	if (ctl & OC_PDCTL) {
		mask |= OC_POWER_DOWN;
		RTSX_WRITE_REG(chip, OCPCTL, SD_DETECT_EN | SD_OCP_INT_EN,
			       SD_DETECT_EN | SD_OCP_INT_EN);
	}
#endif
	
	if (mask) {
		RTSX_WRITE_REG(chip, FPDCTL, mask, 0);
	}
	
	return STATUS_SUCCESS;
}

int rtsx_force_power_down(struct rtsx_chip *chip, u8 ctl)
{
	u8 mask = 0, val = 0;
	
	if (ctl & SSC_PDCTL) {
		mask |= SSC_POWER_DOWN;
		RTSX_WRITE_REG(chip, LDO_SSC_CTL, LDO_EN, 0);
	}
	
#ifdef SUPPORT_OCP
	if (ctl & OC_PDCTL) {
		mask |= OC_POWER_DOWN;
		RTSX_WRITE_REG(chip, OCPCTL, SD_DETECT_EN | SD_OCP_INT_EN, 0);
	}
#endif
	
	if (mask) {
		val = mask;
		RTSX_WRITE_REG(chip, FPDCTL, mask, val);
	}
	
	return STATUS_SUCCESS;
}

int rtsx_ldo_pwr_sel(struct rtsx_chip *chip, u8 ldo_src)
{
	u8 val, mask;
	
	if (CHECK_PID(chip, RTL8411) || CHECK_PID(chip, RTL8411B)) {
		mask = LDO_PWR_SEL_8411;
		if (ldo_src == LDO_FROM_CARD) {
			val = LDO_FROM_CARD_8411;
		} else if (ldo_src == LDO_FROM_DV33) {
			val = LDO_FROM_DV33_8411;
		} else {
			TRACE_RET(chip, STATUS_FAIL);
		}
	} else {
		mask = LDO_PWR_SEL_8402;
		if (ldo_src == LDO_FROM_CARD) {
			val = LDO_FROM_CARD_8402;
		} else if (ldo_src == LDO_FROM_DV33) {
			val = LDO_FROM_DV33_8402;
		} else if (ldo_src == LDO_FROM_NONE) {
			val = LDO_FROM_NONE_8402;
		} else {
			TRACE_RET(chip, STATUS_FAIL);
		}
	}
	
	RTSX_WRITE_REG(chip, LDO_CTL, mask, val);
	
	return STATUS_SUCCESS;
}

static int rtsx_set_handshake_bit(struct rtsx_chip *chip, int index, u8 val)
{
	int retval;
	u32 lval;

	retval = rtsx_read_cfg_dw(chip, 0, 0x744, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	lval &= ~((u32)1 << index);
	lval |= ((u32)(val & 0x01) << index);

	retval = rtsx_write_cfg_dw(chip, 0, 0x744, 0xFFFFFFFF, lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	return STATUS_SUCCESS;
}

static int rtsx_chk_handshake_bit(struct rtsx_chip *chip, int index)
{
	int retval;
	u32 lval;

	retval = rtsx_read_cfg_dw(chip, 0, 0x744, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, -1);

	if (lval & ((u32)1 << index))
		return 1;

	return 0;
}

int rtsx_get_root_aspm_val(struct rtsx_chip *chip, u8 *val)
{
	int retval;
	u32 lval;

	retval = rtsx_read_cfg_dw(chip, 0, 0x744, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	if (val)
		*val = (u8)(lval & 0x03);

	return STATUS_SUCCESS;
}

int rtsx_set_root_aspm_val(struct rtsx_chip *chip, u8 val)
{
	int retval;
	u32 lval;

	retval = rtsx_read_cfg_dw(chip, 0, 0x744, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	lval &= 0xFFFFFFFC;
	lval |= (val & 0x03);

	retval = rtsx_write_cfg_dw(chip, 0, 0x744, 0xFF, lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	return STATUS_SUCCESS;
}

int rtsx_chk_root_aspm_flag(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 2);
}

int rtsx_set_root_aspm_flag(struct rtsx_chip *chip, u8 val)
{
	return rtsx_set_handshake_bit(chip, 2, val);
}

int rtsx_get_aspm_val(struct rtsx_chip *chip, u8 *val)
{
	int retval;
	u32 lval;

	retval = rtsx_read_cfg_dw(chip, 0, 0x744, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	if (val)
		*val = (u8)((lval >> 4) & 0x03);

	return STATUS_SUCCESS;
}

int rtsx_set_aspm_val(struct rtsx_chip *chip, u8 val)
{
	int retval;
	u32 lval;

	retval = rtsx_read_cfg_dw(chip, 0, 0x744, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	lval &= 0xFFFFFFCF;
	lval |= ((val & 0x03) << 4);

	retval = rtsx_write_cfg_dw(chip, 0, 0x744, 0xFF, lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	return STATUS_SUCCESS;
}

int rtsx_chk_aspm_mod_flag(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 6);
}

int rtsx_set_aspm_mod_flag(struct rtsx_chip *chip, u8 val)
{
	return rtsx_set_handshake_bit(chip, 6, val);
}

int rtsx_chk_nic_attach(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 8);
}

int rtsx_set_cr_attach(struct rtsx_chip *chip, u8 val)
{
	return rtsx_set_handshake_bit(chip, 9, val);
}

int rtsx_chk_perf_cr_first(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 20);
}

int rtsx_chk_perf_nic_first(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 21);
}

int rtsx_get_perf_prior(struct rtsx_chip *chip, u8 *val)
{
	int retval;
	u32 lval;

	retval = rtsx_read_cfg_dw(chip, 0, 0x744, &lval);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	if (val)
		*val = (u8)((lval >> 20) & 0x03);

	return STATUS_SUCCESS;
}

int rtsx_set_perf_adjust_nic(struct rtsx_chip *chip, u8 val)
{
	return rtsx_set_handshake_bit(chip, 17, val);
}

int rtsx_chk_perf_adjust_cr(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 16);
}

int rtsx_chk_perf_prior_bios(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 23);
}

static int rtsx_chk_lock_flag0(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 14);
}


static int rtsx_chk_lock_turn(struct rtsx_chip *chip)
{
	return rtsx_chk_handshake_bit(chip, 12);
}


static int rtsx_set_lock_flag1(struct rtsx_chip *chip, u8 val)
{
	return rtsx_set_handshake_bit(chip, 13, val);
}

static int rtsx_set_lock_turn(struct rtsx_chip *chip, u8 val)
{
	return rtsx_set_handshake_bit(chip, 12, val);
}

int rtsx_cfg_try_lock(struct rtsx_chip *chip, int timeout)
{
	int retval;
	int turn, flag0;

	if (timeout == 0)
		timeout = 31415926;

	retval = rtsx_set_lock_flag1(chip, 1);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	do {
		flag0 = rtsx_chk_lock_flag0(chip);
		if (flag0 < 0)
			TRACE_RET(chip, STATUS_FAIL);
		if (flag0 == 0)
			break;

		turn = rtsx_chk_lock_turn(chip);
		if (turn == 0) {
			retval = rtsx_set_lock_flag1(chip, 0);
			if (retval != STATUS_SUCCESS)
				TRACE_RET(chip, STATUS_FAIL);

			while (timeout--) {
				turn = rtsx_chk_lock_turn(chip);
				if (turn < 0);
					TRACE_RET(chip, STATUS_FAIL);
				if (turn == 1)
					break;
				mdelay(1);
			}
			if (turn == 0)
				TRACE_RET(chip, STATUS_TIMEDOUT);

			retval = rtsx_set_lock_flag1(chip, 1);
			if (retval != STATUS_SUCCESS)
				TRACE_RET(chip, STATUS_FAIL);
		}
		
		mdelay(1);
	} while (timeout--);

	return STATUS_SUCCESS;
}

int rtsx_cfg_unlock(struct rtsx_chip *chip)
{
	int retval;

	retval = rtsx_set_lock_turn(chip, 0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	retval = rtsx_set_lock_flag1(chip, 0);
	if (retval != STATUS_SUCCESS)
		TRACE_RET(chip, STATUS_FAIL);

	return STATUS_SUCCESS;
}

