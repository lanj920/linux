// SPDX-License-Identifier: GPL-2.0+
/*
 * RealTek PHY drivers
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Copyright 2010-2011 Freescale Semiconductor, Inc.
 * author Andy Fleming
 *
 */

#include <config.h>
#include <common.h>
#include <phy.h>
#include <bitfield.h>

#define REG_PHY_SPEC_STATUS	0x11
#define REG_DEBUG_ADDR_OFFSET	0x1e
#define REG_DEBUG_DATA		0x1f
#define EXTREG_SLEEP_CONTROL	0x27

#define YTPHY_EXTREG_CHIP_CONFIG	0xa001
#define YTPHY_EXTREG_RGMII_CONFIG1	0xa003
#define YTPHY_PAD_DRIVES_STRENGTH_CFG	0xa010
#define YTPHY_DUPLEX		0x2000
#define YTPHY_DUPLEX_BIT	13
#define YTPHY_SPEED_MODE	0xc000
#define YTPHY_SPEED_MODE_BIT	14
#define YTPHY_RGMII_SW_DR_MASK	GENMASK(5, 4)
#define YTPHY_RGMII_RXC_DR_MASK	GENMASK(15, 13)

#define YT8521_EXT_CLK_GATE	0xc
#define YT8521_EN_SLEEP_SW_BIT	15

#define SPEED_UNKNOWN		-1

#define MOTORCOMM_PHY_ID_MASK           0x00000fff

static int ytphy_read_ext(struct phy_device *phydev, u32 regnum)
{
	int ret;

	ret = phy_write(phydev, MDIO_DEVAD_NONE, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	return phy_read(phydev, MDIO_DEVAD_NONE, REG_DEBUG_DATA);
}

static int ytphy_write_ext(struct phy_device *phydev, u32 regnum, u16 val)
{
	int ret;

	ret = phy_write(phydev, MDIO_DEVAD_NONE, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	return phy_write(phydev, MDIO_DEVAD_NONE, REG_DEBUG_DATA, val);
}

static int ytphy_parse_status(struct phy_device *phydev)
{
	int val;
	int speed, speed_mode, duplex;

	val = phy_read(phydev, MDIO_DEVAD_NONE, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	duplex = (val & YTPHY_DUPLEX) >> YTPHY_DUPLEX_BIT;
	speed_mode = (val & YTPHY_SPEED_MODE) >> YTPHY_SPEED_MODE_BIT;

	printf("speed: %d,duplex: %d\n", speed, duplex );

	switch (speed_mode) {
	case 2:
		speed = SPEED_1000;
		break;
	case 1:
		speed = SPEED_100;
		break;
	default:
		speed = SPEED_10;
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;
	
	return 0;
}

static int ytphy_startup(struct phy_device *phydev)
{
	int retval;

	retval = genphy_update_link(phydev);
	if (retval)
		return retval;
	ytphy_parse_status(phydev);

	return 0;
}

static void ytphy_of_config(struct phy_device *phydev)
{
	u32 val;
	u32 cfg;
	int i;


	val = ytphy_read_ext(phydev, YTPHY_EXTREG_CHIP_CONFIG);
    printf("read reg 0xa001 = 0x%x\n",val);
    val &= (~0x3f);      //mode_sel:UTP_TO_RGMII LDO 3.3V
	val |= (1 << 8);  //rxc_dly_en
    ytphy_write_ext(phydev, 0xa001, val);

	val = ytphy_read_ext(phydev, YTPHY_PAD_DRIVES_STRENGTH_CFG);
	val |= (0x30); //dr_rx_rgmii:strongest
	ytphy_write_ext(phydev, YTPHY_PAD_DRIVES_STRENGTH_CFG, val);

	val = ytphy_read_ext(phydev, YTPHY_EXTREG_RGMII_CONFIG1);
	printf("read reg 0xa003 = 0x%x\n",val);

	ytphy_write_ext(phydev, YTPHY_EXTREG_RGMII_CONFIG1, val);

	ytphy_write_ext(phydev, 0xa000, 0);
}

static int yt8521_config(struct phy_device *phydev)
{
	int ret, val;

	ret = 0;

	/*set delay config*/
	ytphy_of_config(phydev);

	/* disable auto sleep */
	val = ytphy_read_ext(phydev, EXTREG_SLEEP_CONTROL);
	if (val < 0)
		return val;

	val &= ~(1 << YT8521_EN_SLEEP_SW_BIT);
	ret = ytphy_write_ext(phydev, EXTREG_SLEEP_CONTROL, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8521_EXT_CLK_GATE);

	genphy_config_aneg(phydev);
	return 0;
}

static struct phy_driver YT8521_driver = {
	.name = "YuTai YT8521",
	.uid = 0x0000011a,
	.mask = 0x00000fff,
	.features = PHY_GBIT_FEATURES,
	.config = &yt8521_config,
	.startup = &ytphy_startup,
	.shutdown = &genphy_shutdown,
};

int phy_yutai_init(void)
{
	phy_register(&YT8521_driver);

	return 0;
}
