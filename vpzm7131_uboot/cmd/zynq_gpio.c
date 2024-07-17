
#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <command.h>
#include <dm.h>

int io_num = 0;

#define ZYNQ_GPIO_MAX_BANK   4

#define ZYNQ_GPIO_BANK0_NGPIO 32
#define ZYNQ_GPIO_BANK1_NGPIO 22
#define ZYNQ_GPIO_BANK2_NGPIO 32
#define ZYNQ_GPIO_BANK3_NGPIO 32

#define ZYNQ_GPIO_NR_GPIOS  (ZYNQ_GPIO_BANK0_NGPIO + \
							 ZYNQ_GPIO_BANK1_NGPIO + \
							 ZYNQ_GPIO_BANK2_NGPIO + \
							 ZYNQ_GPIO_BANK3_NGPIO)

struct zynq_platform_data
{
	const char *label;
	u16 ngpio;
	u32 max_bank;
	u32 bank_min[ZYNQ_GPIO_MAX_BANK];
	u32 bank_max[ZYNQ_GPIO_MAX_BANK];
};

#define ZYNQ_GPIO_BANK0_PIN_MIN(str) 0
#define ZYNQ_GPIO_BANK0_PIN_MAX(str) (ZYNQ_GPIO_BANK0_PIN_MIN(str) + ZYNQ##str##_GPIO_BANK0_NGPIO - 1)
#define ZYNQ_GPIO_BANK1_PIN_MIN(str) (ZYNQ_GPIO_BANK0_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK1_PIN_MAX(str) (ZYNQ_GPIO_BANK1_PIN_MIN(str) + ZYNQ##str##_GPIO_BANK1_NGPIO - 1)
#define ZYNQ_GPIO_BANK2_PIN_MIN(str) (ZYNQ_GPIO_BANK1_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK2_PIN_MAX(str) (ZYNQ_GPIO_BANK2_PIN_MIN(str) + ZYNQ##str##_GPIO_BANK2_NGPIO - 1)
#define ZYNQ_GPIO_BANK3_PIN_MIN(str) (ZYNQ_GPIO_BANK2_PIN_MAX(str) + 1)
#define ZYNQ_GPIO_BANK3_PIN_MAX(str) (ZYNQ_GPIO_BANK3_PIN_MIN(str) + ZYNQ##str##_GPIO_BANK3_NGPIO - 1)

static const struct zynq_platform_data zynq_gpio_def = {
	.label = "zynq_gpio",
	.ngpio = ZYNQ_GPIO_NR_GPIOS,
	.max_bank = ZYNQ_GPIO_MAX_BANK,
	.bank_min[0] = ZYNQ_GPIO_BANK0_PIN_MIN(),
	.bank_max[0] = ZYNQ_GPIO_BANK0_PIN_MAX(),
	.bank_min[1] = ZYNQ_GPIO_BANK1_PIN_MIN(),
	.bank_max[1] = ZYNQ_GPIO_BANK1_PIN_MAX(),
	.bank_min[2] = ZYNQ_GPIO_BANK2_PIN_MIN(),
	.bank_max[2] = ZYNQ_GPIO_BANK2_PIN_MAX(),
	.bank_min[3] = ZYNQ_GPIO_BANK3_PIN_MIN(),
	.bank_max[3] = ZYNQ_GPIO_BANK3_PIN_MAX(),
};

/**
 *  * @brief copy from zynq_gpio.c
 *   *
 *    */

#define ZYNQ_GPIO_DATA_LSW_OFFSET(BANK) (0x000 + (8 * BANK))
/* MSW Mask & Data -WO */
#define ZYNQ_GPIO_DATA_MSW_OFFSET(BANK) (0x004 + (8 * BANK))
/* Data Register-RW */
#define ZYNQ_GPIO_DATA_RO_OFFSET(BANK) (0x060 + (4 * BANK))
/* Direction mode reg-RW */
#define ZYNQ_GPIO_DIRM_OFFSET(BANK) (0x204 + (0x40 * BANK))
/* Output enable reg-RW */
#define ZYNQ_GPIO_OUTEN_OFFSET(BANK) (0x208 + (0x40 * BANK))
/* Interrupt mask reg-RO */
#define ZYNQ_GPIO_INTMASK_OFFSET(BANK) (0x20C + (0x40 * BANK))
/* Interrupt enable reg-WO */
#define ZYNQ_GPIO_INTEN_OFFSET(BANK) (0x210 + (0x40 * BANK))
/* Interrupt disable reg-WO */
#define ZYNQ_GPIO_INTDIS_OFFSET(BANK) (0x214 + (0x40 * BANK))
/* Interrupt status reg-RO */
#define ZYNQ_GPIO_INTSTS_OFFSET(BANK) (0x218 + (0x40 * BANK))
/* Interrupt type reg-RW */
#define ZYNQ_GPIO_INTTYPE_OFFSET(BANK) (0x21C + (0x40 * BANK))
/* Interrupt polarity reg-RW */
#define ZYNQ_GPIO_INTPOL_OFFSET(BANK) (0x220 + (0x40 * BANK))
/* Interrupt on any, reg-RW */
#define ZYNQ_GPIO_INTANY_OFFSET(BANK) (0x224 + (0x40 * BANK))
/* Mid pin number of a bank */
#define ZYNQ_GPIO_MID_PIN_NUM 16
/* GPIO upper 16 bit mask */
#define ZYNQ_GPIO_UPPER_MASK 0xFFFF0000
/**
 *  * @brief copy and modify from zynq_gpio.c
 *   *
 *    * @param pin_num
 *     * @param bank_num
 *      * @param bank_pin_num
 *       */
void zynq_gpio_get_bank_pin(unsigned int pin_num, unsigned int *bank_num, unsigned int *bank_pin_num)
{
	u32 bank;

	for (bank = 0; bank < zynq_gpio_def.max_bank; bank++)
	{
		if (pin_num >= zynq_gpio_def.bank_min[bank] &&
				pin_num <= zynq_gpio_def.bank_max[bank])
		{
			*bank_num = bank;
			*bank_pin_num = pin_num - zynq_gpio_def.bank_min[bank];
			return;
		}
	}

	if (bank >= zynq_gpio_def.max_bank)
	{
		printf("Invalid bank and pin num\n");
		*bank_num = 0;
		*bank_pin_num = 0;
	}
}

#define ZYNQ_GPIO_BASEADDR 0xE000A000

int zynq_gpio_set_value(unsigned pin, int value)
{
	unsigned int reg_offset, bank_num, bank_pin_num;

//	printf("zynq_gpio_set_value %d pin = %d, value = %d\n", io_num, pin, value);
	zynq_gpio_get_bank_pin(pin, &bank_num, &bank_pin_num);

	if (bank_pin_num >= ZYNQ_GPIO_MID_PIN_NUM)
	{
		/* only 16 data bits in bit maskable reg */
		bank_pin_num -= ZYNQ_GPIO_MID_PIN_NUM;
		reg_offset = ZYNQ_GPIO_DATA_MSW_OFFSET(bank_num);
	}
	else
	{
		reg_offset = ZYNQ_GPIO_DATA_LSW_OFFSET(bank_num);
	}

	/*
	 *      * get the 32 bit value to be written to the mask/data register where
	 *           * the upper 16 bits is the mask and lower 16 bits is the data
	 *                */
	value = !!value;
	value = ~(1 << (bank_pin_num + ZYNQ_GPIO_MID_PIN_NUM)) &
		((value << bank_pin_num) | ZYNQ_GPIO_UPPER_MASK);

	writel(value, ZYNQ_GPIO_BASEADDR + reg_offset);

	return 0;
}

extern int zynq_gpio_direction_output(unsigned gpio, int value)
{
	u32 reg;
	unsigned int bank_num, bank_pin_num;

	zynq_gpio_get_bank_pin(gpio, &bank_num, &bank_pin_num);

	/* set the GPIO pin as output */
	reg = readl(ZYNQ_GPIO_BASEADDR + ZYNQ_GPIO_DIRM_OFFSET(bank_num));
	reg |= BIT(bank_pin_num);
	writel(reg, ZYNQ_GPIO_BASEADDR + ZYNQ_GPIO_DIRM_OFFSET(bank_num));

	/* configure the output enable reg for the pin */
	reg = readl(ZYNQ_GPIO_BASEADDR + ZYNQ_GPIO_OUTEN_OFFSET(bank_num));
	reg |= BIT(bank_pin_num);
	writel(reg, ZYNQ_GPIO_BASEADDR + ZYNQ_GPIO_OUTEN_OFFSET(bank_num));

	/* set the state of the pin */
	zynq_gpio_set_value(gpio, value);
	return 0;
}

void zynq_gpio_toggle(unsigned pin)
{
	if( io_num % 2 == 0 )
	{
		zynq_gpio_direction_output(pin, 0);
	} else {
		zynq_gpio_direction_output(pin, 1);
	}
	io_num ++;
}

static int do_zynq_gpio(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if( argc == 3 )
	{
		int index = simple_strtoul(argv[1], NULL, 10);
		bool onoff = simple_strtoul(argv[2], NULL, 10);

		zynq_gpio_direction_output(index, onoff);
	} else {
		printf("parameter error!");
	}
}

U_BOOT_CMD(
		zynq_gpio, 3, 0,	do_zynq_gpio,
		"zynq_gpio", "zynq_gpio"
		);

