/*6
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>

#include <linux/input/cyttsp.h>
#include <linux/input/ft5x06.h>
#include <linux/input/kxtf9.h>
#include <linux/power/max17042.h>
#include <linux/power/max8903.h>

#include <mach/board-nooktablet.h>
#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <video/omapdss.h>
#include <linux/wl12xx.h>

#include "mux.h"
#include "hsmmc.h"
#include "control.h"
#include "common-board-devices.h"

extern unsigned int system_modelid;

//#define OMAP4_SFH7741_SENSOR_OUTPUT_GPIO	184
//#define OMAP4_SFH7741_ENABLE_GPIO		188

#define WILINK_UART_DEV_NAME "/dev/ttyO1"

#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34

#define CONFIG_SERIAL_OMAP_IDLE_TIMEOUT 5

#define GPIO_WIFI_PMENA			54
#define GPIO_WIFI_IRQ			53

#define CYTTSP_I2C_SLAVEADDRESS 	34
#define OMAP_CYTTSP_GPIO        	37 /*99*/
#define OMAP_CYTTSP_RESET_GPIO 		39 /*46*/

#define FT5x06_I2C_SLAVEADDRESS  	(0x70 >> 1)
#define OMAP_FT5x06_GPIO         	37 /*99*/
#define OMAP_FT5x06_RESET_GPIO   	39 /*46*/

#define TWL6030_RTC_GPIO 		6
#define BLUETOOTH_UART			UART2
#define CONSOLE_UART			UART1

static int max17042_gpio_for_irq = 0;
static int kxtf9_gpio_for_irq = 0;

#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
	printk("board-4430sdp.c: max17042_dev_init ...\n");

	if (gpio_request(max17042_gpio_for_irq, "max17042_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
		return;
	}

	printk("board-4430sdp.c: max17042_dev_init > Init max17042 irq pin %d !\n", max17042_gpio_for_irq);
	gpio_direction_input(max17042_gpio_for_irq);
	printk("max17042 GPIO pin read %d\n", gpio_get_value(max17042_gpio_for_irq));
}
#endif

static void kxtf9_dev_init(void)
{
	printk("board-4430sdp.c: kxtf9_dev_init ...\n");

	if (gpio_request(kxtf9_gpio_for_irq, "kxtf9_irq") < 0)
	{
		printk("Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-4430sdp.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n",
			kxtf9_gpio_for_irq);
	gpio_direction_input(kxtf9_gpio_for_irq);
}


struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXTF9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	/* Map the axes from the sensor to the device */
	/* SETTINGS FOR acclaim */
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.axis_map_x     = 1,
	.axis_map_y     = 0,
	.axis_map_z     = 2,
	.negate_x       = 1,
	.negate_y       = 0,
	.negate_z       = 0,
	.data_odr_init          = ODR12_5F,
	.ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init          = KXTF9_IEN,
	.tilt_timer_init        = 0x03,
	.engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init         = 0x16,
	.wuf_thresh_init        = 0x28,
	.tdt_timer_init         = 0x78,
	.tdt_h_thresh_init      = 0xFF,
	.tdt_l_thresh_init      = 0x14,
	.tdt_tap_timer_init     = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = 0,
};

int ft5x06_dev_init(int resource)
{
	if (resource){
		omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
		omap_mux_init_signal("gpmc_ad15.gpio_39", OMAP_PIN_OUTPUT );

		if (gpio_request(OMAP_FT5x06_RESET_GPIO, "ft5x06_reset") < 0){
			printk(KERN_ERR "can't get ft5x06 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_FT5x06_GPIO, "ft5x06_touch") < 0) {
			printk(KERN_ERR "can't get ft5x06 interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_FT5x06_GPIO);
	} else {
		gpio_free(OMAP_FT5x06_GPIO);
		gpio_free(OMAP_FT5x06_RESET_GPIO);
	}

	return 0;
}

static void ft5x06_platform_suspend(void)
{
	omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT );
}

static void ft5x06_platform_resume(void)
{
	omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
}

static struct ft5x06_platform_data ft5x06_platform_data = {
	.maxx = 600,
	.maxy = 1024,
	.flags = 0,
	.reset_gpio = OMAP_FT5x06_RESET_GPIO,
	.use_st = FT_USE_ST,
	.use_mt = FT_USE_MT,
	.use_trk_id = FT_USE_TRACKING_ID,
	.use_sleep = FT_USE_SLEEP,
	.use_gestures = 1,
	.platform_suspend = ft5x06_platform_suspend,
	.platform_resume = ft5x06_platform_resume,
};

int cyttsp_dev_init(int resource)
{
	if (resource) {
		omap_mux_init_signal("gpmc_ad13.gpio_37", OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
		omap_mux_init_signal("gpmc_ad15.gpio_39", OMAP_PIN_OUTPUT );


		if (gpio_request(OMAP_CYTTSP_RESET_GPIO, "tma340_reset") < 0) {
			printk(KERN_ERR "can't get tma340 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_CYTTSP_GPIO, "cyttsp_touch") < 0) {
			printk(KERN_ERR "can't get cyttsp interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_CYTTSP_GPIO);
		/* omap_set_gpio_debounce(OMAP_CYTTSP_GPIO, 0); */
	} else {
		printk ("\n%s: Free resources",__FUNCTION__);
		gpio_free(OMAP_CYTTSP_GPIO);
		gpio_free(OMAP_CYTTSP_RESET_GPIO);
	}
	return 0;
}

static struct cyttsp_platform_data cyttsp_platform_data = {
	.maxx = 480,
	.maxy = 800,
	.flags = 0,
	.gen = CY_GEN3,
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_SLEEP,
	.use_gestures = CY_USE_GESTURES,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 | CY_GEST_GRP3 | CY_GEST_GRP4 | CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state.
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state.
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
};

#ifdef CONFIG_CHARGER_MAX8903

static struct resource max8903_gpio_resources_evt1a[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.end	= MAX8903_GPIO_CHG_USUS_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.end	= MAX8903_GPIO_CHG_ILM_EVT1A,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct resource max8903_gpio_resources_evt1b[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_EVT1B,
		.end	= MAX8903_GPIO_CHG_USUS_EVT1B,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_EVT1B,
		.end	= MAX8903_GPIO_CHG_ILM_EVT1B,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct resource max8903_gpio_resources_dvt[] = {
	{	.name	= MAX8903_TOKEN_GPIO_CHG_EN,
		.start	= MAX8903_GPIO_CHG_EN,
		.end	= MAX8903_GPIO_CHG_EN,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_FLT,
		.start	= MAX8903_GPIO_CHG_FLT,
		.end	= MAX8903_GPIO_CHG_FLT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_IUSB,
		.start	= MAX8903_GPIO_CHG_IUSB,
		.end	= MAX8903_GPIO_CHG_IUSB,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_USUS,
		.start	= MAX8903_GPIO_CHG_USUS_DVT,
		.end	= MAX8903_GPIO_CHG_USUS_DVT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_ILM,
		.start	= MAX8903_GPIO_CHG_ILM_DVT,
		.end	= MAX8903_GPIO_CHG_ILM_DVT,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_UOK,
		.start	= MAX8903_UOK_GPIO_FOR_IRQ,
		.end	= MAX8903_UOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}, {
		.name	= MAX8903_TOKEN_GPIO_CHG_DOK,
		.start	= MAX8903_DOK_GPIO_FOR_IRQ,
		.end	= MAX8903_DOK_GPIO_FOR_IRQ,
		.flags	= IORESOURCE_IO,
	}
};

static struct platform_device max8903_charger_device = {
	.name           = "max8903_charger",
	.id             = -1,
};

static inline void acclaim_init_charger(void)
{
	const int board_type = acclaim_board_type();

	if (board_type >= DVT) {
		max8903_charger_device.resource = max8903_gpio_resources_dvt;
		max8903_charger_device.num_resources = ARRAY_SIZE(max8903_gpio_resources_dvt);
	} else if (board_type >= EVT1B) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1b;
		max8903_charger_device.num_resources = ARRAY_SIZE(max8903_gpio_resources_evt1b);
	} else if (board_type == EVT1A) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1a;
		max8903_charger_device.num_resources = ARRAY_SIZE(max8903_gpio_resources_evt1a);
	} else {
		pr_err("%s: Acclaim board %d not supported\n", __func__, board_type);
		return;
	}
	platform_device_register(&max8903_charger_device);
}

#endif

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

	.gpio = 0,

};
#endif

static const int sdp4430_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
};
static struct omap_device_pad keypad_pads[] = {
	{	.name   = "kpd_col0.kpd_col0",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE0,
	},
	{	.name   = "kpd_row0.kpd_row0",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE0,
	},
	{	.name   = "kpd_row1.kpd_row1",
		.enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN | OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
};

static struct matrix_keymap_data sdp4430_keymap_data = {
	.keymap			= sdp4430_keymap,
	.keymap_size		= ARRAY_SIZE(sdp4430_keymap),
};

static struct omap4_keypad_platform_data sdp4430_keypad_data = {
	.keymap_data		= &sdp4430_keymap_data,
	.rows			= 2,
	.cols			= 1,
};

static struct omap_board_data keypad_data = {
	.id	    		= 1,
	.pads	 		= keypad_pads,
	.pads_cnt       	= ARRAY_SIZE(keypad_pads),
};

/*static struct gpio_led sdp4430_gpio_leds[] = {
	{
		.name	= "omap4:green:debug0",
		.gpio	= 61,
	},
	{
		.name	= "omap4:green:debug1",
		.gpio	= 30,
	},
	{
		.name	= "omap4:green:debug2",
		.gpio	= 7,
	},
	{
		.name	= "omap4:green:debug3",
		.gpio	= 8,
	},
	{
		.name	= "omap4:green:debug4",
		.gpio	= 50,
	},
	{
		.name	= "omap4:blue:user",
		.gpio	= 169,
	},
	{
		.name	= "omap4:red:user",
		.gpio	= 170,
	},
	{
		.name	= "omap4:green:user",
		.gpio	= 139,
	},

};*/

static struct gpio_keys_button sdp4430_gpio_keys[] = {
/*	{
		.desc			= "Proximity Sensor",
		.type			= EV_SW,
		.code			= SW_FRONT_PROXIMITY,
		.gpio			= OMAP4_SFH7741_SENSOR_OUTPUT_GPIO,
		.active_low		= 0,
	}*/
	{
		.code 		= KEY_POWER,
		.gpio 		= 29,
		.desc 		= "POWER",
		.active_low 	= 0,
		.wakeup 	= 1,
	},
	{

		.code 		= KEY_HOME,
		.gpio 		= 32,
		.desc 		= "HOME",
		.active_low 	= 1,
		.wakeup 	= 1,
	}
};

/*static struct gpio_led_platform_data sdp4430_led_data = {
	.leds	= sdp4430_gpio_leds,
	.num_leds	= ARRAY_SIZE(sdp4430_gpio_leds),
};*/

/*static struct led_pwm sdp4430_pwm_leds[] = {
	{
		.name		= "omap4:green:chrg",
		.pwm_id		= 1,
		.max_brightness	= 255,
		.pwm_period_ns	= 7812500,
	},
};

static struct led_pwm_platform_data sdp4430_pwm_data = {
	.num_leds	= ARRAY_SIZE(sdp4430_pwm_leds),
	.leds		= sdp4430_pwm_leds,
};

static struct platform_device sdp4430_leds_pwm = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_pwm_data,
	},
};*/

/*static int omap_prox_activate(struct device *dev)
{
	gpio_set_value(OMAP4_SFH7741_ENABLE_GPIO , 1);
	return 0;
}

static void omap_prox_deactivate(struct device *dev)
{
	gpio_set_value(OMAP4_SFH7741_ENABLE_GPIO , 0);
}*/

static struct gpio_keys_platform_data sdp4430_gpio_keys_data = {
	.buttons	= sdp4430_gpio_keys,
	.nbuttons	= ARRAY_SIZE(sdp4430_gpio_keys),
//	.enable		= omap_prox_activate,
//	.disable	= omap_prox_deactivate,
};

static struct platform_device sdp4430_gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &sdp4430_gpio_keys_data,
	},
};

/*static struct platform_device sdp4430_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_led_data,
	},
};*/
static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	{
		.modalias		= "boxer_disp_spi",
		.bus_num		= 4,	/* 4: McSPI4 */
		.chip_select		= 0,
		.max_speed_hz		= 375000,
	},
};
/*static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 24000000,
		.irq                    = ETH_KS8851_IRQ,
	},
};

static struct gpio sdp4430_eth_gpios[] __initdata = {
	{ ETH_KS8851_POWER_ON,	GPIOF_OUT_INIT_HIGH,	"eth_power"	},
	{ ETH_KS8851_QUART,	GPIOF_OUT_INIT_HIGH,	"quart"		},
	{ ETH_KS8851_IRQ,	GPIOF_IN,		"eth_irq"	},
};*/

/*static int __init omap_ethernet_init(void)
{
	int status;

	// Request of GPIO lines 
	status = gpio_request_array(sdp4430_eth_gpios,
				    ARRAY_SIZE(sdp4430_eth_gpios));
	if (status)
		pr_err("Cannot request ETH GPIOs\n");

	return status;
}*/

/*static struct regulator_consumer_supply sdp4430_vbat_supply[] = {
	REGULATOR_SUPPLY("vddvibl", "twl6040-vibra"),
	REGULATOR_SUPPLY("vddvibr", "twl6040-vibra"),
};

*/
/*static struct regulator_init_data sdp4430_vbat_data = {
	.constraints = {
		.always_on	= 1,
	},
//	.num_consumer_supplies	= ARRAY_SIZE(sdp4430_vbat_supply),
//	.consumer_supplies	= sdp4430_vbat_supply,
};

static struct fixed_voltage_config sdp4430_vbat_pdata = {
	.supply_name	= "VBAT",
	.microvolts	= 3750000,
	.init_data	= &sdp4430_vbat_data,
	.gpio		= -EINVAL,
};

static struct platform_device sdp4430_vbat = {
	.name		= "reg-fixed-voltage",
	.id		= -1,
	.dev = {
		.platform_data = &sdp4430_vbat_pdata,
	},
};

static int sdp4430_batt_table[] = {
	// adc code for temperature in degree C 
	929, 925, // -2 ,-1 
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, // 00 - 09 
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, // 10 - 19 
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, // 20 - 29 
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, // 30 - 39 
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, // 40 - 49 
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, // 50 - 59 
	511, 504, 496 // 60 - 62 
};

static struct twl4030_bci_platform_data sdp4430_bci_data = {
//	.monitoring_interval		= 10,
//	.max_charger_currentmA		= 1500,
//	.max_charger_voltagemV		= 4560,
//	.max_bat_voltagemV		= 4200,
//	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= sdp4430_batt_table,
	.tblsize			= ARRAY_SIZE(sdp4430_batt_table),
};*/

static struct twl4030_madc_platform_data sdp4430_gpadc_data = {
	.irq_line	= 1,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 100,
};


 static int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
 {
 	static int power_state;
 
 	pr_debug("Powering %s wifi", (power_on ? "on" : "off"));
 
 	if (power_on == power_state)
 		return 0;
 	power_state = power_on;
 
 	if (power_on) {
 		gpio_set_value(GPIO_WIFI_PMENA, 1);
 		mdelay(15);
 		gpio_set_value(GPIO_WIFI_PMENA, 0);
 		mdelay(1);
 		gpio_set_value(GPIO_WIFI_PMENA, 1);
 		mdelay(70);
 	} else {
 		gpio_set_value(GPIO_WIFI_PMENA, 0);
 	}
 
 	return 0;
 }


static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.name		= "internal",
		.caps		=  MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
		.no_off_init	= true,
	},
	{
		.mmc		= 1,
		.name		= "external",
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA, // MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_29_30,
	},
// #ifdef CONFIG_TIWLAN_SDIO
// 	{
// 		.mmc		= 3,
// 		.caps		= MMC_CAP_4_BIT_DATA,
// 		.gpio_cd	= -EINVAL,
// 		.gpio_wp	= 4,
// 		.ocr_mask	= MMC_VDD_165_195,
// 	},
// #else
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
// #endif
	{}	/* Terminator */
};

static struct regulator_consumer_supply sdp4430_vaux1_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};

static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap-hsmmc.0"),
};

// static struct regulator_consumer_supply sdp4430_vemmc_supply[] = {
// 	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
// };

static struct regulator_consumer_supply sdp4430_vwlan_supply[] = {
{
	.supply = "vwlan",
},
};

static struct regulator_consumer_supply omap4_sdp4430_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data sdp4430_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap4_sdp4430_vmmc5_supply,
};

static struct fixed_voltage_config sdp4430_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.8V */
	.gpio			= GPIO_WIFI_PMENA,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &sdp4430_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &sdp4430_vwlan,
	},
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;

 	/* Set the MMC5 (wlan) power function */
 	if (pdev->id == 4)
 		pdata->slots[0].set_power = wifi_set_power;

	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(sdp4430_vaux1_supply),
	.consumer_supplies      = sdp4430_vaux1_supply,
};

static struct regulator_init_data sdp4430_vaux2 = {
        .constraints = {
                .min_uV                 = 1200000,
                .max_uV                 = 2800000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
                        | REGULATOR_MODE_STANDBY,
                .valid_ops_mask  = REGULATOR_CHANGE_VOLTAGE
                        | REGULATOR_CHANGE_MODE
                        | REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
};

static struct regulator_init_data sdp4430_vaux3 = {
        .constraints = {
                .min_uV                 = 1800000,
                .max_uV                 = 1800000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
                        | REGULATOR_MODE_STANDBY,
                .valid_ops_mask  = REGULATOR_CHANGE_VOLTAGE
                        | REGULATOR_CHANGE_MODE
                        | REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
                .always_on      = true,
        },
        .num_consumer_supplies = 1,
        .consumer_supplies = sdp4430_vwlan_supply,
};

static struct regulator_init_data sdp4430_vmmc = {
        .constraints = {
                .min_uV                 = 1200000,
                .max_uV                 = 3000000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
                        | REGULATOR_MODE_STANDBY,
                .valid_ops_mask  = REGULATOR_CHANGE_VOLTAGE
                        | REGULATOR_CHANGE_MODE
                        | REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
        .num_consumer_supplies  = 1,
        .consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
        .constraints = {
                .min_uV                 = 1800000,
                .max_uV                 = 2500000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
                        | REGULATOR_MODE_STANDBY,
                .valid_ops_mask  = REGULATOR_CHANGE_VOLTAGE
                        | REGULATOR_CHANGE_MODE
                        | REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
};
static struct regulator_init_data sdp4430_vusim = {
        .constraints = {
                .min_uV                 = 1200000,
                .max_uV                 = 2900000,
                .apply_uV               = true,
                .valid_modes_mask       = REGULATOR_MODE_NORMAL
                        | REGULATOR_MODE_STANDBY,
                .valid_ops_mask  = REGULATOR_CHANGE_VOLTAGE
                        | REGULATOR_CHANGE_MODE
                        | REGULATOR_CHANGE_STATUS,
                .state_mem = {
                        .enabled        = false,
                        .disabled       = true,
                },
        },
};

static struct regulator_init_data sdp4430_vana = {
  .constraints = {
    .min_uV                 = 2100000,
    .max_uV                 = 2100000,
    .apply_uV               = true,
    .valid_modes_mask       = REGULATOR_MODE_NORMAL
    | REGULATOR_MODE_STANDBY,
    .valid_ops_mask  = REGULATOR_CHANGE_MODE
    | REGULATOR_CHANGE_STATUS,
    .state_mem = {
      .enabled        = false,
      .disabled       = true,
    },
  },
};

static struct regulator_init_data sdp4430_vcxio = {
  .constraints = {
    .min_uV                 = 1800000,
    .max_uV                 = 1800000,
    .apply_uV               = true,
    .valid_modes_mask       = REGULATOR_MODE_NORMAL
    | REGULATOR_MODE_STANDBY,
    .valid_ops_mask  = REGULATOR_CHANGE_MODE
    | REGULATOR_CHANGE_STATUS,
    .state_mem = {
      .enabled        = false,
      .disabled       = true,
    },
    .always_on      = true,
  },
};

static struct regulator_init_data sdp4430_vdac = {
  .constraints = {
    .min_uV                 = 1800000,
    .max_uV                 = 1800000,
    .apply_uV               = true,
    .valid_modes_mask       = REGULATOR_MODE_NORMAL
    | REGULATOR_MODE_STANDBY,
    .valid_ops_mask  = REGULATOR_CHANGE_MODE
    | REGULATOR_CHANGE_STATUS,
    .state_mem = {
      .enabled        = false,
      .disabled       = true,
    },
  },
};


static struct regulator_init_data sdp4430_vusb = {
  .constraints = {
    .min_uV                 = 3300000,
    .max_uV                 = 3300000,
    .apply_uV               = true,
    .valid_modes_mask       = REGULATOR_MODE_NORMAL
    | REGULATOR_MODE_STANDBY,
    .valid_ops_mask  =      REGULATOR_CHANGE_MODE
    | REGULATOR_CHANGE_STATUS,
    .state_mem = {
      .enabled        = false,
      .disabled       = true,
    },
  },
};

static struct regulator_init_data sdp4430_clk32kg = {
  .constraints = {
    .valid_modes_mask       = REGULATOR_MODE_NORMAL,
    .valid_ops_mask  = REGULATOR_CHANGE_STATUS,
    .always_on      = true,
  },
};
//static struct twl4030_codec_data twl6040_codec = {
	/* single-step ramp for headset and handsfree */
//	.hs_left_step	= 0x0f,
//	.hs_right_step	= 0x0f,
//	.hf_left_step	= 0x1d,
//	.hf_right_step	= 0x1d,
//};

/*static struct twl4030_vibra_data twl6040_vibra = {
	.vibldrv_res = 8,
	.vibrdrv_res = 3,
	.viblmotor_res = 10,
	.vibrmotor_res = 10,
	.vddvibl_uV = 0,	// fixed volt supply - VBAT 
	.vddvibr_uV = 0,	// fixed volt supply - VBAT 
};*/

/*static struct twl4030_audio_data twl6040_audio = {
	.codec		= &twl6040_codec,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};*/
static struct twl4030_platform_data sdp4430_twldata = {
        .irq_base       = TWL6030_IRQ_BASE,
        .irq_end        = TWL6030_IRQ_END,

        /* Regulators */
        .vmmc           = &sdp4430_vmmc,
        .vpp            = &sdp4430_vpp,
        .vusim          = &sdp4430_vusim,  //not connected on the board
 //       .vana           = &sdp4430_vana,
 //       .vcxio          = &sdp4430_vcxio,
 //     .vdac           = &sdp4430_vdac,        //not used
//       .vusb           = &sdp4430_vusb,
        .vaux1          = &sdp4430_vaux1,
        .vaux2          = &sdp4430_vaux2,       //proximity sensor not functional switching off vaux2.
        .vaux3          = &sdp4430_vaux3,
        .clk32kg        = &sdp4430_clk32kg,      // always ON for WiFi
        .madc           = &sdp4430_gpadc_data,

};

static struct i2c_board_info __initdata sdp4430_i2c_1_boardinfo[] = {
	{
/*		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &sdp4430_twldata,*/
	},
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = 0,
	},
	{
		I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
		.platform_data = &max17042_platform_data_here,
		.irq = 0,
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CYTTSP_I2C_SLAVEADDRESS),
		.platform_data = &cyttsp_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_CYTTSP_GPIO),
	},
	{
		I2C_BOARD_INFO(FT_I2C_NAME, FT5x06_I2C_SLAVEADDRESS),
		.platform_data = &ft5x06_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_FT5x06_GPIO),
	},
	{
		I2C_BOARD_INFO("tlv320aic3100", 0x18),
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {
//	{
//		I2C_BOARD_INFO("tmp105", 0x48),
//	},
//	{
//		I2C_BOARD_INFO("bh1780", 0x29),
//	},
};
static struct i2c_board_info __initdata sdp4430_i2c_4_boardinfo[] = {
//	{
//		I2C_BOARD_INFO("hmc5843", 0x1e),
//	},
};

static struct usbhs_omap_board_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
//	.phy_reset  = false,
//	.reset_gpio_port[0]  = -EINVAL,
//	.reset_gpio_port[1]  = -EINVAL,
//	.reset_gpio_port[2]  = -EINVAL
};

static int __init omap4_i2c_init(void)
{
	int err;
	omap4_pmic_get_config(&sdp4430_twldata, TWL_COMMON_PDATA_USB,
			TWL_COMMON_REGULATOR_VDAC |
			//TWL_COMMON_REGULATOR_VAUX2 |
			//TWL_COMMON_REGULATOR_VAUX3 |
			//TWL_COMMON_REGULATOR_VMMC |
			//TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB); //|
			//TWL_COMMON_REGULATOR_CLK32KG);
	omap4_pmic_init("twl6030", &sdp4430_twldata);
	err=i2c_register_board_info(1,sdp4430_i2c_1_boardinfo, ARRAY_SIZE(sdp4430_i2c_1_boardinfo));
	if (err)
	  return err;
	//omap_register_i2c_bus(1, 400, sdp4430_i2c_1_boardinfo,
	//			ARRAY_SIZE(sdp4430_i2c_1_boardinfo));
	omap_register_i2c_bus(2, 400, sdp4430_i2c_2_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, sdp4430_i2c_3_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, sdp4430_i2c_4_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_4_boardinfo));
	return 0;
}

/*static void __init omap_sfh7741prox_init(void)
{
	int error;

	error = gpio_request_one(OMAP4_SFH7741_ENABLE_GPIO,
				 GPIOF_OUT_INIT_LOW, "sfh7741");
	if (error < 0)
		pr_err("%s:failed to request GPIO %d, error %d\n",
			__func__, OMAP4_SFH7741_ENABLE_GPIO, error);
}*/
static struct regulator_consumer_supply acclaim_lcd_tp_supply[] = {
	{ .supply = "vtp" },
	{ .supply = "vlcd" },
};

static struct regulator_init_data acclaim_lcd_tp_vinit = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 2,
	.consumer_supplies = acclaim_lcd_tp_supply,
};

static struct fixed_voltage_config acclaim_lcd_touch_reg_data = {
	.supply_name = "vdd_lcdtp",
	.microvolts = 3300000,
	.gpio = 36,
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &acclaim_lcd_tp_vinit,
};

static struct platform_device acclaim_lcd_touch_regulator_device = {
	.name   = "reg-fixed-voltage",
	.id     = -1,
	.dev    = {
		.platform_data = &acclaim_lcd_touch_reg_data,
	},
};
// static void sdp4430_hdmi_mux_init(void)
// {
// 	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
// //	omap_mux_init_signal("hdmi_hpd",
// //			OMAP_PIN_INPUT_PULLUP);
// //	omap_mux_init_signal("hdmi_cec",
// //			OMAP_PIN_INPUT_PULLUP);
// 	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
// //	omap_mux_init_signal("hdmi_ddc_scl",
// //			OMAP_PIN_INPUT_PULLUP);
// //	omap_mux_init_signal("hdmi_ddc_sda",
// //			OMAP_PIN_INPUT_PULLUP);
// }

/*static struct gpio sdp4430_hdmi_gpios[] = {
	{ HDMI_GPIO_HPD,	GPIOF_OUT_INIT_HIGH,	"hdmi_gpio_hpd"   },
	{ HDMI_GPIO_LS_OE,	GPIOF_OUT_INIT_HIGH,	"hdmi_gpio_ls_oe" },
};*/

/*static int sdp4430_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	int status;

	status = gpio_request_array(sdp4430_hdmi_gpios,
				    ARRAY_SIZE(sdp4430_hdmi_gpios));
	if (status)
		pr_err("%s: Cannot request HDMI GPIOs\n", __func__);

	return status;
}*/

/*static void sdp4430_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_free(HDMI_GPIO_LS_OE);
	gpio_free(HDMI_GPIO_HPD);
}*/

/*static struct nokia_dsi_panel_data dsi1_panel = {
		.name		= "taal",
		.reset_gpio	= 102,
		.use_ext_te	= false,
		.ext_te_gpio	= 101,
		.esd_interval	= 0,
};

static struct omap_dss_device sdp4430_lcd_device = {
	.name			= "lcd",
	.driver_name		= "taal",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi1_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,

		.module		= 0,
	},

	.clocks = {
		.dispc = {
			.channel = {
				// Logic Clock = 172.8 MHz 
				.lck_div	= 1,
				// Pixel Clock = 34.56 MHz 
				.pck_div	= 5,
				.lcd_clk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},

		.dsi = {
			.regn		= 16,	// Fint = 2.4 MHz 
			.regm		= 180,	// DDR Clock = 216 MHz 
			.regm_dispc	= 5,	// PLL1_CLK1 = 172.8 MHz 
			.regm_dsi	= 5,	// PLL1_CLK2 = 172.8 MHz 

			.lp_clk_div	= 10,	// LP Clock = 8.64 MHz 
			.dsi_fclk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
		},
	},
	.channel		= OMAP_DSS_CHANNEL_LCD,
};*/

/*static struct nokia_dsi_panel_data dsi2_panel = {
		.name		= "taal",
		.reset_gpio	= 104,
		.use_ext_te	= false,
		.ext_te_gpio	= 103,
		.esd_interval	= 0,
};

static struct omap_dss_device sdp4430_lcd2_device = {
	.name			= "lcd2",
	.driver_name		= "taal",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi2_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,

		.module		= 1,
	},

	.clocks = {
		.dispc = {
			.channel = {
				// Logic Clock = 172.8 MHz 
				.lck_div	= 1,
				// Pixel Clock = 34.56 MHz 
				.pck_div	= 5,
				.lcd_clk_src	= OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},

		.dsi = {
			.regn		= 16,	// Fint = 2.4 MHz 
			.regm		= 180,	// DDR Clock = 216 MHz 
			.regm_dispc	= 5,	// PLL1_CLK1 = 172.8 MHz 
			.regm_dsi	= 5,	// PLL1_CLK2 = 172.8 MHz 

			.lp_clk_div	= 10,	// LP Clock = 8.64 MHz 
			.dsi_fclk_src	= OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DSI,
		},
	},
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};*/

// static void sdp4430_lcd_init(void)
// {
// /*	int r;
// 
// 	r = gpio_request_one(dsi1_panel.reset_gpio, GPIOF_DIR_OUT,
// 		"lcd1_reset_gpio");
// 	if (r)
// 		pr_err("%s: Could not get lcd1_reset_gpio\n", __func__);
// 
// 	r = gpio_request_one(dsi2_panel.reset_gpio, GPIOF_DIR_OUT,
// 		"lcd2_reset_gpio");
// 	if (r)
// 		pr_err("%s: Could not get lcd2_reset_gpio\n", __func__);*/
// }

/*static struct omap_dss_device sdp4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.platform_enable = sdp4430_panel_enable_hdmi,
	.platform_disable = sdp4430_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct picodlp_panel_data sdp4430_picodlp_pdata = {
	.picodlp_adapter_id	= 2,
	.emu_done_gpio		= 44,
	.pwrgood_gpio		= 45,
};*/

/*static void sdp4430_picodlp_init(void)
{
	int r;
	const struct gpio picodlp_gpios[] = {
		{DLP_POWER_ON_GPIO, GPIOF_OUT_INIT_LOW,
			"DLP POWER ON"},
		{sdp4430_picodlp_pdata.emu_done_gpio, GPIOF_IN,
			"DLP EMU DONE"},
		{sdp4430_picodlp_pdata.pwrgood_gpio, GPIOF_OUT_INIT_LOW,
			"DLP PWRGOOD"},
	};

	r = gpio_request_array(picodlp_gpios, ARRAY_SIZE(picodlp_gpios));
	if (r)
		pr_err("Cannot request PicoDLP GPIOs, error %d\n", r);
}*/

/*static int sdp4430_panel_enable_picodlp(struct omap_dss_device *dssdev)
{
	gpio_set_value(DISPLAY_SEL_GPIO, 0);
	gpio_set_value(DLP_POWER_ON_GPIO, 1);

	return 0;
}*/

/*static void sdp4430_panel_disable_picodlp(struct omap_dss_device *dssdev)
{
	gpio_set_value(DLP_POWER_ON_GPIO, 0);
	gpio_set_value(DISPLAY_SEL_GPIO, 1);
}*/

/*static struct omap_dss_device sdp4430_picodlp_device = {
	.name			= "picodlp",
	.driver_name		= "picodlp_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
	.platform_enable	= sdp4430_panel_enable_picodlp,
	.platform_disable	= sdp4430_panel_disable_picodlp,
	.data			= &sdp4430_picodlp_pdata,
};*/

/*static struct omap_dss_device *sdp4430_dss_devices[] = {
	&sdp4430_lcd_device,
	&sdp4430_lcd2_device,
	&sdp4430_hdmi_device,
	&sdp4430_picodlp_device,
};*/

/*static struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices	= ARRAY_SIZE(sdp4430_dss_devices),
	.devices	= sdp4430_dss_devices,
	.default_device	= &sdp4430_lcd_device,
};*/

// static void omap_4430sdp_display_init(void)
// {
// /*	int r;
// 
// 	// Enable LCD2 by default (instead of Pico DLP)
// 	r = gpio_request_one(DISPLAY_SEL_GPIO, GPIOF_OUT_INIT_HIGH,
// 			"display_sel");
// 	if (r)
// 		pr_err("%s: Could not get display_sel GPIO\n", __func__);
// 
// 	sdp4430_lcd_init();
// 	sdp4430_hdmi_mux_init();
// 	sdp4430_picodlp_init();
// 	omap_display_init(&sdp4430_dss_data);*/
// }
#ifdef CONFIG_TI_ST
static bool is_bt_active(void)
{
	struct platform_device  *pdev;
	struct kim_data_s       *kim_gdata;

	pdev = &wl128x_device;
	kim_gdata = dev_get_drvdata(&pdev->dev);
	if (st_ll_getstate(kim_gdata->core_data) != ST_LL_ASLEEP &&
			st_ll_getstate(kim_gdata->core_data) != ST_LL_INVALID)
		return true;
	else
		return false;
}
#else
#define is_bt_active NULL
#endif

/*static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
//		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
//		.plat_hold_wakelock = plat_hold_wakelock,
		.plat_omap_bt_active = NULL,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.cts_padconf	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_MCSPI1_CS1_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
		.wk_mask	=
			OMAP4_MCSPI1_CS1_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_MCSPI1_CS2_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_MCSPI1_CS3_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
//		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
//		.plat_hold_wakelock = plat_hold_wakelock,
		.plat_omap_bt_active = is_bt_active,
		.rts_padconf	= OMAP4_CTRL_MODULE_PAD_UART2_RTS_OFFSET,
		.rts_override	= 0,
		.cts_padconf	= OMAP4_CTRL_MODULE_PAD_UART2_CTS_OFFSET,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART2_RX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
		.wk_mask	=
			OMAP4_UART2_TX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_RX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_RTS_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART2_CTS_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
//		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
//		.plat_hold_wakelock = plat_hold_wakelock,
		.plat_omap_bt_active = NULL,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.cts_padconf	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART3_RX_IRRX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
		.wk_mask	=
			OMAP4_UART3_TX_IRTX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_RX_IRRX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_RTS_SD_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.use_dma	= 0,
//		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
//		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
//		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
//		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
//		.plat_hold_wakelock = NULL,
		.plat_omap_bt_active = NULL,
		.rts_padconf	= 0,
		.rts_override	= 0,
		.cts_padconf	= 0,
		.padconf	= OMAP4_CTRL_MODULE_PAD_UART4_RX_OFFSET,
		.padconf_wake_ev =
			OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
		.wk_mask	=
			OMAP4_UART4_TX_DUPLICATEWAKEUPEVENT_MASK |
			OMAP4_UART4_RX_DUPLICATEWAKEUPEVENT_MASK,
	},
	{
		.flags		= 0
	}
};*/

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
// #ifndef CONFIG_TIWLAN_SDIO
// 	/* WLAN IRQ - GPIO 53 */
// 	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
// 	/* WLAN_EN - GPIO 54 */
// 	OMAP4_MUX(GPMC_NWP, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
// 	/* WLAN SDIO: MMC5 CMD */
// 	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
// 	/* WLAN SDIO: MMC5 CLK */
// 	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
// 	/* WLAN SDIO: MMC5 DAT[0-3] */
// 	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
// 	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
// 	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
// 	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
// #endif
	OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap_device_pad serial2_pads[] __initdata = {
	OMAP_MUX_STATIC("uart2_cts.uart2_cts",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rts.uart2_rts",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rx.uart2_rx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_tx.uart2_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial3_pads[] __initdata = {
	OMAP_MUX_STATIC("uart3_cts_rctx.uart3_cts_rctx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rts_sd.uart3_rts_sd",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rx_irrx.uart3_rx_irrx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_tx_irtx.uart3_tx_irtx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial4_pads[] __initdata = {
	OMAP_MUX_STATIC("uart4_rx.uart4_rx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart4_tx.uart4_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};


static struct omap_board_data serial2_data __initdata = {
	.id		= 1,
	.pads		= serial2_pads,
	.pads_cnt	= ARRAY_SIZE(serial2_pads),
};

static struct omap_board_data serial3_data __initdata = {
	.id		= 2,
	.pads		= serial3_pads,
	.pads_cnt	= ARRAY_SIZE(serial3_pads),
};

static struct omap_board_data serial4_data __initdata = {
	.id		= 3,
	.pads		= serial4_pads,
	.pads_cnt	= ARRAY_SIZE(serial4_pads),
};

static inline void board_serial_init(void)
{
	struct omap_board_data bdata;
	bdata.flags	= 0;
	bdata.pads	= NULL;
	bdata.pads_cnt	= 0;
	bdata.id	= 0;
	// pass dummy data for UART1 
	omap_serial_init_port(&bdata);
	early_print("Serial port 1 init - DONE\n");
	
	omap_serial_init_port(&serial2_data);
	early_print("Serial port 2 init - DONE\n");
	
// 	omap_serial_init_port(&serial3_data);
// 	early_print("Serial port 3 init - DONE\n");
	
	omap_serial_init_port(&serial4_data);
	early_print("Serial port 4 init - DONE\n");
}
static void omap4_sdp4430_wifi_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);

}

static struct wl12xx_platform_data omap4_sdp4430_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,//WL12XX_TCXOCLOCK_26,
};

static void omap4_sdp4430_wifi_init(void)
{
	omap4_sdp4430_wifi_mux_init();
	int ret;
       /* WL12xx WLAN Init */
       ret = wl12xx_set_platform_data(&omap4_sdp4430_wlan_data);
       if (ret)
               pr_err("error setting wl12xx data: %d\n", ret);
 	platform_device_register(&omap_vwlan_device);
}

#define DEFAULT_BACKLIGHT_BRIGHTNESS    105
static void __init show_acclaim_board_revision(const int revision)
{
	switch (revision) {
		case EVT1A:
			printk(KERN_INFO "Board revision %s\n", "EVT1A");
			break;
		case EVT1B:
			printk(KERN_INFO "Board revision %s\n", "EVT1B");
			break;
		case EVT2:
			printk(KERN_INFO "Board revision %s\n", "EVT2");
			break;
		case DVT:
			printk(KERN_INFO "Board revision %s\n", "DVT");
			break;
		case PVT:
			printk(KERN_INFO "Board revision %s\n", "PVT");
			break;
		default:
			printk(KERN_ERR "Board revision UNKNOWN (0x%x)\n", revision);
			break;
	}
}

void __init acclaim_board_init(void)
{
	const int board_type = acclaim_board_type();
	show_acclaim_board_revision(board_type);

	if ( board_type == EVT1A ){
		max17042_gpio_for_irq = 98;
		kxtf9_gpio_for_irq = 99;
	} else if ( board_type >= EVT1B ) {
		max17042_gpio_for_irq = 65;
		kxtf9_gpio_for_irq = 66;
	}

	max17042_platform_data_here.gpio = max17042_gpio_for_irq;
	sdp4430_i2c_1_boardinfo[2].irq = OMAP_GPIO_IRQ(max17042_gpio_for_irq);
	kxtf9_platform_data_here.gpio = kxtf9_gpio_for_irq;
	sdp4430_i2c_1_boardinfo[1].irq = OMAP_GPIO_IRQ(kxtf9_gpio_for_irq);
	omap_mux_init_signal("sys_pwron_reset_out", OMAP_MUX_MODE3);
	omap_mux_init_signal("fref_clk3_req", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN);
}

static void acclaim4430_init_display_led(void)
{
        if (acclaim_board_type() >= EVT2) {
                printk(KERN_INFO "init_display_led: evt2 hardware\n");
                omap_mux_init_signal("abe_dmic_din2.dmtimer11_pwm_evt", OMAP_MUX_MODE5);
        } else {
                printk(KERN_INFO "init_display_led: evt1 hardware\n");
                printk(KERN_INFO "WARNING: brigthness control disabled on EVT1 hardware\n");
                /* mux the brightness control pin as gpio, because on EVT1 it is connected to
                   timer8 and we cannot use timer8 because of audio conflicts causing crash */
                omap_mux_init_signal("usbb1_ulpitll_dat4.gpio_92", OMAP_MUX_MODE3);
                if (gpio_request(92, "EVT1 BACKLIGHT"))
                        printk(KERN_ERR "ERROR: failed to request backlight gpio\n");
                else
                        gpio_direction_output(92, 0);
        }
}

static void acclaim4430_disp_backlight_setpower(struct omap_pwm_led_platform_data *pdata, int state)
{
        if (state)
                gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 1 : 0);
        else
                gpio_direction_output(38, (acclaim_board_type() >= EVT2) ? 0 : 1);
        gpio_direction_output(44, 0);
        gpio_direction_output(45, 0);
        printk("[BL set power] %d\n", state);
}

static struct omap_pwm_led_platform_data acclaim4430_disp_backlight_data = {
        .name            = "lcd-backlight",
        .intensity_timer = 11,
        .def_on          = 0,
        .def_brightness  = DEFAULT_BACKLIGHT_BRIGHTNESS,
        .set_power       = acclaim4430_disp_backlight_setpower,
};

static struct platform_device sdp4430_disp_led = {
        .name   =       "omap_pwm_led",
        .id     =       -1,
        .dev    = {
                .platform_data = &acclaim4430_disp_backlight_data,
        },
};

static struct platform_device *sdp4430_devices[] __initdata = {
	&sdp4430_gpio_keys_device,
//	&sdp4430_leds_gpio,
//	&sdp4430_leds_pwm,
 	&sdp4430_disp_led,
	&acclaim_lcd_touch_regulator_device,
//	&sdp4430_vbat,
};

/*--------------------------------------------------------------------------*/

static void sdp4430_panel_get_resource(void)
{
        int ret_val = 0;

        ret_val = gpio_request(38, "BOXER BL PWR EN");

        if ( ret_val ) {
                printk("%s : Could not request bl pwr en\n",__FUNCTION__);
        }
        ret_val = gpio_request(44, "BOXER CABC0");
        if ( ret_val ){
                printk( "%s : could not request CABC0\n",__FUNCTION__);
        }
        ret_val = gpio_request(45, "BOXER CABC1");
        if ( ret_val ) {
                printk("%s: could not request CABC1\n",__FUNCTION__);
        }
}

static struct boxer_panel_data boxer_panel;

static struct omap_dss_device sdp4430_boxer_device = {
        .name                           = "boxerLCD",
        .driver_name            	= "boxer_panel",
        .type                           = OMAP_DISPLAY_TYPE_DPI,
        .phy.dpi.data_lines     	= 24,
        .channel                        = OMAP_DSS_CHANNEL_LCD2,
        .data                           = &boxer_panel,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
        &sdp4430_boxer_device,
};

static __initdata struct omap_dss_board_info sdp4430_dss_data = {
        .num_devices    =       ARRAY_SIZE(sdp4430_dss_devices),
        .devices        =       sdp4430_dss_devices,
        .default_device =       &sdp4430_boxer_device,
};

void __init acclaim_panel_init(void)
{
        early_print("Panel get resource\n");
	sdp4430_panel_get_resource();
        
	early_print("Init display LED\n"); 
        acclaim4430_init_display_led();
             
	early_print("Display init\n");
	omap_display_init(&sdp4430_dss_data);
        
/*	

	early_print("Platform add devices\n");
        platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));*/
        
}

static void enable_rtc_gpio(void){
	/* To access twl registers we enable gpio6
	 * we need this so the RTC driver can work.
	 */
	gpio_request(TWL6030_RTC_GPIO, "h_SYS_DRM_MSEC");
	gpio_direction_output(TWL6030_RTC_GPIO, 1);

	omap_mux_init_signal("fref_clk0_out.gpio_wk6", \
			OMAP_PIN_OUTPUT | OMAP_PIN_OFF_NONE);
	return;
}
int __init omap4_twl_init(void);

static void __init omap_4430sdp_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;
	
	const int board_type = acclaim_board_type();
	show_acclaim_board_revision(board_type);

	early_print("Starting NOOK TABLET!\n");
	
	if (omap_rev() == OMAP4430_REV_ES1_0)
	{
		package = OMAP_PACKAGE_CBL;
		early_print("Selected CBL package!\n");
		
	}
	
	early_print("Omap mux init\n");
	omap4_mux_init(board_mux, NULL, package);
	
	
	enable_rtc_gpio();
	early_print("I2C init\n");
	omap4_i2c_init();
	
	acclaim_board_init();
	acclaim_init_charger();


	early_print("Add NookTablet devices\n");
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
		
	early_print("Starting OMAP serial init:\n");
	board_serial_init();
	early_print("OMAP SDRC init\n");
	omap_sdrc_init(NULL, NULL);
	
	early_print("TWL6030 HSMMC init\n");
	omap4_twl6030_hsmmc_init(mmc);
	
	early_print("Omap WIFI init\n");
	omap4_sdp4430_wifi_init();
	
//	if (omap4_twl_init()!=0)
//	  early_print("Error init TLW PMIC\n");
	kxtf9_dev_init();
#ifdef CONFIG_BATTERY_MAX17042
	early_print("MAX17042_DEVICE init\n");
	max17042_dev_init();
#endif
	usbhs_init(&usbhs_pdata);
	usb_musb_init(&musb_board_data);

/*	status = omap_ethernet_init();
	if (status) {
		pr_err("Ethernet initialization failed: %d\n", status);
	} else {
		sdp4430_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
		spi_register_board_info(sdp4430_spi_board_info,
				ARRAY_SIZE(sdp4430_spi_board_info));
	}
*/
	status = omap4_keyboard_init(&sdp4430_keypad_data, &keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);
        
 	spi_register_board_info(sdp4430_spi_board_info,
                        ARRAY_SIZE(sdp4430_spi_board_info));

//	early_print("SPI init\n");
        acclaim_panel_init();
	
//	omap_4430sdp_display_init();
}

MACHINE_START(OMAP4_NOOKTABLET, "B@N NOOK TABLET")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap4_map_io,
	.init_early	= omap4430_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap_4430sdp_init,
	.timer		= &omap4_timer,
MACHINE_END
