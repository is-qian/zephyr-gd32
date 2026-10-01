/*
 * TSSOP20 package has no PA9/PA10 pins; the PCB routes UART to pins 16/17
 * (PA11/PA12). Remap them to PA9/PA10 so the usart0 pinctrl (PA9/PA10)
 * reaches the header. Must run before the uart driver init.
 */

#include <zephyr/init.h>
#include <zephyr/sys/util.h>

#define RCU_APBEN   (*(volatile uint32_t *)0x40021044U)
#define SYSCFG_CFG0 (*(volatile uint32_t *)0x40010000U)

static int gd32c231f8_dk_init(void)
{
	RCU_APBEN |= BIT(0);              /* SYSCFG clock */
	SYSCFG_CFG0 |= BIT(3) | BIT(4);   /* PA11->PA9, PA12->PA10 remap */
	return 0;
}

SYS_INIT(gd32c231f8_dk_init, EARLY, 0);
