/*
 * MicroDuck power board MCU (GD32C231F8P6TR)
 *
 * Dynamixel protocol 2.0 device on the single-wire half-duplex bus it shares
 * with the servos: ID 200, 1 Mbps. The IMU and battery voltage sit in a
 * control table (see dxl.c); the host reads everything as one block at
 * address 124 or register by register.
 *
 * The bus engine runs on the main thread and nothing else, so a stuck sensor
 * can never take the bus down. Sensor I2C/ADC work lives in its own thread
 * and only runs while the bus has been quiet for 2 ms.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "dxl.h"
#include "imu.h"

#define ADC_REF_MV     3300
#define ADC_FULL_SCALE 4095
/* divider: Vbat = Vadc x 2.8 */
#define VBAT_X100_NUM  280
#define ADC_AVG        32
#define IMU_RETRY_MS   1000

static const struct gpio_dt_spec adc_ctl =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), adc_ctl_gpios);
static const struct adc_dt_spec adc_chan =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(usart0));

static uint32_t adc_sum;
static uint8_t adc_count;

static void adc_step(void)
{
	uint16_t value;
	struct adc_sequence seq = {
		.channels = BIT(adc_chan.channel_id),
		.buffer = &value,
		.buffer_size = sizeof(value),
		.resolution = 12,
	};

	if (adc_read(adc_chan.dev, &seq) < 0) {
		return;
	}
	adc_sum += value;
	if (++adc_count < ADC_AVG) {
		return;
	}
	uint32_t raw = adc_sum / ADC_AVG;

	adc_sum = 0;
	adc_count = 0;
	dxl_set_battery_mv(raw * ADC_REF_MV / ADC_FULL_SCALE * VBAT_X100_NUM / 100);
}

static void sensor_thread(void *p1, void *p2, void *p3)
{
	uint32_t imu_retry_at = 0;

	imu_init();
	for (;;) {
		if (dxl_bus_active()) {
			k_sleep(K_MSEC(1));
			continue;
		}
		struct imu_state im;

		imu_get_state(&im);
		if (!im.online) {
			uint32_t now = k_uptime_get_32();

			if ((int32_t)(now - imu_retry_at) >= 0) {
				imu_init();
				imu_retry_at = now + IMU_RETRY_MS;
			}
			k_sleep(K_MSEC(10));
		} else {
			imu_step();
			k_sleep(K_MSEC(1));
		}
		if (!dxl_bus_active()) {
			adc_step();
		}
	}
}

K_THREAD_DEFINE(sensor_tid, 2048, sensor_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
	if (!device_is_ready(adc_ctl.port) || !device_is_ready(adc_chan.dev) ||
	    !device_is_ready(uart)) {
		return 0;
	}
	gpio_pin_configure_dt(&adc_ctl, GPIO_OUTPUT_HIGH);
	adc_channel_setup_dt(&adc_chan);

	dxl_init(uart);

	for (;;) {
		dxl_process();
		/* prio 0 must not spin forever or the sensor thread starves */
		k_sleep(K_MSEC(1));
	}
	return 0;
}
