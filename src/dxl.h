#ifndef DXL_H
#define DXL_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/* Battery voltage in mV, shown in the control table in 0.1 V units. */
void dxl_set_battery_mv(uint32_t mv);

void dxl_init(const struct device *uart);

/* Consume received bytes, run timeouts, answer requests. Call in a tight loop. */
void dxl_process(void);

/* Anything in flight or heard on the bus within the last 2 ms. Sensor I2C
 * traffic only runs while this is false, so a slow sensor read can never hold
 * up a bus answer.
 */
bool dxl_bus_active(void);

#endif
