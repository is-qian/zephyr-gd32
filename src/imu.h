#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

struct imu_state {
	int16_t gyro[3];    /* raw, +/-500 dps, 17.5 mdps/LSB */
	int16_t accel[3];   /* raw, +/-2 g, 0.061 mg/LSB */
	int16_t temp_raw;   /* 1/256 C/LSB, 0 at 25 C */
	uint8_t quat[6];    /* SFLP game rotation x/y/z, IEEE half LE */
	uint8_t sample_counter;
	bool online;
	bool quat_valid;
	bool fifo_ovr;
};

int imu_init(void);
/* Bit-bang SCL/SDA out of a slave-held bus lockup, then restore pinctrl. */
void imu_bus_recover(void);
/* One bounded I2C transaction per call; only call while the DXL bus is idle. */
void imu_step(void);
void imu_get_state(struct imu_state *out);

#endif
