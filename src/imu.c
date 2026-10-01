/*
 * LSM6DSV16X over I2C0 (PB6/PB7, addr 0x6A).
 *
 * Gyro runs at +/-500 dps because that is the scale the bus consumer decodes.
 * Orientation comes from the chip's SFLP block: the game-rotation quaternion
 * is only exposed through the FIFO (tag 0x13, three IEEE half floats), so the
 * FIFO runs in stream mode with SFLP batching enabled.
 */

#include "imu.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#define REG_FUNC_CFG_ACCESS  0x01
#define REG_WHO_AM_I         0x0f
#define REG_CTRL1            0x10
#define REG_CTRL2            0x11
#define REG_CTRL3            0x12
#define REG_CTRL6            0x15
#define REG_CTRL8            0x17
#define REG_FIFO_STATUS1     0x1b
#define REG_OUT_TEMP_L       0x20
#define REG_FIFO_DATA_OUT    0x78

#define EMB_FUNC_EN_A        0x04
#define EMB_FUNC_FIFO_EN_A   0x44
#define EMB_SFLP_ODR         0x5e

#define WHO_AM_I_VALUE       0x70

#define TAG_SFLP_GAME_ROT    0x13

#define FIFO_WORD_LEN        7
#define FIFO_DRAIN_MAX       4

static const struct i2c_dt_spec imu_bus = I2C_DT_SPEC_GET(DT_NODELABEL(lsm6dsv16x));

static struct imu_state state;
static uint16_t fifo_level;
static enum { STEP_FIFO_STATUS, STEP_FIFO_DRAIN, STEP_OUT_REGS } step;

static int wr(uint8_t reg, uint8_t value)
{
	return i2c_reg_write_byte_dt(&imu_bus, reg, value);
}

static int rd(uint8_t reg, uint8_t *buf, uint8_t len)
{
	return i2c_burst_read_dt(&imu_bus, reg, buf, len);
}

static int emb_access(bool on)
{
	return wr(REG_FUNC_CFG_ACCESS, on ? 0x80 : 0x00);
}

int imu_init(void)
{
	uint8_t id = 0;

	state.online = false;
	state.quat_valid = false;
	state.fifo_ovr = false;
	if (rd(REG_WHO_AM_I, &id, 1) < 0 || id != WHO_AM_I_VALUE) {
		return -1;
	}
	if (wr(REG_CTRL3, 0x01) < 0) {          /* sw_reset */
		return -1;
	}
	for (volatile int i = 0; i < 100000; i++) {
	}
	if (wr(REG_CTRL3, 0x44) < 0 ||          /* BDU + IF_INC */
	    wr(REG_CTRL6, 0x02) < 0 ||          /* gyro +/-500 dps */
	    wr(REG_CTRL8, 0x00) < 0 ||          /* accel +/-2 g */
	    emb_access(true) < 0 ||
	    wr(EMB_FUNC_FIFO_EN_A, 0x02) < 0 || /* batch SFLP game rotation */
	    emb_access(false) < 0 ||
	    wr(0x0a, 0x06) < 0 ||               /* FIFO_CTRL4: stream mode */
	    wr(REG_CTRL1, 0x06) < 0 ||          /* accel ODR 120 Hz */
	    wr(REG_CTRL2, 0x06) < 0 ||          /* gyro ODR 120 Hz */
	    emb_access(true) < 0 ||
	    wr(EMB_SFLP_ODR, 0x18) < 0 ||       /* SFLP 120 Hz */
	    wr(EMB_FUNC_EN_A, 0x02) < 0 ||      /* sflp_game_en */
	    emb_access(false) < 0) {
		return -1;
	}
	state.online = true;
	step = STEP_FIFO_STATUS;
	return 0;
}

static void fifo_status_step(void)
{
	uint8_t buf[2];

	if (rd(REG_FIFO_STATUS1, buf, 2) < 0) {
		state.online = false;
		return;
	}
	fifo_level = ((uint16_t)(buf[1] & 0x01) << 8) | buf[0];
	if (buf[1] & 0x60) {                    /* full / overrun */
		state.fifo_ovr = true;
	}
	step = fifo_level ? STEP_FIFO_DRAIN : STEP_OUT_REGS;
}

static void fifo_drain_step(void)
{
	uint8_t buf[FIFO_DRAIN_MAX * FIFO_WORD_LEN];
	uint8_t n = fifo_level > FIFO_DRAIN_MAX ? FIFO_DRAIN_MAX : (uint8_t)fifo_level;

	if (rd(REG_FIFO_DATA_OUT, buf, n * FIFO_WORD_LEN) < 0) {
		state.online = false;
		return;
	}
	for (uint8_t i = 0; i < n; i++) {
		const uint8_t *word = &buf[i * FIFO_WORD_LEN];

		if ((word[0] >> 3) == TAG_SFLP_GAME_ROT) {
			memcpy(state.quat, &word[1], 6);
			state.quat_valid = true;
			state.sample_counter++;
		}
	}
	step = STEP_OUT_REGS;
}

static void out_regs_step(void)
{
	uint8_t buf[14];

	if (rd(REG_OUT_TEMP_L, buf, sizeof(buf)) < 0) {
		state.online = false;
		return;
	}
	state.temp_raw = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
	for (int i = 0; i < 3; i++) {
		state.gyro[i] = (int16_t)(buf[2 + 2 * i] | ((uint16_t)buf[3 + 2 * i] << 8));
		state.accel[i] = (int16_t)(buf[8 + 2 * i] | ((uint16_t)buf[9 + 2 * i] << 8));
	}
	step = STEP_FIFO_STATUS;
}

void imu_step(void)
{
	switch (step) {
	case STEP_FIFO_STATUS:
		fifo_status_step();
		break;
	case STEP_FIFO_DRAIN:
		fifo_drain_step();
		break;
	default:
		out_regs_step();
		break;
	}
}

void imu_get_state(struct imu_state *out)
{
	*out = state;
}
