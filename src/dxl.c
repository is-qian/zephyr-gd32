/*
 * Dynamixel protocol 2.0 device side, for the half-duplex single-wire bus.
 *
 * The board answers only its own ID, so it shares the bus with the XL330
 * servos without disturbing them: PING / READ / WRITE / REBOOT addressed to
 * us, plus SYNC READ and FAST SYNC READ where we take our slot in the ID
 * list (appending after earlier devices when we are not listed first).
 *
 * Wire details that must match the host stack (rustypot, mirroring the
 * official SDK): the stuffing byte is 0xFD inserted after an FF FF FD
 * pattern, the length field and the CRC cover the stuffed bytes, and fast
 * sync read responses are never stuffed.
 *
 * The on-board 74LVC1G125/126 pair derives the bus direction from TXD, so
 * the firmware never toggles a direction pin and never hears its own echo.
 */

#include "dxl.h"

#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "imu.h"

#define DXL_ID_DEFAULT   200
#define DXL_ID_BROADCAST 0xfe
#define DXL_MODEL_NUMBER 0x4d44
#define DXL_FW_VERSION   2
#define DXL_BAUD_CODE    3 /* 1 Mbps */

#define INSTR_PING          0x01
#define INSTR_READ          0x02
#define INSTR_WRITE         0x03
#define INSTR_REBOOT        0x08
#define INSTR_STATUS        0x55
#define INSTR_SYNC_READ     0x82
#define INSTR_FAST_SYNC_READ 0x8a

#define ERR_INSTRUCTION 0x02
#define ERR_ACCESS      0x07

/* Control table: identity block 0..9, sensor block 124..146. The 12 bytes at
 * 124 (gyro + quaternion) and the u16/u8 at 144/146 match what robotd reads
 * off the imu_to_dxl board; 124..146 read in one go is the get-everything
 * interface.
 */
#define REG_ID_ADDR     7
#define REG_SENSOR_BASE 124
#define SENSOR_MAP_LEN  23

#define RX_RING_LEN     512
#define PKT_BUF_LEN     300 /* header 4 + id/len 3 + body, generous */
#define MAX_READ_LEN    64
#define STUFFED_LEN(plen) ((plen) + (plen) / 3 + 2)

#define MONITOR_TIMEOUT_MS 30

static const struct device *uart_dev;
static uint8_t dev_id = DXL_ID_DEFAULT;
static uint32_t battery_mv;

static uint8_t rx_ring[RX_RING_LEN];
static volatile uint16_t ring_head;
static volatile uint16_t ring_tail;
static volatile uint32_t last_rx_ms;

static uint8_t pkt[PKT_BUF_LEN]; /* id, len_l, len_h, body... */
static uint16_t pkt_have;
static uint16_t pkt_need;        /* bytes after the header */
static uint32_t pkt_byte_ms;     /* last byte of the in-progress packet */

static enum {
	P_HDR,        /* hunting FF FF FD 00 */
	P_PACKET,     /* filling pkt[] */
	P_SKIP,       /* dropping an oversized packet */
	P_FSR_APPEND, /* fast sync read: consuming the response up to our slot */
	P_FSR_SKIP,   /* fast sync read: our block sent, consuming the rest */
} pstate;

static uint8_t hdr_match; /* matched prefix length, 0..4 */

/* Sync read: number of status packets to see before our turn (0 = inactive). */
static uint8_t sr_wait;
static uint32_t sr_armed_ms;

/* Fast sync read bookkeeping. */
static uint32_t fsr_bytes_before; /* response bytes preceding our block */
static uint32_t fsr_total;        /* response length on the wire */
static uint32_t fsr_consumed;
static uint16_t fsr_crc;
static uint32_t fsr_armed_ms;

/* Read address/length of the collective read in progress. */
static uint16_t coll_addr;
static uint8_t coll_len;

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
	crc ^= (uint16_t)byte << 8;
	for (int i = 0; i < 8; i++) {
		crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : crc << 1;
	}
	return crc;
}

static uint16_t crc16(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0;

	for (uint16_t i = 0; i < len; i++) {
		crc = crc16_update(crc, data[i]);
	}
	return crc;
}

/* Mirrors rustypot's add_stuffing: an extra 0xFD right after each FF FF FD. */
static uint16_t add_stuffing(const uint8_t *in, uint16_t in_len, uint8_t *out)
{
	uint16_t out_len = 0;
	uint8_t run = 0;

	for (uint16_t i = 0; i < in_len; i++) {
		uint8_t b = in[i];

		out[out_len++] = b;
		if (run < 2 && b == 0xff) {
			run++;
		} else if (run == 2 && b == 0xff) {
			run = 2;
		} else if (run == 2 && b == 0xfd) {
			out[out_len++] = 0xfd;
			run = 0;
		} else if (b == 0xff) {
			run = 1;
		} else {
			run = 0;
		}
	}
	return out_len;
}

static uint16_t remove_stuffing(const uint8_t *in, uint16_t in_len, uint8_t *out)
{
	uint16_t out_len = 0;
	uint8_t run = 0;

	for (uint16_t i = 0; i < in_len; i++) {
		uint8_t b = in[i];

		if (run == 3) {
			run = 0;
			if (b == 0xfd) {
				continue;
			}
		}
		if (run < 2 && b == 0xff) {
			run++;
		} else if (run == 2 && b == 0xff) {
			run = 2;
		} else if (run == 2 && b == 0xfd) {
			run = 3;
		} else if (b == 0xff) {
			run = 1;
		} else {
			run = 0;
		}
		out[out_len++] = b;
	}
	return out_len;
}

static void tx_bytes(const uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, data[i]);
	}
}

static void tx_wait_complete(void)
{
	while (!uart_irq_tx_complete(uart_dev)) {
	}
}

static void send_status(uint8_t err, const uint8_t *params, uint16_t plen)
{
	static uint8_t buf[11 + STUFFED_LEN(MAX_READ_LEN)];
	static uint8_t stuffed[STUFFED_LEN(MAX_READ_LEN)];
	uint16_t splen = add_stuffing(params, plen, stuffed);
	uint16_t len = splen + 4; /* instruction + error + crc16 */
	uint16_t pos = 0;
	uint16_t crc;

	buf[pos++] = 0xff;
	buf[pos++] = 0xff;
	buf[pos++] = 0xfd;
	buf[pos++] = 0x00;
	buf[pos++] = dev_id;
	buf[pos++] = len & 0xff;
	buf[pos++] = len >> 8;
	buf[pos++] = INSTR_STATUS;
	buf[pos++] = err;
	memcpy(&buf[pos], stuffed, splen);
	pos += splen;
	crc = crc16(buf, pos);
	buf[pos++] = crc & 0xff;
	buf[pos++] = crc >> 8;
	tx_bytes(buf, pos);
}

/* Sensor shadow laid out as the control table bytes 124..146. */
static void sensor_shadow(uint8_t *s)
{
	struct imu_state im;
	uint16_t voltage;
	int32_t temp_c;

	imu_get_state(&im);
	for (int i = 0; i < 3; i++) {
		s[2 * i] = im.gyro[i] & 0xff;
		s[2 * i + 1] = im.gyro[i] >> 8;
	}
	memcpy(&s[6], im.quat, 6);
	for (int i = 0; i < 3; i++) {
		s[12 + 2 * i] = im.accel[i] & 0xff;
		s[13 + 2 * i] = im.accel[i] >> 8;
	}
	s[18] = im.sample_counter;
	s[19] = (im.online ? 0x01 : 0) | (im.quat_valid ? 0x02 : 0) |
		(im.fifo_ovr ? 0x04 : 0);
	voltage = battery_mv / 100;
	s[20] = voltage & 0xff;
	s[21] = voltage >> 8;
	temp_c = 25 + ((int32_t)im.temp_raw >> 8);
	s[22] = temp_c < 0 ? 0 : (temp_c > 200 ? 200 : temp_c);
}

static int table_read_byte(uint16_t addr, uint8_t *out)
{
	if (addr < 10) {
		static const uint8_t identity[10] = {
			DXL_MODEL_NUMBER & 0xff, DXL_MODEL_NUMBER >> 8,
			0, 0, 0, 0,
			DXL_FW_VERSION,
		};

		if (addr == REG_ID_ADDR) {
			*out = dev_id;
		} else if (addr == 8) {
			*out = DXL_BAUD_CODE;
		} else if (addr == 9) {
			*out = 0;
		} else {
			*out = identity[addr];
		}
		return 0;
	}
	if (addr >= REG_SENSOR_BASE && addr < REG_SENSOR_BASE + SENSOR_MAP_LEN) {
		uint8_t s[SENSOR_MAP_LEN];

		sensor_shadow(s);
		*out = s[addr - REG_SENSOR_BASE];
		return 0;
	}
	return -1;
}

static bool table_write_byte(uint16_t addr, uint8_t value)
{
	switch (addr) {
	case REG_ID_ADDR:
		if (value < 0xfd) {
			dev_id = value;
		}
		return true;
	case 9:
		return true; /* return delay accepted, always 0 */
	default:
		return false;
	}
}

static void handle_read(const uint8_t *params, uint16_t plen)
{
	static uint8_t data[MAX_READ_LEN];
	uint16_t addr, len;

	if (plen != 4) {
		send_status(ERR_INSTRUCTION, NULL, 0);
		return;
	}
	addr = params[0] | ((uint16_t)params[1] << 8);
	len = params[2] | ((uint16_t)params[3] << 8);
	if (len == 0 || len > MAX_READ_LEN) {
		send_status(ERR_INSTRUCTION, NULL, 0);
		return;
	}
	for (uint16_t i = 0; i < len; i++) {
		if (table_read_byte(addr + i, &data[i]) < 0) {
			send_status(ERR_ACCESS, NULL, 0);
			return;
		}
	}
	send_status(0, data, len);
}

static void handle_write(const uint8_t *params, uint16_t plen, bool addressed)
{
	uint16_t addr;

	if (plen < 3) {
		if (addressed) {
			send_status(ERR_INSTRUCTION, NULL, 0);
		}
		return;
	}
	addr = params[0] | ((uint16_t)params[1] << 8);
	for (uint16_t i = 2; i < plen; i++) {
		if (!table_write_byte(addr + i - 2, params[i])) {
			if (addressed) {
				send_status(ERR_ACCESS, NULL, 0);
			}
			return;
		}
	}
	if (addressed) {
		send_status(0, NULL, 0);
	}
}

/* Our index in a sync read id list, or -1. */
static int id_position(const uint8_t *ids, uint16_t count)
{
	for (uint16_t i = 0; i < count; i++) {
		if (ids[i] == dev_id) {
			return i;
		}
	}
	return -1;
}

static void send_block_status(void)
{
	uint8_t data[MAX_READ_LEN];

	for (uint8_t i = 0; i < coll_len; i++) {
		if (table_read_byte(coll_addr + i, &data[i]) < 0) {
			data[i] = 0;
		}
	}
	send_status(0, data, coll_len);
}

static void handle_sync_read(const uint8_t *params, uint16_t plen, bool fast)
{
	uint16_t addr, len;
	int pos;

	if (plen < 4) {
		return;
	}
	addr = params[0] | ((uint16_t)params[1] << 8);
	len = params[2] | ((uint16_t)params[3] << 8);
	if (len == 0 || len > MAX_READ_LEN) {
		return;
	}
	pos = id_position(&params[4], plen - 4);
	if (pos < 0) {
		return;
	}
	coll_addr = addr;
	coll_len = len;

	if (!fast) {
		if (pos == 0) {
			send_block_status();
		} else {
			/* Later slot: answer once that many status packets passed. */
			sr_wait = pos;
			sr_armed_ms = k_uptime_get_32();
		}
		return;
	}

	uint32_t n = plen - 4;

	if (pos == 0) {
		/* First device opens the collective status packet. */
		uint8_t head[10];
		uint16_t plen_field = 1 + n * (len + 4);
		uint16_t crc;
		uint8_t data[MAX_READ_LEN];

		head[0] = 0xff;
		head[1] = 0xff;
		head[2] = 0xfd;
		head[3] = 0x00;
		head[4] = DXL_ID_BROADCAST;
		head[5] = plen_field & 0xff;
		head[6] = plen_field >> 8;
		head[7] = INSTR_STATUS;
		head[8] = 0;        /* error */
		head[9] = dev_id;
		tx_bytes(head, sizeof(head));
		for (uint8_t i = 0; i < len; i++) {
			if (table_read_byte(addr + i, &data[i]) < 0) {
				data[i] = 0;
			}
		}
		tx_bytes(data, len);
		crc = crc16(head, sizeof(head));
		for (uint8_t i = 0; i < len; i++) {
			crc = crc16_update(crc, data[i]);
		}
		uint8_t tail[2] = {crc & 0xff, crc >> 8};

		tx_bytes(tail, sizeof(tail));
	} else {
		/* Append after the earlier devices: consume their bytes while
		 * feeding the running CRC, then transmit our block.
		 */
		fsr_crc = 0;
		fsr_consumed = 0;
		fsr_bytes_before = 8 + (uint32_t)pos * (len + 4);
		fsr_total = 8 + n * (len + 4);
		fsr_armed_ms = k_uptime_get_32();
		pstate = P_FSR_APPEND;
	}
}

static void handle_packet(void)
{
	uint8_t id = pkt[0];
	uint16_t len = pkt[1] | ((uint16_t)pkt[2] << 8);
	const uint8_t *body = &pkt[3];
	uint8_t instr = body[0];

	if (instr == INSTR_STATUS) {
		/* Another device answering a sync read; count it down if we are
		 * waiting for our slot.
		 */
		if (sr_wait > 0 && --sr_wait == 0) {
			send_block_status();
		}
		return;
	}

	if (id != dev_id && id != DXL_ID_BROADCAST) {
		return;
	}
	bool addressed = (id == dev_id);

	/* Destuffed params: body[1 .. len-2); the last two body bytes are the CRC. */
	static uint8_t params[PKT_BUF_LEN];
	uint16_t plen = 0;

	if (len > 3) {
		plen = remove_stuffing(&body[1], len - 3, params);
	}

	switch (instr) {
	case INSTR_PING:
		if (addressed) {
			uint8_t p[3] = {DXL_MODEL_NUMBER & 0xff, DXL_MODEL_NUMBER >> 8,
					DXL_FW_VERSION};

			send_status(0, p, sizeof(p));
		}
		break;
	case INSTR_READ:
		if (addressed) {
			handle_read(params, plen);
		}
		break;
	case INSTR_WRITE:
		handle_write(params, plen, addressed);
		break;
	case INSTR_REBOOT: {
		if (addressed) {
			send_status(0, NULL, 0);
		}
		tx_wait_complete();
		NVIC_SystemReset();
		break;
	}
	case INSTR_SYNC_READ:
		if (id == DXL_ID_BROADCAST) {
			handle_sync_read(params, plen, false);
		}
		break;
	case INSTR_FAST_SYNC_READ:
		if (id == DXL_ID_BROADCAST) {
			handle_sync_read(params, plen, true);
		}
		break;
	default:
		if (addressed) {
			send_status(ERR_INSTRUCTION, NULL, 0);
		}
		break;
	}
}

static void fsr_append_byte(uint8_t b)
{
	fsr_crc = crc16_update(fsr_crc, b);
	if (++fsr_consumed != fsr_bytes_before) {
		return;
	}

	/* Our slot: error, id, data, CRC of the whole packet so far. */
	uint8_t head[2] = {0, dev_id};
	uint8_t data[MAX_READ_LEN];

	fsr_crc = crc16_update(fsr_crc, head[0]);
	fsr_crc = crc16_update(fsr_crc, head[1]);
	for (uint8_t i = 0; i < coll_len; i++) {
		if (table_read_byte(coll_addr + i, &data[i]) < 0) {
			data[i] = 0;
		}
		fsr_crc = crc16_update(fsr_crc, data[i]);
	}
	tx_bytes(head, sizeof(head));
	tx_bytes(data, coll_len);
	uint8_t tail[2] = {fsr_crc & 0xff, fsr_crc >> 8};

	tx_bytes(tail, sizeof(tail));
	fsr_consumed += coll_len + 4; /* our own block is not heard back */
	pstate = P_FSR_SKIP;
}

static void feed_byte(uint8_t b)
{
	switch (pstate) {
	case P_HDR: {
		static const uint8_t hdr[4] = {0xff, 0xff, 0xfd, 0x00};

		if (b == hdr[hdr_match]) {
			if (++hdr_match == 4) {
				hdr_match = 0;
				pkt_have = 0;
				pkt_need = 3;
				pkt_byte_ms = k_uptime_get_32();
				pstate = P_PACKET;
			}
		} else {
			hdr_match = (b == 0xff) ? (hdr_match >= 2 ? 2 : 1) : 0;
		}
		break;
	}
	case P_PACKET:
		pkt_byte_ms = k_uptime_get_32();
		pkt[pkt_have++] = b;
		if (pkt_have == 3) {
			uint16_t len = pkt[1] | ((uint16_t)pkt[2] << 8);

			if (len < 3 || len + 3 > PKT_BUF_LEN) {
				pstate = P_SKIP;
				pkt_need = len;
				break;
			}
			pkt_need = 3 + len;
		}
		if (pkt_have == pkt_need) {
			uint16_t len = pkt[1] | ((uint16_t)pkt[2] << 8);
			static const uint8_t hdr[4] = {0xff, 0xff, 0xfd, 0x00};
			uint16_t crc = crc16(hdr, 4);

			crc = crc16_update(crc, pkt[0]);
			crc = crc16_update(crc, pkt[1]);
			crc = crc16_update(crc, pkt[2]);
			for (uint16_t i = 0; i < len - 2; i++) {
				crc = crc16_update(crc, pkt[3 + i]);
			}
			if ((crc & 0xff) == pkt[pkt_need - 2] && (crc >> 8) == pkt[pkt_need - 1]) {
				handle_packet();
			}
			if (pstate == P_PACKET) {
				pstate = P_HDR;
			}
		}
		break;
	case P_SKIP:
		pkt_byte_ms = k_uptime_get_32();
		if (--pkt_need == 0) {
			pstate = P_HDR;
		}
		break;
	case P_FSR_APPEND:
		fsr_append_byte(b);
		break;
	case P_FSR_SKIP:
		if (++fsr_consumed >= fsr_total) {
			pstate = P_HDR;
		}
		break;
	}
}

static void uart_cb(const struct device *dev, void *user_data)
{
	uint8_t buf[32];

	for (;;) {
		(void)uart_err_check(dev);
		if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
			break;
		}
		int n = uart_fifo_read(dev, buf, sizeof(buf));

		last_rx_ms = k_uptime_get_32();
		for (int i = 0; i < n; i++) {
			uint16_t next = (ring_head + 1) % RX_RING_LEN;

			if (next != ring_tail) {
				rx_ring[ring_head] = buf[i];
				ring_head = next;
			}
		}
	}
}

void dxl_init(const struct device *uart)
{
	/* USART RFCS register: bit 8 enables the hardware RX FIFO */
#define USART_CTL0_UEN BIT(0)
#define USART_RFCS_OFF 0xd0U
#define USART_RFCS_RFEN BIT(8)
	volatile uint32_t *ctl0 = (volatile uint32_t *)DT_REG_ADDR(DT_NODELABEL(usart0));
	volatile uint32_t *rfcs = ctl0 + USART_RFCS_OFF / sizeof(uint32_t);

	uart_dev = uart;
	/* 1M baud needs the RX FIFO: the byte ISR cannot keep up otherwise.
	 * RFEN is only writable while the USART is disabled. */
	*ctl0 &= ~USART_CTL0_UEN;
	*rfcs |= USART_RFCS_RFEN;
	*ctl0 |= USART_CTL0_UEN;
	uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
	uart_irq_rx_enable(uart_dev);
}

void dxl_set_battery_mv(uint32_t mv)
{
	battery_mv = mv;
}

void dxl_process(void)
{
	uint8_t b;

	while (ring_tail != ring_head) {
		b = rx_ring[ring_tail];
		ring_tail = (ring_tail + 1) % RX_RING_LEN;
		feed_byte(b);
	}

	uint32_t now = k_uptime_get_32();

	if (sr_wait > 0 && now - sr_armed_ms > MONITOR_TIMEOUT_MS) {
		sr_wait = 0;
	}
	if ((pstate == P_FSR_APPEND || pstate == P_FSR_SKIP) &&
	    now - fsr_armed_ms > MONITOR_TIMEOUT_MS) {
		pstate = P_HDR;
	}
	if ((pstate == P_PACKET || pstate == P_SKIP) && now - pkt_byte_ms > 20) {
		pstate = P_HDR;
	}
}

bool dxl_bus_active(void)
{
	if (ring_tail != ring_head || pstate != P_HDR || sr_wait > 0) {
		return true;
	}
	return (k_uptime_get_32() - last_rx_ms) < 2;
}
