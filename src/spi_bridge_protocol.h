#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define SPI_BRIDGE_FRAME_LEN 48u
#define SPI_BRIDGE_ACK_LEN 8u

#define SPI_BRIDGE_BOOT_REPORT_LEN 8u
#define SPI_BRIDGE_BOOT_REPORT_MODS_INDEX 0u
#define SPI_BRIDGE_BOOT_REPORT_RESERVED_INDEX 1u
#define SPI_BRIDGE_BOOT_REPORT_KEYS_INDEX 2u
#define SPI_BRIDGE_BOOT_REPORT_KEYS_LEN 6u

#define SPI_BRIDGE_NKRO_BITS_LEN 30u
#define SPI_BRIDGE_NKRO_REPORT_LEN (1u + SPI_BRIDGE_NKRO_BITS_LEN)

#define SPI_BRIDGE_FRAME_HDR0 0xB4u
#define SPI_BRIDGE_FRAME_HDR1 0x4Bu
#define SPI_BRIDGE_FRAME_TYPE_INDEX 2u
#define SPI_BRIDGE_FRAME_VERSION_INDEX 3u
#define SPI_BRIDGE_FRAME_SEQ_INDEX 4u
#define SPI_BRIDGE_FRAME_PAYLOAD_LEN_INDEX 5u
#define SPI_BRIDGE_FRAME_PAYLOAD_INDEX 8u
#define SPI_BRIDGE_FRAME_CRC_INDEX 46u
#define SPI_BRIDGE_FRAME_TAIL_INDEX 47u
#define SPI_BRIDGE_FRAME_TAIL 0x96u
#define SPI_BRIDGE_FRAME_VERSION 1u
#define SPI_BRIDGE_FRAME_PAYLOAD_MAX \
	(SPI_BRIDGE_FRAME_CRC_INDEX - SPI_BRIDGE_FRAME_PAYLOAD_INDEX)

#define SPI_BRIDGE_REPORT_KEYBOARD_BOOT 0x01u
#define SPI_BRIDGE_REPORT_KEYBOARD_NKRO 0x02u
#define SPI_BRIDGE_REPORT_CONSUMER 0x03u
#define SPI_BRIDGE_REPORT_SYSTEM 0x04u

#define SPI_BRIDGE_PROFILE_COUNT 3u

#define SPI_BRIDGE_CTRL_PROFILE_1 0x01u
#define SPI_BRIDGE_CTRL_PROFILE_2 0x02u
#define SPI_BRIDGE_CTRL_PROFILE_3 0x03u
#define SPI_BRIDGE_CTRL_PROFILE_LAST SPI_BRIDGE_CTRL_PROFILE_3
#define SPI_BRIDGE_CTRL_USER_ACTIVITY 0xF1u
#define SPI_BRIDGE_CTRL_DISCONNECT 0xF2u
#define SPI_BRIDGE_CTRL_CLEAR_ACTIVE 0xF3u
#define SPI_BRIDGE_CTRL_CLEAR_ALL 0xF4u
#define SPI_BRIDGE_CTRL_USB_ABSENT 0xF5u
#define SPI_BRIDGE_CTRL_USB_PRESENT 0xF6u
#define SPI_BRIDGE_CTRL_MODE_WIRED 0xF7u
#define SPI_BRIDGE_CTRL_MODE_WIRELESS 0xF8u
#define SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_0 0xE8u
#define SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_1 0xE9u
#define SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_COUNT 0xEAu

#define SPI_BRIDGE_CTRL_HDR0 0xC3u
#define SPI_BRIDGE_CTRL_HDR1 0x3Cu
#define SPI_BRIDGE_CTRL_CMD_INDEX 2u
#define SPI_BRIDGE_CTRL_SEQ_INDEX 3u
#define SPI_BRIDGE_CTRL_CMD_INV_INDEX 4u
#define SPI_BRIDGE_CTRL_SEQ_INV_INDEX 5u
#define SPI_BRIDGE_CTRL_CRC_INDEX 6u
#define SPI_BRIDGE_CTRL_TAIL_INDEX 7u
#define SPI_BRIDGE_CTRL_TAIL 0x69u

#define SPI_BRIDGE_HOST_LED_NUM_LOCK 0x01u
#define SPI_BRIDGE_HOST_LED_CAPS_LOCK 0x02u
#define SPI_BRIDGE_HOST_LED_SCROLL_LOCK 0x04u
#define SPI_BRIDGE_HOST_LED_MASK \
	(SPI_BRIDGE_HOST_LED_NUM_LOCK | SPI_BRIDGE_HOST_LED_CAPS_LOCK | SPI_BRIDGE_HOST_LED_SCROLL_LOCK)

#define SPI_BRIDGE_ACK_LED_PROFILE_SHIFT 3u
#define SPI_BRIDGE_ACK_LED_PROFILE_MASK (0x03u << SPI_BRIDGE_ACK_LED_PROFILE_SHIFT)
#define SPI_BRIDGE_ACK_LED_BLE_IND_ACTIVE 0x20u
#define SPI_BRIDGE_ACK_LED_BLE_IND_ON 0x40u
#define SPI_BRIDGE_ACK_LED_VALID_BITS_MASK \
	(SPI_BRIDGE_HOST_LED_MASK | SPI_BRIDGE_ACK_LED_PROFILE_MASK | \
	 SPI_BRIDGE_ACK_LED_BLE_IND_ACTIVE | SPI_BRIDGE_ACK_LED_BLE_IND_ON)

#define SPI_BRIDGE_ACK_HDR0 0xA5u
#define SPI_BRIDGE_ACK_HDR1 0x5Au
#define SPI_BRIDGE_ACK_COUNT_LO_INDEX 2u
#define SPI_BRIDGE_ACK_COUNT_HI_INDEX 3u
#define SPI_BRIDGE_ACK_LED_INDEX 4u
#define SPI_BRIDGE_ACK_FLAGS_INDEX 5u
#define SPI_BRIDGE_ACK_CTRL_SEQ_INDEX 6u
#define SPI_BRIDGE_ACK_CTRL_CMD_INDEX 7u
#define SPI_BRIDGE_ACK_REPORT_TOKEN 0x00u
#define SPI_BRIDGE_ACK_FLAG_LED_VALID 0x01u
#define SPI_BRIDGE_ACK_FLAG_LINK_ON 0x02u
#define SPI_BRIDGE_ACK_FLAG_PAIRING_ON 0x04u
#define SPI_BRIDGE_ACK_FLAG_PROFILE_VALID 0x08u
#define SPI_BRIDGE_ACK_FLAG_MODE_WIRELESS 0x10u
#define SPI_BRIDGE_ACK_FLAG_MAGIC_MASK 0xE0u
#define SPI_BRIDGE_ACK_FLAG_MAGIC_VALUE 0xA0u

struct spi_bridge_ack_state {
	uint16_t frame_counter;
	uint8_t host_led_bits;
	uint8_t active_profile;
	uint8_t flags;
	uint8_t last_ctrl_seq;
	uint8_t last_ctrl_cmd;
	bool led_valid;
	bool ble_ind_active;
	bool ble_ind_on;
};

static inline uint8_t spi_bridge_crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0u;

	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8u; bit++) {
			crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) :
				(uint8_t)(crc << 1);
		}
	}

	return crc;
}

static inline bool spi_bridge_report_type_valid(uint8_t type)
{
	return type >= SPI_BRIDGE_REPORT_KEYBOARD_BOOT && type <= SPI_BRIDGE_REPORT_SYSTEM;
}

static inline bool spi_bridge_report_payload_len_valid(uint8_t type, uint8_t len)
{
	switch (type) {
	case SPI_BRIDGE_REPORT_KEYBOARD_BOOT:
		return len == SPI_BRIDGE_BOOT_REPORT_LEN;
	case SPI_BRIDGE_REPORT_KEYBOARD_NKRO:
		return len == SPI_BRIDGE_NKRO_REPORT_LEN;
	case SPI_BRIDGE_REPORT_CONSUMER:
	case SPI_BRIDGE_REPORT_SYSTEM:
		return len == sizeof(uint16_t);
	default:
		return false;
	}
}

static inline bool spi_bridge_report_frame_write(uint8_t frame[SPI_BRIDGE_FRAME_LEN],
						  uint8_t type, uint8_t sequence,
						  const uint8_t *payload, uint8_t payload_len)
{
	if (!spi_bridge_report_payload_len_valid(type, payload_len) ||
	    payload_len > SPI_BRIDGE_FRAME_PAYLOAD_MAX || !payload) {
		return false;
	}

	memset(frame, 0, SPI_BRIDGE_FRAME_LEN);
	frame[0] = SPI_BRIDGE_FRAME_HDR0;
	frame[1] = SPI_BRIDGE_FRAME_HDR1;
	frame[SPI_BRIDGE_FRAME_TYPE_INDEX] = type;
	frame[SPI_BRIDGE_FRAME_VERSION_INDEX] = SPI_BRIDGE_FRAME_VERSION;
	frame[SPI_BRIDGE_FRAME_SEQ_INDEX] = sequence;
	frame[SPI_BRIDGE_FRAME_PAYLOAD_LEN_INDEX] = payload_len;
	memcpy(&frame[SPI_BRIDGE_FRAME_PAYLOAD_INDEX], payload, payload_len);
	frame[SPI_BRIDGE_FRAME_CRC_INDEX] =
		spi_bridge_crc8(frame, SPI_BRIDGE_FRAME_CRC_INDEX);
	frame[SPI_BRIDGE_FRAME_TAIL_INDEX] = SPI_BRIDGE_FRAME_TAIL;
	return true;
}

static inline bool spi_bridge_report_frame_read(const uint8_t frame[SPI_BRIDGE_FRAME_LEN],
						 uint8_t *type, uint8_t *sequence,
						 const uint8_t **payload, uint8_t *payload_len)
{
	if (!frame || frame[0] != SPI_BRIDGE_FRAME_HDR0 ||
	    frame[1] != SPI_BRIDGE_FRAME_HDR1 ||
	    frame[SPI_BRIDGE_FRAME_VERSION_INDEX] != SPI_BRIDGE_FRAME_VERSION ||
	    frame[SPI_BRIDGE_FRAME_TAIL_INDEX] != SPI_BRIDGE_FRAME_TAIL ||
	    frame[SPI_BRIDGE_FRAME_CRC_INDEX] !=
		spi_bridge_crc8(frame, SPI_BRIDGE_FRAME_CRC_INDEX)) {
		return false;
	}

	uint8_t report_type = frame[SPI_BRIDGE_FRAME_TYPE_INDEX];
	uint8_t report_len = frame[SPI_BRIDGE_FRAME_PAYLOAD_LEN_INDEX];

	if (!spi_bridge_report_type_valid(report_type) ||
	    !spi_bridge_report_payload_len_valid(report_type, report_len) ||
	    report_len > SPI_BRIDGE_FRAME_PAYLOAD_MAX) {
		return false;
	}

	if (type) {
		*type = report_type;
	}
	if (sequence) {
		*sequence = frame[SPI_BRIDGE_FRAME_SEQ_INDEX];
	}
	if (payload) {
		*payload = &frame[SPI_BRIDGE_FRAME_PAYLOAD_INDEX];
	}
	if (payload_len) {
		*payload_len = report_len;
	}
	return true;
}

static inline bool spi_bridge_control_cmd_valid(uint8_t cmd)
{
	return (cmd >= SPI_BRIDGE_CTRL_PROFILE_1 && cmd <= SPI_BRIDGE_CTRL_PROFILE_LAST) ||
	       cmd == SPI_BRIDGE_CTRL_USER_ACTIVITY || cmd == SPI_BRIDGE_CTRL_DISCONNECT ||
	       cmd == SPI_BRIDGE_CTRL_CLEAR_ACTIVE ||
	       cmd == SPI_BRIDGE_CTRL_CLEAR_ALL || cmd == SPI_BRIDGE_CTRL_USB_ABSENT ||
	       cmd == SPI_BRIDGE_CTRL_USB_PRESENT || cmd == SPI_BRIDGE_CTRL_MODE_WIRED ||
	       cmd == SPI_BRIDGE_CTRL_MODE_WIRELESS ||
	       (cmd >= SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_0 &&
		cmd <= SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_COUNT);
}

static inline void spi_bridge_control_frame_write(uint8_t frame[SPI_BRIDGE_FRAME_LEN],
						   uint8_t cmd, uint8_t sequence)
{
	memset(frame, 0, SPI_BRIDGE_FRAME_LEN);
	frame[0] = SPI_BRIDGE_CTRL_HDR0;
	frame[1] = SPI_BRIDGE_CTRL_HDR1;
	frame[SPI_BRIDGE_CTRL_CMD_INDEX] = cmd;
	frame[SPI_BRIDGE_CTRL_SEQ_INDEX] = sequence;
	frame[SPI_BRIDGE_CTRL_CMD_INV_INDEX] = (uint8_t)~cmd;
	frame[SPI_BRIDGE_CTRL_SEQ_INV_INDEX] = (uint8_t)~sequence;
	frame[SPI_BRIDGE_CTRL_CRC_INDEX] = spi_bridge_crc8(frame, SPI_BRIDGE_CTRL_CRC_INDEX);
	frame[SPI_BRIDGE_CTRL_TAIL_INDEX] = SPI_BRIDGE_CTRL_TAIL;
}

static inline void spi_bridge_ack_frame_write(uint8_t frame[SPI_BRIDGE_FRAME_LEN],
						       uint16_t frame_counter,
						       uint8_t host_led_bits, uint8_t flags,
						       uint8_t last_ctrl_seq, uint8_t last_ctrl_cmd)
{
	memset(frame, 0, SPI_BRIDGE_FRAME_LEN);
	frame[0] = SPI_BRIDGE_ACK_HDR0;
	frame[1] = SPI_BRIDGE_ACK_HDR1;
	frame[SPI_BRIDGE_ACK_COUNT_LO_INDEX] = (uint8_t)(frame_counter & 0xFFu);
	frame[SPI_BRIDGE_ACK_COUNT_HI_INDEX] = (uint8_t)(frame_counter >> 8);
	frame[SPI_BRIDGE_ACK_LED_INDEX] = host_led_bits & SPI_BRIDGE_ACK_LED_VALID_BITS_MASK;
	frame[SPI_BRIDGE_ACK_FLAGS_INDEX] =
		(flags & ~SPI_BRIDGE_ACK_FLAG_MAGIC_MASK) | SPI_BRIDGE_ACK_FLAG_MAGIC_VALUE;
	frame[SPI_BRIDGE_ACK_CTRL_SEQ_INDEX] = last_ctrl_seq;
	frame[SPI_BRIDGE_ACK_CTRL_CMD_INDEX] = last_ctrl_cmd;
}

static inline bool spi_bridge_ack_extract(const uint8_t *rx_buf, uint8_t len,
						   struct spi_bridge_ack_state *ack)
{
	if (!rx_buf || !ack || len < SPI_BRIDGE_ACK_LEN ||
	    rx_buf[0] != SPI_BRIDGE_ACK_HDR0 || rx_buf[1] != SPI_BRIDGE_ACK_HDR1) {
		return false;
	}

	uint8_t flags = rx_buf[SPI_BRIDGE_ACK_FLAGS_INDEX];
	if ((flags & SPI_BRIDGE_ACK_FLAG_MAGIC_MASK) != SPI_BRIDGE_ACK_FLAG_MAGIC_VALUE) {
		return false;
	}

	uint8_t raw_led = rx_buf[SPI_BRIDGE_ACK_LED_INDEX];
	ack->frame_counter = (uint16_t)rx_buf[SPI_BRIDGE_ACK_COUNT_LO_INDEX] |
		((uint16_t)rx_buf[SPI_BRIDGE_ACK_COUNT_HI_INDEX] << 8);
	ack->host_led_bits = raw_led & SPI_BRIDGE_HOST_LED_MASK;
	ack->active_profile =
		(uint8_t)((raw_led & SPI_BRIDGE_ACK_LED_PROFILE_MASK) >> SPI_BRIDGE_ACK_LED_PROFILE_SHIFT);
	ack->ble_ind_active = (raw_led & SPI_BRIDGE_ACK_LED_BLE_IND_ACTIVE) != 0u;
	ack->ble_ind_on = (raw_led & SPI_BRIDGE_ACK_LED_BLE_IND_ON) != 0u;
	ack->flags = flags;
	ack->led_valid = (flags & SPI_BRIDGE_ACK_FLAG_LED_VALID) != 0u;
	ack->last_ctrl_seq = rx_buf[SPI_BRIDGE_ACK_CTRL_SEQ_INDEX];
	ack->last_ctrl_cmd = rx_buf[SPI_BRIDGE_ACK_CTRL_CMD_INDEX];
	return true;
}

static inline bool spi_bridge_frame_control_cmd(const uint8_t frame[SPI_BRIDGE_FRAME_LEN],
						 uint8_t *cmd, uint8_t *sequence)
{
	if (!frame || frame[0] != SPI_BRIDGE_CTRL_HDR0 || frame[1] != SPI_BRIDGE_CTRL_HDR1 ||
	    frame[SPI_BRIDGE_CTRL_TAIL_INDEX] != SPI_BRIDGE_CTRL_TAIL ||
	    frame[SPI_BRIDGE_CTRL_CRC_INDEX] !=
		spi_bridge_crc8(frame, SPI_BRIDGE_CTRL_CRC_INDEX)) {
		return false;
	}

	uint8_t control = frame[SPI_BRIDGE_CTRL_CMD_INDEX];
	uint8_t control_sequence = frame[SPI_BRIDGE_CTRL_SEQ_INDEX];

	if (!spi_bridge_control_cmd_valid(control) ||
	    frame[SPI_BRIDGE_CTRL_CMD_INV_INDEX] != (uint8_t)~control ||
	    frame[SPI_BRIDGE_CTRL_SEQ_INV_INDEX] != (uint8_t)~control_sequence) {
		return false;
	}

	if (cmd) {
		*cmd = control;
	}
	if (sequence) {
		*sequence = control_sequence;
	}
	return true;
}
