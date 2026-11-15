/*
 * SPI (slave) -> BLE HIDS Keyboard bridge
 *
 * Expects a fixed-size, versioned input report or validated bridge control
 * frame from the SPI master.
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "power_manager.h"
#include "watchdog_manager.h"
#include "spi_bridge_protocol.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>
#include <bluetooth/services/hids.h>

LOG_MODULE_REGISTER(spi_hids_bridge, LOG_LEVEL_INF);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION 0x0101

#define INPUT_REPORT_KEYBOARD_LEN SPI_BRIDGE_NKRO_REPORT_LEN
#define INPUT_REPORT_CONSUMER_LEN 2
#define INPUT_REPORT_SYSTEM_LEN   2
#define OUTPUT_REPORT_MAX_LEN     1
#define INPUT_REP_KEYBOARD_REF_ID 1
#define INPUT_REP_CONSUMER_REF_ID 2
#define INPUT_REP_SYSTEM_REF_ID   3
#define OUTPUT_REP_KEYS_REF_ID    INPUT_REP_KEYBOARD_REF_ID
#define INPUT_REP_KEYBOARD_INDEX  0
#define INPUT_REP_CONSUMER_INDEX  1
#define INPUT_REP_SYSTEM_INDEX    2
#define OUTPUT_REPORT_NUM_LOCK_MASK    SPI_BRIDGE_HOST_LED_NUM_LOCK
#define OUTPUT_REPORT_CAPS_LOCK_MASK   SPI_BRIDGE_HOST_LED_CAPS_LOCK
#define OUTPUT_REPORT_SCROLL_LOCK_MASK SPI_BRIDGE_HOST_LED_SCROLL_LOCK

#define SPI_NODE DT_NODELABEL(spi1)
#define PROFILE_COUNT SPI_BRIDGE_PROFILE_COUNT
#define PROFILE_VERSION 1
#define PROFILE_SETTINGS_KEY "spi_hids/profiles"

#define SPI_PAIRING_TIMEOUT_MS 120000
#define SPI_PROFILE_SWITCH_LED_HOLD_MS 2000
#define SPI_CONNECTED_LED_HOLD_MS 3000
#define SPI_ADV_REFRESH_DELAY_MS 20
#define SPI_PROFILE_SWITCH_ADV_DELAY_MS 30
#define SPI_ADV_START_RETRY_DELAY_MS 50U
#define SPI_ADV_START_RETRY_LIMIT 10U
#define SPI_CONNECTION_IDLE_MS 30000U
#define SPI_SLOW_ADV_DELAY_MS 120000U
#define SPI_AUTO_POWEROFF_DELAY_MS 600000U
#define SPI_LINK_IDLE_POWEROFF_MS 1800000U
#define SPI_CONN_ACTIVE_INTERVAL_MIN 6U
#define SPI_CONN_ACTIVE_INTERVAL_MAX 12U
/* Keep connection interval consistent (7.5-15 ms, latency 0, timeout 4000 ms)
 * across active and idle periods to avoid L2CAP/LL parameter renegotiation
 * stalls when resuming typing. */
#define SPI_CONN_IDLE_INTERVAL_MIN 6U
#define SPI_CONN_IDLE_INTERVAL_MAX 12U
#define SPI_CONN_IDLE_LATENCY 0U
#define SPI_CONN_SUPERVISION_TIMEOUT BT_GAP_MS_TO_CONN_TIMEOUT(4000U)
#define SPI_REPORT_QUEUE_LEN 128
#define SPI_CONTROL_QUEUE_LEN 8
#define SPI_REPORT_THREAD_STACK_SIZE 2048
#define SPI_REPORT_THREAD_PRIORITY 5
#define SPI_REPORT_RETRY_DELAY_MS 1
#define SPI_REPORT_RETRY_TIMEOUT_MS 3500U
#define HIDS_NOTIFY_COMPLETE_TIMEOUT_MS 3500U
#define HIDS_RECOVERY_DISCONNECT_TIMEOUT_MS 5000U
#define SPI_REPORT_LOCAL_EPOCH_INDEX (SPI_BRIDGE_FRAME_CRC_INDEX - sizeof(uint32_t))
#define SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX \
	(SPI_BRIDGE_FRAME_PAYLOAD_INDEX + SPI_BRIDGE_NKRO_REPORT_LEN)
#define SPI_REPORT_LOCAL_RELEASE_GENERATION_INDEX \
	(SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX + 1u)
#define SPI_REPORT_LOCAL_CONN_GENERATION_INDEX \
	(SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX + 2u)
#define SPI_LINK_TIMEOUT_MS 1000
#define SPI_QUEUE_DROP_LOG_INTERVAL_MS 1000
#define SPI_DEBUG_LOG_RX_REPORTS 0
#define SPI_DEBUG_FORCE_LED_BITS 0
#define SPI_DEBUG_FORCE_LED_VALUE OUTPUT_REPORT_CAPS_LOCK_MASK
#define SPI_DEBUG_FORCE_LED_BLINK 0
#define SPI_DEBUG_LED_SHOW_CAPS 1

#define SPI_DIAG_LOG_PERIOD_MS 5000U
#define SPI_DIAG_QUEUE_LOG_STEP 8U

#if !DT_NODE_HAS_STATUS(SPI_NODE, okay)
#error "SPI1 must be enabled as nordic,nrf-spis in board overlay"
#endif

BT_HIDS_DEF(hids_obj, OUTPUT_REPORT_MAX_LEN, INPUT_REPORT_KEYBOARD_LEN,
	    INPUT_REPORT_CONSUMER_LEN, INPUT_REPORT_SYSTEM_LEN);

struct conn_mode {
	struct bt_conn *conn;
	bool in_boot_mode;
	uint8_t profile_idx;
	bool profile_idx_valid;
	bt_addr_le_t peer_identity;
	bool peer_identity_valid;
};

struct profile_state {
	bool valid;
	bt_addr_le_t addr;
};

struct profile_store {
	uint8_t version;
	uint8_t active_profile;
	uint8_t valid_mask;
	bt_addr_le_t addrs[PROFILE_COUNT];
};

static struct conn_mode conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];
static struct k_spinlock conn_mode_lock;
static const struct gpio_dt_spec spi_rx_led =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, { 0 });
static struct profile_state profiles[PROFILE_COUNT];
static uint8_t active_profile;
static uint8_t profile_bt_id[PROFILE_COUNT] = {
	BT_ID_DEFAULT, BT_ID_DEFAULT, BT_ID_DEFAULT
};
static struct k_msgq spi_report_msgq;
static uint8_t spi_report_msgq_buffer[SPI_REPORT_QUEUE_LEN * SPI_BRIDGE_FRAME_LEN];
static struct k_spinlock latest_report_lock;
static uint8_t latest_spi_report[SPI_BRIDGE_FRAME_LEN];
static bool latest_spi_report_valid;
static uint32_t latest_spi_report_generation;

enum spi_release_slot {
	SPI_RELEASE_KEYBOARD = 0,
	SPI_RELEASE_CONSUMER,
	SPI_RELEASE_SYSTEM,
	SPI_RELEASE_SLOT_COUNT,
	SPI_RELEASE_SLOT_NONE = UINT8_MAX,
};

struct spi_release_slot_state {
	bool pending;
	bool queued;
	uint8_t generation;
	uint32_t epoch;
	uint8_t conn_generation;
};

static struct spi_release_slot_state spi_release_slots[SPI_RELEASE_SLOT_COUNT];

/* Keep the existing diagnostic names as aliases for the keyboard barrier. */
#define release_report_pending (spi_release_slots[SPI_RELEASE_KEYBOARD].pending)
#define release_report_queued  (spi_release_slots[SPI_RELEASE_KEYBOARD].queued)

static atomic_t spi_report_state_epoch = ATOMIC_INIT(1);
/* QMK owns transport selection after the SPI link is up.  Bluetooth remains
 * radio-silent until a validated MODE_WIRELESS command is received. */
static atomic_t wireless_mode_enabled = ATOMIC_INIT(0);
K_SEM_DEFINE(ble_disconnected_sem, 0, 1);
K_SEM_DEFINE(hids_tx_complete_sem, 0, 1);
K_MUTEX_DEFINE(spi_report_generation_mutex);
K_THREAD_STACK_DEFINE(spi_report_thread_stack, SPI_REPORT_THREAD_STACK_SIZE);
static struct k_thread spi_report_thread;

struct hids_tx_state {
	struct bt_conn *conn;
	uint32_t token;
	uint32_t conn_generation;
	uint32_t report_epoch;
	bool submit_in_progress;
	bool completion_early;
	bool in_flight;
	bool completion_received;
};

static struct hids_tx_state hids_tx_state;
static struct k_spinlock hids_tx_lock;
static uint32_t hids_tx_next_token;
static atomic_t hids_conn_generation = ATOMIC_INIT(1);
static bool pairing_mode;
static struct k_work_delayable pairing_mode_timeout_work;
static struct k_work_delayable profile_switch_adv_refresh_work;
static struct k_work_delayable connection_idle_work;
static struct k_work_delayable slow_advertising_work;
static struct k_work_delayable auto_poweroff_work;
static struct k_work_delayable spi_link_watchdog_work;
static struct k_work_delayable spi_link_idle_poweroff_work;
static struct k_work_delayable spi_diag_log_work;
static struct k_work mode_apply_work;
static struct k_work spi_control_work;
static struct k_work connection_active_work;
K_MSGQ_DEFINE(spi_control_msgq, sizeof(uint8_t), SPI_CONTROL_QUEUE_LEN, 1);
static atomic_t connection_idle_mode = ATOMIC_INIT(0);
static atomic_t spi_link_timed_out = ATOMIC_INIT(0);
static atomic_t usb_host_present = ATOMIC_INIT(0);
static atomic_t usb_presence_known = ATOMIC_INIT(0);
static atomic_t hostless_timer_valid = ATOMIC_INIT(0);
static uint32_t last_wireless_activity_ms;
static uint32_t last_hostless_ms;
static bool slow_advertising;
static uint32_t profile_switch_led_until_ms;
static uint32_t connected_profile_led_until_ms;
static uint32_t spi_queue_drop_log_at_ms;
static uint8_t advertising_start_retry_count;
static uint8_t last_ctrl_seq;
static uint8_t last_ctrl_cmd;
static bool last_ctrl_valid;
static uint8_t last_ack_seq;
static uint8_t last_ack_cmd;

BUILD_ASSERT(SPI_REPORT_LOCAL_EPOCH_INDEX >=
	     SPI_BRIDGE_FRAME_PAYLOAD_INDEX + SPI_BRIDGE_NKRO_REPORT_LEN);
BUILD_ASSERT(SPI_REPORT_LOCAL_CONN_GENERATION_INDEX < SPI_REPORT_LOCAL_EPOCH_INDEX);

struct spi_hids_diag_live_stats {
	uint8_t current_depth;
	uint8_t maximum_depth;
	uint32_t spi_rx;
	uint32_t hid_enqueued;
	uint32_t hid_dequeued;
	uint32_t hid_completed;
	uint16_t queue_full;
	uint16_t send_eagain;
	uint16_t send_enomem;
	uint16_t send_enotconn;
	uint16_t send_eacces;
	uint16_t send_ecanceled;
	uint16_t send_other;
	uint16_t notify_timeout;
	uint16_t send_stall;
	uint16_t disconnects;
	uint16_t all_up_received;
	uint16_t all_up_enqueued;
	uint16_t all_up_dequeued;
	uint16_t all_up_completed;
	uint16_t duplicate_nonzero;
	uint8_t local_rx_sequence;
	int16_t last_hid_error;
	uint16_t conn_interval;
	uint16_t conn_latency;
	uint16_t supervision_timeout;
	uint8_t last_received_report[SPI_BRIDGE_BOOT_REPORT_LEN];
	uint8_t last_dequeued_report[SPI_BRIDGE_BOOT_REPORT_LEN];
	uint8_t last_completed_report[SPI_BRIDGE_BOOT_REPORT_LEN];
	uint32_t send_attempted;
	uint32_t send_submitted;
	uint32_t send_completed;
	uint16_t retry_count;
	uint16_t inflight_timeout;
	uint16_t all_up_submitted;
	uint16_t resubmit_count;
	uint16_t stale_resubmit_blocked;
	uint16_t stale_completion;
	uint8_t last_submitted_report[SPI_BRIDGE_BOOT_REPORT_LEN];
	uint8_t last_resubmitted_report[SPI_BRIDGE_BOOT_REPORT_LEN];
	uint8_t last_error_report[SPI_BRIDGE_BOOT_REPORT_LEN];
	bool last_received_report_valid;
};

struct spi_hids_diag_state {
	struct spi_hids_diag_live_stats live_stats;
};

static struct spi_hids_diag_state spi_diag_state;
static struct k_spinlock spi_diag_lock;
static bool spi_diag_console_seen;

static void spi_diag_print(const char *reason);

static uint32_t last_spi_activity_ms;

static void spi_diag_u16_inc(uint16_t *value)
{
	if (*value != UINT16_MAX) {
		(*value)++;
	}
}

static bool spi_diag_boot_report_all_up(const uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN])
{
	if (report[SPI_BRIDGE_BOOT_REPORT_MODS_INDEX] != 0u) {
		return false;
	}

	for (uint8_t i = SPI_BRIDGE_BOOT_REPORT_KEYS_INDEX;
	     i < SPI_BRIDGE_BOOT_REPORT_KEYS_INDEX + SPI_BRIDGE_BOOT_REPORT_KEYS_LEN; i++) {
		if (report[i] != 0u) {
			return false;
		}
	}

	return true;
}

static void keyboard_nkro_to_boot(const uint8_t nkro[SPI_BRIDGE_NKRO_REPORT_LEN],
				  uint8_t boot[SPI_BRIDGE_BOOT_REPORT_LEN]);

static bool spi_diag_boot_report_copy(const uint8_t frame[SPI_BRIDGE_FRAME_LEN],
				      uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN],
				      uint8_t *sequence)
{
	const uint8_t *payload;
	uint8_t report_type;
	uint8_t payload_len;
	uint8_t report_sequence;

	if (!spi_bridge_report_frame_read(frame, &report_type, &report_sequence,
					  &payload, &payload_len)) {
		return false;
	}

	if (report_type == SPI_BRIDGE_REPORT_KEYBOARD_BOOT &&
	    payload_len == SPI_BRIDGE_BOOT_REPORT_LEN) {
		memcpy(report, payload, SPI_BRIDGE_BOOT_REPORT_LEN);
	} else if (report_type == SPI_BRIDGE_REPORT_KEYBOARD_NKRO &&
		   payload_len == SPI_BRIDGE_NKRO_REPORT_LEN) {
		keyboard_nkro_to_boot(payload, report);
	} else {
		return false;
	}

	if (sequence) {
		*sequence = report_sequence;
	}

	return true;
}

static struct spi_hids_diag_live_stats spi_diag_stats_copy(void)
{
	struct spi_hids_diag_live_stats stats;
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	stats = spi_diag_state.live_stats;
	k_spin_unlock(&spi_diag_lock, key);
	return stats;
}

static void spi_diag_note_queue_depth_locked(uint32_t depth)
{
	uint8_t depth_u8 = MIN(depth, UINT8_MAX);

	spi_diag_state.live_stats.current_depth = depth_u8;
	if (depth_u8 > spi_diag_state.live_stats.maximum_depth) {
		spi_diag_state.live_stats.maximum_depth = depth_u8;
	}
}

static void spi_diag_note_queue_depth(uint32_t depth)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_note_queue_depth_locked(depth);
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_spi_rx(void)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.spi_rx++;
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_received_report(const uint8_t frame[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN];
	uint8_t sequence;
	bool all_up;

	if (!spi_diag_boot_report_copy(frame, report, &sequence)) {
		return;
	}

	all_up = spi_diag_boot_report_all_up(report);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	if (!all_up && spi_diag_state.live_stats.last_received_report_valid &&
	    !spi_diag_boot_report_all_up(spi_diag_state.live_stats.last_received_report) &&
	    memcmp(spi_diag_state.live_stats.last_received_report, report, sizeof(report)) == 0) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.duplicate_nonzero);
	}
	memcpy(spi_diag_state.live_stats.last_received_report, report, sizeof(report));
	spi_diag_state.live_stats.last_received_report_valid = true;
	spi_diag_state.live_stats.local_rx_sequence = sequence;
	if (all_up) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.all_up_received);
	}
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_enqueued(const uint8_t frame[SPI_BRIDGE_FRAME_LEN], uint32_t depth)
{
	uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN];
	bool is_boot_report = spi_diag_boot_report_copy(frame, report, NULL);
	bool all_up = is_boot_report && spi_diag_boot_report_all_up(report);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.hid_enqueued++;
	spi_diag_note_queue_depth_locked(depth);
	if (all_up) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.all_up_enqueued);
	}
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_dequeued(const uint8_t frame[SPI_BRIDGE_FRAME_LEN], uint32_t depth)
{
	uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN];
	bool is_boot_report = spi_diag_boot_report_copy(frame, report, NULL);
	bool all_up = is_boot_report && spi_diag_boot_report_all_up(report);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.hid_dequeued++;
	spi_diag_note_queue_depth_locked(depth);
	if (is_boot_report) {
		memcpy(spi_diag_state.live_stats.last_dequeued_report, report, sizeof(report));
	}
	if (all_up) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.all_up_dequeued);
	}
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_completed(void)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.hid_completed++;
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_queue_full(uint32_t depth)
{
	bool log_now;
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.queue_full);
	spi_diag_note_queue_depth_locked(depth);
	log_now = spi_diag_state.live_stats.queue_full == 1u ||
		  (spi_diag_state.live_stats.queue_full % SPI_DIAG_QUEUE_LOG_STEP) == 0u;
	k_spin_unlock(&spi_diag_lock, key);
	if (log_now) {
		spi_diag_print("queue-full");
	}
}

static void spi_diag_note_hid_error(const uint8_t frame[SPI_BRIDGE_FRAME_LEN], int error)
{
	uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN];
	bool is_boot_report;
	bool log_now = false;
	uint16_t count = 0;

	if (!error) {
		return;
	}

	is_boot_report = spi_diag_boot_report_copy(frame, report, NULL);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.last_hid_error = (int16_t)error;
	if (is_boot_report) {
		memcpy(spi_diag_state.live_stats.last_error_report, report, sizeof(report));
	}
	if (error == -EAGAIN) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.send_eagain);
		count = spi_diag_state.live_stats.send_eagain;
	} else if (error == -ENOMEM) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.send_enomem);
		count = spi_diag_state.live_stats.send_enomem;
	} else if (error == -ENOTCONN) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.send_enotconn);
		log_now = true;
	} else if (error == -EACCES) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.send_eacces);
		log_now = true;
	} else if (error == -ECANCELED) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.send_ecanceled);
		log_now = true;
	} else {
		spi_diag_u16_inc(&spi_diag_state.live_stats.send_other);
		log_now = true;
	}
	log_now = log_now || count == 1u;
	k_spin_unlock(&spi_diag_lock, key);
	if (log_now) {
		spi_diag_print("send-error");
	}
}

static void spi_diag_note_notify_timeout(void)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.notify_timeout);
	k_spin_unlock(&spi_diag_lock, key);
	spi_diag_print("notify-timeout");
}

static void spi_diag_note_send_stall(void)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.send_stall);
	k_spin_unlock(&spi_diag_lock, key);
	spi_diag_print("send-stall");
}

static void spi_diag_note_disconnect(void)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.disconnects);
	k_spin_unlock(&spi_diag_lock, key);
	spi_diag_print("disconnect");
}

static void spi_diag_note_conn_params(uint16_t interval, uint16_t latency, uint16_t timeout)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.conn_interval = interval;
	spi_diag_state.live_stats.conn_latency = latency;
	spi_diag_state.live_stats.supervision_timeout = timeout;
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_send_attempted(const uint8_t frame[SPI_BRIDGE_FRAME_LEN])
{
	ARG_UNUSED(frame);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.send_attempted++;
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_send_submitted(const uint8_t frame[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN];
	bool is_boot_report = spi_diag_boot_report_copy(frame, report, NULL);
	bool all_up = is_boot_report && spi_diag_boot_report_all_up(report);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.send_submitted++;
	if (is_boot_report) {
		memcpy(spi_diag_state.live_stats.last_submitted_report, report, sizeof(report));
	}
	if (all_up) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.all_up_submitted);
	}
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_send_completed(const uint8_t frame[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN];
	bool is_boot_report = spi_diag_boot_report_copy(frame, report, NULL);
	bool all_up = is_boot_report && spi_diag_boot_report_all_up(report);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_state.live_stats.send_completed++;
	if (is_boot_report) {
		memcpy(spi_diag_state.live_stats.last_completed_report, report, sizeof(report));
	}
	if (all_up) {
		spi_diag_u16_inc(&spi_diag_state.live_stats.all_up_completed);
	}
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_retry(void)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.retry_count);
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_inflight_timeout(void)
{
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.inflight_timeout);
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_resubmit(const uint8_t frame[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t report[SPI_BRIDGE_BOOT_REPORT_LEN];
	bool is_boot_report = spi_diag_boot_report_copy(frame, report, NULL);
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.resubmit_count);
	if (is_boot_report) {
		memcpy(spi_diag_state.live_stats.last_resubmitted_report, report, sizeof(report));
	}
	k_spin_unlock(&spi_diag_lock, key);
}

static void spi_diag_note_stale_resubmit_blocked(void)
{
	bool log_now;
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.stale_resubmit_blocked);
	log_now = spi_diag_state.live_stats.stale_resubmit_blocked == 1u ||
		  (spi_diag_state.live_stats.stale_resubmit_blocked %
		   SPI_DIAG_QUEUE_LOG_STEP) == 0u;
	k_spin_unlock(&spi_diag_lock, key);
	if (log_now) {
		spi_diag_print("stale-resubmit-blocked");
	}
}

static void spi_diag_note_stale_completion(void)
{
	bool log_now;
	k_spinlock_key_t key = k_spin_lock(&spi_diag_lock);

	spi_diag_u16_inc(&spi_diag_state.live_stats.stale_completion);
	log_now = spi_diag_state.live_stats.stale_completion == 1u ||
		  (spi_diag_state.live_stats.stale_completion %
		   SPI_DIAG_QUEUE_LOG_STEP) == 0u;
	k_spin_unlock(&spi_diag_lock, key);
	if (log_now) {
		spi_diag_print("stale-completion");
	}
}

static bool spi_diag_console_ready(void)
{
	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	return device_is_ready(console) &&
	       uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr != 0u;
}

static void spi_diag_print(const char *reason)
{
	struct spi_hids_diag_live_stats stats;
	struct hids_tx_state tx_state;
	bool release_pending;
	bool release_queued;

	if (!spi_diag_console_ready()) {
		return;
	}

	stats = spi_diag_stats_copy();
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);

	release_pending = release_report_pending;
	release_queued = release_report_queued;
	k_spin_unlock(&latest_report_lock, key);

	key = k_spin_lock(&hids_tx_lock);
	tx_state = hids_tx_state;
	k_spin_unlock(&hids_tx_lock, key);

	LOG_INF("DIAG[%s] up=%u mode=%u epoch=%u conn_gen=%u q=%u/%u release=%u/%u err=%d",
		reason, (unsigned int)k_uptime_get_32(),
		(unsigned int)(atomic_get(&wireless_mode_enabled) != 0),
		(unsigned int)atomic_get(&spi_report_state_epoch),
		(unsigned int)atomic_get(&hids_conn_generation),
		(unsigned int)stats.current_depth, (unsigned int)stats.maximum_depth,
		(unsigned int)release_pending, (unsigned int)release_queued,
		(int)stats.last_hid_error);
	LOG_INF("DIAG flow rx=%u enq=%u deq=%u local_done=%u attempt=%u submit=%u complete=%u tx_submit=%u tx_flight=%u tx_done=%u",
		(unsigned int)stats.spi_rx, (unsigned int)stats.hid_enqueued,
		(unsigned int)stats.hid_dequeued, (unsigned int)stats.hid_completed,
		(unsigned int)stats.send_attempted, (unsigned int)stats.send_submitted,
		(unsigned int)stats.send_completed, (unsigned int)tx_state.submit_in_progress,
		(unsigned int)tx_state.in_flight,
		(unsigned int)tx_state.completion_received);
	LOG_INF("DIAG errors qfull=%u eagain=%u enomem=%u enotconn=%u eacces=%u canceled=%u other=%u retry=%u notify_to=%u stall=%u inflight_to=%u stale_completion=%u",
		(unsigned int)stats.queue_full, (unsigned int)stats.send_eagain,
		(unsigned int)stats.send_enomem, (unsigned int)stats.send_enotconn,
		(unsigned int)stats.send_eacces, (unsigned int)stats.send_ecanceled,
		(unsigned int)stats.send_other, (unsigned int)stats.retry_count,
		(unsigned int)stats.notify_timeout, (unsigned int)stats.send_stall,
		(unsigned int)stats.inflight_timeout,
		(unsigned int)stats.stale_completion);
	LOG_INF("DIAG all_up rx=%u enq=%u deq=%u submit=%u complete=%u resubmit=%u blocked=%u duplicate=%u seq=%u disconnect=%u",
		(unsigned int)stats.all_up_received, (unsigned int)stats.all_up_enqueued,
		(unsigned int)stats.all_up_dequeued, (unsigned int)stats.all_up_submitted,
		(unsigned int)stats.all_up_completed, (unsigned int)stats.resubmit_count,
		(unsigned int)stats.stale_resubmit_blocked,
		(unsigned int)stats.duplicate_nonzero,
		(unsigned int)stats.local_rx_sequence, (unsigned int)stats.disconnects);
	LOG_INF("DIAG conn interval=%u latency=%u timeout=%u tx_token=%u tx_epoch=%u tx_conn_gen=%u",
		(unsigned int)stats.conn_interval, (unsigned int)stats.conn_latency,
		(unsigned int)stats.supervision_timeout, (unsigned int)tx_state.token,
		(unsigned int)tx_state.report_epoch,
		(unsigned int)tx_state.conn_generation);
	LOG_HEXDUMP_INF(stats.last_received_report, sizeof(stats.last_received_report),
			"DIAG last_received");
	LOG_HEXDUMP_INF(stats.last_dequeued_report, sizeof(stats.last_dequeued_report),
			"DIAG last_dequeued");
	LOG_HEXDUMP_INF(stats.last_submitted_report, sizeof(stats.last_submitted_report),
			"DIAG last_submitted");
	LOG_HEXDUMP_INF(stats.last_completed_report, sizeof(stats.last_completed_report),
			"DIAG last_completed");
	LOG_HEXDUMP_INF(stats.last_resubmitted_report, sizeof(stats.last_resubmitted_report),
			"DIAG last_resubmitted");
	LOG_HEXDUMP_INF(stats.last_error_report, sizeof(stats.last_error_report),
			"DIAG last_error");
}

static void spi_diag_log_work_fn(struct k_work *work)
{
	bool ready = spi_diag_console_ready();

	ARG_UNUSED(work);
	if (ready) {
		spi_diag_print(spi_diag_console_seen ? "periodic" : "usb-open");
	}
	spi_diag_console_seen = ready;
	(void)k_work_schedule(&spi_diag_log_work, K_MSEC(SPI_DIAG_LOG_PERIOD_MS));
}

/* Host output report from the BLE peer, used when real lock LEDs are available. */
static volatile uint8_t host_output_led_bits;
static volatile bool host_output_led_valid;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void advertising_start(void);
static void profiles_save(void);
static void profile_switch_adv_refresh_fn(struct k_work *work);
static int ensure_profile_identities(void);
static void spi_debug_led_set(bool on);
static bool any_conn_active(void);
static bool any_connected(void);
static uint8_t spi_ack_led_bits_get(void);
static bool spi_ack_led_bits_valid(void);
static void host_output_led_reset(void);
static void set_pairing_mode(bool enable);
static void disconnect_all(void);
static void schedule_advertising_refresh(uint32_t delay_ms);
static void spi_report_thread_fn(void *arg1, void *arg2, void *arg3);
static int advertising_configure_accept_list(bool pairing_enabled);
static void hids_tx_cancel(struct bt_conn *conn);
static void spi_report_state_clear(bool clear_last_sent);
static void spi_report_hold_release(const uint8_t report[SPI_BRIDGE_FRAME_LEN],
				    uint32_t report_epoch);
static bool spi_report_submit(const uint8_t report[SPI_BRIDGE_FRAME_LEN]);
static void spi_report_latest_resubmit(void);
static void spi_report_release_resubmit(void);
static bool hids_keyboard_notify_ready(void);
static bool hids_frame_notify_ready(const uint8_t report[SPI_BRIDGE_FRAME_LEN]);
static void hids_report_keyboard_notify_handler(enum bt_hids_notify_evt evt);
static void hids_boot_keyboard_notify_handler(enum bt_hids_notify_evt evt);
static void hids_input_notify_handler(uint8_t report_id,
					      enum bt_hids_notify_evt evt);
static bool conn_mode_peer_identity_get(struct bt_conn *conn, bt_addr_le_t *identity);
static bool conn_mode_profile_index_get(struct bt_conn *conn, uint8_t *profile_idx);
static void conn_mode_set_peer_identity(struct bt_conn *conn,
					    const bt_addr_le_t *identity);
static bool active_profile_matches_peer(const bt_addr_le_t *peer);
static void maybe_exit_pairing_for_peer(const bt_addr_le_t *peer,
						 const char *reason);
static void wireless_mode_set(bool enabled);
static void mode_apply_work_fn(struct k_work *work);
static void spi_control_work_fn(struct k_work *work);
static void connection_active_work_fn(struct k_work *work);
static void ble_prepare_shutdown(void);
static void low_power_note_activity(void);

static bool usb_host_is_present(void)
{
	return atomic_get(&usb_host_present) != 0;
}

static bool addr_equal(const bt_addr_le_t *a, const bt_addr_le_t *b)
{
	return bt_addr_le_cmp(a, b) == 0;
}

static bool any_conn_active(void)
{
	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);
	bool active = false;

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn != NULL) {
			active = true;
			break;
		}
	}

	k_spin_unlock(&conn_mode_lock, key);
	return active;
}

static bool wireless_mode_is_enabled(void)
{
	return atomic_get(&wireless_mode_enabled) != 0;
}

struct conn_param_update_context {
	const struct bt_le_conn_param *params;
	const char *mode;
	bool requested;
};

static const struct bt_le_conn_param active_conn_params =
	BT_LE_CONN_PARAM_INIT(SPI_CONN_ACTIVE_INTERVAL_MIN,
			      SPI_CONN_ACTIVE_INTERVAL_MAX, 0,
			      SPI_CONN_SUPERVISION_TIMEOUT);

static const struct bt_le_conn_param idle_conn_params =
	BT_LE_CONN_PARAM_INIT(SPI_CONN_IDLE_INTERVAL_MIN,
			      SPI_CONN_IDLE_INTERVAL_MAX,
			      SPI_CONN_IDLE_LATENCY,
			      SPI_CONN_SUPERVISION_TIMEOUT);

static void connection_param_update_cb(struct bt_conn *conn, void *data)
{
	struct conn_param_update_context *ctx = data;
	struct bt_conn_info info;
	int ret = bt_conn_get_info(conn, &info);

	if (ret || info.state != BT_CONN_STATE_CONNECTED) {
		return;
	}

	ret = bt_conn_le_param_update(conn, ctx->params);
	if (ret) {
		printk("Connection %s parameter request failed: %d\n",
		       ctx->mode, ret);
		return;
	}

	ctx->requested = true;
	printk("Connection parameter request: %s\n", ctx->mode);
}

static bool connection_params_request(bool idle)
{
	struct conn_param_update_context ctx = {
		.params = idle ? &idle_conn_params : &active_conn_params,
		.mode = idle ? "idle" : "active",
	};

	bt_conn_foreach(BT_CONN_TYPE_LE, connection_param_update_cb, &ctx);
	return ctx.requested;
}

static uint32_t wireless_idle_elapsed_ms(void)
{
	return k_uptime_get_32() - last_wireless_activity_ms;
}

static void connection_idle_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!wireless_mode_is_enabled() || !any_conn_active()) {
		return;
	}

	uint32_t elapsed = wireless_idle_elapsed_ms();

	if (elapsed < SPI_CONNECTION_IDLE_MS) {
		(void)k_work_reschedule(&connection_idle_work,
					K_MSEC(SPI_CONNECTION_IDLE_MS - elapsed));
		return;
	}

	atomic_set(&connection_idle_mode, 1);
	if (!connection_params_request(true)) {
		atomic_set(&connection_idle_mode, 0);
	}
}

static void connection_active_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!wireless_mode_is_enabled() || !any_conn_active()) {
		return;
	}
	if (!connection_params_request(false)) {
		atomic_set(&connection_idle_mode, 1);
	}
}

static void slow_advertising_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!wireless_mode_is_enabled() || any_conn_active()) {
		return;
	}

	uint32_t elapsed = wireless_idle_elapsed_ms();

	if (elapsed < SPI_SLOW_ADV_DELAY_MS) {
		(void)k_work_reschedule(&slow_advertising_work,
					K_MSEC(SPI_SLOW_ADV_DELAY_MS - elapsed));
		return;
	}

	slow_advertising = true;
	advertising_start();
}

static void auto_poweroff_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!atomic_get(&hostless_timer_valid) ||
	    !atomic_get(&usb_presence_known) || usb_host_is_present() || any_conn_active()) {
		return;
	}

	uint32_t elapsed = k_uptime_get_32() - last_hostless_ms;

	if (elapsed < SPI_AUTO_POWEROFF_DELAY_MS) {
		(void)k_work_reschedule(&auto_poweroff_work,
					K_MSEC(SPI_AUTO_POWEROFF_DELAY_MS - elapsed));
		return;
	}

	printk("No USB or BLE host for %u minutes, shutting down\n",
	       (unsigned int)(SPI_AUTO_POWEROFF_DELAY_MS / 60000U));
	atomic_set(&hostless_timer_valid, 0);
	/* Automatic poweroff must remain wakeable by P0.29.  The no-wake path
	 * deliberately requires a battery cycle and is not suitable for an idle
	 * BLE/SPI timeout. */
	power_manager_shutdown();
}

static void spi_link_idle_poweroff_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!wireless_mode_is_enabled() || usb_host_is_present()) {
		return;
	}

	uint32_t elapsed = k_uptime_get_32() - last_spi_activity_ms;

	if (elapsed < SPI_LINK_IDLE_POWEROFF_MS) {
		(void)k_work_reschedule(&spi_link_idle_poweroff_work,
					K_MSEC(SPI_LINK_IDLE_POWEROFF_MS - elapsed));
		return;
	}

	/*
	 * SPI链路长时间无活动（键盘MCU可能已关机或故障），自动关机
	 */
	printk("SPI link idle for %u minutes, shutting down\n",
	       (unsigned int)(SPI_LINK_IDLE_POWEROFF_MS / 60000U));
	/* Keep the physical power button armed even when the SPI peer has been
	 * silent for a long time; otherwise the next wake requires battery removal. */
	power_manager_shutdown();
}

static void radio_idle_timers_cancel(void)
{
	(void)k_work_cancel_delayable(&connection_idle_work);
	(void)k_work_cancel_delayable(&slow_advertising_work);
	(void)k_work_cancel_delayable(&spi_link_idle_poweroff_work);
	atomic_set(&connection_idle_mode, 0);
	last_spi_activity_ms = k_uptime_get_32();
	slow_advertising = false;
}

static void hostless_poweroff_timer_restart(void)
{
	if (!atomic_get(&usb_presence_known) || usb_host_is_present() || any_conn_active()) {
		return;
	}

	last_hostless_ms = k_uptime_get_32();
	atomic_set(&hostless_timer_valid, 1);
	(void)k_work_reschedule(&auto_poweroff_work,
				K_MSEC(SPI_AUTO_POWEROFF_DELAY_MS));
}

static void hostless_poweroff_timer_ensure(void)
{
	if (!atomic_get(&usb_presence_known) || usb_host_is_present() || any_conn_active()) {
		return;
	}

	uint32_t elapsed = k_uptime_get_32() - last_hostless_ms;
	if (!atomic_get(&hostless_timer_valid)) {
		last_hostless_ms = k_uptime_get_32();
		elapsed = 0;
		atomic_set(&hostless_timer_valid, 1);
	}

	uint32_t remaining = elapsed < SPI_AUTO_POWEROFF_DELAY_MS ?
		SPI_AUTO_POWEROFF_DELAY_MS - elapsed : 0U;
	(void)k_work_reschedule(&auto_poweroff_work, K_MSEC(remaining));
}

static void disconnected_idle_timers_start(void)
{
	(void)k_work_cancel_delayable(&connection_idle_work);
	atomic_set(&connection_idle_mode, 0);

	if (!wireless_mode_is_enabled() || usb_host_is_present() || any_conn_active()) {
		return;
	}

	bool start_slow_timer = !k_work_delayable_is_pending(&slow_advertising_work);

	slow_advertising = false;
	if (start_slow_timer) {
		last_wireless_activity_ms = k_uptime_get_32();
		(void)k_work_reschedule(&slow_advertising_work,
						K_MSEC(SPI_SLOW_ADV_DELAY_MS));
	}
	hostless_poweroff_timer_ensure();
}

static void connected_idle_timer_start(void)
{
	(void)k_work_cancel_delayable(&slow_advertising_work);
	slow_advertising = false;
	atomic_set(&connection_idle_mode, 0);

	if (!wireless_mode_is_enabled()) {
		if (!usb_host_is_present() && !any_conn_active()) {
			hostless_poweroff_timer_ensure();
		}
		return;
	}

	atomic_set(&hostless_timer_valid, 0);
	last_wireless_activity_ms = k_uptime_get_32();
	(void)k_work_reschedule(&connection_idle_work,
				K_MSEC(SPI_CONNECTION_IDLE_MS));
}

static void low_power_note_activity(void)
{
	if (!wireless_mode_is_enabled()) {
		return;
	}

	last_wireless_activity_ms = k_uptime_get_32();

	if (any_conn_active()) {
		if (atomic_cas(&connection_idle_mode, 1, 0)) {
			(void)k_work_submit(&connection_active_work);
		}
		(void)k_work_reschedule(&connection_idle_work,
					K_MSEC(SPI_CONNECTION_IDLE_MS));
		return;
	}

	(void)k_work_cancel_delayable(&connection_idle_work);
	atomic_set(&connection_idle_mode, 0);
	if (usb_host_is_present()) {
		return;
	}

	bool restart_fast_advertising = slow_advertising;

	slow_advertising = false;
	(void)k_work_reschedule(&slow_advertising_work,
				K_MSEC(SPI_SLOW_ADV_DELAY_MS));
	hostless_poweroff_timer_restart();
	if (restart_fast_advertising) {
		(void)k_work_submit(&mode_apply_work);
	}
}

static bool conn_mode_peer_identity_get(struct bt_conn *conn, bt_addr_le_t *identity)
{
	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);
	bool found = false;

	if (identity) {
		for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
			if (conn_mode[i].conn == conn && conn_mode[i].peer_identity_valid) {
				*identity = conn_mode[i].peer_identity;
				found = true;
				break;
			}
		}
	}

	k_spin_unlock(&conn_mode_lock, key);
	return found;
}

static bool conn_mode_profile_index_get(struct bt_conn *conn, uint8_t *profile_idx)
{
	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);
	bool found = false;

	if (profile_idx) {
		for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
			if (conn_mode[i].conn == conn && conn_mode[i].profile_idx_valid) {
				*profile_idx = conn_mode[i].profile_idx;
				found = true;
				break;
			}
		}
	}

	k_spin_unlock(&conn_mode_lock, key);
	return found;
}

static void conn_mode_set_peer_identity(struct bt_conn *conn,
					    const bt_addr_le_t *identity)
{
	if (!identity) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			conn_mode[i].peer_identity = *identity;
			conn_mode[i].peer_identity_valid = true;
			break;
		}
	}
	k_spin_unlock(&conn_mode_lock, key);
}

static bool active_profile_matches_peer(const bt_addr_le_t *peer)
{
	if (!peer || active_profile >= PROFILE_COUNT || !profiles[active_profile].valid) {
		return false;
	}

	return addr_equal(&profiles[active_profile].addr, peer);
}

static void maybe_exit_pairing_for_peer(const bt_addr_le_t *peer,
					 const char *reason)
{
	if (!pairing_mode || !active_profile_matches_peer(peer)) {
		return;
	}

	printk("Active profile peer %s, pairing mode OFF\n",
	       reason ? reason : "reconnected");
	set_pairing_mode(false);
}

static bool deadline_is_active(uint32_t deadline_ms)
{
	return deadline_ms != 0u && (int32_t)(deadline_ms - k_uptime_get_32()) > 0;
}

static uint8_t spi_ack_led_bits_get(void)
{
	uint8_t profile_idx = (active_profile < PROFILE_COUNT) ? active_profile : 0u;
	uint8_t bits = (uint8_t)((profile_idx << SPI_BRIDGE_ACK_LED_PROFILE_SHIFT) &
				 SPI_BRIDGE_ACK_LED_PROFILE_MASK);
	bool connected = any_conn_active();

	if (connected && host_output_led_valid) {
		bits |= host_output_led_bits &
			(OUTPUT_REPORT_NUM_LOCK_MASK |
			 OUTPUT_REPORT_CAPS_LOCK_MASK |
			 OUTPUT_REPORT_SCROLL_LOCK_MASK);
	}

	if (!wireless_mode_is_enabled()) {
		return bits;
	}

	if (pairing_mode) {
		bits |= SPI_BRIDGE_ACK_LED_BLE_IND_ACTIVE;
		if (((k_uptime_get_32() / 180U) & 0x01U) == 0U) {
			bits |= SPI_BRIDGE_ACK_LED_BLE_IND_ON;
		}
		return bits;
	}

	if (connected) {
		if (deadline_is_active(connected_profile_led_until_ms)) {
			bits |= SPI_BRIDGE_ACK_LED_BLE_IND_ACTIVE |
				SPI_BRIDGE_ACK_LED_BLE_IND_ON;
		}
		return bits;
	}

	if (deadline_is_active(profile_switch_led_until_ms)) {
		bits |= SPI_BRIDGE_ACK_LED_BLE_IND_ACTIVE;
		if (((k_uptime_get_32() / 250U) & 0x01U) == 0U) {
			bits |= SPI_BRIDGE_ACK_LED_BLE_IND_ON;
		}
	}

	return bits;
}

static bool spi_ack_led_bits_valid(void)
{
	return any_conn_active() && host_output_led_valid;
}

static void host_output_led_reset(void)
{
	host_output_led_bits = 0;
	host_output_led_valid = false;
}

static void schedule_advertising_refresh(uint32_t delay_ms)
{
	if (!wireless_mode_is_enabled() || any_connected()) {
		return;
	}

	/* A new disconnect/profile event starts a fresh bounded convergence window. */
	advertising_start_retry_count = 0;
	k_work_reschedule(&profile_switch_adv_refresh_work, K_MSEC(delay_ms));
}

static void advertising_start_retry_schedule(int err)
{
	if (!wireless_mode_is_enabled() || any_connected()) {
		advertising_start_retry_count = 0;
		return;
	}

	if (advertising_start_retry_count >= SPI_ADV_START_RETRY_LIMIT) {
		printk("Advertising retry limit reached (%d)\n", err);
		return;
	}

	advertising_start_retry_count++;
	printk("Advertising start retry %u/%u after %d\n",
	       advertising_start_retry_count, SPI_ADV_START_RETRY_LIMIT, err);
	(void)k_work_reschedule(&profile_switch_adv_refresh_work,
				K_MSEC(SPI_ADV_START_RETRY_DELAY_MS));
}

static uint8_t spi_release_generation_next(uint8_t generation)
{
	generation++;
	return generation == 0u ? 1u : generation;
}

static void spi_release_state_invalidate_all_locked(uint32_t epoch,
						    bool mark_pending)
{
	uint8_t conn_generation = (uint8_t)atomic_get(&hids_conn_generation);

	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		struct spi_release_slot_state *state = &spi_release_slots[i];

		state->queued = false;
		state->generation = spi_release_generation_next(state->generation);
		state->epoch = epoch;
		state->conn_generation = conn_generation;
		if (mark_pending) {
			state->pending = true;
		}
	}
}

static void spi_report_state_clear(bool clear_last_sent)
{
	k_mutex_lock(&spi_report_generation_mutex, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);

	atomic_inc(&spi_report_state_epoch);
	uint32_t epoch = (uint32_t)atomic_get(&spi_report_state_epoch);
	k_msgq_purge(&spi_report_msgq);
	spi_release_state_invalidate_all_locked(epoch, clear_last_sent);
	latest_spi_report_generation++;
	if (clear_last_sent) {
		latest_spi_report_valid = false;
	}
	k_spin_unlock(&latest_report_lock, key);
	hids_tx_cancel(NULL);
	k_mutex_unlock(&spi_report_generation_mutex);
	spi_diag_note_queue_depth(0);
}

static void spi_report_latest_invalidate(void)
{
	spi_report_state_clear(true);
}

static bool spi_report_is_keyboard(const uint8_t report[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t type;

	if (!spi_bridge_report_frame_read(report, &type, NULL, NULL, NULL)) {
		return false;
	}

	return type == SPI_BRIDGE_REPORT_KEYBOARD_BOOT ||
	       type == SPI_BRIDGE_REPORT_KEYBOARD_NKRO;
}

static bool spi_report_keyboard_all_up(const uint8_t report[SPI_BRIDGE_FRAME_LEN])
{
	const uint8_t *payload;
	uint8_t payload_len;
	uint8_t type;

	if (!spi_bridge_report_frame_read(report, &type, NULL, &payload, &payload_len) ||
	    (type != SPI_BRIDGE_REPORT_KEYBOARD_BOOT &&
	     type != SPI_BRIDGE_REPORT_KEYBOARD_NKRO)) {
		return false;
	}

	for (uint8_t i = 0; i < payload_len; i++) {
		if (payload[i] != 0u) {
			return false;
		}
	}

	return true;
}

static uint8_t spi_release_slot_from_report_type(uint8_t report_type)
{
	switch (report_type) {
	case SPI_BRIDGE_REPORT_KEYBOARD_BOOT:
	case SPI_BRIDGE_REPORT_KEYBOARD_NKRO:
		return SPI_RELEASE_KEYBOARD;
	case SPI_BRIDGE_REPORT_CONSUMER:
		return SPI_RELEASE_CONSUMER;
	case SPI_BRIDGE_REPORT_SYSTEM:
		return SPI_RELEASE_SYSTEM;
	default:
		return SPI_RELEASE_SLOT_NONE;
	}
}

static uint8_t spi_release_slot_from_report(const uint8_t report[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t report_type;

	if (!spi_bridge_report_frame_read(report, &report_type, NULL, NULL, NULL)) {
		return SPI_RELEASE_SLOT_NONE;
	}
	return spi_release_slot_from_report_type(report_type);
}

static bool spi_report_release_payload_matches_slot(
	const uint8_t report[SPI_BRIDGE_FRAME_LEN], uint8_t release_slot)
{
	const uint8_t *payload;
	uint8_t payload_len;
	uint8_t report_type;

	if (!spi_bridge_report_frame_read(report, &report_type, NULL, &payload, &payload_len) ||
	    spi_release_slot_from_report_type(report_type) != release_slot) {
		return false;
	}

	for (uint8_t i = 0; i < payload_len; i++) {
		if (payload[i] != 0u) {
			return false;
		}
	}
	return true;
}

static void spi_report_epoch_set(uint8_t report[SPI_BRIDGE_FRAME_LEN], uint32_t epoch)
{
	sys_put_le32(epoch, &report[SPI_REPORT_LOCAL_EPOCH_INDEX]);
	report[SPI_BRIDGE_FRAME_CRC_INDEX] =
		spi_bridge_crc8(report, SPI_BRIDGE_FRAME_CRC_INDEX);
}

static uint32_t spi_report_epoch_get(const uint8_t report[SPI_BRIDGE_FRAME_LEN])
{
	return sys_get_le32(&report[SPI_REPORT_LOCAL_EPOCH_INDEX]);
}

static void spi_report_local_tag_set(uint8_t report[SPI_BRIDGE_FRAME_LEN],
				     uint8_t release_slot, uint8_t release_generation,
				     uint8_t conn_generation, uint32_t epoch)
{
	report[SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX] = release_slot;
	report[SPI_REPORT_LOCAL_RELEASE_GENERATION_INDEX] = release_generation;
	report[SPI_REPORT_LOCAL_CONN_GENERATION_INDEX] = conn_generation;
	spi_report_epoch_set(report, epoch);
}

static bool spi_report_release_tag_matches_locked(
	const uint8_t report[SPI_BRIDGE_FRAME_LEN], uint32_t report_epoch)
{
	uint8_t release_slot = report[SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX];

	if (release_slot >= SPI_RELEASE_SLOT_COUNT ||
	    !spi_report_release_payload_matches_slot(report, release_slot)) {
		return false;
	}

	const struct spi_release_slot_state *state = &spi_release_slots[release_slot];

	return state->pending && state->queued && state->epoch == report_epoch &&
	       state->generation == report[SPI_REPORT_LOCAL_RELEASE_GENERATION_INDEX] &&
	       state->conn_generation == report[SPI_REPORT_LOCAL_CONN_GENERATION_INDEX];
}

static struct bt_conn *hids_active_conn_ref(bool *boot_mode)
{
	struct bt_conn *conn = NULL;
	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn) {
			conn = bt_conn_ref(conn_mode[i].conn);
			*boot_mode = conn_mode[i].in_boot_mode;
			break;
		}
	}
	k_spin_unlock(&conn_mode_lock, key);
	return conn;
}

static bool hids_keyboard_notify_ready(void)
{
	bool boot_mode = false;
	struct bt_conn *conn = hids_active_conn_ref(&boot_mode);

	if (!conn) {
		return false;
	}

	uint8_t attr_index = boot_mode ? hids_obj.boot_kb_inp_rep.att_ind :
		hids_obj.inp_rep_group.reports[INPUT_REP_KEYBOARD_INDEX].att_ind;
	struct bt_gatt_attr *report_attr = &hids_obj.gp.svc.attrs[attr_index];
	/* A bonded peer can restore CCC before link encryption completes.  Treat
	 * that interval as not ready so reports do not spin on GATT -EPERM. */
	bool ready = bt_conn_get_security(conn) >= BT_SECURITY_L2 &&
		bt_gatt_is_subscribed(conn, report_attr, BT_GATT_CCC_NOTIFY);

	bt_conn_unref(conn);
	return ready;
}

static bool hids_report_notify_ready(uint8_t report_index)
{
	bool boot_mode = false;
	struct bt_conn *conn = hids_active_conn_ref(&boot_mode);

	if (!conn) {
		return false;
	}
	if (boot_mode) {
		bt_conn_unref(conn);
		return false;
	}

	uint8_t attr_index = hids_obj.inp_rep_group.reports[report_index].att_ind;
	struct bt_gatt_attr *report_attr = &hids_obj.gp.svc.attrs[attr_index];
	bool ready = bt_conn_get_security(conn) >= BT_SECURITY_L2 &&
		bt_gatt_is_subscribed(conn, report_attr, BT_GATT_CCC_NOTIFY);

	bt_conn_unref(conn);
	return ready;
}

static bool hids_frame_notify_ready(const uint8_t report[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t report_type;

	if (!spi_bridge_report_frame_read(report, &report_type, NULL, NULL, NULL)) {
		return false;
	}

	switch (report_type) {
	case SPI_BRIDGE_REPORT_KEYBOARD_BOOT:
	case SPI_BRIDGE_REPORT_KEYBOARD_NKRO:
		return hids_keyboard_notify_ready();
	case SPI_BRIDGE_REPORT_CONSUMER:
		return hids_report_notify_ready(INPUT_REP_CONSUMER_INDEX);
	case SPI_BRIDGE_REPORT_SYSTEM:
		return hids_report_notify_ready(INPUT_REP_SYSTEM_INDEX);
	default:
		return false;
	}
}

static int spi_report_release_enqueue_locked(uint8_t report[SPI_BRIDGE_FRAME_LEN],
					     uint32_t epoch)
{
	uint8_t release_payload[SPI_BRIDGE_NKRO_REPORT_LEN] = { 0 };
	struct spi_release_slot_state *state = &spi_release_slots[SPI_RELEASE_KEYBOARD];

	if (!spi_bridge_report_frame_write(report, SPI_BRIDGE_REPORT_KEYBOARD_NKRO,
					   0, release_payload, sizeof(release_payload))) {
		return -EINVAL;
	}
	state->epoch = epoch;
	state->conn_generation = (uint8_t)atomic_get(&hids_conn_generation);
	spi_report_local_tag_set(report, SPI_RELEASE_KEYBOARD, state->generation,
				 state->conn_generation, epoch);
	int ret = k_msgq_put(&spi_report_msgq, report, K_NO_WAIT);

	if (!ret) {
		memcpy(latest_spi_report, report, SPI_BRIDGE_FRAME_LEN);
		latest_spi_report_valid = true;
		latest_spi_report_generation++;
		state->queued = true;
	}
	return ret;
}

static int spi_report_aux_release_enqueue_locked(uint8_t report[SPI_BRIDGE_FRAME_LEN],
						 uint8_t report_type, uint32_t epoch)
{
	uint8_t release_payload[sizeof(uint16_t)] = { 0 };
	uint8_t release_slot = spi_release_slot_from_report_type(report_type);

	if (release_slot != SPI_RELEASE_CONSUMER && release_slot != SPI_RELEASE_SYSTEM) {
		return -EINVAL;
	}
	struct spi_release_slot_state *state = &spi_release_slots[release_slot];

	if (!spi_bridge_report_frame_write(report, report_type, 0, release_payload,
					   sizeof(release_payload))) {
		return -EINVAL;
	}
	state->epoch = epoch;
	state->conn_generation = (uint8_t)atomic_get(&hids_conn_generation);
	spi_report_local_tag_set(report, release_slot, state->generation,
				 state->conn_generation, epoch);
	int ret = k_msgq_put(&spi_report_msgq, report, K_NO_WAIT);

	if (!ret) {
		state->queued = true;
	}
	return ret;
}

static bool spi_release_slot_notify_ready(uint8_t release_slot)
{
	switch (release_slot) {
	case SPI_RELEASE_KEYBOARD:
		return hids_keyboard_notify_ready();
	case SPI_RELEASE_CONSUMER:
		return hids_report_notify_ready(INPUT_REP_CONSUMER_INDEX);
	case SPI_RELEASE_SYSTEM:
		return hids_report_notify_ready(INPUT_REP_SYSTEM_INDEX);
	default:
		return false;
	}
}

static int spi_report_release_enqueue_slot_locked(
	uint8_t report[SPI_BRIDGE_FRAME_LEN], uint8_t release_slot, uint32_t epoch)
{
	switch (release_slot) {
	case SPI_RELEASE_KEYBOARD:
		return spi_report_release_enqueue_locked(report, epoch);
	case SPI_RELEASE_CONSUMER:
		return spi_report_aux_release_enqueue_locked(report,
			SPI_BRIDGE_REPORT_CONSUMER, epoch);
	case SPI_RELEASE_SYSTEM:
		return spi_report_aux_release_enqueue_locked(report,
			SPI_BRIDGE_REPORT_SYSTEM, epoch);
	default:
		return -EINVAL;
	}
}

static void spi_report_hold_release(const uint8_t report[SPI_BRIDGE_FRAME_LEN],
				    uint32_t report_epoch)
{
	bool reset = false;

	k_mutex_lock(&spi_report_generation_mutex, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);

	if ((uint32_t)atomic_get(&spi_report_state_epoch) == report_epoch) {
		uint8_t release_slot = report[SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX];

		if (release_slot == SPI_RELEASE_SLOT_NONE ||
		    spi_report_release_tag_matches_locked(report, report_epoch)) {
			atomic_inc(&spi_report_state_epoch);
			uint32_t epoch = (uint32_t)atomic_get(&spi_report_state_epoch);

			k_msgq_purge(&spi_report_msgq);
			latest_spi_report_valid = false;
			latest_spi_report_generation++;
			spi_release_state_invalidate_all_locked(epoch, true);
			reset = true;
		}
	}
	k_spin_unlock(&latest_report_lock, key);
	k_mutex_unlock(&spi_report_generation_mutex);
	if (reset) {
		spi_diag_note_queue_depth(0);
	}
}

static void spi_report_release_complete(const uint8_t report[SPI_BRIDGE_FRAME_LEN],
					uint32_t report_epoch)
{
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);
	uint8_t release_slot = report[SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX];

	if ((uint32_t)atomic_get(&spi_report_state_epoch) == report_epoch &&
	    release_slot < SPI_RELEASE_SLOT_COUNT &&
	    spi_report_release_tag_matches_locked(report, report_epoch)) {
		spi_release_slots[release_slot].pending = false;
		spi_release_slots[release_slot].queued = false;
	}
	k_spin_unlock(&latest_report_lock, key);
}

static void spi_report_release_resubmit(void)
{
	uint8_t release_frames[SPI_RELEASE_SLOT_COUNT][SPI_BRIDGE_FRAME_LEN];
	bool ready[SPI_RELEASE_SLOT_COUNT] = { false };
	bool enqueued[SPI_RELEASE_SLOT_COUNT] = { false };
	bool queue_failed = false;
	bool release_waiting = false;

	if (!wireless_mode_is_enabled() || !any_conn_active()) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);
	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		if (spi_release_slots[i].pending && !spi_release_slots[i].queued) {
			release_waiting = true;
			break;
		}
	}
	k_spin_unlock(&latest_report_lock, key);
	if (!release_waiting) {
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		ready[i] = spi_release_slot_notify_ready((uint8_t)i);
	}

	k_mutex_lock(&spi_report_generation_mutex, K_FOREVER);
	key = k_spin_lock(&latest_report_lock);
	uint32_t epoch = (uint32_t)atomic_get(&spi_report_state_epoch);

	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		struct spi_release_slot_state *state = &spi_release_slots[i];
		int ret;

		if (!state->pending || state->queued || !ready[i]) {
			continue;
		}
		ret = spi_report_release_enqueue_slot_locked(release_frames[i],
							(uint8_t)i, epoch);
			if (ret) {
				queue_failed = true;
				continue;
			}
			enqueued[i] = true;
	}
	k_spin_unlock(&latest_report_lock, key);
	k_mutex_unlock(&spi_report_generation_mutex);

	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		if (!enqueued[i]) {
			continue;
		}
		spi_diag_note_enqueued(release_frames[i],
					       k_msgq_num_used_get(&spi_report_msgq));
		if (i == SPI_RELEASE_KEYBOARD) {
			spi_diag_note_resubmit(release_frames[i]);
		}
	}
	if (queue_failed) {
		spi_diag_note_queue_full(k_msgq_num_used_get(&spi_report_msgq));
	}
}

static bool spi_report_submit(const uint8_t report[SPI_BRIDGE_FRAME_LEN])
{
	uint8_t queued_report[SPI_BRIDGE_FRAME_LEN];
	uint8_t release_frames[SPI_RELEASE_SLOT_COUNT][SPI_BRIDGE_FRAME_LEN];
	bool ready[SPI_RELEASE_SLOT_COUNT] = { false };
	bool release_enqueued[SPI_RELEASE_SLOT_COUNT] = { false };
	bool queue_failed = false;
	uint8_t release_slot = spi_release_slot_from_report(report);
	bool keyboard;
	int ret = 0;

	if (!wireless_mode_is_enabled() ||
	    release_slot == SPI_RELEASE_SLOT_NONE) {
		return false;
	}

	keyboard = spi_report_is_keyboard(report);
	memcpy(queued_report, report, sizeof(queued_report));

	bool release_waiting = false;
	k_spinlock_key_t pre_key = k_spin_lock(&latest_report_lock);
	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		if (spi_release_slots[i].pending && !spi_release_slots[i].queued) {
			release_waiting = true;
			break;
		}
	}
	k_spin_unlock(&latest_report_lock, pre_key);

	if (release_waiting) {
		for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
			ready[i] = spi_release_slot_notify_ready((uint8_t)i);
		}
	}

	k_mutex_lock(&spi_report_generation_mutex, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);
	uint32_t epoch = (uint32_t)atomic_get(&spi_report_state_epoch);

	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		struct spi_release_slot_state *state = &spi_release_slots[i];

		if (!state->pending || state->queued || !ready[i]) {
			continue;
		}
		ret = spi_report_release_enqueue_slot_locked(release_frames[i],
							(uint8_t)i, epoch);
		if (ret) {
			queue_failed = true;
			continue;
		}
		release_enqueued[i] = true;
	}

	/* A keyboard release is a global barrier.  An aux release only blocks
	 * another report of the same HIDS type, so one stuck CCC cannot reorder
	 * unrelated reports. */
	bool blocked = release_report_pending ||
		spi_release_slots[release_slot].pending;
	if (!blocked) {
		spi_report_local_tag_set(queued_report, SPI_RELEASE_SLOT_NONE, 0u,
					 (uint8_t)atomic_get(&hids_conn_generation), epoch);
		ret = k_msgq_put(&spi_report_msgq, queued_report, K_NO_WAIT);
		if (!ret && keyboard) {
			memcpy(latest_spi_report, queued_report, SPI_BRIDGE_FRAME_LEN);
			latest_spi_report_valid = true;
			latest_spi_report_generation++;
		}
	}
	k_spin_unlock(&latest_report_lock, key);
	k_mutex_unlock(&spi_report_generation_mutex);

	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		if (!release_enqueued[i]) {
			continue;
		}
		spi_diag_note_enqueued(release_frames[i],
					       k_msgq_num_used_get(&spi_report_msgq));
		if (i == SPI_RELEASE_KEYBOARD) {
			spi_diag_note_resubmit(release_frames[i]);
		}
	}
	if (queue_failed) {
		spi_diag_note_queue_full(k_msgq_num_used_get(&spi_report_msgq));
	}

	if (blocked || ret) {
		if (ret) {
			uint32_t now = k_uptime_get_32();

			if ((int32_t)(now - spi_queue_drop_log_at_ms) >= 0) {
				printk("SPI report queue backpressure: %d\n", ret);
				spi_queue_drop_log_at_ms = now + SPI_QUEUE_DROP_LOG_INTERVAL_MS;
			}
		}
		return false;
	}

	spi_diag_note_enqueued(queued_report, k_msgq_num_used_get(&spi_report_msgq));
	last_spi_activity_ms = k_uptime_get_32();
	return true;
}

static void spi_report_latest_resubmit(void)
{
	uint8_t report[SPI_BRIDGE_FRAME_LEN];
	uint32_t generation;
	bool valid;

	/* CCC/security/protocol changes first drain every type-specific release. */
	spi_report_release_resubmit();

	if (!wireless_mode_is_enabled() || !any_conn_active() ||
	    !hids_keyboard_notify_ready()) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);
	valid = latest_spi_report_valid;
	generation = latest_spi_report_generation;
	if (valid) {
		memcpy(report, latest_spi_report, sizeof(report));
	}
	bool release_pending = release_report_pending;
	bool release_queued = release_report_queued;
	k_spin_unlock(&latest_report_lock, key);

	if (release_pending || release_queued || !valid) {
		return;
	}
	if (!spi_report_keyboard_all_up(report)) {
		spi_diag_note_stale_resubmit_blocked();
		return;
	}

	k_mutex_lock(&spi_report_generation_mutex, K_FOREVER);
	key = k_spin_lock(&latest_report_lock);
	uint32_t epoch = (uint32_t)atomic_get(&spi_report_state_epoch);
	if (release_report_pending || release_report_queued ||
	    !latest_spi_report_valid || latest_spi_report_generation != generation ||
	    memcmp(latest_spi_report, report, sizeof(report)) != 0) {
		k_spin_unlock(&latest_report_lock, key);
		k_mutex_unlock(&spi_report_generation_mutex);
		spi_diag_note_stale_resubmit_blocked();
		return;
	}
	spi_report_local_tag_set(report, SPI_RELEASE_SLOT_NONE, 0u,
				(uint8_t)atomic_get(&hids_conn_generation), epoch);
	int ret = k_msgq_put(&spi_report_msgq, report, K_NO_WAIT);

	k_spin_unlock(&latest_report_lock, key);
	k_mutex_unlock(&spi_report_generation_mutex);
	if (ret) {
		spi_diag_note_queue_full(k_msgq_num_used_get(&spi_report_msgq));
		return;
	}
	spi_diag_note_enqueued(report, k_msgq_num_used_get(&spi_report_msgq));
	spi_diag_note_resubmit(report);
}

static void hids_keyboard_notify_event(bool boot_mode,
					       enum bt_hids_notify_evt evt)
{
	bool current_boot_mode = false;
	struct bt_conn *conn = hids_active_conn_ref(&current_boot_mode);

	if (!conn || current_boot_mode != boot_mode) {
		if (conn) {
			bt_conn_unref(conn);
		}
		return;
	}
	bt_conn_unref(conn);

	if (evt == BT_HIDS_CCCD_EVT_NOTIFY_ENABLED) {
		spi_report_latest_resubmit();
	} else if (evt == BT_HIDS_CCCD_EVT_NOTIFY_DISABLED) {
		spi_report_latest_invalidate();
	}
}

static void hids_report_keyboard_notify_handler(enum bt_hids_notify_evt evt)
{
	hids_keyboard_notify_event(false, evt);
}

static void hids_boot_keyboard_notify_handler(enum bt_hids_notify_evt evt)
{
	hids_keyboard_notify_event(true, evt);
}

static void hids_input_notify_handler(uint8_t report_id,
				      enum bt_hids_notify_evt evt)
{
	if (report_id != INPUT_REP_CONSUMER_REF_ID &&
	    report_id != INPUT_REP_SYSTEM_REF_ID) {
		return;
	}
	if (evt == BT_HIDS_CCCD_EVT_NOTIFY_ENABLED) {
		spi_report_latest_resubmit();
	} else if (evt == BT_HIDS_CCCD_EVT_NOTIFY_DISABLED) {
		spi_report_latest_invalidate();
	}
}

static void spi_link_watchdog_fn(struct k_work *work)
{
	uint8_t release_frames[3][SPI_BRIDGE_FRAME_LEN];
	bool release_enqueued[3] = { false };
	bool keyboard_ready;
	bool consumer_ready;
	bool system_ready;
	bool queue_failed = false;

	ARG_UNUSED(work);

	if (!wireless_mode_is_enabled() || !any_conn_active()) {
		return;
	}

	spi_diag_print("spi-timeout");
	atomic_set(&spi_link_timed_out, 1);
	keyboard_ready = hids_keyboard_notify_ready();
	consumer_ready = hids_report_notify_ready(INPUT_REP_CONSUMER_INDEX);
	system_ready = hids_report_notify_ready(INPUT_REP_SYSTEM_INDEX);

	k_mutex_lock(&spi_report_generation_mutex, K_FOREVER);
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);

	atomic_inc(&spi_report_state_epoch);
	uint32_t epoch = (uint32_t)atomic_get(&spi_report_state_epoch);
	k_msgq_purge(&spi_report_msgq);
	latest_spi_report_valid = false;
	spi_release_state_invalidate_all_locked(epoch, true);
	latest_spi_report_generation++;
	bool ready[ARRAY_SIZE(spi_release_slots)] = {
		keyboard_ready, consumer_ready, system_ready,
	};
	for (size_t i = 0; i < ARRAY_SIZE(spi_release_slots); i++) {
		if (!ready[i]) {
			continue;
		}
		int ret = spi_report_release_enqueue_slot_locked(release_frames[i],
							(uint8_t)i, epoch);

		release_enqueued[i] = ret == 0;
		queue_failed = queue_failed || ret != 0;
	}
	k_spin_unlock(&latest_report_lock, key);
	hids_tx_cancel(NULL);
	k_mutex_unlock(&spi_report_generation_mutex);

	spi_diag_note_queue_depth(0);
	for (size_t i = 0; i < ARRAY_SIZE(release_frames); i++) {
		if (release_enqueued[i]) {
			spi_diag_note_enqueued(release_frames[i],
					       k_msgq_num_used_get(&spi_report_msgq));
		}
	}
	if (release_enqueued[0]) {
		spi_diag_note_resubmit(release_frames[0]);
	}
	if (queue_failed) {
		spi_diag_note_queue_full(k_msgq_num_used_get(&spi_report_msgq));
	}
	printk("SPI link timeout; queued subscribed BLE releases\n");

	/* SPI链路超时后，启动空闲关机计时器 */
	last_spi_activity_ms = k_uptime_get_32();
	(void)k_work_reschedule(&spi_link_idle_poweroff_work,
				K_MSEC(SPI_LINK_IDLE_POWEROFF_MS));
}

static void spi_activity_note(void)
{
	last_spi_activity_ms = k_uptime_get_32();
	/* The already scheduled worker compares this timestamp with its deadline
	 * and reschedules itself when activity is recent. */
}

static void spi_link_watchdog_feed(void)
{
	if (!wireless_mode_is_enabled() || !any_conn_active()) {
		return;
	}

	k_work_reschedule(&spi_link_watchdog_work,
			  K_MSEC(SPI_LINK_TIMEOUT_MS));
}

static uint8_t active_profile_bt_id(void)
{
	if (active_profile < PROFILE_COUNT) {
		return profile_bt_id[active_profile];
	}

	return BT_ID_DEFAULT;
}

static void accept_list_bond_cb(const struct bt_bond_info *info, void *user_data)
{
	int *added = user_data;

	if (bt_le_filter_accept_list_add(&info->addr) == 0) {
		(*added)++;
		if (active_profile < PROFILE_COUNT &&
		    !addr_equal(&profiles[active_profile].addr, &info->addr)) {
			profiles[active_profile].addr = info->addr;
		}
	}
}

static int advertising_configure_accept_list(bool pairing_enabled)
{
	int err;
	int added = 0;

	err = bt_le_filter_accept_list_clear();
	if (err && err != -EALREADY) {
		printk("accept_list_clear failed: %d\n", err);
		return err;
	}

	if (pairing_enabled || active_profile >= PROFILE_COUNT ||
	    !profiles[active_profile].valid) {
		return 0;
	}

	bt_foreach_bond(active_profile_bt_id(), accept_list_bond_cb, &added);
	if (added > 0) {
		return 0;
	}

	err = bt_le_filter_accept_list_add(&profiles[active_profile].addr);
	if (err) {
		char addr[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(&profiles[active_profile].addr, addr, sizeof(addr));
		printk("accept_list_add failed for %s: %d\n", addr, err);
		return err;
	}

	return 0;
}

static void profiles_clear(void)
{
	memset(profiles, 0, sizeof(profiles));
	active_profile = 0;
}

static bool profiles_any_valid(void)
{
	for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
		if (profiles[i].valid) {
			return true;
		}
	}
	return false;
}

static int ensure_profile_identities(void)
{
	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
	size_t count = ARRAY_SIZE(addrs);
	uint8_t slot = 0;

	bt_id_get(addrs, &count);

	for (uint8_t id = 0; id < count && slot < PROFILE_COUNT; id++) {
		if (bt_addr_le_cmp(&addrs[id], BT_ADDR_LE_ANY) != 0) {
			profile_bt_id[slot++] = id;
		}
	}

	while (slot < PROFILE_COUNT) {
		int new_id = bt_id_create(NULL, NULL);

		if (new_id < 0) {
			printk("bt_id_create failed: %d (need CONFIG_BT_ID_MAX >= %u)\n",
			       new_id, PROFILE_COUNT);
			return new_id;
		}

		profile_bt_id[slot++] = (uint8_t)new_id;
	}

	count = ARRAY_SIZE(addrs);
	bt_id_get(addrs, &count);

	for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
		char addr_str[BT_ADDR_LE_STR_LEN];
		uint8_t id = profile_bt_id[i];

		if (id < count) {
			bt_addr_le_to_str(&addrs[id], addr_str, sizeof(addr_str));
			printk("Profile %u uses BT ID %u addr %s\n",
			       (unsigned int)(i + 1), (unsigned int)id, addr_str);
		} else {
			printk("Profile %u uses BT ID %u\n",
			       (unsigned int)(i + 1), (unsigned int)id);
		}
	}

	return 0;
}

static bool profile_index_from_bt_id(uint8_t id, uint8_t *profile_idx)
{
	for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
		if (profile_bt_id[i] == id) {
			if (profile_idx) {
				*profile_idx = i;
			}
			return true;
		}
	}

	return false;
}

struct bond_iter_ctx {
	uint8_t slot;
	bool found;
};

static void bond_iter_cb(const struct bt_bond_info *info, void *user_data)
{
	struct bond_iter_ctx *ctx = user_data;

	if (ctx->slot >= PROFILE_COUNT || ctx->found) {
		return;
	}

	profiles[ctx->slot].valid = true;
	profiles[ctx->slot].addr = info->addr;
	ctx->found = true;
}

static void profiles_bootstrap_from_bonds(void)
{
	uint8_t restored = 0;

	if (profiles_any_valid()) {
		return;
	}

	for (uint8_t slot = 0; slot < PROFILE_COUNT; slot++) {
		struct bond_iter_ctx ctx = {
			.slot = slot,
		};

		bt_foreach_bond(profile_bt_id[slot], bond_iter_cb, &ctx);
		if (ctx.found) {
			restored++;
		}
	}

	if (restored > 0) {
		profiles_save();
		printk("Bootstrapped %u profile slot(s) from stored bonds\n",
		       (unsigned int)restored);
	}
}

static void profiles_save(void)
{
	struct profile_store store = { 0 };
	int err;

	store.version = PROFILE_VERSION;
	store.active_profile = active_profile;
	for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
		if (profiles[i].valid) {
			store.valid_mask |= BIT(i);
			store.addrs[i] = profiles[i].addr;
		}
	}

	err = settings_save_one(PROFILE_SETTINGS_KEY, &store, sizeof(store));
	if (err) {
		printk("profiles_save failed: %d\n", err);
	}
}

static int profiles_settings_set(const char *name, size_t len,
				 settings_read_cb read_cb, void *cb_arg)
{
	struct profile_store store;
	int err;

	if (strcmp(name, "profiles") != 0) {
		return -ENOENT;
	}
	if (len != sizeof(store)) {
		return -EINVAL;
	}

	err = read_cb(cb_arg, &store, sizeof(store));
	if (err < 0) {
		return err;
	}
	if (store.version != PROFILE_VERSION) {
		return -EINVAL;
	}

	profiles_clear();
	active_profile = (store.active_profile < PROFILE_COUNT) ? store.active_profile : 0;
	for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
		if (store.valid_mask & BIT(i)) {
			profiles[i].valid = true;
			profiles[i].addr = store.addrs[i];
		}
	}

	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(spi_hids_profiles, "spi_hids",
			       NULL, profiles_settings_set, NULL, NULL);

static void pairing_mode_timeout_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	pairing_mode = false;
	printk("Pairing mode timeout\n");
	if (active_profile >= PROFILE_COUNT || !profiles[active_profile].valid) {
		disconnect_all();
	}
	advertising_start();
}

static void disconnect_all(void)
{
	struct bt_conn *connections[CONFIG_BT_HIDS_MAX_CLIENT_COUNT] = { 0 };
	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn) {
			connections[i] = bt_conn_ref(conn_mode[i].conn);
		}
	}
	k_spin_unlock(&conn_mode_lock, key);

	for (size_t i = 0; i < ARRAY_SIZE(connections); i++) {
		if (connections[i]) {
			(void)bt_conn_disconnect(connections[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			bt_conn_unref(connections[i]);
		}
	}
}

static bool any_connected(void)
{
	return any_conn_active();
}

static void set_pairing_mode(bool enable)
{
	pairing_mode = enable;
	if (enable) {
		k_work_reschedule(&pairing_mode_timeout_work,
				  K_MSEC(SPI_PAIRING_TIMEOUT_MS));
		printk("Pairing mode ON\n");
	} else {
		(void)k_work_cancel_delayable(&pairing_mode_timeout_work);
		printk("Pairing mode OFF\n");
	}

	advertising_start();
}

static void advertising_start(void)
{
	int err;
	uint32_t adv_opts = BT_LE_ADV_OPT_CONN;
	uint32_t interval_min = slow_advertising ? BT_GAP_ADV_SLOW_INT_MIN :
						 BT_GAP_MS_TO_ADV_INTERVAL(20);
	uint32_t interval_max = slow_advertising ? BT_GAP_ADV_SLOW_INT_MAX :
						 BT_GAP_MS_TO_ADV_INTERVAL(30);
	struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
		adv_opts,
		interval_min,
		interval_max,
		NULL);

	if (!wireless_mode_is_enabled() || any_connected()) {
		advertising_start_retry_count = 0;
		return;
	}

	err = bt_le_adv_stop();
	if (err && err != -EALREADY) {
		printk("adv_stop failed: %d\n", err);
	}
	if (!pairing_mode && (active_profile >= PROFILE_COUNT ||
				      !profiles[active_profile].valid)) {
		printk("Advertising stopped: empty profile outside pairing window\n");
		advertising_start_retry_count = 0;
		return;
	}

	err = advertising_configure_accept_list(pairing_mode);
	if (err) {
		advertising_start_retry_schedule(err);
		return;
	}

	if (!pairing_mode && active_profile < PROFILE_COUNT &&
	    profiles[active_profile].valid) {
		adv_opts |= BT_LE_ADV_OPT_FILTER_CONN;
	}

	adv_param.id = active_profile_bt_id();
	adv_param.options = adv_opts;

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err == -EALREADY) {
		advertising_start_retry_count = 0;
		return;
	}
	if (err) {
		printk("Advertising start failed: %d\n", err);
		advertising_start_retry_schedule(err);
		return;
	}
	advertising_start_retry_count = 0;

	printk("Advertising started (%s, %s, profile %u, bt_id %u)\n",
	       pairing_mode ? "pairing" : "normal",
	       slow_advertising ? "slow" : "fast",
	       (unsigned int)(active_profile + 1),
	       (unsigned int)adv_param.id);
}

static void profile_switch_adv_refresh_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	advertising_start();
}

static void hids_outp_rep_handler(struct bt_hids_rep *rep, struct bt_conn *conn, bool write)
{
	char addr[BT_ADDR_LE_STR_LEN];
	uint8_t led_bits = 0;

	if (!write) {
		return;
	}

	if (rep && rep->data) {
		led_bits = *rep->data;
	}
	host_output_led_bits = led_bits;
	host_output_led_valid = true;
#if SPI_DEBUG_LED_SHOW_CAPS
	spi_debug_led_set((led_bits & OUTPUT_REPORT_CAPS_LOCK_MASK) != 0);
#endif

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Output report from %s, Num=%d Caps=%d Scr=%d raw=0x%02X\n",
	       addr,
	       (led_bits & OUTPUT_REPORT_NUM_LOCK_MASK) ? 1 : 0,
	       (led_bits & OUTPUT_REPORT_CAPS_LOCK_MASK) ? 1 : 0,
	       (led_bits & OUTPUT_REPORT_SCROLL_LOCK_MASK) ? 1 : 0,
	       led_bits);
}

static void hids_boot_kb_outp_rep_handler(struct bt_hids_rep *rep, struct bt_conn *conn, bool write)
{
	hids_outp_rep_handler(rep, conn, write);
}

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn)
{
	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn != conn) {
			continue;
		}

		if (evt == BT_HIDS_PM_EVT_BOOT_MODE_ENTERED) {
			conn_mode[i].in_boot_mode = true;
		} else if (evt == BT_HIDS_PM_EVT_REPORT_MODE_ENTERED) {
			conn_mode[i].in_boot_mode = false;
		}
		break;
	}
	k_spin_unlock(&conn_mode_lock, key);
	spi_report_latest_invalidate();
	spi_report_latest_resubmit();
}

static void hid_init(void)
{
	struct bt_hids_init_param hids_init_obj = { 0 };
	struct bt_hids_inp_rep *hids_inp_rep;
	struct bt_hids_outp_feat_rep *hids_outp_rep;

	static const uint8_t report_map[] = {
		0x05, 0x01,                         /* Usage Page (Generic Desktop) */
		0x09, 0x06,                         /* Usage (Keyboard) */
		0xA1, 0x01,                         /* Collection (Application) */
		0x85, INPUT_REP_KEYBOARD_REF_ID,    /* Report ID */
		0x05, 0x07,                         /* Usage Page (Keyboard/Keypad) */
		0x19, 0xE0,                         /* Usage Minimum (Left Control) */
		0x29, 0xE7,                         /* Usage Maximum (Right GUI) */
		0x15, 0x00,                         /* Logical Minimum (0) */
		0x25, 0x01,                         /* Logical Maximum (1) */
		0x75, 0x01,                         /* Report Size (1) */
		0x95, 0x08,                         /* Report Count (8 modifiers) */
		0x81, 0x02,                         /* Input (Data, Variable, Absolute) */
		0x05, 0x07,                         /* Usage Page (Keyboard/Keypad) */
		0x19, 0x00,                         /* Usage Minimum (0) */
		0x2A, 0xEF, 0x00,                   /* Usage Maximum (239) */
		0x15, 0x00,                         /* Logical Minimum (0) */
		0x25, 0x01,                         /* Logical Maximum (1) */
		0x75, 0x01,                         /* Report Size (1) */
		0x96, 0xF0, 0x00,                   /* Report Count (240 keys) */
		0x81, 0x02,                         /* Input (Data, Variable, Absolute) */
		0x85, OUTPUT_REP_KEYS_REF_ID,       /* Keyboard LED Report ID */
		0x05, 0x08,                         /* Usage Page (LEDs) */
		0x19, 0x01,                         /* Usage Minimum (Num Lock) */
		0x29, 0x05,                         /* Usage Maximum (Kana) */
		0x95, 0x05,                         /* Report Count (5) */
		0x75, 0x01,                         /* Report Size (1) */
		0x91, 0x02,                         /* Output (Data, Variable, Absolute) */
		0x95, 0x01,                         /* Report Count (1) */
		0x75, 0x03,                         /* Report Size (3) */
		0x91, 0x01,                         /* Output (Constant) */
		0xC0,                               /* End Keyboard Collection */

		0x05, 0x0C,                         /* Usage Page (Consumer) */
		0x09, 0x01,                         /* Usage (Consumer Control) */
		0xA1, 0x01,                         /* Collection (Application) */
		0x85, INPUT_REP_CONSUMER_REF_ID,    /* Report ID */
		0x19, 0x00,                         /* Usage Minimum (0) */
		0x2A, 0xA0, 0x02,                   /* Usage Maximum (0x02A0) */
		0x15, 0x00,                         /* Logical Minimum (0) */
		0x26, 0xA0, 0x02,                   /* Logical Maximum (0x02A0) */
		0x75, 0x10,                         /* Report Size (16) */
		0x95, 0x01,                         /* Report Count (1) */
		0x81, 0x00,                         /* Input (Data, Array, Absolute) */
		0xC0,                               /* End Consumer Collection */

		0x05, 0x01,                         /* Usage Page (Generic Desktop) */
		0x09, 0x80,                         /* Usage (System Control) */
		0xA1, 0x01,                         /* Collection (Application) */
		0x85, INPUT_REP_SYSTEM_REF_ID,      /* Report ID */
		0x19, 0x00,                         /* Usage Minimum (0) */
		0x2A, 0xB7, 0x00,                   /* Usage Maximum (0x00B7) */
		0x15, 0x00,                         /* Logical Minimum (0) */
		0x26, 0xB7, 0x00,                   /* Logical Maximum (0x00B7) */
		0x75, 0x10,                         /* Report Size (16) */
		0x95, 0x01,                         /* Report Count (1) */
		0x81, 0x00,                         /* Input (Data, Array, Absolute) */
		0xC0                                /* End System Collection */
	};

	hids_init_obj.rep_map.data = report_map;
	hids_init_obj.rep_map.size = sizeof(report_map);
	hids_init_obj.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_obj.info.b_country_code = 0x00;
	hids_init_obj.info.flags = BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE;

	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_KEYBOARD_INDEX];
	hids_inp_rep->size = INPUT_REPORT_KEYBOARD_LEN;
	hids_inp_rep->id = INPUT_REP_KEYBOARD_REF_ID;
	hids_inp_rep->handler = hids_report_keyboard_notify_handler;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_CONSUMER_INDEX];
	hids_inp_rep->size = INPUT_REPORT_CONSUMER_LEN;
	hids_inp_rep->id = INPUT_REP_CONSUMER_REF_ID;
	hids_inp_rep->handler_ext = hids_input_notify_handler;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_SYSTEM_INDEX];
	hids_inp_rep->size = INPUT_REPORT_SYSTEM_LEN;
	hids_inp_rep->id = INPUT_REP_SYSTEM_REF_ID;
	hids_inp_rep->handler_ext = hids_input_notify_handler;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_outp_rep = &hids_init_obj.outp_rep_group_init.reports[0];
	hids_outp_rep->size = OUTPUT_REPORT_MAX_LEN;
	hids_outp_rep->id = OUTPUT_REP_KEYS_REF_ID;
	hids_outp_rep->handler = hids_outp_rep_handler;
	hids_init_obj.outp_rep_group_init.cnt++;

	hids_init_obj.is_kb = true;
	hids_init_obj.boot_kb_notif_handler = hids_boot_keyboard_notify_handler;
	hids_init_obj.boot_kb_outp_rep_handler = hids_boot_kb_outp_rep_handler;
	hids_init_obj.pm_evt_handler = hids_pm_evt_handler;

	if (bt_hids_init(&hids_obj, &hids_init_obj)) {
		printk("HIDS init failed\n");
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int ret;
	struct bt_conn_info info;
	uint8_t connection_profile;
	const bt_addr_le_t *peer = bt_conn_get_dst(conn);

	bt_addr_le_to_str(peer, addr, sizeof(addr));
	if (err) {
		printk("Connect failed: %s err 0x%02x\n", addr, err);
		return;
	}

	printk("Connected: %s\n", addr);
	ret = bt_conn_get_info(conn, &info);
	if (ret || !profile_index_from_bt_id(info.id, &connection_profile) ||
	    connection_profile != active_profile) {
		printk("Rejecting connection for inactive identity (id=%u active_profile=%u)\n",
		       ret ? UINT8_MAX : info.id, (unsigned int)(active_profile + 1));
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	spi_report_state_clear(true);
	atomic_inc(&hids_conn_generation);

	host_output_led_reset();
	profile_switch_led_until_ms = 0;
	connected_profile_led_until_ms = k_uptime_get_32() + SPI_CONNECTED_LED_HOLD_MS;

	ret = bt_hids_connected(&hids_obj, conn);
	if (ret) {
		printk("bt_hids_connected failed: %d\n", ret);
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (!conn_mode[i].conn) {
			conn_mode[i].conn = bt_conn_ref(conn);
			conn_mode[i].in_boot_mode = false;
			conn_mode[i].profile_idx = connection_profile;
			conn_mode[i].profile_idx_valid = true;
			conn_mode[i].peer_identity_valid = false;
			break;
		}
	}
	k_spin_unlock(&conn_mode_lock, key);
	connected_idle_timer_start();

	if (bt_addr_le_is_identity(peer)) {
		conn_mode_set_peer_identity(conn, peer);
		maybe_exit_pairing_for_peer(peer, "reconnected directly");
	}

	ret = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (ret) {
		printk("Set security failed: %d\n", ret);
	}

	if (!wireless_mode_is_enabled()) {
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	spi_report_latest_resubmit();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int ret;
	bool tracked = false;
	struct bt_conn *tracked_conn = NULL;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected: %s reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));
	hids_tx_cancel(conn);
	atomic_inc(&hids_conn_generation);

	k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			tracked = true;
			tracked_conn = conn_mode[i].conn;
			conn_mode[i].conn = NULL;
			conn_mode[i].in_boot_mode = false;
			conn_mode[i].profile_idx_valid = false;
			conn_mode[i].peer_identity_valid = false;
		}
	}
	k_spin_unlock(&conn_mode_lock, key);
	if (tracked_conn) {
		bt_conn_unref(tracked_conn);
	}

	if (tracked) {
		spi_diag_note_disconnect();
		ret = bt_hids_disconnected(&hids_obj, conn);
		if (ret) {
			printk("bt_hids_disconnected failed: %d\n", ret);
		}
	}

	if (!any_conn_active()) {
		host_output_led_reset();
		connected_profile_led_until_ms = 0;
		spi_report_state_clear(true);
		k_sem_give(&ble_disconnected_sem);
		if (wireless_mode_is_enabled()) {
			disconnected_idle_timers_start();
		} else if (!usb_host_is_present()) {
			hostless_poweroff_timer_ensure();
		}
	}

	schedule_advertising_refresh(SPI_ADV_REFRESH_DELAY_MS);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	const bt_addr_le_t *peer = bt_conn_get_dst(conn);

	bt_addr_le_to_str(peer, addr, sizeof(addr));
	if (!err && level >= BT_SECURITY_L2) {
		printk("Security changed: %s level %u\n", addr, level);
		spi_report_latest_resubmit();
	} else if (!err) {
		printk("Security changed: %s level %u (waiting for level 2)\n",
		       addr, level);
	} else {
		printk("Security failed: %s level %u err %d %s\n",
		       addr, level, err, bt_security_err_to_str(err));
		if (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING ||
		    err == BT_SECURITY_ERR_AUTH_FAIL) {
			if (pairing_mode && peer) {
				(void)bt_unpair(active_profile_bt_id(), peer);
			}
			(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
	}
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	spi_diag_note_conn_params(interval, latency, timeout);
	uint32_t interval_us = (uint32_t)interval * 1250U;

	printk("Connection parameters: interval=%u.%03u ms latency=%u timeout=%u ms\n",
	       (unsigned int)(interval_us / 1000U),
	       (unsigned int)(interval_us % 1000U),
	       (unsigned int)latency, (unsigned int)timeout * 10U);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t stored_peer;
	uint8_t connection_profile;
	const bt_addr_le_t *peer = bt_conn_get_dst(conn);
	bool stored_peer_valid = conn_mode_peer_identity_get(conn, &stored_peer);

	bt_addr_le_to_str(peer, addr, sizeof(addr));
	printk("Pairing complete: %s bonded=%u\n", addr, bonded ? 1 : 0);

	if (!bonded) {
		return;
	}
	if (!conn_mode_profile_index_get(conn, &connection_profile)) {
		printk("Pairing completed without a profile identity\n");
		return;
	}

	profiles[connection_profile].valid = true;
	if (stored_peer_valid) {
		profiles[connection_profile].addr = stored_peer;
	} else {
		profiles[connection_profile].addr = *peer;
	}
	profiles_save();
	if (connection_profile == active_profile) {
		set_pairing_mode(false);
	}
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing failed: %s reason=%d %s\n", addr, reason,
	       bt_security_err_to_str(reason));
}

static void bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bool changed = false;
	bt_addr_le_to_str(peer, addr, sizeof(addr));

	for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
		if (profile_bt_id[i] != id) {
			continue;
		}

		if (profiles[i].valid) {
			profiles[i].valid = false;
			changed = true;
		}
	}

	if (changed) {
		profiles_save();
		printk("Bond deleted: %s (profile id %u slot cleared)\n",
		       addr, (unsigned int)id);
	}
}

#if defined(CONFIG_BT_SMP)
static void identity_resolved(struct bt_conn *conn,
			      const bt_addr_le_t *rpa,
			      const bt_addr_le_t *identity)
{
	char rpa_str[BT_ADDR_LE_STR_LEN];
	char id_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(rpa, rpa_str, sizeof(rpa_str));
	bt_addr_le_to_str(identity, id_str, sizeof(id_str));
	printk("Identity resolved: %s -> %s\n", rpa_str, id_str);

	conn_mode_set_peer_identity(conn, identity);
	maybe_exit_pairing_for_peer(identity, "reconnected from bonded peer");
}
#endif

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
	.bond_deleted = bond_deleted,
};

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_SMP)
	.identity_resolved = identity_resolved,
#endif
};

static void keyboard_boot_to_nkro(const uint8_t boot[SPI_BRIDGE_BOOT_REPORT_LEN],
				  uint8_t nkro[SPI_BRIDGE_NKRO_REPORT_LEN])
{
	memset(nkro, 0, SPI_BRIDGE_NKRO_REPORT_LEN);
	nkro[0] = boot[SPI_BRIDGE_BOOT_REPORT_MODS_INDEX];

	for (uint8_t i = 0; i < SPI_BRIDGE_BOOT_REPORT_KEYS_LEN; i++) {
		uint8_t usage = boot[SPI_BRIDGE_BOOT_REPORT_KEYS_INDEX + i];

		/* Boot usage 0 is No Event; 1..3 are rollover/error codes. */
		if (usage >= 4u && usage < (SPI_BRIDGE_NKRO_BITS_LEN * 8u)) {
			nkro[1u + (usage >> 3)] |= BIT(usage & 0x07u);
		}
	}
}

static void keyboard_nkro_to_boot(const uint8_t nkro[SPI_BRIDGE_NKRO_REPORT_LEN],
				  uint8_t boot[SPI_BRIDGE_BOOT_REPORT_LEN])
{
	uint8_t key_count = 0;

	memset(boot, 0, SPI_BRIDGE_BOOT_REPORT_LEN);
	boot[SPI_BRIDGE_BOOT_REPORT_MODS_INDEX] = nkro[0];

	for (uint16_t usage = 4u; usage < (SPI_BRIDGE_NKRO_BITS_LEN * 8u) &&
					 key_count < SPI_BRIDGE_BOOT_REPORT_KEYS_LEN; usage++) {
		if ((nkro[1u + (usage >> 3)] & BIT(usage & 0x07u)) != 0u) {
			boot[SPI_BRIDGE_BOOT_REPORT_KEYS_INDEX + key_count++] = (uint8_t)usage;
		}
	}
}

static bool hids_tx_matches_locked(struct bt_conn *conn, uint32_t token)
{
	return token != 0u && hids_tx_state.token == token && hids_tx_state.conn == conn;
}

static void hids_tx_state_reset_locked(void)
{
	hids_tx_state.conn = NULL;
	hids_tx_state.token = 0u;
	hids_tx_state.conn_generation = 0u;
	hids_tx_state.report_epoch = 0u;
	hids_tx_state.submit_in_progress = false;
	hids_tx_state.completion_early = false;
	hids_tx_state.in_flight = false;
	hids_tx_state.completion_received = false;
}

static void hids_tx_cancel(struct bt_conn *conn)
{
	bool wake = false;
	k_spinlock_key_t key = k_spin_lock(&hids_tx_lock);

	if (hids_tx_state.token != 0u && (!conn || hids_tx_state.conn == conn)) {
		if (hids_tx_state.completion_received) {
			wake = true;
		} else {
			wake = hids_tx_state.submit_in_progress || hids_tx_state.in_flight;
			hids_tx_state_reset_locked();
		}
	}
	k_spin_unlock(&hids_tx_lock, key);
	if (wake) {
		k_sem_give(&hids_tx_complete_sem);
	}
}

static int hids_tx_begin(struct bt_conn *conn, uint32_t report_epoch, uint32_t *token)
{
	k_sem_reset(&hids_tx_complete_sem);
	k_spinlock_key_t key = k_spin_lock(&hids_tx_lock);

	if ((uint32_t)atomic_get(&spi_report_state_epoch) != report_epoch) {
		k_spin_unlock(&hids_tx_lock, key);
		return -ECANCELED;
	}
	if (hids_tx_state.submit_in_progress || hids_tx_state.in_flight ||
	    hids_tx_state.completion_received) {
		k_spin_unlock(&hids_tx_lock, key);
		return -EBUSY;
	}

	hids_tx_next_token++;
	if (hids_tx_next_token == 0u) {
		hids_tx_next_token++;
	}
	hids_tx_state.conn = conn;
	hids_tx_state.token = hids_tx_next_token;
	hids_tx_state.conn_generation = (uint32_t)atomic_get(&hids_conn_generation);
	hids_tx_state.report_epoch = report_epoch;
	hids_tx_state.submit_in_progress = true;
	hids_tx_state.completion_early = false;
	hids_tx_state.in_flight = false;
	hids_tx_state.completion_received = false;
	*token = hids_tx_state.token;
	k_spin_unlock(&hids_tx_lock, key);
	return 0;
}

static void hids_tx_abort(struct bt_conn *conn, uint32_t token)
{
	k_spinlock_key_t key = k_spin_lock(&hids_tx_lock);

	if (hids_tx_matches_locked(conn, token)) {
		hids_tx_state_reset_locked();
	}
	k_spin_unlock(&hids_tx_lock, key);
}

static int hids_tx_mark_submitted(struct bt_conn *conn, uint32_t token)
{
	int result;
	k_spinlock_key_t key = k_spin_lock(&hids_tx_lock);

	if (!hids_tx_matches_locked(conn, token) ||
	    hids_tx_state.conn_generation != (uint32_t)atomic_get(&hids_conn_generation) ||
	    hids_tx_state.report_epoch != (uint32_t)atomic_get(&spi_report_state_epoch)) {
		result = -ECANCELED;
	} else {
		hids_tx_state.submit_in_progress = false;
		if (hids_tx_state.completion_early) {
			hids_tx_state.completion_received = true;
			result = 1;
		} else {
			hids_tx_state.in_flight = true;
			result = 0;
		}
	}
	k_spin_unlock(&hids_tx_lock, key);
	return result;
}

static int hids_tx_finish(struct bt_conn *conn, uint32_t token, k_timeout_t timeout)
{
	int wait_err = k_sem_take(&hids_tx_complete_sem, timeout);
	int result;
	k_spinlock_key_t key = k_spin_lock(&hids_tx_lock);

	if (hids_tx_matches_locked(conn, token) && hids_tx_state.completion_received &&
	    hids_tx_state.conn_generation == (uint32_t)atomic_get(&hids_conn_generation) &&
	    hids_tx_state.report_epoch == (uint32_t)atomic_get(&spi_report_state_epoch)) {
		hids_tx_state_reset_locked();
		result = 0;
	} else if (hids_tx_matches_locked(conn, token) &&
		   hids_tx_state.completion_received) {
		hids_tx_state_reset_locked();
		result = -ECANCELED;
	} else if (!wait_err && !hids_tx_matches_locked(conn, token)) {
		result = -ECANCELED;
	} else if (hids_tx_matches_locked(conn, token)) {
		hids_tx_state_reset_locked();
		result = -ETIMEDOUT;
	} else {
		result = -ECANCELED;
	}
	k_spin_unlock(&hids_tx_lock, key);
	return result;
}

static void hids_notify_complete(struct bt_conn *conn, void *user_data)
{
	uint32_t token = (uint32_t)(uintptr_t)user_data;
	bool stale = false;
	bool wake = false;
	k_spinlock_key_t key = k_spin_lock(&hids_tx_lock);

	if (!hids_tx_matches_locked(conn, token) ||
	    hids_tx_state.conn_generation != (uint32_t)atomic_get(&hids_conn_generation) ||
	    hids_tx_state.report_epoch != (uint32_t)atomic_get(&spi_report_state_epoch)) {
		stale = true;
	} else if (hids_tx_state.submit_in_progress) {
		hids_tx_state.completion_early = true;
	} else if (hids_tx_state.in_flight && !hids_tx_state.completion_received) {
		hids_tx_state.in_flight = false;
		hids_tx_state.completion_received = true;
		wake = true;
	} else {
		stale = true;
	}
	k_spin_unlock(&hids_tx_lock, key);

	if (stale) {
		spi_diag_note_stale_completion();
	}
	if (wake) {
		k_sem_give(&hids_tx_complete_sem);
	}
}

static int hids_boot_kb_inp_rep_send_userdata(struct bt_conn *conn,
						      const uint8_t *report, uint16_t len,
						      bt_gatt_complete_func_t callback,
						      void *user_data)
{
	uint8_t report_index = hids_obj.boot_kb_inp_rep.att_ind;
	struct bt_gatt_attr *report_attr = &hids_obj.gp.svc.attrs[report_index];

	if (!bt_gatt_is_subscribed(conn, report_attr, BT_GATT_CCC_NOTIFY)) {
		return -EACCES;
	}

	struct bt_hids_conn_data *conn_data = bt_conn_ctx_get(hids_obj.conn_ctx, conn);

	if (!conn_data) {
		return -EINVAL;
	}
	if (len > sizeof(conn_data->hids_boot_kb_inp_rep_ctx)) {
		bt_conn_ctx_release(hids_obj.conn_ctx, conn_data);
		return -EINVAL;
	}

	uint8_t *report_data = conn_data->hids_boot_kb_inp_rep_ctx;

	memcpy(report_data, report, len);
	memset(&report_data[len], 0, sizeof(conn_data->hids_boot_kb_inp_rep_ctx) - len);
	struct bt_gatt_notify_params params = {
		.attr = report_attr,
		.data = report_data,
		.len = sizeof(conn_data->hids_boot_kb_inp_rep_ctx),
		.func = callback,
		.user_data = user_data,
	};
	int err = bt_gatt_notify_cb(conn, &params);

	bt_conn_ctx_release(hids_obj.conn_ctx, conn_data);
	return err;
}

static int bridge_report_con_send(const uint8_t frame[SPI_BRIDGE_FRAME_LEN], bool boot_mode,
					  struct bt_conn *conn, bool *notification_pending,
					  void *user_data)
{
	const uint8_t *payload;
	uint8_t payload_len;
	uint8_t report_type;

	*notification_pending = false;

	if (!spi_bridge_report_frame_read(frame, &report_type, NULL, &payload, &payload_len)) {
		return -EINVAL;
	}

	switch (report_type) {
	case SPI_BRIDGE_REPORT_KEYBOARD_BOOT:
		*notification_pending = true;
		if (boot_mode) {
			return hids_boot_kb_inp_rep_send_userdata(conn, payload, payload_len,
								       hids_notify_complete, user_data);
		} else {
			uint8_t nkro[SPI_BRIDGE_NKRO_REPORT_LEN];

			keyboard_boot_to_nkro(payload, nkro);
			return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
				INPUT_REP_KEYBOARD_INDEX, nkro, sizeof(nkro),
				hids_notify_complete, user_data);
		}

	case SPI_BRIDGE_REPORT_KEYBOARD_NKRO:
		*notification_pending = true;
		if (boot_mode) {
			uint8_t boot[SPI_BRIDGE_BOOT_REPORT_LEN];

			keyboard_nkro_to_boot(payload, boot);
			return hids_boot_kb_inp_rep_send_userdata(conn, boot, sizeof(boot),
								       hids_notify_complete, user_data);
		}
		return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
			INPUT_REP_KEYBOARD_INDEX, payload, payload_len,
			hids_notify_complete, user_data);

	case SPI_BRIDGE_REPORT_CONSUMER:
		if (boot_mode) {
			return 0;
		}
		*notification_pending = true;
		return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
			INPUT_REP_CONSUMER_INDEX, payload, payload_len,
			hids_notify_complete, user_data);

	case SPI_BRIDGE_REPORT_SYSTEM:
		if (boot_mode) {
			return 0;
		}
		*notification_pending = true;
		return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
			INPUT_REP_SYSTEM_INDEX, payload, payload_len,
			hids_notify_complete, user_data);

	default:
		return -EINVAL;
	}
}

static int bridge_report_con_send_wait(const uint8_t frame[SPI_BRIDGE_FRAME_LEN],
					       bool boot_mode, struct bt_conn *conn,
					       uint32_t report_epoch)
{
	bool notification_pending = false;
	uint32_t token;
	int err;

	k_mutex_lock(&spi_report_generation_mutex, K_FOREVER);
	err = hids_tx_begin(conn, report_epoch, &token);
	if (err) {
		k_mutex_unlock(&spi_report_generation_mutex);
		return err;
	}

	err = bridge_report_con_send(frame, boot_mode, conn, &notification_pending,
				     (void *)(uintptr_t)token);
	k_mutex_unlock(&spi_report_generation_mutex);
	if (notification_pending) {
		spi_diag_note_send_attempted(frame);
	}
	if (err || !notification_pending) {
		hids_tx_abort(conn, token);
		return err;
	}

	spi_diag_note_send_submitted(frame);
	int submit_state = hids_tx_mark_submitted(conn, token);

	if (submit_state < 0) {
		hids_tx_abort(conn, token);
		return submit_state;
	}
	if (submit_state > 0) {
		err = hids_tx_finish(conn, token, K_NO_WAIT);
	} else {
		err = hids_tx_finish(conn, token,
				     K_MSEC(HIDS_NOTIFY_COMPLETE_TIMEOUT_MS));
	}
	if (!err) {
		spi_diag_note_send_completed(frame);
	}
	return err;
}

static int bridge_report_send_all(const uint8_t frame[SPI_BRIDGE_FRAME_LEN],
				  uint32_t report_epoch)
{
	bool sent = false;
	int result = 0;

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		int err;
		struct bt_conn *conn = NULL;
		bool boot_mode = false;
		k_spinlock_key_t key = k_spin_lock(&conn_mode_lock);

		if (conn_mode[i].conn) {
			conn = bt_conn_ref(conn_mode[i].conn);
			boot_mode = conn_mode[i].in_boot_mode;
		}
		k_spin_unlock(&conn_mode_lock, key);

		if (!conn) {
			continue;
		}

		sent = true;
		err = bridge_report_con_send_wait(frame, boot_mode, conn, report_epoch);
		bt_conn_unref(conn);
		if (err) {
			result = err;
			break;
		}
	}

	return result ? result : sent ? 0 : -ENOTCONN;
}

static void spi_report_recover_disconnect(void)
{
	spi_diag_print("recover");
	/* Drop the absolute states that could otherwise replay a stale key-down
	 * while the ATT bearer is being torn down. */
	k_sem_reset(&ble_disconnected_sem);
	spi_report_state_clear(true);

	if (!any_conn_active()) {
		return;
	}

	disconnect_all();
	if (k_sem_take(&ble_disconnected_sem,
		       K_MSEC(HIDS_RECOVERY_DISCONNECT_TIMEOUT_MS)) != 0) {
		printk("HIDS recovery disconnect timed out\n");
		spi_report_state_clear(true);
	}
}

static void spi_report_thread_fn(void *arg1, void *arg2, void *arg3)
{
	uint8_t report[SPI_BRIDGE_FRAME_LEN];

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		if (k_msgq_get(&spi_report_msgq, report, K_FOREVER) != 0) {
			continue;
		}
		watchdog_manager_feed();
		uint32_t report_epoch = spi_report_epoch_get(report);

		spi_diag_note_dequeued(report, k_msgq_num_used_get(&spi_report_msgq));
		if ((uint32_t)atomic_get(&spi_report_state_epoch) != report_epoch) {
			continue;
		}
		if (report[SPI_REPORT_LOCAL_RELEASE_SLOT_INDEX] !=
		    SPI_RELEASE_SLOT_NONE) {
			k_spinlock_key_t key = k_spin_lock(&latest_report_lock);
			bool current_release =
				spi_report_release_tag_matches_locked(report, report_epoch);
			k_spin_unlock(&latest_report_lock, key);
			if (!current_release) {
				continue;
			}
		}
		if (!wireless_mode_is_enabled()) {
			spi_report_hold_release(report, report_epoch);
			continue;
		}

		uint32_t retry_started_ms = k_uptime_get_32();

		while (1) {
			int ret;

			if ((uint32_t)atomic_get(&spi_report_state_epoch) != report_epoch) {
				break;
			}
			if (!wireless_mode_is_enabled()) {
				spi_report_hold_release(report, report_epoch);
				break;
			}

			ret = bridge_report_send_all(report, report_epoch);
				if (ret == 0) {
					spi_diag_note_completed();
					spi_report_release_complete(report, report_epoch);
					/* A completion can free the only queue slot that was
					 * blocking a persistent release of another HIDS type. */
					spi_report_release_resubmit();
					break;
				}
			spi_diag_note_hid_error(report, ret);
			if (ret == -ENOTCONN || ret == -EACCES || ret == -EPERM) {
				spi_report_hold_release(report, report_epoch);
				break;
			}
			if (ret == -ETIMEDOUT) {
				spi_diag_note_notify_timeout();
				spi_diag_note_inflight_timeout();
				printk("HIDS notification timed out after %u ms; disconnecting\n",
				       HIDS_NOTIFY_COMPLETE_TIMEOUT_MS);
				spi_report_recover_disconnect();
				break;
			}

			if (ret == -ENOMEM || ret == -EAGAIN) {
				if ((uint32_t)(k_uptime_get_32() - retry_started_ms) >=
				    SPI_REPORT_RETRY_TIMEOUT_MS) {
					spi_diag_note_send_stall();
					printk("HIDS send stalled for %u ms (%d); disconnecting\n",
					       SPI_REPORT_RETRY_TIMEOUT_MS, ret);
					spi_report_recover_disconnect();
					break;
				}
				/* These errors occur before ATT accepts the notification, so a
				 * bounded retry cannot duplicate an already-sent key state. */
				spi_diag_note_retry();
				k_sleep(K_MSEC(SPI_REPORT_RETRY_DELAY_MS));
				continue;
			}

			printk("HIDS send error: %d\n", ret);
			spi_report_hold_release(report, report_epoch);
			break;
		}
	}
}

static void wireless_mode_set(bool enabled)
{
	bool was_enabled = wireless_mode_is_enabled();

	atomic_set(&wireless_mode_enabled, enabled ? 1 : 0);
	if (was_enabled != enabled) {
		advertising_start_retry_count = 0;
		spi_report_state_clear(true);
		printk(enabled ? "Transport mode: wireless\n" :
				 "Transport mode: wired (Bluetooth idle)\n");
		/* The SPIS loop only accepts the desired route. HCI advertising and
		 * disconnect calls run in deferred work so the next SPI
		 * transaction is armed immediately. */
		(void)k_work_submit(&mode_apply_work);
	}
}

static void mode_apply_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!wireless_mode_is_enabled()) {
		pairing_mode = false;
		(void)k_work_cancel_delayable(&pairing_mode_timeout_work);
		radio_idle_timers_cancel();
		(void)k_work_cancel_delayable(&spi_link_watchdog_work);
		atomic_set(&spi_link_timed_out, 0);
		(void)k_work_cancel_delayable(&profile_switch_adv_refresh_work);

		int err = bt_le_adv_stop();

		if (err && err != -EALREADY) {
			printk("Advertising stop failed: %d\n", err);
		}
		if (wireless_mode_is_enabled()) {
			(void)k_work_submit(&mode_apply_work);
			return;
		}
		/* Disconnecting the link releases the old BLE HID state.  Never wait for
		 * notification completion during transport handoff. */
		if (any_conn_active()) {
			disconnect_all();
		}
		if (!usb_host_is_present()) {
			hostless_poweroff_timer_ensure();
		}
		return;
	}

	if (any_conn_active()) {
		connected_idle_timer_start();
	} else if (usb_host_is_present()) {
		(void)k_work_cancel_delayable(&slow_advertising_work);
		atomic_set(&hostless_timer_valid, 0);
	} else {
		disconnected_idle_timers_start();
	}

	if (!wireless_mode_is_enabled()) {
		(void)k_work_submit(&mode_apply_work);
		return;
	}
	if (active_profile < PROFILE_COUNT && !profiles[active_profile].valid &&
	    !pairing_mode) {
		set_pairing_mode(true);
	} else {
		if (!any_conn_active() && !deadline_is_active(profile_switch_led_until_ms)) {
			profile_switch_led_until_ms =
				k_uptime_get_32() + SPI_PROFILE_SWITCH_LED_HOLD_MS;
		}
		advertising_start();
	}
	if (!wireless_mode_is_enabled()) {
		(void)k_work_submit(&mode_apply_work);
		return;
	}

	last_spi_activity_ms = k_uptime_get_32();
	if (!usb_host_is_present()) {
		(void)k_work_reschedule(&spi_link_idle_poweroff_work,
					K_MSEC(SPI_LINK_IDLE_POWEROFF_MS));
	}
}

static void ble_prepare_shutdown(void)
{
	atomic_set(&wireless_mode_enabled, 0);
	pairing_mode = false;
	radio_idle_timers_cancel();
	atomic_set(&hostless_timer_valid, 0);
	(void)k_work_cancel_delayable(&pairing_mode_timeout_work);
	(void)k_work_cancel_delayable(&spi_link_watchdog_work);
	(void)k_work_cancel_delayable(&profile_switch_adv_refresh_work);
	spi_report_state_clear(true);
	(void)bt_le_adv_stop();
	if (any_conn_active()) {
		disconnect_all();
	}
}

static void select_profile(uint8_t profile_idx)
{
	if (profile_idx >= PROFILE_COUNT) {
		return;
	}

	slow_advertising = false;
	if (profile_idx == active_profile && any_conn_active()) {
		connected_profile_led_until_ms =
			k_uptime_get_32() + SPI_CONNECTED_LED_HOLD_MS;
		printk("Active profile %u already connected\n",
		       (unsigned int)(active_profile + 1));
		return;
	}

	bool had_conn = any_conn_active();

	spi_report_state_clear(true);
	active_profile = profile_idx;
	host_output_led_reset();
	profile_switch_led_until_ms = 0;
	connected_profile_led_until_ms = 0;
	profiles_save();
	printk("Active profile -> %u%s\n", (unsigned int)(active_profile + 1),
	       profiles[active_profile].valid ? "" : " (empty)");

	if (profiles[active_profile].valid) {
		profile_switch_led_until_ms =
			k_uptime_get_32() + SPI_PROFILE_SWITCH_LED_HOLD_MS;
		pairing_mode = false;
		(void)k_work_cancel_delayable(&pairing_mode_timeout_work);
		printk("Pairing mode OFF\n");
	} else {
		pairing_mode = true;
		k_work_reschedule(&pairing_mode_timeout_work,
				  K_MSEC(SPI_PAIRING_TIMEOUT_MS));
		printk("Pairing mode ON\n");
	}

	if (had_conn) {
		disconnect_all();
		schedule_advertising_refresh(SPI_PROFILE_SWITCH_ADV_DELAY_MS);
	} else {
		advertising_start();
	}
}

static void clear_active_profile_bond(void)
{
	slow_advertising = false;
	disconnect_all();
	(void)bt_unpair(active_profile_bt_id(), BT_ADDR_LE_ANY);
	spi_report_state_clear(true);
	profiles[active_profile].valid = false;
	host_output_led_reset();
	profile_switch_led_until_ms = 0;
	connected_profile_led_until_ms = 0;
	profiles_save();
	printk("Cleared bond in profile %u\n", (unsigned int)(active_profile + 1));
	set_pairing_mode(true);
	if (any_conn_active()) {
		schedule_advertising_refresh(SPI_ADV_REFRESH_DELAY_MS);
	}
}

static void spi_control_work_fn(struct k_work *work)
{
	uint8_t cmd;

	ARG_UNUSED(work);

	while (k_msgq_get(&spi_control_msgq, &cmd, K_NO_WAIT) == 0) {
		if (cmd >= SPI_BRIDGE_CTRL_PROFILE_1 &&
		    cmd <= SPI_BRIDGE_CTRL_PROFILE_LAST) {
			select_profile(cmd - SPI_BRIDGE_CTRL_PROFILE_1);
			continue;
		}

		switch (cmd) {
		case SPI_BRIDGE_CTRL_CLEAR_ACTIVE:
			clear_active_profile_bond();
			break;
		case SPI_BRIDGE_CTRL_DISCONNECT:
		case SPI_BRIDGE_CTRL_CLEAR_ALL:
		default:
			break;
		}
	}
}

static bool spi_control_defer(uint8_t cmd)
{
	if (k_msgq_put(&spi_control_msgq, &cmd, K_NO_WAIT) != 0) {
		printk("SPI control queue full, retry cmd 0x%02X\n", cmd);
		return false;
	}

	(void)k_work_submit(&spi_control_work);
	return true;
}

static bool handle_spi_control(uint8_t cmd)
{
	if (cmd >= SPI_BRIDGE_CTRL_PROFILE_1 && cmd <= SPI_BRIDGE_CTRL_PROFILE_LAST) {
		return spi_control_defer(cmd);
	}

	switch (cmd) {
	case SPI_BRIDGE_CTRL_USER_ACTIVITY:
		if (wireless_mode_is_enabled()) {
			low_power_note_activity();
		} else {
			hostless_poweroff_timer_restart();
		}
		break;
	case SPI_BRIDGE_CTRL_CLEAR_ACTIVE:
		return spi_control_defer(cmd);
	case SPI_BRIDGE_CTRL_DISCONNECT:
	case SPI_BRIDGE_CTRL_CLEAR_ALL:
		break;
	case SPI_BRIDGE_CTRL_USB_ABSENT:
		atomic_set(&usb_host_present, 0);
		atomic_set(&usb_presence_known, 1);
		/* Transport choice and physical host presence are independent.  Wired
		 * standby without a USB host must still reach System OFF. */
		hostless_poweroff_timer_ensure();
		if (wireless_mode_is_enabled()) {
			/* Presence is refreshed every five seconds.  Do not restart the
			 * advertising-idle window on each refresh, otherwise a radio that
			 * already reached slow advertising would be pulled back to fast
			 * advertising forever. */
			if (!any_conn_active() && !slow_advertising &&
			    !k_work_delayable_is_pending(&slow_advertising_work)) {
				disconnected_idle_timers_start();
			}
			last_spi_activity_ms = k_uptime_get_32();
			(void)k_work_reschedule(&spi_link_idle_poweroff_work,
						K_MSEC(SPI_LINK_IDLE_POWEROFF_MS));
		}
		printk("USB host: absent\n");
		break;
	case SPI_BRIDGE_CTRL_USB_PRESENT:
		atomic_set(&usb_host_present, 1);
		atomic_set(&usb_presence_known, 1);
		atomic_set(&hostless_timer_valid, 0);
		(void)k_work_cancel_delayable(&spi_link_idle_poweroff_work);
		printk("USB host: present\n");
		break;
	case SPI_BRIDGE_CTRL_MODE_WIRED:
		keyboard_power_set(true);
		wireless_mode_set(false);
		if (!usb_host_is_present()) {
			hostless_poweroff_timer_ensure();
		}
		break;
	case SPI_BRIDGE_CTRL_MODE_WIRELESS:
		wireless_mode_set(true);
		break;
	case SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_0:
	case SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_1:
	case SPI_BRIDGE_CTRL_DEBUG_FEEDBACK_COUNT:
		break;
	default:
		printk("Unknown SPI control cmd: 0x%02X\n", cmd);
		return false;
	}

	return true;
}

static void spi_debug_led_init(void)
{
	if (!spi_rx_led.port) {
		return;
	}

	if (!device_is_ready(spi_rx_led.port)) {
		printk("SPI debug LED device not ready\n");
		return;
	}

	if (gpio_pin_configure_dt(&spi_rx_led, GPIO_OUTPUT_INACTIVE) < 0) {
		printk("SPI debug LED configure failed\n");
		return;
	}

	spi_debug_led_set(false);
}

static void spi_debug_led_set(bool on)
{
	if (!spi_rx_led.port) {
		return;
	}

	(void)gpio_pin_set_dt(&spi_rx_led, on ? 1 : 0);
}

#if !SPI_DEBUG_LED_SHOW_CAPS
static void spi_debug_led_toggle(void)
{
	static bool on;
	on = !on;
	spi_debug_led_set(on);
}
#endif

static bool spi_report_state_equal(const uint8_t left[SPI_BRIDGE_FRAME_LEN],
				   const uint8_t right[SPI_BRIDGE_FRAME_LEN])
{
	const uint8_t *left_payload;
	const uint8_t *right_payload;
	uint8_t left_type;
	uint8_t right_type;
	uint8_t left_len;
	uint8_t right_len;

	if (!spi_bridge_report_frame_read(left, &left_type, NULL, &left_payload, &left_len) ||
	    !spi_bridge_report_frame_read(right, &right_type, NULL, &right_payload, &right_len)) {
		return false;
	}

	return left_type == right_type && left_len == right_len &&
	       memcmp(left_payload, right_payload, left_len) == 0;
}

static void spi_bridge_loop(void)
{
	static const struct spi_config spis_cfg = {
		.frequency = 1000000U,
		.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
		.slave = 0,
	};
	static const struct device *const spis = DEVICE_DT_GET(SPI_NODE);

	uint8_t tx_ack[SPI_BRIDGE_FRAME_LEN] = { 0 };
	uint8_t rx_report[SPI_BRIDGE_FRAME_LEN];
	uint8_t last_input_report[SPI_BRIDGE_FRAME_LEN] = { 0 };
	uint8_t last_forwarded_report[SPI_BRIDGE_FRAME_LEN] = { 0 };
	bool have_last_input_report = false;
	bool have_last_forwarded_report = false;
	uint8_t last_report_seq = 0;
	bool have_last_report_seq = false;
	bool report_ack_valid = false;
	uint32_t report_state_epoch = (uint32_t)atomic_get(&spi_report_state_epoch);

	struct spi_buf tx_buf = {
		.buf = tx_ack,
		.len = sizeof(tx_ack),
	};
	struct spi_buf rx_buf = {
		.buf = rx_report,
		.len = sizeof(rx_report),
	};
	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	if (!device_is_ready(spis)) {
		printk("SPIS device is not ready\n");
		return;
	}

	spi_debug_led_init();
	printk("SPIS bridge started, waiting keyboard reports...\n");
	printk("SPIS MISO ACK format: A5 5A <count_lo> <count_hi> <kbd_leds> <flags> <ctrl_seq> <ctrl_cmd>\n");

	uint32_t rx_count = 0;
	while (1) {
		uint8_t ack_led_bits;
		uint8_t ack_flags = 0;
		uint8_t ack_seq = last_ack_seq;
		uint8_t ack_cmd = last_ack_cmd;
		uint8_t ctrl_cmd;
		uint8_t ctrl_seq;
		uint32_t current_report_state_epoch =
			(uint32_t)atomic_get(&spi_report_state_epoch);

		if (current_report_state_epoch != report_state_epoch) {
			report_state_epoch = current_report_state_epoch;
			have_last_report_seq = false;
			have_last_input_report = false;
			have_last_forwarded_report = false;
			report_ack_valid = false;
		}
		if (ack_cmd == SPI_BRIDGE_ACK_REPORT_TOKEN && !report_ack_valid) {
			ack_cmd = UINT8_MAX;
		}

#if SPI_DEBUG_FORCE_LED_BITS
#if SPI_DEBUG_FORCE_LED_BLINK
		ack_led_bits = ((k_uptime_get_32() / 500U) & 0x01U) ? SPI_DEBUG_FORCE_LED_VALUE : 0x00;
#else
		ack_led_bits = SPI_DEBUG_FORCE_LED_VALUE;
#endif
#else
		ack_led_bits = spi_ack_led_bits_get();
#endif
		if (spi_ack_led_bits_valid()) {
			ack_flags |= SPI_BRIDGE_ACK_FLAG_LED_VALID;
		}
		if (any_conn_active()) {
			ack_flags |= SPI_BRIDGE_ACK_FLAG_LINK_ON;
		}
		if (pairing_mode) {
			ack_flags |= SPI_BRIDGE_ACK_FLAG_PAIRING_ON;
		}
		if (profiles[active_profile].valid) {
			ack_flags |= SPI_BRIDGE_ACK_FLAG_PROFILE_VALID;
		}
		if (wireless_mode_is_enabled()) {
			ack_flags |= SPI_BRIDGE_ACK_FLAG_MODE_WIRELESS;
		}
		spi_bridge_ack_frame_write(tx_ack, (uint16_t)rx_count, ack_led_bits, ack_flags,
					   ack_seq, ack_cmd);

		int ret = spi_transceive(spis, &spis_cfg, &tx, &rx);

		watchdog_manager_feed();
		if (ret < 0) {
			printk("spi_transceive error: %d\n", ret);
			k_sleep(K_MSEC(20));
			continue;
		}
		rx_count++;
		spi_diag_note_spi_rx();
#if !SPI_DEBUG_LED_SHOW_CAPS
		spi_debug_led_toggle();
#endif
#if SPI_DEBUG_LOG_RX_REPORTS
		printk("SPI RX %lu: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		       (unsigned long)rx_count,
		       rx_report[0], rx_report[1], rx_report[2], rx_report[3],
		       rx_report[4], rx_report[5], rx_report[6], rx_report[7]);
#endif
		if (spi_bridge_frame_control_cmd(rx_report, &ctrl_cmd, &ctrl_seq)) {
			spi_link_watchdog_feed();
			spi_activity_note();
			bool duplicate_ctrl = last_ctrl_valid && last_ctrl_cmd == ctrl_cmd &&
				last_ctrl_seq == ctrl_seq;
			bool accepted = duplicate_ctrl;

			if (!duplicate_ctrl) {
				accepted = handle_spi_control(ctrl_cmd);
			}
			if (accepted && !duplicate_ctrl) {
				if (ctrl_cmd != SPI_BRIDGE_CTRL_USER_ACTIVITY &&
				    ctrl_cmd != SPI_BRIDGE_CTRL_MODE_WIRED &&
				    ctrl_cmd != SPI_BRIDGE_CTRL_MODE_WIRELESS &&
				    ctrl_cmd != SPI_BRIDGE_CTRL_USB_PRESENT &&
				    ctrl_cmd != SPI_BRIDGE_CTRL_USB_ABSENT) {
					if (wireless_mode_is_enabled()) {
						low_power_note_activity();
					} else {
						hostless_poweroff_timer_restart();
					}
				}
				last_ctrl_cmd = ctrl_cmd;
				last_ctrl_seq = ctrl_seq;
				last_ctrl_valid = true;
			}
			if (accepted) {
				last_ack_seq = ctrl_seq;
				last_ack_cmd = ctrl_cmd;
				report_ack_valid = false;
			}
			have_last_report_seq = false;
			have_last_forwarded_report = false;
			continue;
		}

		uint8_t report_seq;

		if (!spi_bridge_report_frame_read(rx_report, NULL, &report_seq, NULL, NULL)) {
			continue;
		}
		spi_diag_note_received_report(rx_report);

		spi_link_watchdog_feed();
		spi_activity_note();
		current_report_state_epoch =
			(uint32_t)atomic_get(&spi_report_state_epoch);

		if (current_report_state_epoch != report_state_epoch) {
			report_state_epoch = current_report_state_epoch;
			have_last_report_seq = false;
			have_last_input_report = false;
			have_last_forwarded_report = false;
			report_ack_valid = false;
		}
		if (atomic_cas(&spi_link_timed_out, 1, 0)) {
			have_last_report_seq = false;
			have_last_forwarded_report = false;
		}

		bool duplicate = have_last_report_seq && last_report_seq == report_seq &&
			have_last_input_report &&
			spi_report_state_equal(last_input_report, rx_report);

		if (duplicate) {
			if ((uint32_t)atomic_get(&spi_report_state_epoch) !=
			    report_state_epoch) {
				report_ack_valid = false;
				continue;
			}
			last_ack_seq = report_seq;
			last_ack_cmd = SPI_BRIDGE_ACK_REPORT_TOKEN;
			report_ack_valid = true;
			continue;
		}

		if (!wireless_mode_is_enabled() || !any_conn_active() ||
		    !hids_frame_notify_ready(rx_report)) {
			/* Only keyboard state has a periodic QMK heartbeat. Keep an aux
			 * report unacknowledged until its own CCC is ready so a one-shot
			 * consumer/system release cannot be lost before encryption. */
			if (!spi_report_is_keyboard(rx_report)) {
				report_ack_valid = false;
				continue;
			}

			/* Keyboard reports are absolute states. ACK/drop states received
			 * before encryption or CCC readiness; the release barrier runs
			 * first and the heartbeat then restores the current state. */
			have_last_forwarded_report = false;
		} else if (!have_last_forwarded_report ||
			   !spi_report_state_equal(last_forwarded_report, rx_report)) {
			/* Do not ACK until the report is safely queued for BLE. */
			if (!spi_report_submit(rx_report)) {
				continue;
			}

			memcpy(last_forwarded_report, rx_report, SPI_BRIDGE_FRAME_LEN);
			have_last_forwarded_report = true;
		}

		bool input_changed = !have_last_input_report ||
			!spi_report_state_equal(last_input_report, rx_report);

		if (input_changed) {
			memcpy(last_input_report, rx_report, SPI_BRIDGE_FRAME_LEN);
			have_last_input_report = true;
			low_power_note_activity();
		}

		last_report_seq = report_seq;
		have_last_report_seq = true;
		if ((uint32_t)atomic_get(&spi_report_state_epoch) !=
		    report_state_epoch) {
			report_ack_valid = false;
			continue;
		}
		last_ack_seq = report_seq;
		last_ack_cmd = SPI_BRIDGE_ACK_REPORT_TOKEN;
		report_ack_valid = true;
	}
}

int main(void)
{
	int err;

	printk("SPI -> BLE HIDS keyboard bridge\n");
	(void)power_manager_init(ble_prepare_shutdown);
	profiles_clear();
	k_msgq_init(&spi_report_msgq, spi_report_msgq_buffer, SPI_BRIDGE_FRAME_LEN,
		    SPI_REPORT_QUEUE_LEN);
	k_work_init_delayable(&pairing_mode_timeout_work, pairing_mode_timeout_fn);
	k_work_init_delayable(&profile_switch_adv_refresh_work,
			      profile_switch_adv_refresh_fn);
	k_work_init_delayable(&connection_idle_work, connection_idle_work_fn);
	k_work_init_delayable(&slow_advertising_work, slow_advertising_work_fn);
	k_work_init_delayable(&auto_poweroff_work, auto_poweroff_work_fn);
	k_work_init_delayable(&spi_link_watchdog_work, spi_link_watchdog_fn);
	k_work_init_delayable(&spi_link_idle_poweroff_work, spi_link_idle_poweroff_fn);
	k_work_init_delayable(&spi_diag_log_work, spi_diag_log_work_fn);
	k_work_init(&mode_apply_work, mode_apply_work_fn);
	k_work_init(&spi_control_work, spi_control_work_fn);
	k_work_init(&connection_active_work, connection_active_work_fn);
	k_thread_create(&spi_report_thread, spi_report_thread_stack,
			K_THREAD_STACK_SIZEOF(spi_report_thread_stack),
			spi_report_thread_fn, NULL, NULL, NULL,
			SPI_REPORT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&spi_report_thread, "spi_report_tx");
	(void)k_work_schedule(&spi_diag_log_work, K_MSEC(SPI_DIAG_LOG_PERIOD_MS));
	hid_init();

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed: %d\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
	power_manager_start();

	err = watchdog_manager_init();
	if (err) {
		printk("Watchdog init warning: %d\n", err);
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_load();
		if (err) {
			printk("Settings load failed: %d\n", err);
		}
	}

	err = ensure_profile_identities();
	if (err) {
		printk("Identity setup failed: %d\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		profiles_bootstrap_from_bonds();
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Auth info cb register failed: %d\n", err);
		return 0;
	}

	/* 初始化SPI活动时间戳 */
	last_spi_activity_ms = k_uptime_get_32();

	/* QMK is the transport policy owner.  Stay radio-silent until its first
	 * validated MODE_WIRELESS command, so USB and BLE never race at startup. */
	printk("Transport mode: wired standby; waiting for QMK policy\n");
	spi_bridge_loop();
	return 0;
}
