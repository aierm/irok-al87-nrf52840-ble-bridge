/*
 * SPI (slave) -> BLE HIDS Keyboard bridge
 *
 * Expects a fixed-size, versioned input report or validated bridge control
 * frame from the SPI master.
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "power_manager.h"
#include "spi_bridge_protocol.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>
#include <bluetooth/services/hids.h>

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
#define SPI_ADV_REFRESH_DELAY_MS 180
#define SPI_PROFILE_SWITCH_ADV_DELAY_MS 250
#define SPI_ADV_START_RETRY_DELAY_MS 150U
#define SPI_ADV_START_RETRY_LIMIT 10U
#define SPI_CONNECTION_IDLE_MS 30000U
#define SPI_SLOW_ADV_DELAY_MS 120000U
#define SPI_AUTO_POWEROFF_DELAY_MS 600000U
#define SPI_LINK_IDLE_POWEROFF_MS 1800000U
#define SPI_CONN_ACTIVE_INTERVAL_MIN 6U
#define SPI_CONN_ACTIVE_INTERVAL_MAX 6U
#define SPI_CONN_IDLE_INTERVAL_MIN BT_GAP_MS_TO_CONN_INTERVAL(30U)
#define SPI_CONN_IDLE_INTERVAL_MAX BT_GAP_MS_TO_CONN_INTERVAL(50U)
#define SPI_CONN_IDLE_LATENCY 4U
#define SPI_CONN_SUPERVISION_TIMEOUT BT_GAP_MS_TO_CONN_TIMEOUT(4200U)
#define SPI_REPORT_QUEUE_LEN 128
#define SPI_CONTROL_QUEUE_LEN 8
#define SPI_REPORT_THREAD_STACK_SIZE 2048
#define SPI_REPORT_THREAD_PRIORITY 5
#define SPI_REPORT_RETRY_DELAY_MS 1
#define SPI_REPORT_RETRY_TIMEOUT_MS 1000U
#define HIDS_NOTIFY_COMPLETE_TIMEOUT_MS 1000U
#define HIDS_RECOVERY_DISCONNECT_TIMEOUT_MS 5000U
#define SPI_LINK_TIMEOUT_MS 1000
#define SPI_QUEUE_DROP_LOG_INTERVAL_MS 1000
#define SPI_DEBUG_LOG_RX_REPORTS 0
#define SPI_DEBUG_FORCE_LED_BITS 0
#define SPI_DEBUG_FORCE_LED_VALUE OUTPUT_REPORT_CAPS_LOCK_MASK
#define SPI_DEBUG_FORCE_LED_BLINK 0
#define SPI_DEBUG_LED_SHOW_CAPS 1

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
/* QMK owns transport selection after the SPI link is up.  Bluetooth remains
 * radio-silent until a validated MODE_WIRELESS command is received. */
static atomic_t wireless_mode_enabled = ATOMIC_INIT(0);
K_SEM_DEFINE(ble_disconnected_sem, 0, 1);
K_SEM_DEFINE(hids_notify_complete_sem, 0, 1);
K_THREAD_STACK_DEFINE(spi_report_thread_stack, SPI_REPORT_THREAD_STACK_SIZE);
static struct k_thread spi_report_thread;
static bool pairing_mode;
static struct k_work_delayable pairing_mode_timeout_work;
static struct k_work_delayable profile_switch_adv_refresh_work;
static struct k_work_delayable connection_idle_work;
static struct k_work_delayable slow_advertising_work;
static struct k_work_delayable auto_poweroff_work;
static struct k_work_delayable spi_link_watchdog_work;
static struct k_work_delayable spi_link_idle_poweroff_work;
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

/* Fault snapshots are intentionally kept on the nRF side; the SN32 image has
 * no spare RAM for persistent counters.  These counters do not participate in
 * flow control and are printed only when a recovery path is taken. */
static atomic_t spi_diag_rx_count;
static atomic_t spi_diag_report_enqueued;
static atomic_t spi_diag_report_dequeued;
static atomic_t spi_diag_report_completed;
static atomic_t spi_diag_queue_full;
static atomic_t spi_diag_hids_timeout;
static atomic_t spi_diag_hids_stall;

static uint32_t last_spi_activity_ms;

static void spi_diag_print(const char *reason)
{
	printk("SPI/HIDS stats (%s): rx=%u enq=%u deq=%u done=%u qfull=%u "
	       "timeout=%u stall=%u depth=%u\n",
	       reason,
	       (unsigned int)atomic_get(&spi_diag_rx_count),
	       (unsigned int)atomic_get(&spi_diag_report_enqueued),
	       (unsigned int)atomic_get(&spi_diag_report_dequeued),
	       (unsigned int)atomic_get(&spi_diag_report_completed),
	       (unsigned int)atomic_get(&spi_diag_queue_full),
	       (unsigned int)atomic_get(&spi_diag_hids_timeout),
	       (unsigned int)atomic_get(&spi_diag_hids_stall),
	       (unsigned int)k_msgq_num_used_get(&spi_report_msgq));
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
static void spi_report_state_clear(bool clear_last_sent);
static bool spi_report_submit(const uint8_t report[SPI_BRIDGE_FRAME_LEN]);
static void spi_report_latest_resubmit(void);
static void hids_input_notify_handler(enum bt_hids_notify_evt evt);
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
	power_manager_shutdown_no_wake();
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
	power_manager_shutdown_no_wake();
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

static uint8_t active_profile_led_bit(void)
{
	if (active_profile >= PROFILE_COUNT) {
		return 0;
	}

	return BIT(active_profile);
}

static bool deadline_is_active(uint32_t deadline_ms)
{
	return deadline_ms != 0u && (int32_t)(deadline_ms - k_uptime_get_32()) > 0;
}

static uint8_t spi_ack_led_bits_get(void)
{
	uint8_t profile_led = active_profile_led_bit();

	if (pairing_mode) {
		return ((k_uptime_get_32() / 180U) & 0x01U) ? profile_led : 0x00;
	}

	if (any_conn_active()) {
		if (host_output_led_valid) {
			return host_output_led_bits &
			       (OUTPUT_REPORT_NUM_LOCK_MASK |
				OUTPUT_REPORT_CAPS_LOCK_MASK |
				OUTPUT_REPORT_SCROLL_LOCK_MASK);
		}

		return deadline_is_active(connected_profile_led_until_ms) ? profile_led : 0x00;
	}

	return deadline_is_active(profile_switch_led_until_ms) ? profile_led : 0x00;
}

static bool spi_ack_led_bits_valid(void)
{
	if (pairing_mode) {
		return true;
	}

	if (any_conn_active()) {
		return host_output_led_valid || deadline_is_active(connected_profile_led_until_ms);
	}

	return deadline_is_active(profile_switch_led_until_ms);
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

static void spi_report_state_clear(bool clear_last_sent)
{
	k_msgq_purge(&spi_report_msgq);

	if (clear_last_sent) {
		k_spinlock_key_t key = k_spin_lock(&latest_report_lock);

		latest_spi_report_valid = false;
		k_spin_unlock(&latest_report_lock, key);
	}
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

static bool spi_report_submit(const uint8_t report[SPI_BRIDGE_FRAME_LEN])
{
	if (!wireless_mode_is_enabled()) {
		return false;
	}

	int ret = k_msgq_put(&spi_report_msgq, report, K_NO_WAIT);

	if (ret) {
		atomic_inc(&spi_diag_queue_full);
		uint32_t now = k_uptime_get_32();

		if ((int32_t)(now - spi_queue_drop_log_at_ms) >= 0) {
			printk("SPI report queue backpressure: %d\n", ret);
			spi_queue_drop_log_at_ms = now + SPI_QUEUE_DROP_LOG_INTERVAL_MS;
		}
		return false;
	}
	atomic_inc(&spi_diag_report_enqueued);

	if (spi_report_is_keyboard(report)) {
		k_spinlock_key_t key = k_spin_lock(&latest_report_lock);

		memcpy(latest_spi_report, report, SPI_BRIDGE_FRAME_LEN);
		latest_spi_report_valid = true;
		k_spin_unlock(&latest_report_lock, key);
	}

	last_spi_activity_ms = k_uptime_get_32();
	return true;
}

static void spi_report_latest_resubmit(void)
{
	uint8_t report[SPI_BRIDGE_FRAME_LEN];
	k_spinlock_key_t key = k_spin_lock(&latest_report_lock);
	bool valid = latest_spi_report_valid;

	if (valid) {
		memcpy(report, latest_spi_report, sizeof(report));
	}
	k_spin_unlock(&latest_report_lock, key);

	if (valid && wireless_mode_is_enabled() && any_conn_active()) {
		(void)spi_report_submit(report);
	}
}

static void hids_input_notify_handler(enum bt_hids_notify_evt evt)
{
	if (evt == BT_HIDS_CCCD_EVT_NOTIFY_ENABLED) {
		spi_report_latest_resubmit();
	}
}

static void spi_link_watchdog_fn(struct k_work *work)
{
	uint8_t release_frame[SPI_BRIDGE_FRAME_LEN];
	uint8_t release_report[SPI_BRIDGE_NKRO_REPORT_LEN] = { 0 };

	ARG_UNUSED(work);

	if (!wireless_mode_is_enabled() || !any_conn_active()) {
		return;
	}

	spi_diag_print("spi-timeout");
	atomic_set(&spi_link_timed_out, 1);
	k_msgq_purge(&spi_report_msgq);
	(void)spi_bridge_report_frame_write(release_frame,
		SPI_BRIDGE_REPORT_KEYBOARD_NKRO, 0, release_report,
		sizeof(release_report));
	(void)spi_report_submit(release_frame);
	(void)spi_bridge_report_frame_write(release_frame,
		SPI_BRIDGE_REPORT_CONSUMER, 0, release_report, sizeof(uint16_t));
	(void)spi_report_submit(release_frame);
	(void)spi_bridge_report_frame_write(release_frame,
		SPI_BRIDGE_REPORT_SYSTEM, 0, release_report, sizeof(uint16_t));
	(void)spi_report_submit(release_frame);
	printk("SPI link timeout; released all BLE keys\n");

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

static int advertising_configure_accept_list(bool pairing_enabled)
{
	int err;

	err = bt_le_filter_accept_list_clear();
	if (err && err != -EALREADY) {
		printk("accept_list_clear failed: %d\n", err);
		return err;
	}

	if (pairing_enabled || active_profile >= PROFILE_COUNT ||
	    !profiles[active_profile].valid) {
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

static uint8_t active_profile_bt_id(void)
{
	if (active_profile < PROFILE_COUNT) {
		return profile_bt_id[active_profile];
	}

	return BT_ID_DEFAULT;
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
						 BT_GAP_ADV_FAST_INT_MIN_2;
	uint32_t interval_max = slow_advertising ? BT_GAP_ADV_SLOW_INT_MAX :
						 BT_GAP_ADV_FAST_INT_MAX_2;
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
		adv_opts |= BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ;
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
	hids_inp_rep->handler = hids_input_notify_handler;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_CONSUMER_INDEX];
	hids_inp_rep->size = INPUT_REPORT_CONSUMER_LEN;
	hids_inp_rep->id = INPUT_REP_CONSUMER_REF_ID;
	hids_inp_rep->handler = hids_input_notify_handler;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_SYSTEM_INDEX];
	hids_inp_rep->size = INPUT_REPORT_SYSTEM_LEN;
	hids_inp_rep->id = INPUT_REP_SYSTEM_REF_ID;
	hids_inp_rep->handler = hids_input_notify_handler;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_outp_rep = &hids_init_obj.outp_rep_group_init.reports[0];
	hids_outp_rep->size = OUTPUT_REPORT_MAX_LEN;
	hids_outp_rep->id = OUTPUT_REP_KEYS_REF_ID;
	hids_outp_rep->handler = hids_outp_rep_handler;
	hids_init_obj.outp_rep_group_init.cnt++;

	hids_init_obj.is_kb = true;
	hids_init_obj.boot_kb_notif_handler = hids_input_notify_handler;
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

	host_output_led_reset();
	profile_switch_led_until_ms = 0;
	connected_profile_led_until_ms = k_uptime_get_32() + SPI_CONNECTED_LED_HOLD_MS;

	ret = bt_hids_connected(&hids_obj, conn);
	if (ret) {
		printk("bt_hids_connected failed: %d\n", ret);
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
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int ret;
	bool tracked = false;
	struct bt_conn *tracked_conn = NULL;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected: %s reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

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

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d %s\n",
		       addr, level, err, bt_security_err_to_str(err));
	}
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
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

static void hids_notify_complete(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(user_data);
	k_sem_give(&hids_notify_complete_sem);
}

static int bridge_report_con_send(const uint8_t frame[SPI_BRIDGE_FRAME_LEN], bool boot_mode,
					  struct bt_conn *conn, bool *notification_pending)
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
			return bt_hids_boot_kb_inp_rep_send(&hids_obj, conn, payload,
							      payload_len, hids_notify_complete);
		} else {
			uint8_t nkro[SPI_BRIDGE_NKRO_REPORT_LEN];

			keyboard_boot_to_nkro(payload, nkro);
			return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
				INPUT_REP_KEYBOARD_INDEX, nkro, sizeof(nkro),
				hids_notify_complete, NULL);
		}

	case SPI_BRIDGE_REPORT_KEYBOARD_NKRO:
		*notification_pending = true;
		if (boot_mode) {
			uint8_t boot[SPI_BRIDGE_BOOT_REPORT_LEN];

			keyboard_nkro_to_boot(payload, boot);
			return bt_hids_boot_kb_inp_rep_send(&hids_obj, conn, boot,
							      sizeof(boot), hids_notify_complete);
		}
		return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
			INPUT_REP_KEYBOARD_INDEX, payload, payload_len,
			hids_notify_complete, NULL);

	case SPI_BRIDGE_REPORT_CONSUMER:
		if (boot_mode) {
			return 0;
		}
		*notification_pending = true;
		return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
			INPUT_REP_CONSUMER_INDEX, payload, payload_len,
			hids_notify_complete, NULL);

	case SPI_BRIDGE_REPORT_SYSTEM:
		if (boot_mode) {
			return 0;
		}
		*notification_pending = true;
		return bt_hids_inp_rep_send_userdata(&hids_obj, conn,
			INPUT_REP_SYSTEM_INDEX, payload, payload_len,
			hids_notify_complete, NULL);

	default:
		return -EINVAL;
	}
}

static int bridge_report_send_all(const uint8_t frame[SPI_BRIDGE_FRAME_LEN])
{
	bool sent = false;
	int result = 0;

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		int err;
		struct bt_conn *conn = NULL;
		bool boot_mode = false;
		bool notification_pending = false;
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
		k_sem_reset(&hids_notify_complete_sem);
		err = bridge_report_con_send(frame, boot_mode, conn,
					     &notification_pending);
		if (!err && notification_pending &&
		    k_sem_take(&hids_notify_complete_sem,
			       K_MSEC(HIDS_NOTIFY_COMPLETE_TIMEOUT_MS)) != 0) {
			err = -ETIMEDOUT;
		}
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
		atomic_inc(&spi_diag_report_dequeued);
		if (!wireless_mode_is_enabled()) {
			continue;
		}

		uint32_t retry_started_ms = k_uptime_get_32();

		while (1) {
			int ret;

			if (!wireless_mode_is_enabled()) {
				break;
			}

			ret = bridge_report_send_all(report);
			if (ret == 0) {
				atomic_inc(&spi_diag_report_completed);
				break;
			}
			if (ret == -ENOTCONN || ret == -EACCES) {
				break;
			}
			if (ret == -ETIMEDOUT) {
				atomic_inc(&spi_diag_hids_timeout);
				printk("HIDS notification timed out after %u ms; disconnecting\n",
				       HIDS_NOTIFY_COMPLETE_TIMEOUT_MS);
				spi_report_recover_disconnect();
				break;
			}

			if (ret == -ENOMEM || ret == -EAGAIN) {
				if ((uint32_t)(k_uptime_get_32() - retry_started_ms) >=
				    SPI_REPORT_RETRY_TIMEOUT_MS) {
					atomic_inc(&spi_diag_hids_stall);
					printk("HIDS send stalled for %u ms (%d); disconnecting\n",
					       SPI_REPORT_RETRY_TIMEOUT_MS, ret);
					spi_report_recover_disconnect();
					break;
				}
				/* These errors occur before ATT accepts the notification, so a
				 * bounded retry cannot duplicate an already-sent key state. */
				k_sleep(K_MSEC(SPI_REPORT_RETRY_DELAY_MS));
				continue;
			}

			printk("HIDS send error: %d\n", ret);
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

	spi_report_state_clear(true);
	active_profile = profile_idx;
	host_output_led_reset();
	profile_switch_led_until_ms = 0;
	connected_profile_led_until_ms = 0;
	profiles_save();
	printk("Active profile -> %u%s\n", (unsigned int)(active_profile + 1),
	       profiles[active_profile].valid ? "" : " (empty)");

	if (profiles[active_profile].valid) {
		profile_switch_led_until_ms = k_uptime_get_32() + SPI_PROFILE_SWITCH_LED_HOLD_MS;
		set_pairing_mode(false);
	} else {
		set_pairing_mode(true);
	}

	disconnect_all();
	schedule_advertising_refresh(SPI_PROFILE_SWITCH_ADV_DELAY_MS);
}

static void clear_active_profile_bond(void)
{
	(void)bt_unpair(active_profile_bt_id(), BT_ADDR_LE_ANY);
	spi_report_state_clear(true);
	profiles[active_profile].valid = false;
	host_output_led_reset();
	profile_switch_led_until_ms = 0;
	connected_profile_led_until_ms = 0;
	profiles_save();
	printk("Cleared bond in profile %u\n", (unsigned int)(active_profile + 1));
	disconnect_all();
	set_pairing_mode(true);
}

static void clear_all_bonds(void)
{
	for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
		(void)bt_unpair(profile_bt_id[i], BT_ADDR_LE_ANY);
	}
	spi_report_state_clear(true);
	host_output_led_reset();
	profile_switch_led_until_ms = 0;
	connected_profile_led_until_ms = 0;
	profiles_clear();
	profiles_save();
	printk("Cleared all bonds\n");
	set_pairing_mode(true);
	disconnect_all();
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
		case SPI_BRIDGE_CTRL_DISCONNECT:
			disconnect_all();
			break;
		case SPI_BRIDGE_CTRL_CLEAR_ACTIVE:
			clear_active_profile_bond();
			break;
		case SPI_BRIDGE_CTRL_CLEAR_ALL:
			clear_all_bonds();
			break;
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
	case SPI_BRIDGE_CTRL_DISCONNECT:
		return spi_control_defer(cmd);
	case SPI_BRIDGE_CTRL_CLEAR_ACTIVE:
		return spi_control_defer(cmd);
	case SPI_BRIDGE_CTRL_CLEAR_ALL:
		return spi_control_defer(cmd);
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
		uint8_t ctrl_cmd;
		uint8_t ctrl_seq;

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
					   last_ack_seq, last_ack_cmd);

		int ret = spi_transceive(spis, &spis_cfg, &tx, &rx);

		if (ret < 0) {
			printk("spi_transceive error: %d\n", ret);
			k_sleep(K_MSEC(20));
			continue;
		}
		rx_count++;
		atomic_inc(&spi_diag_rx_count);
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
			}
			have_last_report_seq = false;
			have_last_forwarded_report = false;
			continue;
		}

		uint8_t report_seq;

		if (!spi_bridge_report_frame_read(rx_report, NULL, &report_seq, NULL, NULL)) {
			continue;
		}

		spi_link_watchdog_feed();
		spi_activity_note();
		if (atomic_cas(&spi_link_timed_out, 1, 0)) {
			have_last_report_seq = false;
			have_last_forwarded_report = false;
		}

		bool duplicate = have_last_report_seq && last_report_seq == report_seq &&
			have_last_input_report &&
			spi_report_state_equal(last_input_report, rx_report);

		if (duplicate) {
			last_ack_seq = report_seq;
			last_ack_cmd = SPI_BRIDGE_ACK_REPORT_TOKEN;
			continue;
		}

		if (!wireless_mode_is_enabled() || !any_conn_active()) {
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
		last_ack_seq = report_seq;
		last_ack_cmd = SPI_BRIDGE_ACK_REPORT_TOKEN;
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
	k_work_init(&mode_apply_work, mode_apply_work_fn);
	k_work_init(&spi_control_work, spi_control_work_fn);
	k_work_init(&connection_active_work, connection_active_work_fn);
	k_thread_create(&spi_report_thread, spi_report_thread_stack,
			K_THREAD_STACK_SIZEOF(spi_report_thread_stack),
			spi_report_thread_fn, NULL, NULL, NULL,
			SPI_REPORT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&spi_report_thread, "spi_report_tx");
	hid_init();

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed: %d\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
	power_manager_start();

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
