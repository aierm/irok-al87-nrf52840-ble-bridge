#include "power_manager.h"

#include <errno.h>
#include <stdint.h>

#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_power.h>

#define BATTERY_NODE DT_NODELABEL(battery_sense)

#define BATTERY_SAMPLE_INTERVAL K_SECONDS(30)
#define BATTERY_FILTER_DOWN_WEIGHT 3  /* Falling voltage: (old*3 + new)/4 */
#define BATTERY_FILTER_UP_WEIGHT 15   /* Load recovery: (old*15 + new)/16 */
#define POWER_BUTTON_HOLD_MS 1000
#define BATTERY_IDLE_SAMPLE_COUNT 3            /* 空闲3次采样后取平均 */
#define POWER_BUTTON_RELEASE_DEBOUNCE_MS 30
/* Power-on needs the same deliberate hold as power-off.  Without this a single
 * edge on P0.29 booted the whole keyboard, so an accidental brush in a bag
 * turned it on and drained the battery. */
#define POWER_BUTTON_ON_HOLD_MS 1000
#define POWER_BUTTON_ON_POLL_MS 20
/* Let the internal pull-up settle before the first read.  main() runs this only
 * a few hundred microseconds after GPIO init. */
#define POWER_BUTTON_SETTLE_MS 5
/* The wake source may be a capacitive touch pad rather than a clean mechanical
 * contact, so its output can chatter during a hold.  Require the button to be
 * down for this share of the samples instead of every single one. */
#define POWER_BUTTON_ON_HOLD_MIN_PERCENT 70
/* Consecutive released samples that count as a real release (5 * 20ms = 100ms).
 * Anything shorter is treated as chatter inside a continuing hold. */
#define POWER_BUTTON_RELEASE_SAMPLES 5U
/* Upper bound on any wait-for-release loop.  A pad stuck active must not keep
 * the CPU spinning with the rails already down. */
#define POWER_BUTTON_RELEASE_TIMEOUT_MS 5000U
/* Length of the confirmation sampling window on the shutdown path.  The hold
 * work is scheduled this much earlier than POWER_BUTTON_HOLD_MS and then samples
 * for this long, so a pass still means the button was down for roughly
 * POWER_BUTTON_HOLD_MS in total. */
#define POWER_BUTTON_HOLD_CONFIRM_MS 200U

struct battery_curve_point {
	uint16_t millivolts;
	uint8_t percent;
};

static const struct battery_curve_point battery_curve[] = {
	{ 4200, 100 },
	{ 4100, 90 },
	{ 4000, 80 },
	{ 3920, 70 },
	{ 3850, 60 },
	{ 3790, 50 },
	{ 3740, 40 },
	{ 3700, 30 },
	{ 3650, 20 },
	{ 3550, 10 },
	{ 3400, 5 },
	{ 3300, 0 },
};

static const struct gpio_dt_spec external_vcc_enable =
	GPIO_DT_SPEC_GET(DT_NODELABEL(external_vcc), gpios);
static const struct gpio_dt_spec keyboard_load_enable =
	GPIO_DT_SPEC_GET(DT_NODELABEL(keyboard_power), gpios);
static const struct gpio_dt_spec power_button =
	GPIO_DT_SPEC_GET(DT_NODELABEL(power_button), gpios);
static const struct adc_dt_spec battery_adc = ADC_DT_SPEC_GET(BATTERY_NODE);

static atomic_t keyboard_powered = ATOMIC_INIT(1);
static atomic_t shutdown_started = ATOMIC_INIT(0);
static bool power_gpio_ready;
K_MUTEX_DEFINE(power_mutex);

static bool battery_ready;
static bool battery_calibrate = true;
static int32_t battery_filtered_mv;
static int32_t battery_samples[BATTERY_IDLE_SAMPLE_COUNT];
static uint8_t battery_sample_index;
static uint8_t battery_sample_count;
static struct k_work_delayable battery_work;

static bool power_button_ready;
static struct gpio_callback power_button_callback;
static struct k_work_delayable power_button_hold_work;
static power_shutdown_prepare_cb_t shutdown_prepare_cb;

static uint8_t battery_percent_from_mv(int32_t millivolts)
{
	if (millivolts >= battery_curve[0].millivolts) {
		return battery_curve[0].percent;
	}

	for (size_t i = 1; i < ARRAY_SIZE(battery_curve); i++) {
		const struct battery_curve_point *high = &battery_curve[i - 1];
		const struct battery_curve_point *low = &battery_curve[i];

		if (millivolts >= low->millivolts) {
			return low->percent +
			       (uint32_t)(high->percent - low->percent) *
			       (millivolts - low->millivolts) /
			       (high->millivolts - low->millivolts);
		}
	}

	return 0;
}

static int battery_sample_mv(int32_t *battery_mv)
{
	int16_t raw;
	struct adc_sequence sequence = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
		.calibrate = battery_calibrate,
	};

	/* Hardware SAADC oversampling (4x = 16 samples in devicetree) averages
	 * high-frequency noise in a single conversion without stalling the
	 * system workqueue across multiple k_msleep() delays. */
	int ret = adc_sequence_init_dt(&battery_adc, &sequence);
	if (ret < 0) {
		return ret;
	}

	sequence.calibrate = battery_calibrate;
	ret = adc_read_dt(&battery_adc, &sequence);
	if (ret < 0) {
		return ret;
	}
	battery_calibrate = false;

	int32_t divider_mv = raw;
	ret = adc_raw_to_millivolts_dt(&battery_adc, &divider_mv);
	if (ret < 0) {
		return ret;
	}

	/* 2倍分压，恢复实际电池电压 */
	*battery_mv = divider_mv * (uint64_t)DT_PROP(BATTERY_NODE, full_ohms) /
		      DT_PROP(BATTERY_NODE, output_ohms);
	return 0;
}

/* Average consecutive sampling periods to reduce load transients. */
static int battery_get_stable_mv(int32_t *stable_mv)
{
	int32_t sample_mv;
	int ret = battery_sample_mv(&sample_mv);

	if (ret < 0) {
		return ret;
	}

	/* 累积多次采样 */
	battery_samples[battery_sample_index] = sample_mv;
	battery_sample_index = (battery_sample_index + 1) % BATTERY_IDLE_SAMPLE_COUNT;
	if (battery_sample_count < BATTERY_IDLE_SAMPLE_COUNT) {
		battery_sample_count++;
	}

	/* 计算平均值（去除负载瞬态影响） */
	int64_t sum = 0;
	for (uint8_t i = 0; i < battery_sample_count; i++) {
		sum += battery_samples[i];
	}
	*stable_mv = (int32_t)(sum / battery_sample_count);
	return 0;
}

static void battery_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	int32_t millivolts;
	int ret = battery_get_stable_mv(&millivolts);

	if (ret < 0) {
		/* Never turn an ADC error into a plausible but false battery value. */
		printk("Battery sample failed: %d\n", ret);
		(void)k_work_reschedule(&battery_work, BATTERY_SAMPLE_INTERVAL);
		return;
	}

	/* 初次读数或滤波值为0时，直接使用当前值 */
	if (battery_filtered_mv == 0) {
		battery_filtered_mv = millivolts;
	} else {
		/* Voltage rebounds when the key/LED load is removed.  Accept rises much
		 * more slowly than falls so reconnecting does not produce large jumps,
		 * while real charging can still move the estimate upward. */
		const int32_t weight = millivolts > battery_filtered_mv ?
			BATTERY_FILTER_UP_WEIGHT : BATTERY_FILTER_DOWN_WEIGHT;

		battery_filtered_mv =
			(battery_filtered_mv * weight + millivolts) / (weight + 1);
	}

	uint8_t percent = battery_percent_from_mv(battery_filtered_mv);

	ret = bt_bas_set_battery_level(percent);
	if (ret < 0 && ret != -EINVAL) {
		/* 不打印错误，避免未连接时刷屏 */
	}
	/* Keep both values in RTT so the divider/ADC can be calibrated separately
	 * from the load-dependent percentage curve. */
	printk("Battery: sample=%d mV, filtered=%d mV, %u%%\n",
	       millivolts, battery_filtered_mv, percent);

	(void)k_work_reschedule(&battery_work, BATTERY_SAMPLE_INTERVAL);
}

bool keyboard_power_is_enabled(void)
{
	return atomic_get(&keyboard_powered) != 0;
}

void keyboard_power_set(bool enabled)
{
	if (!power_gpio_ready) {
		printk("Keyboard power command ignored: GPIOs not ready\n");
		return;
	}

	k_mutex_lock(&power_mutex, K_FOREVER);
	if (enabled == keyboard_power_is_enabled()) {
		k_mutex_unlock(&power_mutex);
		return;
	}

	int ret;

	if (enabled) {
		ret = gpio_pin_set_dt(&external_vcc_enable, 1);
		if (ret == 0) {
			k_msleep(2);
			ret = gpio_pin_set_dt(&keyboard_load_enable, 1);
		}
	} else {
		ret = gpio_pin_set_dt(&keyboard_load_enable, 0);
		if (ret == 0) {
			k_msleep(2);
			ret = gpio_pin_set_dt(&external_vcc_enable, 0);
		}
	}

	if (ret < 0) {
		printk("Keyboard power GPIO write failed: %d\n", ret);
	} else {
		atomic_set(&keyboard_powered, enabled ? 1 : 0);
		printk("Keyboard power %s (P0.02=%u, P0.13=%u)\n",
		       enabled ? "on" : "off", enabled ? 1U : 0U,
		       enabled ? 1U : 0U);
	}
	k_mutex_unlock(&power_mutex);
}

static bool power_button_is_pressed(void)
{
	int value = gpio_pin_get_dt(&power_button);

	if (value < 0) {
		printk("Power button read failed: %d\n", value);
		return false;
	}

	return value != 0;
}

/* Sample the button over window_ms and report whether it read pressed for at
 * least POWER_BUTTON_ON_HOLD_MIN_PERCENT of the samples.
 *
 * A single gpio_pin_get_dt() is not enough to decide "still held": the wake
 * source is a touch pad whose output chatters, so an unlucky instant reads
 * released while the finger is still down.  That is what made shutdown feel
 * unreliable -- the 1 s check landed in a gap and silently did nothing. */
static bool power_button_held_over(uint32_t window_ms)
{
	const uint32_t samples = window_ms / POWER_BUTTON_ON_POLL_MS;
	const uint32_t needed = (samples * POWER_BUTTON_ON_HOLD_MIN_PERCENT) / 100U;
	uint32_t pressed_samples = 0;
	uint32_t consecutive_release = 0;

	for (uint32_t i = 0; i < samples; i++) {
		if (power_button_is_pressed()) {
			pressed_samples++;
			consecutive_release = 0;
		} else if (++consecutive_release >= POWER_BUTTON_RELEASE_SAMPLES) {
			/* Solid release: the user let go, no point sampling on. */
			break;
		}
		k_msleep(POWER_BUTTON_ON_POLL_MS);
	}

	printk("Power button hold: %u/%u samples (need %u)\n",
	       (unsigned int)pressed_samples, (unsigned int)samples,
	       (unsigned int)needed);
	return pressed_samples >= needed;
}

/* Block until the button reads released for POWER_BUTTON_RELEASE_SAMPLES in a
 * row, so chatter mid-release does not end the wait early.  Bounded by
 * POWER_BUTTON_RELEASE_TIMEOUT_MS: a stuck-low pad must not wedge the caller,
 * because on the shutdown path that would keep the CPU out of System OFF and
 * drain the battery with the rails already down. */
static void power_button_wait_for_release(void)
{
	uint32_t consecutive_release = 0;
	uint32_t waited_ms = 0;

	while (consecutive_release < POWER_BUTTON_RELEASE_SAMPLES) {
		if (power_button_is_pressed()) {
			consecutive_release = 0;
		} else {
			consecutive_release++;
		}

		if (waited_ms >= POWER_BUTTON_RELEASE_TIMEOUT_MS) {
			printk("Power button still active after %ums; continuing\n",
			       (unsigned int)waited_ms);
			return;
		}

		k_msleep(POWER_BUTTON_ON_POLL_MS);
		waited_ms += POWER_BUTTON_ON_POLL_MS;
	}
}

static void power_manager_shutdown_impl(bool allow_gpio_wake)
{
	if (!atomic_cas(&shutdown_started, 0, 1)) {
		return;
	}

	printk("System shutdown requested\n");
	if (power_button_ready) {
		(void)gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_DISABLE);
	}
	if (battery_ready) {
		(void)k_work_cancel_delayable(&battery_work);
	}

	if (shutdown_prepare_cb) {
		shutdown_prepare_cb();
	}

	keyboard_power_set(false);

	/* nRF52840 hardware watchdog cannot be stopped while running; entering System OFF
	 * with active WDT would cause false wakeups after the timeout expires.
	 * We set GPREGRET magic and reset the SOC, entering a clean System OFF on next boot
	 * BEFORE the watchdog is ever started.
	 */
	if (allow_gpio_wake && power_button_ready) {
		power_button_wait_for_release();
		k_msleep(POWER_BUTTON_RELEASE_DEBOUNCE_MS);
		nrf_power_gpregret_set(NRF_POWER, 0, POWER_GPREGRET_SHUTDOWN_WAKE);
		printk("Resetting SOC to enter clean System OFF (wake enabled)...\n");
	} else {
		nrf_power_gpregret_set(NRF_POWER, 0, POWER_GPREGRET_SHUTDOWN_NO_WAKE);
		printk("Resetting SOC to enter clean System OFF (wake disabled)...\n");
	}

	k_msleep(10);
	sys_reboot(SYS_REBOOT_COLD);
}

void power_manager_shutdown(void)
{
	power_manager_shutdown_impl(true);
}

void power_manager_shutdown_no_wake(void)
{
	power_manager_shutdown_impl(false);
}

static void power_button_hold_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Confirm over a window instead of a single read at the 1 s mark.  The
	 * work fires POWER_BUTTON_HOLD_CONFIRM_MS early so a pass still means the
	 * button was down for about POWER_BUTTON_HOLD_MS in total. */
	if (power_button_held_over(POWER_BUTTON_HOLD_CONFIRM_MS)) {
		power_manager_shutdown();
	} else {
		printk("Power button released early; no shutdown\n");
	}
}

static void power_button_handler(const struct device *port,
				 struct gpio_callback *callback,
				 gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);

	(void)k_work_schedule(&power_button_hold_work,
			      K_MSEC(POWER_BUTTON_HOLD_MS -
				     POWER_BUTTON_HOLD_CONFIRM_MS));
}

/* True when this boot came from System OFF (i.e. a P0.29 wake) rather than a
 * cold start such as a fresh flash, USB attach, or pin reset.
 *
 * A cold start has nobody holding the button, so gating it on a hold would drop
 * straight back into System OFF and look like dead hardware.  Only a wake from
 * System OFF is required to prove intent. */
static bool boot_is_wake_from_system_off(void)
{
	uint32_t reas = nrf_power_resetreas_get(NRF_POWER);

	/* Clear so the next boot sees only its own cause. */
	nrf_power_resetreas_clear(NRF_POWER, reas);

	printk("RESETREAS=0x%08X", (unsigned int)reas);
	if (reas & NRF_POWER_RESETREAS_DOG_MASK) {
		printk(" [WATCHDOG RESET - auto self-healed]");
	}
	if (reas & NRF_POWER_RESETREAS_SREQ_MASK) {
		printk(" [SOFTWARE RESET]");
	}
	if (reas & NRF_POWER_RESETREAS_OFF_MASK) {
		printk(" [SYSTEM OFF WAKE]");
	}
	if (reas & NRF_POWER_RESETREAS_RESETPIN_MASK) {
		printk(" [PIN RESET]");
	}
	printk("\n");

	return (reas & NRF_POWER_RESETREAS_OFF_MASK) != 0U;
}

/* Wait for a deliberate hold on P0.29 before powering the keyboard.
 *
 * Returns true if the button was down for at least
 * POWER_BUTTON_ON_HOLD_MIN_PERCENT of the sampling window.  Runs before SPI and
 * Bluetooth are up, so polling is fine.  A brief edge (bag brush, or the release
 * bounce of the shutdown press itself) fails the threshold and the caller goes
 * back to System OFF instead of booting the keyboard.
 *
 * The threshold rather than an all-or-nothing check matters because the wake
 * source may be a capacitive touch pad whose output chatters while held. */
static bool power_button_wait_for_on_hold(void)
{
	/* Let the internal pull-up settle before the first read. */
	k_msleep(POWER_BUTTON_SETTLE_MS);

	return power_button_held_over(POWER_BUTTON_ON_HOLD_MS);
}

/* Re-arm P0.29 as the wake source and go back to System OFF without touching
 * the keyboard rails -- they were never brought up on this boot. */
static void power_manager_abort_boot(void)
{
	printk("Power button not held; returning to System OFF\n");
	(void)gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_DISABLE);

	power_button_wait_for_release();
	k_msleep(POWER_BUTTON_RELEASE_DEBOUNCE_MS);

	int ret = gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_LEVEL_ACTIVE);

	if (ret < 0) {
		printk("Power button wake configuration failed: %d\n", ret);
	}
	k_msleep(5);
	sys_poweroff();
}

int power_manager_init(power_shutdown_prepare_cb_t shutdown_prepare)
{
	shutdown_prepare_cb = shutdown_prepare;

	/* If boot came from shutdown transition, enter clean System OFF immediately
	 * before watchdog is ever started.
	 */
	uint32_t gpregret = nrf_power_gpregret_get(NRF_POWER, 0);
	if (gpregret == POWER_GPREGRET_SHUTDOWN_WAKE ||
	    gpregret == POWER_GPREGRET_SHUTDOWN_NO_WAKE) {
		nrf_power_gpregret_set(NRF_POWER, 0, 0);
		printk("Shutdown transition detected (GPREGRET=0x%02X)\n", (unsigned int)gpregret);

		if (gpio_is_ready_dt(&external_vcc_enable)) {
			(void)gpio_pin_configure_dt(&external_vcc_enable, GPIO_OUTPUT_INACTIVE);
		}
		if (gpio_is_ready_dt(&keyboard_load_enable)) {
			(void)gpio_pin_configure_dt(&keyboard_load_enable, GPIO_OUTPUT_INACTIVE);
		}

		if (gpregret == POWER_GPREGRET_SHUTDOWN_WAKE && gpio_is_ready_dt(&power_button)) {
			(void)gpio_pin_configure_dt(&power_button, GPIO_INPUT);
			power_button_wait_for_release();
			k_msleep(POWER_BUTTON_RELEASE_DEBOUNCE_MS);
			int ret = gpio_pin_interrupt_configure_dt(&power_button, GPIO_INT_LEVEL_ACTIVE);
			if (ret < 0) {
				printk("Power button wake configuration failed: %d\n", ret);
			}
			printk("Entering clean System OFF (WDT stopped); press P0.29 to wake\n");
		} else {
			printk("Entering clean System OFF (WDT stopped); cycle switch to restart\n");
		}

		k_msleep(5);
		sys_poweroff();
		return 0;
	}

	if (!gpio_is_ready_dt(&external_vcc_enable) ||
	    !gpio_is_ready_dt(&keyboard_load_enable)) {
		printk("Keyboard power GPIO device not ready\n");
		return -ENODEV;
	}

	/* Start with both rails off.  The devicetree gpio-hog that used to force
	 * them high before main() is gone, so nothing is powered until the
	 * power-on hold below is satisfied. */
	int ret = gpio_pin_configure_dt(&external_vcc_enable, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&keyboard_load_enable, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}
	power_gpio_ready = true;
	atomic_set(&keyboard_powered, 0);

	/* The power button must be readable to gate the boot.  If it is not, fall
	 * back to powering up unconditionally so the keyboard is never bricked. */
	if (!gpio_is_ready_dt(&power_button)) {
		printk("Power button GPIO not ready; powering up unconditionally\n");
		keyboard_power_set(true);
		return 0;
	}

	ret = gpio_pin_configure_dt(&power_button, GPIO_INPUT);
	if (ret < 0) {
		printk("Power button setup failed: %d; powering up unconditionally\n", ret);
		keyboard_power_set(true);
		return 0;
	}
	power_button_ready = true;

	/* Only a wake from System OFF has to prove intent with a hold.  A cold
	 * start (fresh flash, USB attach, pin reset) powers up directly -- nobody
	 * is holding the button then, and refusing to boot would look like dead
	 * hardware. */
	if (boot_is_wake_from_system_off() && !power_button_wait_for_on_hold()) {
		power_manager_abort_boot();
		/* Not reached. */
		return 0;
	}

	keyboard_power_set(true);
	printk("Keyboard power on (TPS22919 ON=P0.02)\n");

	if (!adc_is_ready_dt(&battery_adc)) {
		printk("Battery ADC device not ready\n");
	} else {
		ret = adc_channel_setup_dt(&battery_adc);
		if (ret < 0) {
			printk("Battery ADC setup failed: %d\n", ret);
		} else {
			battery_ready = true;
			k_work_init_delayable(&battery_work, battery_work_fn);
		}
	}

	/* The button pin itself was already configured above to gate the boot;
	 * only the shutdown-hold work and its callback remain to be wired up. */
	k_work_init_delayable(&power_button_hold_work, power_button_hold_fn);
	gpio_init_callback(&power_button_callback, power_button_handler,
			   BIT(power_button.pin));
	ret = gpio_add_callback(power_button.port, &power_button_callback);
	if (ret < 0) {
		printk("Power button callback setup failed: %d\n", ret);
		power_button_ready = false;
		return 0;
	}

	printk("Power button configured on P0.29 (active low)\n");
	return 0;
}

void power_manager_start(void)
{
	if (battery_ready) {
		(void)k_work_reschedule(&battery_work, K_NO_WAIT);
	}

	if (power_button_ready) {
		int ret = gpio_pin_interrupt_configure_dt(&power_button,
						  GPIO_INT_EDGE_TO_ACTIVE);

		if (ret < 0) {
			printk("Power button interrupt setup failed: %d\n", ret);
		}
	}
}
