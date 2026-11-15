#include "watchdog_manager.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/printk.h>

#define WDT_TIMEOUT_MS 6000U
#define WDT_FEED_INTERVAL_MS 1500U

static const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int wdt_channel_id = -1;
static struct k_work_delayable wdt_feed_work;

void watchdog_manager_feed(void)
{
	if (wdt_channel_id >= 0 && device_is_ready(wdt)) {
		wdt_feed(wdt, wdt_channel_id);
	}
}

static void wdt_feed_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	watchdog_manager_feed();
	(void)k_work_reschedule(&wdt_feed_work, K_MSEC(WDT_FEED_INTERVAL_MS));
}

int watchdog_manager_init(void)
{
	if (!device_is_ready(wdt)) {
		printk("Watchdog device not ready\n");
		return -ENODEV;
	}

	struct wdt_timeout_cfg wdt_config = {
		.flags = WDT_FLAG_RESET_SOC,
		.window.min = 0U,
		.window.max = WDT_TIMEOUT_MS,
		.callback = NULL,
	};

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	if (wdt_channel_id < 0) {
		printk("Watchdog install timeout failed: %d\n", wdt_channel_id);
		return wdt_channel_id;
	}

	int ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (ret < 0) {
		printk("Watchdog setup failed: %d\n", ret);
		return ret;
	}

	printk("Hardware watchdog started: timeout=%u ms, channel=%d\n",
	       WDT_TIMEOUT_MS, wdt_channel_id);

	k_work_init_delayable(&wdt_feed_work, wdt_feed_work_fn);
	(void)k_work_schedule(&wdt_feed_work, K_MSEC(WDT_FEED_INTERVAL_MS));
	return 0;
}
