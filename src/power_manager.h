#ifndef POWER_MANAGER_H_
#define POWER_MANAGER_H_

#include <stdbool.h>

#define POWER_GPREGRET_SHUTDOWN_WAKE    0xA5U
#define POWER_GPREGRET_SHUTDOWN_NO_WAKE 0x5AU

typedef void (*power_shutdown_prepare_cb_t)(void);

int power_manager_init(power_shutdown_prepare_cb_t shutdown_prepare);
void power_manager_start(void);
void keyboard_power_set(bool enabled);
bool keyboard_power_is_enabled(void);
void power_manager_shutdown(void);
void power_manager_shutdown_no_wake(void);

#endif /* POWER_MANAGER_H_ */
