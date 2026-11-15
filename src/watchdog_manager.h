#ifndef WATCHDOG_MANAGER_H_
#define WATCHDOG_MANAGER_H_

#include <stdbool.h>

int watchdog_manager_init(void);
void watchdog_manager_feed(void);

#endif /* WATCHDOG_MANAGER_H_ */
