#ifndef WATCHDOG_H
#define WATCHDOG_H

void watchdog_init(void);
void watchdog_service_cpu0(void);
void watchdog_service_cpu1(void);

#endif
