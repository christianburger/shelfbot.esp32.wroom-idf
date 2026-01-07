#ifndef WIFI_STATION_H
#define WIFI_STATION_H

#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"

void wifi_init_sta();

#endif // WIFI_STATION_H
