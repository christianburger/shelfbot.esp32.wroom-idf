#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "motor_control.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"

void start_webserver(void);
void stop_webserver(void);

#endif // HTTP_SERVER_H
