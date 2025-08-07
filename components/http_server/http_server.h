#ifndef MAIN_HTTP_SERVER_H_
#define MAIN_HTTP_SERVER_H_

#include "esp_err.h"

// Starts the web server
esp_err_t start_webserver();

// Stops the web server
esp_err_t stop_webserver();

#endif /* MAIN_HTTP_SERVER_H_ */
