#include "shelfbot.h"

// The ESP-IDF framework requires a C-style `app_main` entry point.
// We use `extern "C"` to prevent C++ name mangling.
extern "C" void app_main(void)
{
    Shelfbot shelfbot;
    shelfbot.begin();
}
