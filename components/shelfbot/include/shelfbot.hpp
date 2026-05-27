#pragma once
#include <idf_c_includes.hpp>

class Shelfbot {
public:
    static Shelfbot& get_instance();
    esp_err_t begin();

private:
    Shelfbot() = default;
    static Shelfbot* instance_;
};