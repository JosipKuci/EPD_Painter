#include "epd_pin_driver.h"
#include "epd_painter_powerctl.h"

void EPD_SRPin::set(bool high) {
    _sr->sr_set_bit(_index, high);
}
