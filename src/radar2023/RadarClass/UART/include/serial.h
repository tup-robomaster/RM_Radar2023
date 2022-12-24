#ifndef SERIAL_H
#define SERIAL_H

#include "../../Common/include/public.h"

class MySerial
{
public:
    int fd = -1;
    int flag;
    int wr_num = 0;
    int rr_num = 0;
    struct termios options, newstate;
    speed_t baud_rate_i, baud_rate_o;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    MySerial();
    ~MySerial();

    void initSerial();
    bool _is_open();

    void msread(unsigned char buffer[], size_t size);
    void mswrite(unsigned char buffer[], size_t size);
};

#endif