#ifndef SERIAL_H
#define SERIAL_H

#include "./public.h"

class MySerial
{
public:
    int fd, flag, wr_num = 0, rr_num = 0;
    struct termios options, newstate;
    speed_t baud_rate_i, baud_rate_o;

public:
    MySerial();
    ~MySerial();

    void initSerial();
    bool _is_open();

    void msread(unsigned char buffer[], size_t size);
    void mswrite(unsigned char buffer[], size_t size);
};

#endif