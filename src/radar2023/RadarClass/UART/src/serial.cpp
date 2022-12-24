#include "../include/serial.h"

MySerial::MySerial()
{
}

MySerial::~MySerial()
{
}

void MySerial::initSerial()
{
    if(this->fd != -1)
        return;
    this->fd = open(SerialPortNAME, O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (this->fd == -1)
    {
        // fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
        //            "[ERROR], {}!\n", "Serial init failed");
        return;
    }

    // if (fcntl(this->fd, F_SETFL, 0) < 0) //改为阻塞模式
        // fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
        //            "[ERROR], {}!\n", "fcntl failed");
    // else
    //     fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
    //                "[INFO], fcntl={}...\n", fcntl(this->fd, F_SETFL, 0));

    tcgetattr(this->fd, &this->options);

    //设置波特率
    cfsetispeed(&this->options, B115200);
    cfsetospeed(&this->options, B115200);

    //获取波特率
    tcgetattr(this->fd, &this->newstate);
    this->baud_rate_i = cfgetispeed(&this->newstate);
    this->baud_rate_o = cfgetospeed(&this->newstate);

    //串口设置
    this->options.c_cflag |= (CLOCAL | CREAD);
    this->options.c_cflag &= ~PARENB; //设置无奇偶校验位，N
    this->options.c_cflag &= ~CSTOPB; //设置停止位1
    this->options.c_cflag &= ~CSIZE;
    this->options.c_cflag |= CS8;  //设置数据位
    this->options.c_cc[VTIME] = 0; //阻塞模式的设置
    this->options.c_cc[VMIN] = 1;

    //激活新配置
    tcsetattr(this->fd, TCSANOW, &this->options);
}

bool MySerial::_is_open()
{
    return this->fd != -1;
}

void MySerial::msread(unsigned char buffer[], size_t size)
{
    this->rr_num = read(this->fd, buffer, size);
}

void MySerial::mswrite(unsigned char buffer[], size_t size)
{
    this->wr_num = write(fd, buffer, size);
}