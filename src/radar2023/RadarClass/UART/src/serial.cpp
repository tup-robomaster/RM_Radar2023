#include "../include/serial.h"

MySerial::MySerial()
{
}

MySerial::~MySerial()
{
}

void MySerial::initSerial(std::string sername, std::string password)
{
    if (this->fd != -1)
        return;
    if (access(sername.c_str(), 0) == -1)
    {
        this->logger->warn("Serial :Serial Port Not Found !");
        return;
    }
    if (system(("echo " + password + " | sudo -S chmod a+rw " + sername).c_str()) != 0)
    {
        this->logger->error("Serial :Failed to get permission!");
        return;
    }
    this->fd = open(sername.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (this->fd == -1)
    {
        this->logger->error("Serial init failed");
        return;
    }

    if (fcntl(this->fd, F_SETFL, FNDELAY) < 0)
        this->logger->error("fcntl failed");
    else
        this->logger->info("fcntl={}", fcntl(this->fd, F_SETFL, FNDELAY));

    tcgetattr(this->fd, &this->options);

    // 设置波特率
    cfsetispeed(&this->options, B115200);
    cfsetospeed(&this->options, B115200);

    // 获取波特率
    tcgetattr(this->fd, &this->newstate);
    this->baud_rate_i = cfgetispeed(&this->newstate);
    this->baud_rate_o = cfgetospeed(&this->newstate);

    // 串口设置
    this->options.c_cflag |= (CLOCAL | CREAD);
    this->options.c_cflag &= ~PARENB; // 设置无奇偶校验位，N
    this->options.c_iflag &= ~INPCK; 
    this->options.c_cflag &= ~CSTOPB; // 设置停止位1
    this->options.c_cflag &= ~CSIZE;
    this->options.c_cflag |= CS8;  // 设置数据位
    this->options.c_cc[VTIME] = 150; // 阻塞模式的设置
    this->options.c_cc[VMIN] = 0;
    this->options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    this->options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    this->options.c_iflag &= ~(ICRNL | IGNCR);
    this->options.c_oflag &= ~OPOST;
    tcflush(this->fd, TCIOFLUSH);
    // 激活新配置
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