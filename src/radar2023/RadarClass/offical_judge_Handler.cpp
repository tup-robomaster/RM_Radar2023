#include "../PublicInclude/offical_Judge_Handler.h"

Offical_Judge_Handler::Offical_Judge_Handler()
{
}

Offical_Judge_Handler::~Offical_Judge_Handler()
{
}

unsigned char Offical_Judge_Handler::myGet_CRC8_Check_Sum(unsigned char *pchMessage, int dwLength, unsigned char ucCRC8)
{
    int tmp = 0;
    while (dwLength > 0)
    {
        --dwLength;
        unsigned char ucIndex = ucCRC8 ^ pchMessage[tmp];
        tmp++;
        ucCRC8 = this->myCRC8_TAB[ucIndex];
    }
    return ucCRC8;
}

bool Offical_Judge_Handler::myVerify_CRC8_Check_Sum(unsigned char *pchMessage, int dwLength)
{
    if (*pchMessage == 0U || dwLength <= 2)
        return false;
    unsigned char ucExpected = myGet_CRC8_Check_Sum(pchMessage, dwLength - 1, this->myCRC8_INIT);
    return ucExpected == pchMessage[dwLength - 1];
}

void Offical_Judge_Handler::Append_CRC8_Check_Sum(unsigned char *pchMessage, int dwLength)
{
    if (*pchMessage == 0U || dwLength <= 2)
        return;
    unsigned char ucCRC = myGet_CRC8_Check_Sum(pchMessage, dwLength - 1, myCRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

unsigned int Offical_Judge_Handler::myGet_CRC16_Check_Sum(unsigned char *pchMessage, int dwLength, unsigned int wCRC)
{
    if (*pchMessage == 0U)
        return 0xFFFF;
    int tmp = 0;
    while (dwLength)
    {
        --dwLength;
        unsigned char chData = pchMessage[tmp];
        ++tmp;
        wCRC = (wCRC >> 8) ^ mywCRC_Table[(wCRC ^ chData) & 0x00ff];
    }
    return wCRC;
}

bool Offical_Judge_Handler::myVerify_CRC16_Check_Sum(unsigned char *pchMessage, int dwLength)
{
    if (*pchMessage == 0U || dwLength <= 2)
        return false;
    unsigned char wExpected = myGet_CRC16_Check_Sum(pchMessage, dwLength - 2, myCRC16_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void Offical_Judge_Handler::Append_CRC16_Check_Sum(unsigned char *pchMessage, int dwLength)
{
    if (*pchMessage == 0 || dwLength <= 2)
        return;
    unsigned char wCRC = myGet_CRC16_Check_Sum(pchMessage, dwLength - 2, myCRC16_INIT);
    pchMessage[dwLength - 2] = (wCRC & 0x00ff);
    pchMessage[dwLength - 1] = ((wCRC >> 8) & 0x00ff);
}