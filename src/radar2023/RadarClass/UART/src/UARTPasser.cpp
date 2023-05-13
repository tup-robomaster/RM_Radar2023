#include "../include/UARTPasser.h"

UARTPasser::UARTPasser()
{
}

UARTPasser::~UARTPasser()
{
}

int UARTPasser::bytes2Int(unsigned char a, unsigned char b)
{
    return (0x0000 | a) | (b << 8);
}

float UARTPasser::bytesToFloat(unsigned char bytes[])
{
    return *((float *)bytes);
}

void UARTPasser::_judge_max_hp(int _HP[16])
{
    int temp[10];
    int count = 0;
    for (int i = 0; i < 16; ++i)
    {
        if (i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 8 || i == 9 || i == 10 || i == 11 || i == 12)
        {
            temp[count] = _HP[i];
            count++;
        }
    }
    for (int i = 0; i < 10; ++i)
    {
        if (i == 1 || i == 9)
            continue;
        if (temp[i] - this->_last_hp[i] > 30 && temp[i] - this->_last_hp[i] <= 80 && this->_last_hp > 0)
            this->_max_hp[i] += 50;
        if (temp[i] - this->_last_hp[i] <= 80 && this->_last_hp > 0)
            this->_max_hp[i] += 100;
        if (temp[i] > this->_max_hp[i])
        {
            for (int j = 0; j < 9; ++j)
            {
                if (temp[i] < this->_hp_up[i])
                    this->_max_hp[i] = this->_hp_up[i];
            }
        }
    }
    count = 0;
    for (int i = 0; i < 16; ++i)
    {
        if (i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 8 || i == 9 || i == 10 || i == 11 || i == 12)
        {
            _HP[i] = temp[count];
            count++;
        }
    }
}

void UARTPasser::push_loc(vector<vector<float>> &location)
{
    this->_robot_location.swap(location);
}

vector<vector<float>> UARTPasser::get_position()
{
    return this->_robot_location;
}

void UARTPasser::get_message()
{
    // TODO:信息获取接口
}

void UARTPasser::Refree_MapLocationSelf_Message()
{
    // TODO:位置获取接口
}

void UARTPasser::Referee_Update_GameData(unsigned char *buffer)
{
    if (this->_Now_stage < 2 && ((buffer[7] >> 4) == 2 || (buffer[7] >> 4) == 3 || (buffer[7] >> 4) == 4))
    {
        this->_Game_Start_Flag = true;
        this->_set_max_flag = true;
        this->Remain_time = 420;
        this->logger->critical("GAME START !");
    }
    if (this->_Now_stage < 5 && (buffer[7] >> 4) == 5)
    {
        this->_Game_End_Flag = true;
        this->logger->critical("GAME END !");
        for (int i = 0; i < 10; ++i)
        {
            this->_max_hp[i] = this->_init_hp[i];
        }
        this->Remain_time = 0;
    }
    this->_Now_stage = buffer[7] >> 4;
}

void UARTPasser::Referee_Robot_HP(unsigned char *buffer)
{
    for (int i = 0; i < 16; ++i)
    {
        this->_HP[0] = this->bytes2Int(buffer[(i + 4) * 2 - 1], buffer[(i + 4) * 2]);
    }
}

BOData UARTPasser::One_compete_end()
{
    BOData temp;
    if (this->_Game_End_Flag)
    {
        this->_Game_End_Flag = false;
        ++this->_BO;
        temp.GameEndFlag = true;
        temp.remainBO = this->_BO - MAXBO;
    }
    return temp;
}

bool UARTPasser::One_compete_start()
{
    if (this->_Game_Start_Flag)
    {
        this->_Game_Start_Flag = false;
        return true;
    }
    else
        return false;
}

void UARTPasser::Receive_Robot_Data(unsigned char *buffer)
{
    if ((0x0000 | buffer[7]) | ((buffer[8] << 8) == 0x0200))
        this->logger->info("received");
}