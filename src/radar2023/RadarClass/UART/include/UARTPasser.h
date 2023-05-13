#ifndef UARTPASSER_H
#define UARTPASSER_H

#include "../../Common/include/public.h"

/**
 * @brief UART传递
 * 官方裁判系统Demo的C++版本
 * 负责处理裁判系统数据
 */
class UARTPasser
{
private:
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    int _hp_up[9] = {100, 150, 200, 250, 300, 350, 400, 450, 500};
    int _init_hp[10] = {500};
    int _last_hp[10] = {500};
    int _HP[16] = {500};
    int _max_hp[10] = {500};
    bool _set_max_flag = false;
    vector<vector<float>> _robot_location = vector<vector<float>>(5, vector<float>(2, 0.f));
    int _BO = 0;
    vector<string> _stage = {"NOT START", "PREPARING", "CHECKING", "5S", "PLAYING", "END"};
    int _Now_stage = 0;
    bool _Game_Start_Flag = false;
    bool _Game_End_Flag = false;
    int Remain_time = 1;
    int _HP_thres = 10;
    float _prevent_time[6] = {2.f, 2.f, 2.f, 2.f, 2.f, 2.f};
    float _event_prevent[6] = {0.f};
    int loop_send = 0;

public:
    UARTPasser();
    ~UARTPasser();

    int bytes2Int(unsigned char a, unsigned char b);
    float bytesToFloat(unsigned char bytes[]);

    void _judge_max_hp(int _HP[16]);
    void push_loc(vector<vector<float>> &location);
    vector<vector<float>> get_position();
    void get_message();
    void Refree_MapLocationSelf_Message();
    void Referee_Update_GameData(unsigned char *buffer);
    void Referee_Robot_HP(unsigned char *buffer);
    BOData One_compete_end();
    bool One_compete_start();
    void Receive_Robot_Data(unsigned char *buffer);
};

#endif