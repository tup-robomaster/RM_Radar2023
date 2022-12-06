#ifndef __GAMEDATA_H
#define __GAMEDATA_H

#include "../../Common/include/public.h"

class game_state
{
public:
    unsigned char game_type = 4;
    unsigned char game_progress = 4;
    unsigned char stage_remain_time[2] = {0U, 0U};
};

class game_result
{
public:
    unsigned char winner = 0U;
};

class game_robot_HP
{
public:
    unsigned char red_1_robot_HP[2] = {0U, 0U};
    unsigned char red_2_robot_HP[2] = {0U, 0U};
    unsigned char red_3_robot_HP[2] = {0U, 0U};
    unsigned char red_4_robot_HP[2] = {0U, 0U};
    unsigned char red_5_robot_HP[2] = {0U, 0U};
    unsigned char red_7_robot_HP[2] = {0U, 0U};
    unsigned char red_outpost_HP[2] = {0U, 0U};
    unsigned char red_base_HP[2] = {0U, 0U};
    unsigned char blue_1_robot_HP[2] = {0U, 0U};
    unsigned char blue_2_robot_HP[2] = {0U, 0U};
    unsigned char blue_3_robot_HP[2] = {0U, 0U};
    unsigned char blue_4_robot_HP[2] = {0U, 0U};
    unsigned char blue_5_robot_HP[2] = {0U, 0U};
    unsigned char blue_7_robot_HP[2] = {0U, 0U};
    unsigned char blue_outpost_HP[2] = {0U, 0U};
    unsigned char blue_base_HP[2] = {0U, 0U};
};

class dart_status
{
public:
    unsigned char dart_belong = 0U;
    unsigned char stage_remaining_time[2] = {0U, 0U};
};

class event_data
{
public:
    unsigned char event_type[4] = {0U, 0U, 0U, 0U};
};

class supply_projectile_action
{
public:
    unsigned char supply_projectile_id = 0U;
    unsigned char supply_robot_id = 0U;
    unsigned char supply_projectile_step = 0U;
    unsigned char supply_projectile_num = 0U;
};

class refree_warning
{
public:
    unsigned char level = 0U;
    unsigned char foul_robot_id = 0U;
};

class dart_remaining_time
{
public:
    unsigned char time = 0U;
};

class custom_data0U
{
public:
    unsigned char data1[4] = {0U, 0U, 0U, 0U};
    unsigned char data2[4] = {0U, 0U, 0U, 0U};
    unsigned char data3[4] = {0U, 0U, 0U, 0U};
    unsigned char masks = 0U;
};

class graphic_data_struct
{
public:
    vector<unsigned char> data;
    int datalength = 0U;
    unsigned char graphic_name[4] = {0U, 0U, 0U, 0U};
    unsigned char operate_tpye[4] = {0U, 0U, 0U, 0U};
    unsigned char graphic_tpye[4] = {0U, 0U, 0U, 0U};
    unsigned char layer[4] = {0U, 0U, 0U, 0U};
    unsigned char color[4] = {0U, 0U, 0U, 0U};
    unsigned char start_angle[4] = {0U, 0U, 0U, 0U};
    unsigned char end_angle[4] = {0U, 0U, 0U, 0U};
    unsigned char width[4] = {0U, 0U, 0U, 0U};
    unsigned char start_x[4] = {0U, 0U, 0U, 0U};
    unsigned char start_y[4] = {0U, 0U, 0U, 0U};
    unsigned char radius[4] = {0U, 0U, 0U, 0U};
    unsigned char end_x[4] = {0U, 0U, 0U, 0U};
    unsigned char end_y[4] = {0U, 0U, 0U, 0U};

public:
    void Add()
    {
        vector<unsigned char> temp;
        this->data.swap(temp);
        this->datalength = 15;
    };
};

class robot_location
{
public:
    vector<vector<float>> loc;

public:
    robot_location()
    {
        vector<vector<float>> temp(2, vector<float>(5, 0.f));
        this->loc.swap(temp);
    };

    void push(vector<vector<float>> loc)
    {
        this->loc.swap(loc);
    };
};

#endif