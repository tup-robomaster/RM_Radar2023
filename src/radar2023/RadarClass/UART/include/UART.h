#ifndef UART_H
#define UART_H

#include "../../Common/include/public.h"
#include "../include/offical_Judge_Handler.h"
#include "../include/GameData.h"
#include "../include/serial.h"
#include "../include/UARTPasser.h"

class UART
{
public:
    int ind = 0;
    int Id_red = 1;
    int Id_blue = 101;

    int buffercnt = 0;
    unsigned char buffer[1000] = {0U};
    unsigned int cmdID = 0;
    int indecode = 0;
    UARTPasser myUARTPasser;
    Offical_Judge_Handler myHandler;
    game_state Game_state;
    game_result Game_result;
    game_robot_HP Game_robot_HP;
    dart_status Game_dart_status;
    event_data Game_event_data;
    supply_projectile_action Game_supply_projectile_action;
    refree_warning Game_refree_warning;
    dart_remaining_time Game_dart_remaining_time;

private:
    union FloatAndByte
    {
        float union_float;
        unsigned char union_byte[4];
    } FAB;

    void FloatToBytes(unsigned char *data, float float_to_byte)
    {
        FAB.union_float = float_to_byte;
        for (int i = 0; i < 4; i++)
        {
            data[i] = FAB.union_byte[i];
        }
    }

    void Refree_Arial_Message();
    void Judge_Refresh_Result();
    void Referee_Game_Result();
    void Referee_dart_status();
    void Referee_event_data();
    void Refree_supply_projectile_action();
    void Refree_Warning();
    void Refree_dart_remaining_time();

    void Referee_Transmit_BetweenCar(unsigned int dataID, unsigned char ReceiverId, unsigned char data[4], MySerial &ser);
    void Referee_Transmit_Map(unsigned int cmdID, int datalength, int targetId, float x, float y, MySerial &ser);
    void Robot_Data_Transmit_Map(MySerial &ser);

public:
    UART();
    ~UART();

    void ControlLoop_red();
    void ControlLoop_blue();
    void read(MySerial &ser);
    void write(MySerial &ser);
};

#endif