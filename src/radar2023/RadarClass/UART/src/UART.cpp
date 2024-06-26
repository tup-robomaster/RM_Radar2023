#include "../include/UART.h"

UART::UART(int ENEMY)
{
    this->ENEMY = ENEMY;
}

UART::~UART()
{
}

void UART::Judge_Refresh_Result()
{
    this->logger->debug("Judge_Refresh_Result");
}

void UART::Referee_Game_Result()
{
    this->Game_result.winner = this->buffer[7];
}

void UART::Referee_dart_status()
{
    this->Game_dart_status.dart_belong = this->buffer[7];
    this->Game_dart_status.stage_remaining_time[0] = this->buffer[8];
    this->Game_dart_status.stage_remaining_time[1] = this->buffer[9];
}

void UART::Referee_event_data()
{
    this->Game_event_data.event_type[0] = this->buffer[7];
    this->Game_event_data.event_type[1] = this->buffer[8];
    this->Game_event_data.event_type[2] = this->buffer[9];
    this->Game_event_data.event_type[3] = this->buffer[10];
}

void UART::Refree_supply_projectile_action()
{
    this->Game_supply_projectile_action.supply_projectile_id = this->buffer[7];
    this->Game_supply_projectile_action.supply_robot_id = this->buffer[8];
    this->Game_supply_projectile_action.supply_projectile_step = this->buffer[9];
    this->Game_supply_projectile_action.supply_projectile_num = this->buffer[10];
}

void UART::Refree_Warning()
{
    this->Game_refree_warning.level = this->buffer[7];
    this->Game_refree_warning.foul_robot_id = this->buffer[8];
}

void UART::Refree_dart_remaining_time()
{
    this->Game_dart_remaining_time.time = this->buffer[8];
}

void UART::Referee_Transmit_BetweenCar(unsigned int dataID, unsigned char ReceiverId, unsigned char data[48], MySerial::Ptr ser)
{
    unsigned char local_buffer[200];
    local_buffer[0] = 0xA5;
    local_buffer[1] = (54) & 0x00ff; // 数据帧中 data 的长度
    local_buffer[2] = ((54) & 0xff00) >> 8;
    local_buffer[3] = 0;
    local_buffer[4] = this->myHandler.Get_CRC8_Check_Sum(local_buffer, 5 - 1, 0xff);
    local_buffer[5] = 0x01;
    local_buffer[6] = 0x03;
    local_buffer[7] = dataID & 0x00ff;
    local_buffer[8] = (dataID & 0xff00) >> 8;
    if (this->ENEMY)
        local_buffer[9] = 9;
    else
        local_buffer[9] = 109;
    local_buffer[10] = 0;
    local_buffer[11] = ReceiverId;
    local_buffer[12] = 0;
    local_buffer[13] = data[0];
    local_buffer[14] = data[1];
    local_buffer[15] = data[2];
    local_buffer[16] = data[3];
    local_buffer[17] = data[4];
    local_buffer[18] = data[5];
    local_buffer[19] = data[6];
    local_buffer[20] = data[7];
    local_buffer[21] = data[8];
    local_buffer[22] = data[9];
    local_buffer[23] = data[10];
    local_buffer[24] = data[11];
    local_buffer[25] = data[12];
    local_buffer[26] = data[13];
    local_buffer[27] = data[14];
    local_buffer[28] = data[15];
    local_buffer[29] = data[16];
    local_buffer[30] = data[17];
    local_buffer[31] = data[18];
    local_buffer[32] = data[19];
    local_buffer[33] = data[20];
    local_buffer[34] = data[21];
    local_buffer[35] = data[22];
    local_buffer[36] = data[23];
    local_buffer[37] = data[24];
    local_buffer[38] = data[25];
    local_buffer[39] = data[26];
    local_buffer[40] = data[27];
    local_buffer[41] = data[28];
    local_buffer[42] = data[29];
    local_buffer[43] = data[30];
    local_buffer[44] = data[31];
    local_buffer[45] = data[32];
    local_buffer[46] = data[33];
    local_buffer[47] = data[34];
    local_buffer[48] = data[35];
    local_buffer[49] = data[36];
    local_buffer[50] = data[37];
    local_buffer[51] = data[38];
    local_buffer[52] = data[39];
    local_buffer[53] = data[40];
    local_buffer[54] = data[41];
    local_buffer[55] = data[42];
    local_buffer[56] = data[43];
    local_buffer[57] = data[44];
    local_buffer[58] = data[45];
    local_buffer[59] = data[46];
    local_buffer[60] = data[47];
    myHandler.Append_CRC16_Check_Sum(local_buffer, 54 + 9);
    unsigned char buffer_tmp_array[54 + 9];
    for (int i = 0; i < 54 + 9; ++i)
        buffer_tmp_array[i] = local_buffer[i];
    ser->mswrite(buffer_tmp_array, 54 + 9);
}

void UART::Referee_Transmit_Map(unsigned int cmdID, int targetId, float x, float y, MySerial::Ptr ser)
{
    unsigned char t_x[4], t_y[4];
    this->FloatToBytes(t_x, x);
    this->FloatToBytes(t_y, y);
    unsigned char local_buffer[200];
    local_buffer[0] = 0xA5;
    local_buffer[1] = (14) & 0x00ff; // 数据帧中 data 的长度
    local_buffer[2] = ((14) & 0xff00) >> 8;
    local_buffer[3] = 0;
    local_buffer[4] = this->myHandler.Get_CRC8_Check_Sum(local_buffer, 5 - 1, 0xff);
    local_buffer[5] = cmdID & 0x00ff;
    local_buffer[6] = (cmdID & 0xff00) >> 8;

    local_buffer[7] = targetId;
    local_buffer[8] = 0;
    local_buffer[9] = t_x[0];
    local_buffer[10] = t_x[1];
    local_buffer[11] = t_x[2];
    local_buffer[12] = t_x[3];
    local_buffer[13] = t_y[0];
    local_buffer[14] = t_y[1];
    local_buffer[15] = t_y[2];
    local_buffer[16] = t_y[3];
    local_buffer[17] = 0;
    local_buffer[18] = 0;
    local_buffer[19] = 0;
    local_buffer[20] = 0;
    myHandler.Append_CRC16_Check_Sum(local_buffer, 14 + 9);
    unsigned char buffer_tmp_array[14 + 9];
    for (int i = 0; i < 14 + 9; ++i)
        buffer_tmp_array[i] = local_buffer[i];
    ser->mswrite(buffer_tmp_array, 14 + 9);
}

void UART::Robot_Data_Transmit_Map(MySerial::Ptr ser)
{
    bool flag;
    vector<float> location = myUARTPasser.get_position()[this->ind];
    if (location[0] == 0 && location[1] == 0)
        flag = false;
    else
        flag = true;
    if (!this->ENEMY && flag)
    {
        this->Referee_Transmit_Map(0x0305, this->Id_red, _Float32(location[0]), _Float32(location[1]), ser);
    }
    else if (flag)
    {
        this->Referee_Transmit_Map(0x0305, this->Id_blue, _Float32(location[0]), _Float32(location[1]), ser);
    }
    if (flag)
        ++this->myUARTPasser.loop_send;
    if (this->ind == 5)
    {
        if (this->myUARTPasser.loop_send == 0)
            this->myUARTPasser.loop_send = 0;
    }
    this->ControlLoop_red();
    this->ControlLoop_blue();
    this->ind = (this->ind + 1) % 6;
}

void UART::ControlLoop_red()
{
    if (this->Id_red == 5)
        this->Id_red = 7;
    else if (this->Id_red == 7)
        this->Id_red = 1;
    else
        ++this->Id_red;
}

void UART::ControlLoop_blue()
{
    if (this->Id_blue == 105)
        this->Id_blue = 107;
    else if (this->Id_blue == 107)
        this->Id_blue = 101;
    else
        ++this->Id_blue;
}

void UART::read(MySerial::Ptr ser)
{
    unsigned char tempBuffer[1];
    ser->msread(tempBuffer, 1);
    int s = (int)tempBuffer[0];
    if (this->buffercnt > 50)
        this->buffercnt = 0;
    this->buffer[this->buffercnt] = s;
    if (this->buffercnt == 0)
    {
        if (this->buffer[this->buffercnt] != 0xa5)
        {
            this->buffercnt = 0;
            return;
        }
    }
    if (this->buffercnt == 5)
    {
        if (myHandler.Verify_CRC8_Check_Sum(this->buffer, 5) == 0)
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 7)
        this->cmdID = (0x0000 | buffer[5]) | (buffer[6] << 8);
    if (this->buffercnt == 25 && this->cmdID == 0x0203)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 25))
        {
            this->myUARTPasser.Refree_MapLocationSelf_Message();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 10 && this->cmdID == 0x0002)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->Referee_Game_Result();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 20 && this->cmdID == 0x0001)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 20))
        {
            this->myUARTPasser.Referee_Update_GameData(this->buffer);
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 41 && this->cmdID == 0x0003)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 41))
        {
            this->myUARTPasser.Referee_Robot_HP(this->buffer);
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 12 && this->cmdID == 0x0004)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 12))
        {
            this->Referee_dart_status();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 13 && this->cmdID == 0x0101)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 13))
        {
            this->Referee_event_data();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 13 && this->cmdID == 0x0102)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 13))
        {
            this->Refree_supply_projectile_action();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 11 && this->cmdID == 0x0104)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 11))
        {
            this->Refree_Warning();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 10 && this->cmdID == 0x0105)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->Refree_dart_remaining_time();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 17 && this->cmdID == 0x301)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 17))
        {
            this->myUARTPasser.Receive_Robot_Data(this->buffer);
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 25 && this->cmdID == 0x202)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 25))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 25 && this->cmdID == 0x203)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 25))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 27 && this->cmdID == 0x201)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 27))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 10 && this->cmdID == 0x204)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 10 && this->cmdID == 0x206)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 13 && this->cmdID == 0x209)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 13))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 20 && this->cmdID == 0x0301)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 20))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 36 && this->cmdID == 0x020B)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 36))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    if (this->buffercnt == 4 && this->cmdID == 0x0104)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 4))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    ++this->buffercnt;
}

void UART::write(MySerial::Ptr ser)
{
    this->Robot_Data_Transmit_Map(ser);
    unsigned int dataID = 0x0200 + (1 & 0xFF);
    unsigned char receiverId;
    if (this->ENEMY)
        receiverId = 7;
    else
        receiverId = 107;
    unsigned char data[48];
    vector<vector<float>> positions = myUARTPasser.get_position();
    for (int i = 0; i < 6; ++i)
    {
        unsigned char t_x[4], t_y[4];
        this->FloatToBytes(t_x, positions[i][0]);
        this->FloatToBytes(t_y, positions[i][1]);
        int data_p = i * 8;
        data[data_p + 0] = t_x[0];
        data[data_p + 1] = t_x[1];
        data[data_p + 2] = t_x[2];
        data[data_p + 3] = t_x[3];
        data[data_p + 4] = t_y[0];
        data[data_p + 5] = t_y[1];
        data[data_p + 6] = t_y[2];
        data[data_p + 7] = t_y[3];
    }
    Referee_Transmit_BetweenCar(dataID, receiverId, data, ser);
    usleep(10000);
}
