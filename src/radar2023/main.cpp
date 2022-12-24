#include "./RadarClass/Radar/include/Radar.h"
#include "./RadarClass/Logger/include/Logger.h"

int main(int argc, char **argv)
{
    Logger myLogger;
    myLogger.registerLogger(LOGPATH, (char *)"RadarLogger");
    Radar myRadar;
    while (myRadar.alive())
    {
        myRadar.spin(argc, argv);
        waitKey(1);
    }
    cout << "---------Program END---------" << endl;
    return 0;
}