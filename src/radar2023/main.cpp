#include "./RadarClass/Radar/include/Radar.h"

int main(int argc, char **argv)
{
    Radar myRadar;
    while (myRadar.alive())
    {
        myRadar.spin(argc, argv);
        waitKey(1);
    }
    cout << "---------Program END---------" << endl;
    return 0;
}