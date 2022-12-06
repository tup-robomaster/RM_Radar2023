#include "./RadarClass/Radar/include/Radar.h"

int main(int argc, char **argv)
{
    Radar myLidar(argc, argv);
    while (myLidar.alive())
    {
        myLidar.spin(argc, argv);
        waitKey(1);
    }
    cout << "---------Program END---------" << endl;
    return 0;
}