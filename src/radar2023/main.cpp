#include "PublicInclude/Radar.h"

int main(int argc, char **argv)
{
    Radar myLidar(argc, argv);
    while (true)
    {
        myLidar.spin(argc, argv);
    }
    myLidar.stop();
    cout << "---------Programe Ended---------" << endl;
    return 0;
}