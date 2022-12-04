#include "PublicInclude/Lidar.h"

int main(int argc, char **argv)
{
    Lidar myLidar(argc, argv);
    while (true)
    {
        myLidar.spin(argc, argv);
        waitKey(10);
    }
    myLidar.stop();
    cout << "---------Programe Ended---------" << endl;
    return 0;
}