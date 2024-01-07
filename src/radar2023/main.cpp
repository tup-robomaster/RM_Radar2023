#include "./RadarClass/Radar/include/Radar.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar2023");
    ros::NodeHandle nh;
    SpdLogger myLogger;
    std::string share_path = ros::package::getPath("radar2023");
    myLogger.registerLogger((share_path + "/logs/").c_str(), (char *)"RadarLogger");
    Radar::Ptr myRadar = std::make_shared<Radar>();
    myRadar->setRosNodeHandle(nh);
    myRadar->setRosPackageSharedPath(share_path);
    while (myRadar->alive())
    {
        myRadar->spin();
        waitKey(1);
    }
    cout << "---------Program END---------" << endl;
    return 0;
}