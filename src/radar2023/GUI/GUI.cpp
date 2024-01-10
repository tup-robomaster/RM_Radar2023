#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <radar2023/Location.h>
#include <radar2023/Locations.h>
#include <unistd.h>

using namespace std;
using namespace cv;

static cv_bridge::CvImageConstPtr result_image;
static vector<radar2023::Location> locs;

void locations_msgCallback(const radar2023::Locations::ConstPtr &msg)
{
    locs.clear();
    for (auto &it : msg->locations)
    {
        locs.emplace_back(it);
    }
}

void image_msgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        result_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gui_node");
    ros::NodeHandle nh;
    std::string share_path = ros::package::getPath("radar2023");
    ros::Subscriber msg_sub = nh.subscribe("/radar2023/locations", 1, locations_msgCallback);
    image_transport::ImageTransport it_(nh);
    image_transport::Subscriber image_sub_;
    image_sub_ = it_.subscribe("/radar2023/result_view", 1, image_msgCallback);
    namedWindow("GUI", WINDOW_GUI_NORMAL);
    createTrackbar("Exit Program", "GUI", 0, 1, nullptr);
    setTrackbarPos("Exit Program", "GUI", 0);
    createTrackbar("Recorder", "GUI", 0, 1, nullptr);
    setTrackbarPos("Recorder", "GUI", 1);
    createTrackbar("CoverDepth", "GUI", 0, 1, nullptr);
    setTrackbarPos("CoverDepth", "GUI", 1);
    bool if_exit_program = false;
    bool _if_record = true;
    bool _if_coverDepth = true;
    string param_name, map_name;
    int ENEMY = 0;
    if (nh.searchParam("/gui/CoverDepth", param_name))
    {
        nh.getParam(param_name, _if_coverDepth);
        _if_coverDepth ? setTrackbarPos("CoverDepth", "GUI", 1) : setTrackbarPos("CoverDepth", "GUI", 0);
    }
    else
    {
        ROS_WARN("Parameter CoverDepth not defined");
    }
    if (nh.searchParam("/gui/MapRMUC", param_name))
    {
        nh.getParam(param_name, map_name);
    }
    else
    {
        ROS_WARN("Parameter SerialPortName not defined");
    }
    if (nh.searchParam("/radar2023/EnemyType", param_name))
    {
        nh.getParam(param_name, ENEMY);
    }
    else
    {
        ROS_WARN("Parameter SerialPortName not defined");
    }
    Mat map;
    if (!map_name.empty() && access((share_path + "/resources/" + map_name).c_str(), F_OK) == 0)
    {
        map = cv::imread(share_path + "/resources/" + map_name);
        if (ENEMY == 1)
            cv::flip(map, map, -1);
    }
    else
    {
        ROS_WARN("NON MAP !");
    }
    while (ros::ok() && !if_exit_program)
    {
        ros::spinOnce();
        if (!result_image || result_image->image.empty())
            continue;
        Mat display = Mat::zeros(Size(result_image->image.cols + result_image->image.rows * 0.54, result_image->image.rows), CV_8UC3);
        result_image->image.copyTo(display(Rect(0, 0, result_image->image.cols, result_image->image.rows)));
        if (!map.empty())
        {
            cv::resize(map, display(Rect(result_image->image.cols, 0, result_image->image.rows * 0.54, result_image->image.rows)), Size(result_image->image.rows * 0.54, result_image->image.rows));
        }
        for (auto it : locs)
        {
            if (it.x == 0 && it.y == 0)
                continue;
            if (ENEMY == 1)
            {
                it.x = 28.0 - it.x;
                it.y = 15.0 - it.y;
            }
            Scalar color;
            string text;
            if (it.id < 6)
            {
                color = Scalar(255, 0, 0);
                text += "B";
            }
            else
            {
                color = Scalar(0, 0, 255);
                text += "R";
            }
            Point2f center = Point2f((it.y / 15.0) * result_image->image.rows * 0.54 + result_image->image.cols, (it.x / 28.0) * result_image->image.rows);
            circle(display, center, 40.0, color, 3);
            text += to_string(it.id);
            int baseline;
            Size text_size = getTextSize(text, FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
            center.x = center.x - text_size.width / 2;
            center.y = center.y + (text_size.height) / 2;
            putText(display, text, center, FONT_HERSHEY_SIMPLEX, 1, color, 2);
        }
        cv::imshow("GUI", display);
        cv::resizeWindow("GUI", Size(1536, 864));
        bool if_exit_program_current = getTrackbarPos("Exit Program", "GUI") == 1 ? true : false;
        bool _if_record_current = getTrackbarPos("Recorder", "GUI") == 1 ? true : false;
        bool _if_coverDepth_current = getTrackbarPos("CoverDepth", "GUI") == 1 ? true : false;
        if (if_exit_program_current != if_exit_program)
        {
            if_exit_program = if_exit_program_current;
            nh.setParam("/radar2023/ExitProgram", if_exit_program);
        }
        if (_if_record_current != _if_record)
        {
            _if_record = _if_record_current;
            nh.setParam("/radar2023/Recorder", _if_record);
        }
        if (_if_coverDepth_current != _if_coverDepth)
        {
            _if_coverDepth = _if_coverDepth_current;
            nh.setParam("/gui/CoverDepth", _if_coverDepth);
        }
        cv::waitKey(1);
    }
    cv::destroyAllWindows();
    ROS_INFO("GUI EXIT");
}