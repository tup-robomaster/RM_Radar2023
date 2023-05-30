#include "../include/general.h"

bool read_param(Mat &K_0, Mat &C_0, Mat &E_0)
{
    if (access(CAMERA_PARAM_PATH, F_OK) != 0)
        return false;
    cv::FileStorage fs;
    try
    {
        fs = cv::FileStorage(CAMERA_PARAM_PATH, FileStorage::READ);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    fs["K_0"] >> K_0;
    fs["C_0"] >> C_0;
    fs["E_0"] >> E_0;
    return true;
}

float f_min(float x, float y)
{
    return floor((x + y - abs(x - y)) / 2);
}

float f_max(float x, float y)
{
    return floor((x + y + abs(x - y)) / 2);
}

void makeRectSafe(Rect &rect, Mat &src)
{
    rect &= Rect(0, 0, src.cols, src.rows);
}