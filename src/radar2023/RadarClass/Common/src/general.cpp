#include "../include/general.h"

bool is_inside(std::vector<Point2f> &box, Point2f point)
{
    Point2f AM = point - box[0];
    Point2f AB = box[1] - box[0];
    Point2f BM = point - box[1];
    Point2f BC = box[2] - box[1];
    Point2f CM = point - box[2];
    Point2f CD = box[3] - box[2];
    Point2f DM = point - box[3];
    Point2f DA = box[0] - box[3];
    float a = AM.x * AB.y - AB.x * AM.y;
    float b = BM.x * BC.y - BC.x * BM.y;
    float c = CM.x * CD.y - CD.x * CM.y;
    float d = DM.x * DA.y - DA.x * DM.y;
    return (a >= 0 && b >= 0 && c >= 0 && d >= 0) ||
           (a <= 0 && b <= 0 && c <= 0 && d <= 0);
}

int mod(int a, int b)
{
    return a - floor(a / b) * b;
}

Rect rectCenterScale(Rect rect, Size size)
{
    rect = rect + size;
    Point pt;
    pt.x = cvRound(size.width / 2.0);
    pt.y = cvRound(size.height / 2.0);
    return (rect - pt);
}

Rect reMapRect(Rect &rect, int blocksizeW, int blocksizeH)
{
    return Rect(rect.x * blocksizeW, rect.y * blocksizeH, rect.width * blocksizeW, rect.height * blocksizeH);
}

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