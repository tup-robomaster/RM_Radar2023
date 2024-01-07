#include "../include/general.h"

bool read_param(Mat &K_0, Mat &C_0, Mat &E_0, String path)
{
    if (access(path.c_str(), F_OK) != 0)
        return false;
    cv::FileStorage fs;
    try
    {
        fs = cv::FileStorage(path, FileStorage::READ);
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

#ifdef Experimental
float sumConfAverage(std::vector<bboxAndRect> &items)
{
    float sum = 0.;
    for (const auto &data : items)
    {
        sum += data.armor.conf;
    }

    return items.size() != 0 ? sum / items.size() : 0.;
}
#endif