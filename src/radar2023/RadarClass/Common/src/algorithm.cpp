#include "../include/algorithm.h"

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

#ifdef ExperimentalOutput
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

void hsv_to_bgr(int h, int s, int v, int &b, int &g, int &r)
{
    h = std::max(0, std::min(255, h));
    s = std::max(0, std::min(255, s));
    v = std::max(0, std::min(255, v));

    float hh = h / 42.5f;
    int i = static_cast<int>(floor(hh));
    float f = hh - i;
    float p = v * (1.0f - s / 255.0f);
    float q = v * (1.0f - s / 255.0f * f);
    float t = v * (1.0f - s / 255.0f * (1.0f - f));

    switch (i)
    {
    case 0:
        b = round(p);
        g = round(t);
        r = round(v);
        break;
    case 1:
        b = round(p);
        g = round(v);
        r = round(q);
        break;
    case 2:
        b = round(t);
        g = round(v);
        r = round(p);
        break;
    case 3:
        b = round(v);
        g = round(q);
        r = round(p);
        break;
    case 4:
        b = round(v);
        g = round(p);
        r = round(t);
        break;
    case 5:
        b = round(q);
        g = round(p);
        r = round(v);
        break;
    }
}