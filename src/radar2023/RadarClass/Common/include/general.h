#ifndef GENERAL_H
#define GENERAL_H

#include "./public.h"

bool is_inside(std::vector<Point2f> &box, Point2f point);

int mod(int a, int b);

Rect rectCenterScale(Rect rect, Size size);

Rect reMapRect(Rect &rect, int blocksizeW, int blocksizeH);

bool read_param(Mat &K_0, Mat &C_0, Mat &E_0);

#endif