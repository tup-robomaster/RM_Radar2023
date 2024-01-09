#ifndef GENERAL_H
#define GENERAL_H

#include "./public.h"

bool read_param(Mat &K_0, Mat &C_0, Mat &E_0, String path);

float f_min(float x, float y);

float f_max(float x, float y);

void makeRectSafe(Rect &rect, Mat &src);

Rect rectCenterScale(Rect rect, Size size);

Rect reMapRect(Rect &rect, int blocksizeW, int blocksizeH);

#ifdef ExperimentalOutput
float sumConfAverage(std::vector<bboxAndRect> &items);
#endif

void hsv_to_bgr(int h, int s, int v, int &b, int &g, int &r);

#endif