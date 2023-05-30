#ifndef GENERAL_H
#define GENERAL_H

#include "./public.h"

bool read_param(Mat &K_0, Mat &C_0, Mat &E_0);

float f_min(float x, float y);

float f_max(float x, float y);

void makeRectSafe(Rect &rect, Mat &src);

#endif