#ifndef GENERAL_H
#define GENERAL_H

#include "./public.h"

bool is_inside(std::vector<Point2f> &box, Point2f point);

int mod(int a, int b);

Rect rectCenterScale(Rect rect, Size size);

Rect reMapRect(Rect &rect, int blocksizeW, int blocksizeH);

bool read_param(Mat &K_0, Mat &C_0, Mat &E_0);

float f_min(float x, float y);

float f_max(float x, float y);

map<string, Point3f> location_targets = {{"red_base",Point3f(1.760, -15. + 7.539, 0.200 + 0.920)},
                                         {"blue_outpost",Point3f(16.776, -15. + 12.565, 1.760)},
                                         {"red_outpost",Point3f(11.176, -15. + 2.435, 1.760)},
                                         {"blue_base",Point3f(26.162, -15. + 7.539, 0.200 + 0.920)},
                                         {"r_rt",Point3f(8.805, -5.728 - 0.660, 0.120 + 0.495)},
                                         {"r_lt",Point3f(8.805, -5.728, 0.120 + 0.495)},
                                         {"b_rt",Point3f(19.200, -9.272 + 0.660, 0.120 + 0.495)},
                                         {"b_lt",Point3f(19.200, -9.272, 0.120 + 0.495)}};

#endif