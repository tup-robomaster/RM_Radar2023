#ifndef _VIDEORECODER_H
#define _VIDEORECODER_H

#include "../../Common/include/public.h"

class VideoRecoder
{
private:
    VideoWriter vw;

public:
    VideoRecoder();
    ~VideoRecoder();

    void init(char *videoPath, int coder, Size size);
    void write(Mat &src);
    void release();
};

#endif