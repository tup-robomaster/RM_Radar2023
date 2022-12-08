#include "../include/VideoRecoder.h"

VideoRecoder::VideoRecoder()
{
}

VideoRecoder::~VideoRecoder()
{
    this->release();
}

void VideoRecoder::init(char *videoPath, int coder, Size size)
{
    if (!this->vw.isOpened())
    {
        char filename[1024];
        time_t currentTime = time(NULL);
        char chCurrentTime[256];
        strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d %H%M%S", localtime(&currentTime));
        strcpy(filename, videoPath);
        strcat(filename, chCurrentTime);
        this->vw = VideoWriter(filename, coder, 60.0, size, true);
    }
}

void VideoRecoder::write(Mat &src)
{
    if (!this->vw.isOpened())
        return;
    this->vw.write(src);
}

void VideoRecoder::release()
{
    if (this->vw.isOpened())
        this->vw.release();
}