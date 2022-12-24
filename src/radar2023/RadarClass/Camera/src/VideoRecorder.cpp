#include "../include/VideoRecorder.h"

VideoRecorder::VideoRecorder()
{
}

VideoRecorder::~VideoRecorder()
{
    this->close();
}

void VideoRecorder::init(char *videoPath, int coder, Size size)
{
    if (!this->vw.isOpened())
    {
        char filename[1024];
        time_t currentTime = time(NULL);
        char chCurrentTime[256];
        strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d %H%M%S", localtime(&currentTime));
        strcat(chCurrentTime, ".mp4");
        strcpy(filename, videoPath);
        strcat(filename, chCurrentTime);
        this->vw = VideoWriter();
        if (!this->vw.open(filename, coder, 15.0, size, true))
        {
            this->logger->warn("Block Video Recoder");
        }
    }
}

void VideoRecorder::write(Mat &src)
{
    if (!this->vw.isOpened())
        return;
    this->vw.write(src);
}

void VideoRecorder::close()
{
    this->vw.release();
}