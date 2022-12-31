#ifndef _VIDEORECORDER_H
#define _VIDEORECORDER_H

#include "../../Common/include/public.h"

/**
 * @brief 视频录制类
 * 提供对控制窗口图像的录制功能
 */
class VideoRecorder
{
private:
    VideoWriter vw;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    
public:
    VideoRecorder();
    ~VideoRecorder();

    void init(char *videoPath, int coder, Size size);
    void write(Mat src);
    void close();
};

#endif