#include "../include/camera.h"

#ifndef UsingVideo
// unsigned char camera_match_index = (char)0;                  //相机索引号

MV_Camera::MV_Camera()
{
}

MV_Camera::MV_Camera(bool Is_init)
{
    CameraSdkInit(1);
    this->iStatus = CameraEnumerateDevice(this->tCameraEnumList, &this->iCameraCounts);
    if (this->iCameraCounts == 0)
    {
        this->logger->error("No camera found!");
        return;
    }
    // this->iStatus = -1;
    // for(int i=0;i < this->iCameraCounts;i++){
    //     if(camera_match_index == this->tCameraEnumList[i].uInstance)
    //         iStatus = 0;
    // }
    // if(this->iStatus==-1){
    //     cout<<"None Using Camera Found!"<<endl;
    //     return;
    // }
    try
    {
        this->iStatus = CameraInit(&this->tCameraEnumList[0], -1, -1, &this->hCamera);
        this->logger->info("CameraIniting ...Process");
    }
    catch (const std::exception &e)
    {
        this->logger->error("CameraInitERROR!{}", e.what());
        return;
    }
    if (this->iStatus != CAMERA_STATUS_SUCCESS)
    {
        this->logger->error("CameraInit Failed!{}", this->iStatus);
        return;
    }
    else
    {
        this->logger->info("CameraIniting ...Done");
    }
    CameraGetCapability(this->hCamera, &this->tCapability);
    if (!tCapability.sIspCapacity.bMonoSensor)
        CameraSetIspOutFormat(this->hCamera, CAMERA_MEDIA_TYPE_BGR8);
    else
    {
        this->logger->error("None suitable camera!");
    }
    CameraSetTriggerMode(this->hCamera, 0);
    if (Is_init)
        CameraReadParameterFromFile(this->hCamera, CameraConfigPath);
    CameraSetAeState(this->hCamera, 0);
    CameraPlay(this->hCamera);
    int frameBufferSize = this->tCapability.sResolutionRange.iWidthMax * this->tCapability.sResolutionRange.iHeightMax * 3;
    this->pFrameBuffer = CameraAlignMalloc(frameBufferSize, 16);
    this->logger->info("Camera setup complete");
}

MV_Camera::~MV_Camera()
{
}

FrameBag MV_Camera::read()
{
    FrameBag framebag;
    if (this->hCamera == -1)
    {
        this->logger->error("No handled camera found!");
        return framebag;
    }
    CameraGetImageBuffer(this->hCamera, &this->sFrameInfo, &this->pRawDataBuffer, 200);
    if (CameraImageProcess(this->hCamera, this->pRawDataBuffer, this->pFrameBuffer, &this->sFrameInfo) != CAMERA_STATUS_SUCCESS)
    {
        this->logger->error("Can not process Image!");
        return framebag;
    }
    framebag.frame = cv::Mat(
        Size(this->sFrameInfo.iWidth, this->sFrameInfo.iHeight),
        CV_8UC3,
        this->pFrameBuffer);
    CameraReleaseImageBuffer(this->hCamera, this->pRawDataBuffer);
    framebag.flag = true;
    return framebag;
}

void MV_Camera::uninit()
{
    CameraUnInit(this->hCamera);
    // CameraAlignFree(this->pFrameBuffer);
    // CameraAlignFree(this->pRawDataBuffer);
}

void MV_Camera::setExposureTime(int ex)
{
    if (this->hCamera == -1)
        return;
    CameraSetExposureTime(this->hCamera, ex);
}

void MV_Camera::setGain(int gain)
{
    if (this->hCamera == -1)
        return;
    CameraSetAnalogGain(this->hCamera, gain);
}

void MV_Camera::saveParam(char tCameraConfigPath[23])
{
    if (access(tCameraConfigPath, F_OK) == 0)
        return;
    if (this->hCamera == -1)
        return;
    CameraSaveParameterToFile(this->hCamera, tCameraConfigPath);
}

void MV_Camera::disableAutoEx()
{
    if (this->hCamera == -1)
        return;
    CameraSetAeState(this->hCamera, 0);
}

int MV_Camera::getExposureTime()
{
    if (this->hCamera == -1)
        return -1;
    double ex;
    CameraGetExposureTime(this->hCamera, &ex);
    return int(ex);
}

int MV_Camera::getAnalogGain()
{
    if (this->hCamera == -1)
        return -1;
    int gain;
    CameraGetAnalogGain(this->hCamera, &gain);
    return gain;
}
#endif

CameraThread::CameraThread()
{
}

CameraThread::~CameraThread()
{
}

void CameraThread::start()
{
    while (!this->_open)
    {
        this->open();
    }
}

void CameraThread::stop()
{
    this->_is_init = false;
    this->_open = false;
    this->_alive = false;
    this->release();
}

#ifndef UsingVideo
void CameraThread::openCamera(bool is_init)
{
    bool initFlag = false;
    try
    {
        this->logger->info("Camera opening ...Process");
        this->_cap = MV_Camera(is_init);
        FrameBag framebag = this->_cap.read();
        if (!framebag.flag)
        {
            this->logger->warn("Camera not inited");
            return;
        }
        framebag = this->_cap.read();
        if (!is_init && framebag.flag)
        {
            namedWindow("PREVIEW", WINDOW_NORMAL);
            resizeWindow("PREVIEW", Size(840, 640));
            setWindowProperty("PREVIEW", WND_PROP_TOPMOST, 1);
            moveWindow("PREVIEW", 100, 100);
            imshow("PREVIEW", framebag.frame);
            int key = waitKey(0);
            destroyWindow("PREVIEW");
            this->_cap.disableAutoEx();
            if (key == 84 || key == 116)
                this->adjustExposure();
            this->_cap.saveParam(CameraConfigPath);
        }
        initFlag = true;
        this->logger->info("Camera opening ...Done");
    }
    catch (const std::exception &e)
    {
        this->logger->error("CameraThread::openCamera(){}", e.what());
        this->_cap.uninit();
        return;
    }
    this->_cap._openflag = initFlag;
}

void CameraThread::adjustExposure()
{
    namedWindow("EXPOSURE Press Q to Exit", WINDOW_NORMAL);
    resizeWindow("EXPOSURE Press Q to Exit", 1280, 960);
    moveWindow("EXPOSURE Press Q to Exit", 200, 200);
    setWindowProperty("EXPOSURE Press Q to Exit", WND_PROP_TOPMOST, 1);
    createTrackbar("ex", "EXPOSURE Press Q to Exit", 0, 30000);
    setTrackbarPos("ex", "EXPOSURE Press Q to Exit", this->_cap.getExposureTime() != -1 ? this->_cap.getExposureTime() : 0);
    createTrackbar("gain", "EXPOSURE Press Q to Exit", 0, 256);
    setTrackbarPos("gain", "EXPOSURE Press Q to Exit", this->_cap.getAnalogGain() != -1 ? this->_cap.getAnalogGain() : 0);
    createTrackbar("Quit", "EXPOSURE Press Q to Exit", 0, 1);
    setTrackbarPos("Quit", "EXPOSURE Press Q to Exit", 0);
    FrameBag framebag = this->_cap.read();
    while (framebag.flag && waitKey(1) != 81 && waitKey(1) != 113 && getTrackbarPos("Quit", "EXPOSURE Press Q to Exit") == 0)
    {
        this->_cap.setExposureTime(getTrackbarPos("ex", "EXPOSURE Press Q to Exit"));
        this->_cap.setGain(getTrackbarPos("gain", "EXPOSURE Press Q to Exit"));
        imshow("EXPOSURE Press Q to Exit", framebag.frame);
        framebag = this->_cap.read();
        waitKey(1);
    }
    int ex = this->_cap.getExposureTime();
    int gain = this->_cap.getAnalogGain();
    this->logger->info("Setting Expoure Time {}us", ex);
    this->logger->info("Setting analog gain {}", gain);
    destroyWindow("EXPOSURE Press Q to Exit");
}
#endif

void CameraThread::open()
{
#ifdef UsingVideo
    if (!this->_open)
    {
        this->_cap = VideoCapture(TestVideoPath);
        this->_open = true;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#else
    if (!this->_open && this->_alive)
    {
        this->openCamera(this->_is_init);
        this->_open = this->_cap._openflag;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#endif
}

bool CameraThread::is_open()
{
    return this->_open;
}

FrameBag CameraThread::read()
{
    FrameBag framebag;
    if (this->_open && this->_alive)
    {
#ifdef UsingVideo
        this->frame_counter += 1;
        if (this->frame_counter == int(_cap.get(cv::CAP_PROP_FRAME_COUNT)))
        {
            this->frame_counter = 0;
            _cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        }
        this->_cap.read(framebag.frame);
        framebag.flag = !framebag.frame.empty();
#else
        framebag = this->_cap.read();
#endif
    }
    else if (this->_alive)
    {
        this->logger->warn("Camera closed !");
    }
    if (!framebag.flag && this->_alive)
    {
        this->logger->error("Failed to get frame!");
    }
    return framebag;
}

void CameraThread::release()
{
#ifndef UsingVideo
    this->_cap.uninit();
#endif
    this->_open = false;
}