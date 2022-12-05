#include "../PublicInclude/camera.h"

// unsigned char camera_match_index = (char)0;                  //相机索引号

MV_Camera::MV_Camera(bool Is_init)
{
    CameraSdkInit(1);
    this->iStatus = CameraEnumerateDevice(this->tCameraEnumList, &this->iCameraCounts);
    if (this->iCameraCounts == 0)
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], {}!\n", "No camera found!");
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
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], {}!\n", "CameraInit!");
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "[ERROR]CameraInitERROR!" << '\n';
        return;
    }
    if (this->iStatus != CAMERA_STATUS_SUCCESS)
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], {} {}!\n", "CameraInit Failed!", this->iStatus);
        return;
    }
    CameraGetCapability(this->hCamera, &this->tCapability);
    if (!tCapability.sIspCapacity.bMonoSensor)
        CameraSetIspOutFormat(this->hCamera, CAMERA_MEDIA_TYPE_BGR8);
    else
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], {}!\n", "None suitable camera!");
    }
    CameraSetTriggerMode(this->hCamera, 0);
    if (!Is_init)
        CameraReadParameterFromFile(this->hCamera, CameraConfigPath);
    CameraSetAeState(this->hCamera, 0);
    CameraPlay(this->hCamera);
    int frameBufferSize = this->tCapability.sResolutionRange.iWidthMax * this->tCapability.sResolutionRange.iHeightMax * 3;
    this->pFrameBuffer = CameraAlignMalloc(frameBufferSize, 16);
}

MV_Camera::~MV_Camera()
{
}

FrameBag MV_Camera::read()
{
#ifdef SpeedTest
    clock_t start, finish;
    start = clock();
#endif

    FrameBag framebag;
    if (this->hCamera == -1)
        return framebag;
    try
    {
        CameraGetImageBuffer(this->hCamera, &this->sFrameInfo, &this->pRawDataBuffer, 200);
        if (CameraImageProcess(this->hCamera, this->pRawDataBuffer, this->pFrameBuffer, &this->sFrameInfo) != CAMERA_STATUS_SUCCESS)
        {
            fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                       "[ERROR], {}!\n", "Can not process Image!");
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
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "[ERROR]Failed to get frame!" << '\n';
        return framebag;
    }

#ifdef SpeedTest
    finish = clock();
    cout << "MV_Camera::read()|" << framebag.flag << "|" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "|FPS:" << 1000 / (double(finish - start) / CLOCKS_PER_SEC * 1000) << endl;
#endif

    return framebag;
}

void MV_Camera::uninit()
{
    CameraUnInit(this->hCamera);
    free(this->pFrameBuffer);
    free(this->pRawDataBuffer);
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

CameraThread::CameraThread()
{
    while (!this->is_open())
    {
        this->open();
    }
}

CameraThread::~CameraThread()
{
    this->release();
}

MV_Camera CameraThread::openCamera(bool is_init)
{
    MV_Camera cap;
    bool initFlag = false;
    try
    {
        cap = MV_Camera(is_init);
        FrameBag framebag = cap.read();
        if (!framebag.flag)
        {
            fmt::print(fg(fmt::color::yellow) | fmt::emphasis::bold,
                       "[WARN], {}!\n", "Camera not init");
            return cap;
        }
        framebag = cap.read();
        if (!is_init && framebag.flag)
        {
            namedWindow("PREVIEW", WINDOW_NORMAL);
            resizeWindow("PREVIEW", Size(840, 640));
            setWindowProperty("PREVIEW", WND_PROP_TOPMOST, 1);
            moveWindow("PREVIEW", 100, 100);
            imshow("PREVIEW", framebag.frame);
            waitKey(1);
            int key = waitKey(0);
            destroyWindow("PREVIEW");
            cap.disableAutoEx();
            if (key == 84)
                this->adjustExposure(cap);
            cap.saveParam(CameraConfigPath);
        }
        initFlag = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "[ERROR]CameraThread::openCamera()" << '\n';
        cap.uninit();
    }
    cap._openflag = initFlag;
    return cap;
}

void CameraThread::adjustExposure(MV_Camera &cap)
{
    namedWindow("EXPOSURE Press Q to Exit", WINDOW_NORMAL);
    resizeWindow("EXPOSURE Press Q to Exit", 1280, 960);
    moveWindow("EXPOSURE Press Q to Exit", 200, 200);
    setWindowProperty("EXPOSURE Press Q to Exit", WND_PROP_TOPMOST, 1);
    createTrackbar("ex", "EXPOSURE Press Q to Exit", 0, 1);
    setTrackbarMax("ex", "EXPOSURE Press Q to Exit", 30000);
    setTrackbarMin("ex", "EXPOSURE Press Q to Exit", 0);
    setTrackbarPos("ex", "EXPOSURE Press Q to Exit", cap.getExposureTime() != -1 ? cap.getExposureTime() : 0);
    createTrackbar("gain", "EXPOSURE Press Q to Exit", 0, 1);
    setTrackbarMax("gain", "EXPOSURE Press Q to Exit", 256);
    setTrackbarMin("gain", "EXPOSURE Press Q to Exit", 0);
    setTrackbarPos("gain", "EXPOSURE Press Q to Exit", cap.getAnalogGain() != -1 ? cap.getAnalogGain() : 0);
    FrameBag framebag = cap.read();
    while (!framebag.flag && waitKey(1) != 81)
    {
        cap.setExposureTime(getTrackbarPos("ex", "EXPOSURE Press Q to Exit"));
        cap.setGain(getTrackbarPos("gain", "EXPOSURE Press Q to Exit"));
        imshow("EXPOSURE Press Q to Exit", framebag.frame);
        framebag = cap.read();
        waitKey(1);
    }
    int ex = cap.getExposureTime();
    int gain = cap.getAnalogGain();
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], {}{}us!\n", "Setting Expoure Time ", ex);
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], {}{}!\n", "Setting analog gain ", gain);
    destroyWindow("EXPOSURE Press Q to Exit");
}

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
    if (!this->_open)
    {
        MV_Camera c = this->openCamera(this->_is_init);
        this->_open = c._openflag;
        this->_cap = c;
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
    if (this->_open)
    {
#ifdef UsingVideo
        this->_cap.read(framebag.frame);
        framebag.flag = true;
#else
        framebag = this->_cap.read();
#endif
    }
    if (!framebag.flag)
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                       "[ERROR], {}!\n", "Failed to get frame!--CameraThread::read()");
        this->release();
    }
    return framebag;
}

void CameraThread::release()
{
    this->_cap.uninit();
    this->_open = false;
}