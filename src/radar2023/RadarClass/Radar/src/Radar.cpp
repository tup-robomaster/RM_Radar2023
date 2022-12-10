#include "../include/Radar.h"

static const char lidarTopicName[13] = "/livox/lidar"; // 雷达点云节点名称

static vector<DepthQueue> mainDqBox;       // 考虑到后续可能的设备改变，预留容器
static vector<MovementDetector> mainMDBox; // 考虑到后续可能的设备改变，预留容器
static vector<ArmorDetector> mainADBox;    // 考虑到后续可能的设备改变，预留容器
static vector<CarDetector> mainCDBox;      // 考虑到后续可能的设备改变，预留容器
static vector<CameraThread> mainCamBox;    // 考虑到后续可能的设备改变，预留容器
static vector<MapMapping> mainMMBox;       // 考虑到后续可能的设备改变，预留容器
static vector<UART> mainUARTBox;           // 考虑到后续可能的设备改变，预留容器
static vector<MySerial> mainSerBox;        // 考虑到后续可能的设备改变，预留容器
static vector<VideoRecoder> mainVRBox;     // 考虑到后续可能的设备改变，预留容器
static vector<vector<float>> publicDepth;  // 共享深度图
static int depthResourceCount;             // 深度图资源计数
static shared_timed_mutex myMutex;         // 读写锁
static vector<Rect> SeqTargets;            // 共享分割目标
static int separation_mode = 0;            // 图像分割模式
static SharedQueue<Mat> myFrames;          // 图像帧队列

static void armor_filter(vector<ArmorBoundingBox> &armors)
{
    vector<ArmorBoundingBox> results;
    int ids[10] = {1, 2, 3, 4, 5, 8, 9, 10, 11, 12};
    for (int i = 0; i < 10; ++i)
    {
        int max_id = 0;
        float max_conf = 0.f;
        for (size_t j = 0; j < armors.size(); ++j)
        {
            if ((int)armors[j].cls == ids[i] && armors[j].conf - max_conf > 0)
            {
                max_id = j;
                max_conf = armors[j].conf;
            }
        }
        if (max_conf != 0.f)
            results.emplace_back(armors[max_id]);
    }
    armors.swap(results);
}

static void detectDepth(vector<ArmorBoundingBox> &armorBoundingBoxs)
{
    if (armorBoundingBoxs.size() == 0)
        return;
    for (size_t i = 0; i < armorBoundingBoxs.size(); ++i)
    {
        float count = 0;
        vector<float> tempBox;
        float center[2] = {armorBoundingBoxs.at(i).x0 + armorBoundingBoxs[i].w / 2, armorBoundingBoxs[i].y0 + armorBoundingBoxs[i].h / 2};
        for (int j = int(max<float>(center[1] - armorBoundingBoxs[i].h, 0.)); j < int(min<float>(center[1] + armorBoundingBoxs[i].h, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - armorBoundingBoxs[i].w, 0.)); k < int(min<float>(center[0] + armorBoundingBoxs[i].w, ImageW)); ++k)
            {
                if (publicDepth[i][j] == 0)
                    continue;
                tempBox.emplace_back(publicDepth[i][j]);
                ++count;
            }
        }
        float tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        armorBoundingBoxs[i].depth = tempNum / count;
    }
}

static void send_judge(judge_message &message, UART &myUART)
{
    vector<vector<float>> loc;
    switch (message.task)
    {
    case 1:
        for (int i = 1; i < 6; ++i)
        {
            vector<float> temp_location;
            temp_location.emplace_back(message.loc[i + ENEMY * 5].x);
            temp_location.emplace_back(message.loc[i + ENEMY * 5].y);
            loc.emplace_back(temp_location);
        }
        myUART.myUARTPasser.push_loc(loc);
        break;

    default:
        break;
    }
}

Radar::Radar(int argc, char **argv)
{
}

Radar::~Radar()
{
    if (this->is_alive)
        this->stop();
}

void Radar::init(int argc, char **argv)
{
    if (this->_init_flag)
        return;
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], Initing ...Process\n");
    if (ENEMY)
        fmt::print(fg(fmt::color::antique_white) | fmt::emphasis::bold,
                   "[GAME], YOU ARE RED\n");
    else
        fmt::print(fg(fmt::color::antique_white) | fmt::emphasis::bold,
                   "[GAME], YOU ARE BLUE\n");
    Matrix<float, 3, 3> K_0;
    Matrix<float, 1, 5> C_0;
    Matrix<float, 4, 4> E_0;
    Mat K_0_Mat;
    Mat C_0_Mat;
    Mat E_0_Mat;
    if (!read_param(K_0_Mat, C_0_Mat, E_0_Mat))
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], Can't read CAMERA_PARAM: {}!\n", CAMERA_PARAM_PATH);
        return;
    }
    cv2eigen(K_0_Mat, K_0);
    cv2eigen(C_0_Mat, C_0);
    cv2eigen(E_0_Mat, E_0);
    // TODO: CHECK HERE
    if (mainDqBox.size() == 0)
    {
        mainDqBox.emplace_back(DepthQueue(K_0, C_0, E_0));
    }
    if (mainMDBox.size() == 0)
    {
        mainMDBox.emplace_back(MovementDetector());
    }
    if (mainADBox.size() == 0)
    {
        mainADBox.emplace_back(ArmorDetector());
    }
    if (mainCDBox.size() == 0)
    {
        mainCDBox.emplace_back(CarDetector());
    }
    if (mainCamBox.size() == 0)
    {
        mainCamBox.emplace_back(CameraThread());
    }
    if (mainVRBox.size() == 0)
    {
        mainVRBox.emplace_back(VideoRecoder());
    }
    if (mainMMBox.size() == 0)
    {
        mainMMBox.emplace_back(MapMapping());
    }
    if (mainUARTBox.size() == 0)
    {
        mainUARTBox.emplace_back(UART());
    }
    if (mainSerBox.size() == 0)
    {
        mainSerBox.emplace_back(MySerial());
    }
    if (!this->_init_flag)
    {
        namedWindow("ControlPanel", WindowFlags::WINDOW_NORMAL);
        createTrackbar("Exit Program", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Exit Program", "ControlPanel", 0);
        createTrackbar("Separation mode", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Separation mode", "ControlPanel", 0);
        this->LidarListenerBegin(argc, argv);
        if (!mainADBox[0].initModel())
        {
            this->stop();
            return;
        }
        this->carInferAvailable = mainCDBox[0].initModel() ? true : false;
        mainSerBox[0].initSerial();
        mainVRBox[0].init(VideoRecoderRath, VideoWriter::fourcc('m', 'p', '4', 'v'), Size(ImageW, ImageH));
        mainCamBox[0].start();
        this->_init_flag = true;
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "[INFO], Init Done\n");
    }
}

void Radar::LidarListenerBegin(int argc, char **argv)
{
    if (this->_is_LidarInited)
        return;
    ros::init(argc, argv, "laser_listener");
    ros::NodeHandle nh;
    sub = nh.subscribe(lidarTopicName, LidarQueueSize, &Radar::LidarCallBack, this);
    this->_is_LidarInited = true;
}

void Radar::LidarMainLoop(future<void> futureObj)
{
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        if (ros::ok())
            ros::spinOnce();
    }
}

void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    std::vector<std::vector<float>> tempDepth = mainDqBox[0].pushback(*pc);
    unique_lock<shared_timed_mutex> ulk(myMutex);
    publicDepth.swap(tempDepth);
    if (depthResourceCount < 5)
        ++depthResourceCount;
    ulk.unlock();
}

void Radar::SeparationLoop(future<void> futureObj)
{
    unique_lock<shared_timed_mutex> ulk(myMutex);
    ulk.unlock();
    shared_lock<shared_timed_mutex> slk(myMutex);
    slk.unlock();
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        vector<Rect> tempSeqTargets;
        if (separation_mode == 0 && depthResourceCount > 0)
        {
            ulk.lock();
            depthResourceCount--;
            ulk.unlock();
            slk.lock();
            tempSeqTargets = mainMDBox[0].applyMovementDetector(publicDepth);
            slk.unlock();
        }
        else if (separation_mode == 1)
        {
            slk.lock();
            FrameBag image = mainCamBox[0].read();
            slk.unlock();
            tempSeqTargets = mainCDBox[0].infer(image.frame);
        }
        ulk.lock();
        SeqTargets.swap(tempSeqTargets);
        ulk.unlock();
    }
}

void Radar::SerReadLoop()
{
    mainUARTBox[0].read(mainSerBox[0]);
}

void Radar::SerWriteLoop()
{
    mainUARTBox[0].write(mainSerBox[0]);
}

void Radar::MainProcessLoop(future<void> futureObj)
{
    unique_lock<shared_timed_mutex> ulk(myMutex);
    ulk.unlock();
    shared_lock<shared_timed_mutex> slk(myMutex);
    slk.unlock();
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        slk.lock();
        int check_count = SeqTargets.size();
        slk.unlock();
        FrameBag frameBag = mainCamBox[0].read();
        if (check_count > 0)
        {
            if (!mainCamBox[0].is_open())
            {
                mainCamBox[0].open();
                continue;
            }
            vector<ArmorBoundingBox> armorBoundingBoxs;
            if (frameBag.flag)
            {
                vector<Rect> tempSeqTargets;
                ulk.lock();
                tempSeqTargets.swap(SeqTargets);
                ulk.unlock();
                armorBoundingBoxs = mainADBox[0].infer(frameBag.frame, tempSeqTargets);
                if (armorBoundingBoxs.size() == 0)
                    continue;
                // TODO: 加入防抖层
                armor_filter(armorBoundingBoxs);
                slk.lock();
                detectDepth(armorBoundingBoxs);
                slk.unlock();
                vector<ArmorBoundingBox> IouArmors;
                mainMMBox[0].mergeUpdata(armorBoundingBoxs, IouArmors);
                judge_message myJudge_message;
                myJudge_message.task = 1;
                myJudge_message.loc = mainMMBox[0].getloc();
                send_judge(myJudge_message, mainUARTBox[0]);
            }
            else
                continue;
        }
        if (frameBag.flag)
            myFrames.push(frameBag.frame);
    }
}

void Radar::VideoRecoderLoop(future<void> futureObj)
{
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        mainVRBox[0].write(myFrames.front());
    }
}

void Radar::spin(int argc, char **argv)
{
    this->init(argc, argv);
    if (!this->_init_flag)
        return;
    if (getTrackbarPos("Exit Program", "ControlPanel") == 1)
    {
        this->stop();
        return;
    }
    if (this->carInferAvailable)
        separation_mode = getTrackbarPos("Separation mode", "ControlPanel");
    else
        separation_mode = 0;
    if (!mainMMBox[0]._is_pass() || waitKey(1) == 76 || waitKey(1) == 108)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Locate pick start ...Process\n");
        // TODO: Fix here
        unique_lock<shared_timed_mutex> ulk(myMutex);
        Location myLocation = Location();
        Mat rvec, tvec;
        if (!myLocation.locate_pick(mainCamBox[0], ENEMY, rvec, tvec))
        {
            ulk.unlock();
            return;
        }
        ulk.unlock();
        mainMMBox[0].push_T(rvec, tvec);
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "[INFO], Locate pick Done \n");
    }
    if (!this->_thread_working)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Thread starting ...");
        this->_thread_working = true;
        this->exitSignal1 = promise<void>();
        this->exitSignal2 = promise<void>();
        this->exitSignal3 = promise<void>();
        this->exitSignal4 = promise<void>();
        future<void> futureObj1 = exitSignal1.get_future();
        future<void> futureObj2 = exitSignal2.get_future();
        future<void> futureObj3 = exitSignal3.get_future();
        future<void> futureObj4 = exitSignal4.get_future();
        this->mainloop = thread(&this->LidarMainLoop, move(futureObj1));
        this->Seqloop = thread(&this->SeparationLoop, move(futureObj2));
        this->processLoop = thread(&this->MainProcessLoop, move(futureObj3));
        this->videoRecoderLoop = thread(&this->VideoRecoderLoop, move(futureObj4));
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (!mainSerBox[0]._is_open())
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Serial initing ...");
        mainSerBox[0].initSerial();
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (!this->_Ser_working && mainSerBox[0]._is_open())
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], SerThread initing ...");
        this->_Ser_working = true;
        this->serRead = thread(&this->SerReadLoop);
        this->serR_t = this->serRead.native_handle();
        this->serWrite = thread(&this->SerWriteLoop);
        this->serW_t = this->serWrite.native_handle();
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (myFrames.size() > 0)
    {
        Mat frame = myFrames.front();
        imshow("ControlPanel", frame);
        resizeWindow("ControlPanel", 1920, 1080);
        myFrames.pop();
    }
}

void Radar::stop()
{
    fmt::print(fg(fmt::color::orange_red) | fmt::emphasis::bold | fmt::v9::bg(fmt::color::white),
               "[WARN], Start Shutdown Process...");
    cv::destroyAllWindows();
    if (this->_thread_working)
    {
        this->_thread_working = false;
        this->exitSignal1.set_value();
        this->exitSignal2.set_value();
        this->exitSignal3.set_value();
        this->exitSignal4.set_value();
        pthread_cancel(this->serR_t);
        pthread_cancel(this->serW_t);
        this->mainloop.join();
        this->Seqloop.join();
        this->processLoop.join();
        this->videoRecoderLoop.join();
    }
    if (mainCamBox[0].is_open())
        mainCamBox[0].stop();
    mainVRBox[0].close();
    this->is_alive = false;
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "Done.\n");
}

bool Radar::alive()
{
    return this->is_alive;
}