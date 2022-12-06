#include "../include/Radar.h"

static const char lidarTopicName[13] = "/livox/lidar"; //雷达点云节点名称

static vector<DepthQueue> mainDqBox;       //考虑到后续可能的设备改变，预留容器
static vector<MovementDetector> mainMDBox; //考虑到后续可能的设备改变，预留容器
static vector<ArmorDetector> mainADBox;    //考虑到后续可能的设备改变，预留容器
static vector<CarDetector> mainCDBox;      //考虑到后续可能的设备改变，预留容器
static vector<CameraThread> mainCamBox;    //考虑到后续可能的设备改变，预留容器
static vector<MapMapping> mainMMBox;       //考虑到后续可能的设备改变，预留容器
static vector<UART> mainUARTBox;           //考虑到后续可能的设备改变，预留容器
static vector<MySerial> mainSerBox;        //考虑到后续可能的设备改变，预留容器
static vector<vector<float>> publicDepth;  //共享深度图
static int depthResourceCount;             //深度图资源计数
static shared_timed_mutex myMutex;         //读写锁
static vector<Rect> SeqTargets;            //共享分割目标
static int separation_mode = 0;            //图像分割模式

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

static void detectDepth(vector<ArmorBoundingBox> *armorBoundingBoxs)
{
#ifdef SpeedTest
    clock_t start, finish;
    start = clock();
#endif

    if (armorBoundingBoxs->size() == 0)
        return;
    for (size_t i = 0; i < armorBoundingBoxs->size(); ++i)
    {
        float count = 0;
        vector<float> tempBox;
        float center[2] = {armorBoundingBoxs->at(i).x0 + armorBoundingBoxs->at(i).w / 2, armorBoundingBoxs->at(i).y0 + armorBoundingBoxs->at(i).h / 2};
        for (int j = int(max<float>(center[1] - armorBoundingBoxs->at(i).h, 0.)); j < int(min<float>(center[1] + armorBoundingBoxs->at(i).h, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - armorBoundingBoxs->at(i).w, 0.)); k < int(min<float>(center[0] + armorBoundingBoxs->at(i).w, ImageW)); ++k)
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
        armorBoundingBoxs->at(i).depth = tempNum / count;
    }

#ifdef SpeedTest
    finish = clock();
    cout << "DepthQueue::detectDepth()|" << depthBox.size() << "|" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "|FPS:" << 1000 / (double(finish - start) / CLOCKS_PER_SEC * 1000) << endl;
#endif
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
    if(this->is_alive)
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
    if (!read_yaml(K_0_Mat, C_0_Mat, E_0_Mat))
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], Can't read yaml !\n");
        return;
    }
    cv2eigen(K_0_Mat, K_0);
    cv2eigen(C_0_Mat, C_0);
    cv2eigen(E_0_Mat, E_0);
    // TODO: CHECK HERE
    if (mainDqBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding DepthQueue ...");
        mainDqBox.emplace_back(DepthQueue(K_0, C_0, E_0));
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (mainMDBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding MovementDetector ...");
        mainMDBox.emplace_back(MovementDetector());
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (mainADBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding ArmorDetector ...");
        mainADBox.emplace_back(ArmorDetector());
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (mainCDBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding CarDetector ...");
        mainCDBox.emplace_back(CarDetector());
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (mainCamBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding CameraThread ...");
        mainCamBox.emplace_back(CameraThread());
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (mainMMBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding MapMapping ...");
        mainMMBox.emplace_back(MapMapping());
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (mainUARTBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding UART ...");
        mainUARTBox.emplace_back(UART());
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (mainSerBox.size() == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Adding MySerial ...");
        mainSerBox.emplace_back(MySerial());
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (!this->_init_flag)
    {
        namedWindow("ControlPanel", WindowFlags::WINDOW_NORMAL);
        createTrackbar("Exit Program", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Exit Program", "ControlPanel", 0);
        createTrackbar("Locate Pick", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Locate Pick", "ControlPanel", 0);
        createTrackbar("Separation mode", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Separation mode", "ControlPanel", 0);
        this->LidarListenerBegin(argc, argv);
        if (!mainADBox[0].initModel())
        {
            this->stop();
            return;
        }
        if (!mainCDBox[0].initModel())
        {
            this->stop();
            return;
        }
        mainSerBox[0].initSerial();
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
        ros::spin();
    }
}

void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
#ifdef ThreadSpeedTest
    clock_t start, finish;
    start = clock();
#endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    std::vector<std::vector<float>> tempDepth = mainDqBox[0].pushback(*pc);
    unique_lock<shared_timed_mutex> ulk(myMutex);
    publicDepth.swap(tempDepth);
    if (depthResourceCount < 5)
        depthResourceCount++;
    ulk.unlock();

#ifdef ThreadSpeedTest
    finish = clock();
    cout << "Lidar::LidarCallBack()|" << pc->points.size() << "|" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "|FPS:" << 1000 / (double(finish - start) / CLOCKS_PER_SEC * 1000) << endl;
#endif
}

void Radar::SeparationLoop(future<void> futureObj)
{
    unique_lock<shared_timed_mutex> ulk(myMutex);
    ulk.unlock();
    shared_lock<shared_timed_mutex> slk(myMutex);
    slk.unlock();
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        if (depthResourceCount > 0)
        {

#ifdef ThreadSpeedTest
            clock_t start, finish;
            start = clock();
#endif
            vector<Rect> tempSeqTargets;
            ulk.lock();
            depthResourceCount--;
            ulk.unlock();
            if (separation_mode == 0)
            {
                slk.lock();
                tempSeqTargets = mainMDBox[0].applyMovementDetector(publicDepth);
                slk.unlock();
            }
            else if (separation_mode == 1)
            {
                FrameBag image = mainCamBox[0].read();
                tempSeqTargets = mainCDBox[0].infer(image.frame);
            }
            ulk.lock();
            SeqTargets.swap(tempSeqTargets);
            ulk.unlock();

#ifdef ThreadSpeedTest
            finish = clock();
            cout << "Lidar::MovementDetectorLoop()|" << depthResourceCount << "|" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "|FPS:" << 1000 / (double(finish - start) / CLOCKS_PER_SEC * 1000) << endl;
#endif
        }
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
#ifdef ThreadSpeedTest
        clock_t start, finish;
        start = clock();
#endif
        slk.lock();
        int check_count = SeqTargets.size();
        slk.unlock();
        if (check_count > 0)
        {
            if (!mainCamBox[0].is_open())
            {
                mainCamBox[0].open();
                continue;
            }
            FrameBag frameBag = mainCamBox[0].read();
            vector<ArmorBoundingBox> armorBoundingBoxs;
            if (frameBag.flag)
            {
                slk.lock();
                vector<Rect> tempSeqTargets = SeqTargets;
                slk.unlock();

                armorBoundingBoxs = mainADBox[0].infer(frameBag.frame, tempSeqTargets);
                if (armorBoundingBoxs.size() == 0)
                    continue;
                // TODO: 加入防抖层
                armor_filter(armorBoundingBoxs);
                slk.lock();
                detectDepth(&armorBoundingBoxs);
                slk.unlock();
                // TODO: working here
                vector<ArmorBoundingBox> IouArmors;
                mainMMBox[0].mergeUpdata(armorBoundingBoxs, IouArmors);
                judge_message myJudge_message;
                myJudge_message.task = 1;
                myJudge_message.loc = mainMMBox[0].updata();
                send_judge(myJudge_message, mainUARTBox[0]);
            }
            else
                continue;
        }

#ifdef ThreadSpeedTest
        finish = clock();
        cout << "Lidar::MovementDetectorLoop()|" << depthResourceCount << "|" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "|FPS:" << 1000 / (double(finish - start) / CLOCKS_PER_SEC * 1000) << endl;
#endif
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
    separation_mode = getTrackbarPos("Separation mode", "ControlPanel");
    if (!mainMMBox[0]._is_pass() || getTrackbarPos("Locate Pick", "ControlPanel") == 1)
    {
        setTrackbarPos("Locate Pick", "ControlPanel", 0);
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Locate pick start ...Process\n");
        Location myLocation = Location();
        Mat rvec, tvec;
        if (!myLocation.locate_pick(mainCamBox[0], ENEMY, rvec, tvec))
            return;
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
        future<void> futureObj1 = exitSignal1.get_future();
        future<void> futureObj2 = exitSignal2.get_future();
        future<void> futureObj3 = exitSignal3.get_future();
        this->mainloop = thread(&this->LidarMainLoop, move(futureObj1));
        this->Seqloop = thread(&this->SeparationLoop, move(futureObj2));
        this->processLoop = thread(&this->MainProcessLoop, move(futureObj3));
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
        this->serWrite = thread(&this->SerWriteLoop);
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
}

void Radar::stop()
{
    fmt::print(fg(fmt::color::orange_red) | fmt::emphasis::bold | fmt::v9::bg(fmt::color::white),
               "[WARN], Start Shutdown Process...");
    this->is_alive = false;
    if (this->_thread_working)
    {
        this->_thread_working = false;
        this->exitSignal1.set_value();
        this->exitSignal2.set_value();
        this->exitSignal3.set_value();
        this->mainloop.join();
        this->Seqloop.join();
        this->processLoop.join();
        mainCamBox[0].stop();
    }
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "Done.\n");
}

bool Radar::alive()
{
    return this->is_alive;
}