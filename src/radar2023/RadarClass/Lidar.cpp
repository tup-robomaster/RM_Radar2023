#include "../PublicInclude/Lidar.h"

static const char lidarTopicName[13] = "/livox/lidar"; //雷达点云节点名称

static vector<DepthQueue> mainDqBox;       //考虑到后续可能的设备改变，预留容器
static vector<MovementDetector> mainMDBox; //考虑到后续可能的设备改变，预留容器
static vector<ArmorDetector> mainADBox;    //考虑到后续可能的设备改变，预留容器
static vector<CameraThread> mainCamBox;    //考虑到后续可能的设备改变，预留容器
static vector<MapMapping> mainMMBox;       //考虑到后续可能的设备改变，预留容器
static vector<UART> mainUARTBox;           //考虑到后续可能的设备改变，预留容器
static vector<MySerial> mainSerBox;        //考虑到后续可能的设备改变，预留容器
static vector<vector<float>> publicDepth;  //共享深度图
static int depthResourceCount;             //深度图资源计数
static shared_timed_mutex myMutex;         //读写锁
static vector<Rect> MTs;                   //共享运动目标

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

Lidar::Lidar(int argc, char **argv)
{
    this->init(argc, argv);
}

Lidar::~Lidar()
{
}

void Lidar::init(int argc, char **argv)
{
    if (!this->_init_flag)
    {
        cout << "[INFO]"
             << "Initing ..." << endl;
        if (ENEMY)
            cout << "[INFO]"
                 << "YOU ARE RED" << endl;
        else
            cout << "[INFO]"
                 << "YOU ARE BLUE" << endl;
        Matrix<float, 3, 3> K_0;
        Matrix<float, 1, 5> C_0;
        Matrix<float, 4, 4> E_0;
        Mat K_0_Mat;
        Mat C_0_Mat;
        Mat E_0_Mat;
        if (!read_yaml(K_0_Mat, C_0_Mat, E_0_Mat))
            return;
        cv2eigen(K_0_Mat, K_0);
        cv2eigen(C_0_Mat, C_0);
        cv2eigen(E_0_Mat, E_0);
        // TODO: CHECK HERE
        if (mainDqBox.size() == 0)
            mainDqBox.emplace_back(DepthQueue(K_0, C_0, E_0));
        if (mainMDBox.size() == 0)
            mainMDBox.emplace_back(MovementDetector());
        if (mainADBox.size() == 0)
            mainADBox.emplace_back(ArmorDetector());
        if (mainCamBox.size() == 0)
            mainCamBox.emplace_back(CameraThread());
        if (mainMMBox.size() == 0)
            mainMMBox.emplace_back(MapMapping());
        if (mainUARTBox.size() == 0)
            mainUARTBox.emplace_back(UART());
        if (mainSerBox.size() == 0)
            mainSerBox.emplace_back(MySerial());
        if (!this->_init_flag)
        {
            this->LidarListenerBegin(argc, argv);
            mainADBox[0].initModel();
            mainSerBox[0].initSerial();
            this->_init_flag = true;
            cout << "[INFO]"
                 << "Init Done" << endl;
        }
    }
}

void Lidar::LidarListenerBegin(int argc, char **argv)
{
    ros::init(argc, argv, "laser_listener");
    ros::NodeHandle nh;
    sub = nh.subscribe(lidarTopicName, LidarQueueSize, &Lidar::LidarCallBack, this);
}

void Lidar::LidarMainLoop(future<void> futureObj)
{
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        ros::spin();
    }
}

void Lidar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
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

void Lidar::MovementDetectorLoop(future<void> futureObj)
{
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        if (depthResourceCount > 0)
        {

#ifdef ThreadSpeedTest
            clock_t start, finish;
            start = clock();
#endif

            unique_lock<shared_timed_mutex> ulk(myMutex);
            depthResourceCount--;
            ulk.unlock();
            shared_lock<shared_timed_mutex> slk(myMutex);
            vector<Rect> tempMTs = mainMDBox[0].applyMovementDetector(publicDepth);
            slk.unlock();
            ulk.lock();
            MTs.swap(tempMTs);
            ulk.unlock();

#ifdef ThreadSpeedTest
            finish = clock();
            cout << "Lidar::MovementDetectorLoop()|" << depthResourceCount << "|" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "|FPS:" << 1000 / (double(finish - start) / CLOCKS_PER_SEC * 1000) << endl;
#endif
        }
    }
}

void Lidar::SerReadLoop()
{
    mainUARTBox[0].read(mainSerBox[0]);
}

void Lidar::SerWriteLoop()
{
    mainUARTBox[0].write(mainSerBox[0]);
}

void Lidar::MainProcessLoop(future<void> futureObj)
{
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
#ifdef ThreadSpeedTest
        clock_t start, finish;
        start = clock();
#endif

        unique_lock<shared_timed_mutex> ulk(myMutex);
        ulk.unlock();
        shared_lock<shared_timed_mutex> slk(myMutex);
        int check_count = MTs.size();
        slk.unlock();
        if (check_count > 0)
        {
            if (!mainCamBox[0].is_open())
            {
                mainCamBox[0].open();
                return;
            }
            FrameBag frameBag = mainCamBox[0].read();
            vector<ArmorBoundingBox> armorBoundingBoxs;
            if (frameBag.flag)
            {
                slk.lock();
                vector<Rect> tempMTs = MTs;
                slk.unlock();
                armorBoundingBoxs = mainADBox[0].infer(frameBag.frame, tempMTs);
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
                return;
        }

#ifdef ThreadSpeedTest
        finish = clock();
        cout << "Lidar::MovementDetectorLoop()|" << depthResourceCount << "|" << double(finish - start) / CLOCKS_PER_SEC * 1000 << "|FPS:" << 1000 / (double(finish - start) / CLOCKS_PER_SEC * 1000) << endl;
#endif
    }
}

void Lidar::spin(int argc, char **argv)
{
    this->init(argc, argv);
    if (!this->_init_flag)
        return;
    if (!mainMMBox[0]._is_pass())
    {
        cout << "[INFO]"
             << "Locate pick start ..." << endl;
        Location myLocation = Location();
        Mat revc, tvec;
        if (!myLocation.locate_pick(mainCamBox[0], ENEMY, revc, tvec))
            return;
        mainMMBox[0].push_T(revc, tvec);
        cout << "[INFO]"
             << "Locate pick Done" << endl;
    }
    if (!this->_thread_working)
    {
        cout << "[INFO]"
             << "Thread starting ..." << endl;
        this->_thread_working = true;
        this->exitSignal1 = promise<void>();
        this->exitSignal2 = promise<void>();
        this->exitSignal3 = promise<void>();
        future<void> futureObj1 = exitSignal1.get_future();
        future<void> futureObj2 = exitSignal2.get_future();
        future<void> futureObj3 = exitSignal2.get_future();
        this->mainloop = thread(&this->LidarMainLoop, move(futureObj1));
        this->MDloop = thread(&this->MovementDetectorLoop, move(futureObj2));
        this->processLoop = thread(&this->MainProcessLoop, move(futureObj3));
        cout << "[INFO]"
             << "Thread start Done" << endl;
    }
    if (!mainSerBox[0]._is_open())
    {
        cout << "[INFO]"
             << "Serial initing ..." << endl;
        mainSerBox[0].initSerial();
        cout << "[INFO]"
             << "Serial init Done" << endl;
    }
    if (!this->_Ser_working && mainSerBox[0]._is_open())
    {
        cout << "[INFO]"
             << "SerThread starting ..." << endl;
        this->_Ser_working = true;
        this->serRead = thread(&this->SerReadLoop);
        this->serWrite = thread(&this->SerWriteLoop);
        cout << "[INFO]"
             << "SerThread start Done" << endl;
    }
}

void Lidar::stop()
{
    if (this->_thread_working)
    {
        this->_thread_working = false;
        this->exitSignal1.set_value();
        this->exitSignal2.set_value();
        this->exitSignal3.set_value();
        this->mainloop.join();
        this->MDloop.join();
        this->processLoop.join();
    }
}