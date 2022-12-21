#include "../include/Radar.h"

void Radar::armor_filter(vector<bboxAndRect> &pred)
{
    vector<bboxAndRect> results;
    int ids[10] = {1, 2, 3, 4, 5, 8, 9, 10, 11, 12};
    for (int i = 0; i < 10; ++i)
    {
        int max_id = 0;
        float max_conf = 0.f;
        for (size_t j = 0; j < pred.size(); ++j)
        {
            if ((int)pred[j].armor.cls == ids[i] && pred[j].armor.conf - max_conf > 0)
            {
                max_id = j;
                max_conf = pred[j].armor.conf;
            }
        }
        if (max_conf != 0.f)
            results.emplace_back(pred[max_id]);
    }
    pred.swap(results);
}

void Radar::detectDepth(vector<bboxAndRect> &pred)
{
    if (pred.size() == 0)
        return;
    for (size_t i = 0; i < pred.size(); ++i)
    {
        float count = 0;
        vector<float> tempBox;
        float center[2] = {pred.at(i).armor.x0 + pred[i].armor.w / 2, pred[i].armor.y0 + pred[i].armor.h / 2};
        for (int j = int(max<float>(center[1] - pred[i].armor.h, 0.)); j < int(min<float>(center[1] + pred[i].armor.h, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - pred[i].armor.w, 0.)); k < int(min<float>(center[0] + pred[i].armor.w, ImageW)); ++k)
            {
                if (this->publicDepth[i][j] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[i][j]);
                ++count;
            }
        }
        float tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        pred[i].armor.depth = tempNum / count;
    }
}

void Radar::detectDepth(vector<ArmorBoundingBox> &armors)
{
    if (armors.size() == 0)
        return;
    for (size_t i = 0; i < armors.size(); ++i)
    {
        float count = 0;
        vector<float> tempBox;
        float center[2] = {armors.at(i).x0 + armors[i].w / 2, armors[i].y0 + armors[i].h / 2};
        for (int j = int(max<float>(center[1] - armors[i].h, 0.)); j < int(min<float>(center[1] + armors[i].h, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - armors[i].w, 0.)); k < int(min<float>(center[0] + armors[i].w, ImageW)); ++k)
            {
                if (this->publicDepth[i][j] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[i][j]);
                ++count;
            }
        }
        float tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        armors[i].depth = tempNum / count;
    }
}

void Radar::send_judge(judge_message &message, UART &myUART)
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

Radar::Radar()
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
    this->depthQueue = DepthQueue(K_0, C_0, E_0);
    this->movementDetector = MovementDetector();
    this->armorDetector = ArmorDetector();
    this->carDetector = CarDetector();
    this->cameraThread = CameraThread();
    this->videoRecorder = VideoRecorder();
    this->myLocation = Location();
    this->mapMapping = MapMapping();
    this->myUART = UART();
    this->mySerial = MySerial();
    if (!this->_init_flag)
    {
        namedWindow("ControlPanel", WindowFlags::WINDOW_NORMAL);
        createTrackbar("Exit Program", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Exit Program", "ControlPanel", 0);
        createTrackbar("Separation mode", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Separation mode", "ControlPanel", 0);
        createTrackbar("Recorder", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Recorder", "ControlPanel", 0);
        this->LidarListenerBegin(argc, argv);
        if (!this->armorDetector.initModel())
        {
            this->stop();
            return;
        }
        this->carInferAvailable = this->carDetector.initModel() ? true : false;
        if(this->carInferAvailable)
            setTrackbarPos("Separation mode", "ControlPanel", 1);
        this->mySerial.initSerial();
        this->videoRecorder.init(VideoRecoderRath, VideoWriter::fourcc('m', 'p', '4', 'v'), Size(ImageW, ImageH));
        this->cameraThread.start();
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

void Radar::LidarMainLoop(Radar *radar)
{
    while (radar->__LidarMainLoop_working)
    {
        if (ros::ok())
            ros::spinOnce();
    }
}

void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    std::vector<std::vector<float>> tempDepth = this->depthQueue.pushback(*pc);
    unique_lock<shared_timed_mutex> ulk(myMutex);
    this->publicDepth.swap(tempDepth);
    ulk.unlock();
}

void Radar::SeparationLoop(Radar *radar)
{
    unique_lock<shared_timed_mutex> ulk(radar->myMutex);
    ulk.unlock();
    shared_lock<shared_timed_mutex> slk(radar->myMutex);
    slk.unlock();
    while (radar->__SeparationLoop_working)
    {
        vector<Rect> tempSeqTargets;
        if (radar->separation_mode == 0)
        {
            slk.lock();
            if(radar->publicDepth.size() > 0)
                tempSeqTargets = radar->movementDetector.applyMovementDetector(radar->publicDepth);
            slk.unlock();
        }
        else if (radar->separation_mode == 1)
        {
            ulk.lock();
            FrameBag image = radar->cameraThread.read();
            ulk.unlock();
            tempSeqTargets = radar->carDetector.infer(image.frame);
        }
        ulk.lock();
        radar->SeqTargets.swap(tempSeqTargets);
        ulk.unlock();
    }
}

void Radar::SerReadLoop(Radar *radar)
{
    radar->myUART.read(radar->mySerial);
}

void Radar::SerWriteLoop(Radar *radar)
{
    radar->myUART.write(radar->mySerial);
}

void Radar::MainProcessLoop(Radar *radar)
{
    unique_lock<shared_timed_mutex> ulk(radar->myMutex);
    ulk.unlock();
    shared_lock<shared_timed_mutex> slk(radar->myMutex);
    slk.unlock();
    while (radar->__MainProcessLoop_working)
    {
        slk.lock();
        int check_count = radar->SeqTargets.size();
        slk.unlock();
        FrameBag frameBag = radar->cameraThread.read();
        if (check_count > 0)
        {
            if (!radar->cameraThread.is_open())
            {
                radar->cameraThread.open();
                continue;
            }
            vector<bboxAndRect> pred;
            if (frameBag.flag)
            {
                vector<Rect> tempSeqTargets;
                ulk.lock();
                tempSeqTargets.swap(radar->SeqTargets);
                ulk.unlock();
                pred = radar->armorDetector.infer(frameBag.frame, tempSeqTargets);
                if (pred.size() == 0)
                    continue;
                radar->armor_filter(pred);
                slk.lock();
                radar->detectDepth(pred);
                slk.unlock();
                vector<ArmorBoundingBox> IouArmors;
                if(radar->separation_mode == 1)
                {
                    IouArmors = radar->mapMapping._IoU_prediction(pred);
                    radar->detectDepth(IouArmors);
                }
                radar->mapMapping.mergeUpdata(pred, IouArmors, radar->separation_mode);
                judge_message myJudge_message;
                myJudge_message.task = 1;
                myJudge_message.loc = radar->mapMapping.getloc();
                radar->send_judge(myJudge_message, radar->myUART);
            }
            else
                continue;
        }
        if (frameBag.flag)
            radar->myFrames.push(frameBag.frame);
    }
}

void Radar::VideoRecorderLoop(Radar *radar)
{
    while (radar->__VideoRecorderLoop_working)
    {
        if (radar->_if_record && radar->myFrames.size() > 0)
            radar->videoRecorder.write(radar->myFrames.front());
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
    this->_if_record = getTrackbarPos("Recorder", "ControlPanel");
    if (this->carInferAvailable)
        separation_mode = getTrackbarPos("Separation mode", "ControlPanel");
    else
        separation_mode = 0;
    if (!this->mapMapping._is_pass() || waitKey(1) == 76 || waitKey(1) == 108)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Locate pick start ...Process\n");
        // TODO: Fix here
        Mat rvec, tvec;
        unique_lock<shared_timed_mutex> ulk(myMutex);
        if (!this->myLocation.locate_pick(this->cameraThread, ENEMY, rvec, tvec))
        {
            ulk.unlock();
            return;
        }
        ulk.unlock();
        this->mapMapping.push_T(rvec, tvec);
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "[INFO], Locate pick Done \n");
    }
    if (!this->_thread_working)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Thread starting ...");
        this->_thread_working = true;
        if (!this->__LidarMainLoop_working)
        {
            this->__LidarMainLoop_working = true;
            this->lidarMainloop = thread(std::bind(&Radar::LidarMainLoop, this));
        }
        if (!this->__SeparationLoop_working)
        {
            this->__SeparationLoop_working = true;
            this->seqloop = thread(std::bind(&Radar::SeparationLoop, this));
        }
        if (!this->__MainProcessLoop_working)
        {
            this->__MainProcessLoop_working = true;
            this->processLoop = thread(std::bind(&Radar::MainProcessLoop, this));
        }
        if (!this->__VideoRecorderLoop_working)
        {
            this->__VideoRecorderLoop_working = true;
            this->videoRecoderLoop = thread(std::bind(&Radar::VideoRecorderLoop, this));
        }
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (!this->mySerial._is_open())
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Serial initing ...");
        this->mySerial.initSerial();
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                   "Done.\n");
    }
    if (!this->_Ser_working && this->mySerial._is_open())
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], SerThread initing ...");
        this->_Ser_working = true;
        this->serRead = thread(std::bind(&Radar::SerReadLoop, this));
        this->serR_t = this->serRead.native_handle();
        this->serWrite = thread(std::bind(&Radar::SerWriteLoop, this));
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
    this->is_alive = false;
    fmt::print(fg(fmt::color::orange_red) | fmt::emphasis::bold | fmt::v9::bg(fmt::color::white),
               "[WARN], Start Shutdown Process...");
    cv::destroyAllWindows();
    if (this->_thread_working)
    {
        this->_thread_working = false;
        this->__LidarMainLoop_working = false;
        this->__MainProcessLoop_working = false;
        this->__SeparationLoop_working = false;
        this->__VideoRecorderLoop_working = false;
        pthread_cancel(this->serR_t);
        pthread_cancel(this->serW_t);
        this->videoRecoderLoop.join(); 
        this->seqloop.join();
        this->processLoop.join();
        this->lidarMainloop.join(); 
    }
    if (this->cameraThread.is_open())
        this->cameraThread.stop();
    this->videoRecorder.close();
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "Done.\n");
}

bool Radar::alive()
{
    return this->is_alive;
}