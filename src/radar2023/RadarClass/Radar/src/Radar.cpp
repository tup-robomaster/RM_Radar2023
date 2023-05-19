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
        vector<float> tempBox;
        float center[2] = {pred[i].armor.x0 + pred[i].armor.w / 2, pred[i].armor.y0 + pred[i].armor.h / 2};
        for (int j = int(max<float>(center[1] - pred[i].armor.h, 0.)); j < int(min<float>(center[1] + pred[i].armor.h, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - pred[i].armor.w, 0.)); k < int(min<float>(center[0] + pred[i].armor.w, ImageW)); ++k)
            {
                if (this->publicDepth[i][j] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[i][j]);
            }
        }
        float tempDepth = 0;
        for (const auto &jt : tempBox)
        {
            tempDepth += jt;
        }
        pred[i].armor.depth = tempDepth / tempBox.size();
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

void Radar::drawBbox(vector<Rect> &bboxs, Mat &img)
{
    for (Rect &it : bboxs)
    {
        cv::rectangle(img, it, Scalar(0, 255, 0), 2);
    }
}

void Radar::drawArmorsForDebug(vector<ArmorBoundingBox> &armors, Mat &img)
{
    for (auto &it : armors)
    {
        Rect temp = Rect(it.x0, it.y0, it.w, it.h);
        cv::rectangle(img, temp, Scalar(255, 255, 0), 2);
        stringstream ss;
        ss << it.cls << "[Depth]" << it.depth << "[Conf]" << it.conf;
        cv::putText(img, ss.str(), Point2i(int(it.x0 + it.w / 2), int(it.y0 + it.h / 2)), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
    }
}

void Radar::drawArmorsForDebug(vector<bboxAndRect> &armors, Mat &img)
{
    for (auto &it : armors)
    {
        Rect temp = Rect(it.armor.x0, it.armor.y0, it.armor.w, it.armor.h);
        cv::rectangle(img, temp, Scalar(255, 255, 0), 2);
        stringstream ss;
        ss << it.armor.cls << "[Depth]" << it.armor.depth << "[Conf]" << it.armor.conf;
        cv::putText(img, ss.str(), Point2i(int(it.armor.x0 + it.armor.w / 2), int(it.armor.y0 + it.armor.h / 2)), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
    }
}

Radar::Radar()
{
}

Radar::~Radar()
{
    if (this->is_alive)
        this->stop();
    this->logger->flush();
}

void Radar::init(int argc, char **argv)
{
    if (this->_init_flag)
        return;
    this->logger->info("Initing ...Process");
    if (ENEMY)
        this->logger->critical("YOU ARE RED");
    else
        this->logger->critical("YOU ARE BLUE");
    Matrix<float, 3, 3> K_0;
    Matrix<float, 1, 5> C_0;
    Matrix<float, 4, 4> E_0;
    if (!read_param(this->K_0_Mat, this->C_0_Mat, this->E_0_Mat))
    {
        this->logger->error("Can't read CAMERA_PARAM: {}!", CAMERA_PARAM_PATH);
        return;
    }
    cv2eigen(this->K_0_Mat, K_0);
    cv2eigen(this->C_0_Mat, C_0);
    cv2eigen(this->E_0_Mat, E_0);
    // TODO: CHECK HERE
    this->depthQueue = DepthQueue(K_0, C_0, E_0);
    if (!this->_init_flag)
    {
        namedWindow("ControlPanel", WindowFlags::WINDOW_NORMAL);
        createTrackbar("Exit Program", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Exit Program", "ControlPanel", 0);
        createTrackbar("Recorder", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Recorder", "ControlPanel", 0);
        this->LidarListenerBegin(argc, argv);
        if (!(this->armorDetector.initModel() && this->carDetector.initModel()))
        {
            this->stop();
            this->logger->flush();
            return;
        }
        this->mySerial.initSerial(SerialPortNAME, PASSWORD);
        this->videoRecorder.init(VideoRecoderRath, VideoWriter::fourcc('m', 'p', '4', 'v'), Size(ImageW, ImageH)) ? setTrackbarPos("Recorder", "ControlPanel", 1) : setTrackbarPos("Recorder", "ControlPanel", 0);
        this->cameraThread.start();
        this->_init_flag = true;
        this->logger->info("Init Done");
    }
    this->is_alive = true;
}

void Radar::LidarListenerBegin(int argc, char **argv)
{
    if (this->_is_LidarInited)
        return;
    ros::init(argc, argv, "laser_listener");
    ros::NodeHandle nh;
    sub = nh.subscribe(lidarTopicName, LidarQueueSize, &Radar::LidarCallBack, this);
    this->_is_LidarInited = true;
    this->logger->info("Lidar inited");
}

void Radar::LidarMainLoop()
{
    while (this->__LidarMainLoop_working)
    {
        if (ros::ok())
            ros::spinOnce();
    }
    this->logger->critical("LidarMainLoop Exit");
}

void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    std::vector<std::vector<float>> tempDepth = this->depthQueue.pushback(*pc);
    unique_lock<shared_timed_mutex> ulk(this->myMutex_publicDepth);
    this->publicDepth.swap(tempDepth);
    ++this->_if_DepthUpdated;
    ulk.unlock();
}

void Radar::SerReadLoop()
{
    while (this->_Ser_working)
    {
        this->myUART.read(this->mySerial);
    }
    this->logger->critical("SerReadLoop Exit");
}

void Radar::SerWriteLoop()
{
    while (this->_Ser_working)
    {
        this->myUART.write(this->mySerial);
    }
    this->logger->critical("SerWriteLoop Exit");
}

void Radar::MainProcessLoop()
{
    while (this->__MainProcessLoop_working)
    {
        auto start_t = std::chrono::system_clock::now().time_since_epoch();
        FrameBag frameBag = this->cameraThread.read();
        if (!this->cameraThread.is_open())
        {
            this->cameraThread.open();
            continue;
        }
        if (frameBag.flag)
        {
            vector<Rect> sepTargets = this->carDetector.infer(frameBag.frame);
            vector<bboxAndRect> pred = this->armorDetector.infer(frameBag.frame, sepTargets);
#ifdef Test
            this->drawBbox(sepTargets, frameBag.frame);
#endif
            if (pred.size() != 0)
            {
                this->armor_filter(pred);
                if (this->_if_DepthUpdated == 0)
                {
                    this->logger->info("No Lidar Msg , Return");
                    continue;
                }
                cout << 1 << endl;
                this->detectDepth(pred);
                cout << 1 << endl;
                vector<ArmorBoundingBox> IouArmors = this->mapMapping._IoU_prediction(pred, sepTargets);
                cout << 1 << endl;
                this->detectDepth(IouArmors);
                cout << 1 << endl;
#ifdef Test
                this->drawArmorsForDebug(pred, frameBag.frame);
                this->drawArmorsForDebug(IouArmors, frameBag.frame);
#endif
                this->mapMapping.mergeUpdata(pred, IouArmors);
                cout << 1 << endl;
                judge_message myJudge_message;
                myJudge_message.task = 1;
                myJudge_message.loc = this->mapMapping.getloc();
                cout << 1 << endl;
                this->send_judge(myJudge_message, this->myUART);
                cout << 1 << endl;
            }
        }
        else
            continue;
        auto end_t = std::chrono::system_clock::now().time_since_epoch();
#ifdef Test
        char ch[255];
        sprintf(ch, "FPS %d", int(std::chrono::nanoseconds(1000000000).count() / (end_t - start_t).count()));
        std::string fps_str = ch;
        cv::putText(frameBag.frame, fps_str, {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 0});
#endif
        if (frameBag.flag)
            this->myFrames.push(frameBag.frame);
    }
    this->logger->critical("MainProcessLoop Exit");
}

void Radar::VideoRecorderLoop()
{
    while (this->__VideoRecorderLoop_working)
    {
        if (this->_if_record && this->myFrames.size() > 0)
        {
            this->videoRecorder.write(this->myFrames.front().clone());
        }
        else if (!this->_if_record)
        {
            this->videoRecorder.close();
        }
    }
    this->logger->critical("VideoRecorderLoop Exit");
}

void Radar::spin(int argc, char **argv)
{
    // TODO: 增加更多标志位来扩展流程控制
    this->init(argc, argv);
    if (!this->_init_flag)
        return;
    if (getTrackbarPos("Exit Program", "ControlPanel") == 1)
    {
        this->stop();
        return;
    }
    this->_if_record = getTrackbarPos("Recorder", "ControlPanel");
    if (!this->mapMapping._is_pass() || waitKey(1) == 76 || waitKey(1) == 108)
    {
        this->logger->info("Locate pick start ...Process");
        // TODO: Fix here
        Mat rvec, tvec;
        unique_lock<shared_timed_mutex> ulk(myMutex_cameraThread);
        try
        {
            if (!this->myLocation.locate_pick(this->cameraThread, ENEMY, rvec, tvec))
            {
                ulk.unlock();
                return;
            }
            ulk.unlock();
        }
        catch (const std::exception &e)
        {
            this->logger->error(e.what());
            return;
        }
        this->mapMapping.push_T(rvec, tvec);
        this->logger->info("Locate pick Done");
    }
    if (!this->_thread_working && this->is_alive)
    {
        this->logger->info("Thread starting ...Process");
        this->_thread_working = true;
        if (!this->__LidarMainLoop_working)
        {
            this->__LidarMainLoop_working = true;
            this->lidarMainloop = thread(std::bind(&Radar::LidarMainLoop, this));
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
        this->logger->info("Thread starting ...Done");
    }
    if (!this->mySerial._is_open() && this->is_alive)
    {
        this->logger->info("Serial initing ...Process");
        this->mySerial.initSerial(SerialPortNAME, PASSWORD);
        this->logger->info("Serial initing ...Done");
    }
    if (!this->_Ser_working && this->mySerial._is_open() && this->is_alive)
    {
        this->logger->info("SerThread initing ...Process");
        this->_Ser_working = true;
        this->serRead = thread(std::bind(&Radar::SerReadLoop, this));
        this->serWrite = thread(std::bind(&Radar::SerWriteLoop, this));
        this->logger->info("SerThread initing ...Done");
    }
    if (myFrames.size() > 0 && this->is_alive)
    {
        Mat frame = myFrames.front().clone();
        this->mapMapping._plot_region_rect(this->location_show, frame, this->K_0_Mat, this->C_0_Mat);
        imshow("ControlPanel", frame);
        resizeWindow("ControlPanel", 1920, 1080);
        myFrames.pop();
    }
}

void Radar::stop()
{
    this->is_alive = false;
    this->logger->warn("Start Shutdown Process...");
    this->logger->flush();
    if (this->cameraThread.is_open())
        this->cameraThread.stop();
    this->videoRecorder.close();
    this->_if_record = false;
    cv::destroyAllWindows();
    if (this->_thread_working)
    {
        this->_thread_working = false;
        this->__LidarMainLoop_working = false;
        this->__MainProcessLoop_working = false;
        this->__VideoRecorderLoop_working = false;
        this->_Ser_working = false;
        this->videoRecoderLoop.join();
        this->processLoop.join();
        this->lidarMainloop.join();
        this->serRead.join();
        this->serWrite.join();
    }
    this->armorDetector.unInit();
    this->carDetector.unInit();
    this->logger->warn("Program Shutdown");
}

bool Radar::alive()
{
    return this->is_alive;
}