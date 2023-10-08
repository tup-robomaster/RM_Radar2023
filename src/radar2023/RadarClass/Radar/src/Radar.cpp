#include "../include/Radar.h"

void Radar::armor_filter(vector<bboxAndRect> &pred)
{
    vector<bboxAndRect> results;
    for (int i = 0; i < int(this->mapMapping._ids.size()); ++i)
    {
        int max_id = 0;
        float max_conf = 0.f;
        for (size_t j = 0; j < pred.size(); ++j)
        {
            if ((int)pred[j].armor.cls == this->ids[i] && pred[j].armor.conf - max_conf > 0)
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
        float center[2] = {pred[i].armor.x0 + pred[i].armor.w / 2.f, pred[i].armor.y0 + pred[i].armor.h / 2.f};
        for (int j = int(max<float>(center[1] - pred[i].armor.h / 2.f, 0.)); j < int(min<float>(center[1] + pred[i].armor.h / 2.f, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - pred[i].armor.w / 2.f, 0.)); k < int(min<float>(center[0] + pred[i].armor.w / 2.f, ImageW)); ++k)
            {
                if (this->publicDepth[j][k] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[j][k]);
            }
        }
        float tempDepth = 0;
        for (const auto &jt : tempBox)
        {
            tempDepth += jt;
        }
        pred[i].armor.depth = tempDepth / tempBox.size();
        this->logger->info("Depth: [CLS] " + to_string(pred[i].armor.cls) + " [Depth] " + to_string(pred[i].armor.depth));
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
        float center[2] = {armors.at(i).x0 + armors[i].w / 2.f, armors[i].y0 + armors[i].h / 2.f};
        for (int j = int(max<float>(center[1] - armors[i].h / 2.f, 0.)); j < int(min<float>(center[1] + armors[i].h / 2.f, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - armors[i].w / 2.f, 0.)); k < int(min<float>(center[0] + armors[i].w / 2.f, ImageW)); ++k)
            {
                if (this->publicDepth[j][k] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[j][k]);
                ++count;
            }
        }
        float tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        armors[i].depth = tempNum / count;
        this->logger->info("Depth: [CLS] " + to_string(armors[i].cls) + " [Depth] " + to_string(armors[i].depth));
    }
}

void Radar::send_judge(judge_message &message)
{
    vector<vector<float>> loc;
    switch (message.task)
    {
    case 1:
        for (int i = 0; i < int(message.loc.size() / 2); ++i)
        {
            vector<float> temp_location;
            temp_location.emplace_back(message.loc[i + ENEMY * 6].x);
            temp_location.emplace_back(message.loc[i + ENEMY * 6].y);
            loc.emplace_back(temp_location);
        }
        this->myUART.myUARTPasser.push_loc(loc);
        break;

    default:
        break;
    }
}

void Radar::drawBbox(vector<DetectBox> &bboxs, Mat &img)
{
    for (DetectBox &it : bboxs)
    {
        cv::rectangle(img, Rect(it.x1, it.y1, it.x2 - it.x1, it.y2 - it.y1), Scalar(0, 255, 0), 2);
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
        cv::rectangle(img, temp, Scalar(0, 255, 0), 2);
        cv::putText(img, to_string(int(it.armor.cls)), cv::Point2i(temp.x, temp.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 3);
    }
}

Radar::Radar()
{
    this->myFrames.setDepth(FRAME_DEPTH);
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
    this->depthQueue = DepthQueue(K_0, C_0, E_0);
    if (!this->_init_flag)
    {
        namedWindow("ControlPanel", WindowFlags::WINDOW_NORMAL);
        createTrackbar("Exit Program", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Exit Program", "ControlPanel", 0);
        createTrackbar("Recorder", "ControlPanel", 0, 1, nullptr);
        setTrackbarPos("Recorder", "ControlPanel", 0);
        this->LidarListenerBegin(argc, argv);
        this->armorDetector.accessModelTest();

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
        this->carDetector.accessModelTest();
#endif

        if (!this->armorDetector.initModel())
        {
            this->stop();
            this->logger->flush();
            return;
        }

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
        if (!this->carDetector.initModel())
        {
            this->stop();
            this->logger->flush();
            return;
        }
#endif

#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
        this->dsTracker = std::make_shared<DsTracker>(DsTracker(SORT_ONNX_PATH, SORT_ENGINE_PATH));
#endif

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
    ros::shutdown();
    this->logger->critical("LidarMainLoop Exit");
}

void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    std::vector<std::vector<float>> tempDepth = this->depthQueue.pushback(*pc);
    unique_lock<shared_timed_mutex> ulk(this->myMutex_publicDepth);
    this->publicDepth.swap(tempDepth);
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
        if (!this->cameraThread.is_open())
        {
            this->cameraThread.open();
            continue;
        }
        FrameBag frameBag = this->cameraThread.read();
        if (frameBag.flag)
        {
#ifndef UseOneLayerInfer
#ifdef UsePointCloudSepTarget
            shared_lock<shared_timed_mutex> slk_md(this->myMutex_publicDepth);
            vector<Rect> sepTargets = this->movementDetector.applyMovementDetector(this->publicDepth);
            slk_md.unlock();
            vector<bboxAndRect> pred = this->movementDetector._ifHistoryBuild() ? this->armorDetector.infer(frameBag.frame, sepTargets) : {};
#else
            vector<DetectBox> sepTargets = this->carDetector.infer(frameBag.frame);
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
            this->dsTracker->sort(frameBag.frame, sepTargets);
#endif
            vector<bboxAndRect> pred = this->armorDetector.infer(frameBag.frame, sepTargets);
#endif
#else
            vector<bboxAndRect> pred = this->armorDetector.infer(frameBag.frame);
#endif
#if defined Test && defined TestWithVis
#ifndef UseOneLayerInfer
            this->drawBbox(sepTargets, frameBag.frame);
#endif
            this->drawArmorsForDebug(pred, frameBag.frame);
#endif

            if (pred.size() != 0)
            {
                this->armor_filter(pred);
                shared_lock<shared_timed_mutex> slk(this->myMutex_publicDepth);
                if (this->publicDepth.size() == 0)
                {
                    slk.unlock();
                    this->logger->info("No Lidar Msg , Return");
                }
                else
                {
                    this->detectDepth(pred);
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
                    this->mapMapping._DeepSort_prediction(pred, sepTargets);
#endif
#ifndef UseOneLayerInfer
                    vector<ArmorBoundingBox> IouArmors = this->mapMapping._IoU_prediction(pred, sepTargets);
#else
                    vector<ArmorBoundingBox> IouArmors = {};
#endif
                    this->detectDepth(IouArmors);
                    slk.unlock();
                    this->mapMapping.mergeUpdata(pred, IouArmors, this->K_0_Mat, this->C_0_Mat);
                    judge_message myJudge_message;
                    myJudge_message.task = 1;
                    myJudge_message.loc = this->mapMapping.getloc();
                    this->send_judge(myJudge_message);
                }
            }
            auto end_t = std::chrono::system_clock::now().time_since_epoch();
#ifdef Test
            char ch[255];
            sprintf(ch, "FPS %d", int(std::chrono::nanoseconds(1000000000).count() / (end_t - start_t).count()));
            std::string fps_str = ch;
            cv::putText(frameBag.frame, fps_str, {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 0}, 3);
#endif
            this->myFrames.push(frameBag.frame);
            this->logger->flush();
        }
        else
            continue;
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
        this->mapMapping._plot_region_rect(this->show_region, frame, this->K_0_Mat, this->C_0_Mat);
        cv::Mat map1, map2;
        cv::Size imageSize = frame.size();
        cv::initUndistortRectifyMap(this->K_0_Mat, this->C_0_Mat, cv::Mat(), cv::getOptimalNewCameraMatrix(this->K_0_Mat, this->C_0_Mat, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
        cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);
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
    cv::destroyAllWindows();
    if (this->_thread_working)
    {
        this->_thread_working = false;
        if (this->__LidarMainLoop_working)
        {
            this->__LidarMainLoop_working = false;
            this->lidarMainloop.join();
        }
        if (this->__MainProcessLoop_working)
        {
            this->__MainProcessLoop_working = false;
            this->processLoop.join();
        }
        if (this->__VideoRecorderLoop_working)
        {
            this->__VideoRecorderLoop_working = false;
            this->videoRecoderLoop.join();
        }
        if (this->_Ser_working)
        {
            this->_Ser_working = false;
            this->serRead.join();
            this->serWrite.join();
        }
    }
    this->videoRecorder.close();
    this->_if_record = false;
    this->armorDetector.unInit();
#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    this->carDetector.unInit();
#endif
    this->logger->warn("Program Shutdown");
}

bool Radar::alive()
{
    return this->is_alive;
}