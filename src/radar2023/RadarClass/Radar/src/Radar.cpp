#include "../include/Radar.h"

void Radar::armor_filter(vector<bboxAndRect> &pred)
{
    vector<bboxAndRect> results;
    for (int i = 0; i < int(this->mapMapping->_ids.size()); ++i)
    {
        int max_id = 0;
        float max_conf = 0.f;
        for (size_t j = 0; j < pred.size(); ++j)
        {
            if ((int)pred[j].armor.cls == this->ids[i] && pred[j].armor.conf - max_conf > Epsilon)
            {
                max_id = j;
                max_conf = pred[j].armor.conf;
            }
        }
        if (fabs(max_conf) > Epsilon)
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
        if (pred[i].armor.x0 > ImageW || pred[i].armor.y0 > ImageH || pred[i].armor.x0 + pred[i].armor.w > ImageW || pred[i].armor.y0 + pred[i].armor.h > ImageH)
            continue;
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
        pred[i].armor.depth = tempBox.size() != 0 ? tempDepth / tempBox.size() : 0.;
        this->logger->info("Depth: [CLS] " + to_string(pred[i].armor.cls) + " [Depth] " + to_string(pred[i].armor.depth));
    }
}

void Radar::detectDepth(vector<ArmorBoundingBox> &armors)
{
    if (armors.size() == 0)
        return;
    for (size_t i = 0; i < armors.size(); ++i)
    {
        if (armors[i].x0 > ImageW || armors[i].y0 > ImageH || armors[i].x0 + armors[i].w > ImageW || armors[i].y0 + armors[i].h > ImageH)
            continue;
        int count = 0;
        vector<float> tempBox;
        float center[2] = {armors[i].x0 + armors[i].w / 2.f, armors[i].y0 + armors[i].h / 2.f};
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
        int tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        armors[i].depth = count != 0 ? (float)tempNum / (float)count : 0.;
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
            temp_location.emplace_back(message.loc[i + this->ENEMY * 6].x);
            temp_location.emplace_back(message.loc[i + this->ENEMY * 6].y);
            loc.emplace_back(temp_location);
        }
        this->myUART->myUARTPasser.push_loc(loc);
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

void Radar::init()
{
    assert(this->nh);
    assert(!share_path.empty());
    if (this->_init_flag)
        return;
    this->logger->info("Initing ...Process");
    if (ENEMY)
        this->logger->critical("YOU ARE RED");
    else
        this->logger->critical("YOU ARE BLUE");
    this->GUI_image_pub_ = this->image_transport->advertise("/radar2023/result_view", 1, true);
    this->pub_locations = this->nh->advertise<radar2023::Locations>("/radar2023/locations", 1, true);
    std::string param_name;
    if (this->nh->searchParam("/radar2023/EnemyType", param_name))
    {
        this->nh->getParam(param_name, this->ENEMY);
    }
    else
    {
        ROS_WARN("Parameter EnemyType not defined");
    }
    if (this->nh->searchParam("/radar2023/CameraParam", param_name))
    {
        this->nh->getParam(param_name, this->CAMERA_PARAM);
    }
    else
    {
        ROS_WARN("Parameter CameraParam not defined");
    }
    if (this->nh->searchParam("/radar2023/AuthPassword", param_name))
    {
        this->nh->getParam(param_name, this->PASSWORD);
    }
    else
    {
        ROS_WARN("Parameter AuthPassword not defined");
    }
    if (this->nh->searchParam("/radar2023/EngineForArmor", param_name))
    {
        this->nh->getParam(param_name, this->EngineForArmor);
    }
    else
    {
        ROS_WARN("Parameter EngineForArmor not defined");
    }
    if (this->nh->searchParam("/radar2023/OnnxForArmor", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForArmor);
    }
    else
    {
        ROS_WARN("Parameter OnnxForArmor not defined");
    }
    if (this->nh->searchParam("/radar2023/CameraConfig", param_name))
    {
        this->nh->getParam(param_name, this->CameraConfig);
    }
    else
    {
        ROS_WARN("Parameter CameraConfig not defined");
    }
    if (this->nh->searchParam("/radar2023/SerialPortName", param_name))
    {
        this->nh->getParam(param_name, this->SerialPortName);
    }
    else
    {
        ROS_WARN("Parameter SerialPortName not defined");
    }

#ifdef UsingVideo
    if (this->nh->searchParam("/radar2023/TestVideo", param_name))
    {
        this->nh->getParam(param_name, this->TestVideo);
    }
    else
    {
        ROS_WARN("Parameter TestVideo not defined");
    }
#endif
#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    if (this->nh->searchParam("/radar2023/EngineForCar", param_name))
    {
        this->nh->getParam(param_name, this->EngineForCar);
    }
    else
    {
        ROS_WARN("Parameter EngineForCar not defined");
    }
    if (this->nh->searchParam("/radar2023/OnnxForCar", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForCar);
    }
    else
    {
        ROS_WARN("Parameter OnnxForCar not defined");
    }
#endif
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    if (this->nh->searchParam("/radar2023/EngineForSort", param_name))
    {
        this->nh->getParam(param_name, this->EngineForSort);
    }
    else
    {
        ROS_WARN("Parameter EngineForSort not defined");
    }
    if (this->nh->searchParam("/radar2023/OnnxForSort", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForSort);
    }
    else
    {
        ROS_WARN("Parameter OnnxForSort not defined");
    }
#endif

    Matrix<float, 3, 3> K_0;
    Matrix<float, 1, 5> C_0;
    Matrix<float, 4, 4> E_0;
    if (!read_param(this->K_0_Mat, this->C_0_Mat, this->E_0_Mat, this->share_path + "/params/" + this->CAMERA_PARAM))
    {
        this->logger->error("Can't read CAMERA_PARAM: {}!", this->share_path + "/params/" + this->CAMERA_PARAM);
        return;
    }
    cv2eigen(this->K_0_Mat, K_0);
    cv2eigen(this->C_0_Mat, C_0);
    cv2eigen(this->E_0_Mat, E_0);
    this->depthQueue = std::make_shared<DepthQueue>(K_0, C_0, E_0);
    this->myUART = std::make_shared<UART>(this->ENEMY);
    this->myLocation = std::make_shared<Location>();
    this->videoRecorder = std::make_shared<VideoRecorder>();
    this->mySerial = std::make_shared<MySerial>();
    this->mapMapping = std::make_shared<MapMapping>();
    this->myLocation->decodeMapPoints(this->share_path + "/params/MapMappingPoints.json");

#ifdef UsingVideo
    this->cameraThread = std::make_shared<CameraThread>(this->share_path + "/params/" + this->CameraConfig,
                                                        this->share_path + "/resources/" + this->TestVideo);
#else
    this->cameraThread = std::make_shared<CameraThread>(this->share_path + "/params/" + this->CameraConfig);
#endif

    this->nh->setParam("/radar2023/ExitProgram", false);
    this->nh->setParam("/radar2023/Recorder", true);
    this->LidarListenerBegin();
    this->armorDetector = std::make_shared<ArmorDetector>(this->share_path + "/models/" + this->EngineForArmor,
                                                          this->share_path + "/models/" + this->OnnxForArmor);
    this->armorDetector->accessModelTest();

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    this->carDetector = std::make_shared<CarDetector>(this->share_path + "/models/" + this->EngineForCar,
                                                      this->share_path + "/models/" + this->OnnxForCar);
    this->carDetector->accessModelTest();
#endif

    if (!this->armorDetector->initModel())
    {
        this->stop();
        this->logger->flush();
        return;
    }

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    if (!this->carDetector->initModel())
    {
        this->stop();
        this->logger->flush();
        return;
    }
#endif
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    this->dsTracker = std::make_shared<DsTracker>(this->share_path + "/models/" + this->OnnxForSort,
                                                  this->share_path + "/models/" + this->EngineForSort);
#endif
#ifdef ExperimentalOutput
    this->myExpLog = std::make_shared<ExpLog>();
    this->myExpLog->init(this->share_path + "/ExpResultDir/");
#endif

    this->_recorder_block = !this->videoRecorder->init((this->share_path + "/Record/").c_str(), VideoWriter::fourcc('m', 'p', '4', 'v'), Size(ImageW, ImageH));
    this->cameraThread->start();
    this->_init_flag = true;
    this->logger->info("Init Done");
    this->is_alive = true;
}

void Radar::setRosNodeHandle(ros::NodeHandle &nh)
{
    this->nh = std::make_shared<ros::NodeHandle>(nh);
}

void Radar::setRosImageTransport(image_transport::ImageTransport &image_transport)
{
    this->image_transport = std::make_shared<image_transport::ImageTransport>(image_transport);
}

void Radar::setRosPackageSharedPath(String &path)
{
    this->share_path = path;
}

void Radar::LidarListenerBegin()
{
    assert(this->nh);
    if (this->_is_LidarInited)
        return;
    this->sub_lidar = this->nh->subscribe(lidarTopicName, LidarQueueSize, &Radar::LidarCallBack, this);
    this->_is_LidarInited = true;
    this->logger->info("Lidar inited");
}

void Radar::RosSpinLoop()
{
    while (this->__RosSpinLoop_working)
    {
        if (ros::ok())
            ros::spinOnce();
    }
    ros::shutdown();
    this->logger->critical("RosSpinLoop Exit");
}

void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    std::vector<std::vector<float>> tempDepth = this->depthQueue->pushback(*pc);
    unique_lock<shared_timed_mutex> ulk(this->myMutex_publicDepth);
    this->publicDepth.swap(tempDepth);
    ulk.unlock();
}

void Radar::SerReadLoop()
{
    while (this->_Ser_working)
    {
        this->myUART->read(this->mySerial);
    }
    this->logger->critical("SerReadLoop Exit");
}

void Radar::SerWriteLoop()
{
    while (this->_Ser_working)
    {
        this->myUART->write(this->mySerial);
    }
    this->logger->critical("SerWriteLoop Exit");
}

void Radar::MainProcessLoop()
{
    while (this->__MainProcessLoop_working)
    {
        auto start_t = std::chrono::system_clock::now().time_since_epoch();
        if (!this->cameraThread->is_open())
        {
            this->cameraThread->open();
            continue;
        }
        FrameBag frameBag = this->cameraThread->read();
        if (frameBag.flag)
        {
            if (this->_if_record)
            {
                Mat record_frame = frameBag.frame.clone();
                this->myFrames.push(record_frame);
            }

#ifndef UseOneLayerInfer
#ifdef UsePointCloudSepTarget
            shared_lock<shared_timed_mutex> slk_md(this->myMutex_publicDepth);
            vector<Rect> sepTargets = this->movementDetector->applyMovementDetector(this->publicDepth);
            slk_md.unlock();
            vector<bboxAndRect> pred = this->movementDetector->_ifHistoryBuild() ? this->armorDetector.infer(frameBag.frame, sepTargets) : {};
#else
            vector<DetectBox> sepTargets = this->carDetector->infer(frameBag.frame);
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
            this->dsTracker->sort(frameBag.frame, sepTargets);
#endif
            vector<bboxAndRect> pred = this->armorDetector->infer(frameBag.frame, sepTargets);
#endif
#else
            vector<bboxAndRect> pred = this->armorDetector->infer(frameBag.frame);
#endif
#if defined Test && defined TestWithVis
#ifndef UseOneLayerInfer
            this->drawBbox(sepTargets, frameBag.frame);
#endif
            this->drawArmorsForDebug(pred, frameBag.frame);
#endif
#ifdef ExperimentalOutput
            int pred_size = pred.size();
            float pred_conf_average = sumConfAverage(pred);
#endif

#ifdef Test
#ifdef ShowDepth
            if (this->_if_coverDepth)
            {
                shared_lock<shared_timed_mutex> slk_pd(this->myMutex_publicDepth);
                for (int i = 0; i < ImageH; ++i)
                {
                    uchar *data = frameBag.frame.ptr<uchar>(i);
                    int k = 0;
                    for (int j = 0; j < ImageW; ++j)
                    {
                        if (fabs(this->publicDepth[i][j]) > Epsilon)
                        {
                            int b, g, r;
                            hsv_to_bgr((this->publicDepth[i][j] / 100.) * 255, 255, 255, b, g, r);
                            data[k] = b;
                            data[k + 1] = g;
                            data[k + 2] = r;
                        }
                        k += 3;
                    }
                }
                slk_pd.unlock();
            }
#endif
#endif

            if (pred.size() != 0)
            {
                this->armor_filter(pred);
                shared_lock<shared_timed_mutex> slk_pd(this->myMutex_publicDepth);
                if (this->publicDepth.size() == 0)
                {
                    slk_pd.unlock();
                    this->logger->info("No Lidar Msg , Return");
                }
                else
                {
                    this->detectDepth(pred);

#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
                    this->mapMapping._DeepSort_prediction(pred, sepTargets);
#endif
#ifndef UseOneLayerInfer
                    vector<ArmorBoundingBox> IouArmors = this->mapMapping->_IoU_prediction(pred, sepTargets);
#else
                    vector<ArmorBoundingBox> IouArmors = {};
#endif

                    this->detectDepth(IouArmors);
                    slk_pd.unlock();
                    this->mapMapping->mergeUpdata(pred, IouArmors, this->K_0_Mat, this->C_0_Mat);
                    judge_message myJudge_message;
                    myJudge_message.task = 1;
                    myJudge_message.loc = this->mapMapping->getloc();
                    this->send_judge(myJudge_message);
                    if (myJudge_message.loc.size() > 0)
                    {
                        radar2023::Locations locations_msg;
                        for (int i = 0, N = myJudge_message.loc.size(); i < N; ++i)
                        {
                            radar2023::Location location_msg;
                            location_msg.id = myJudge_message.loc[i].id;
                            location_msg.x = myJudge_message.loc[i].x;
                            location_msg.y = myJudge_message.loc[i].y;
                            locations_msg.locations.emplace_back(location_msg);
                        }
                        locations_msg.header = std_msgs::Header();
                        locations_msg.header.frame_id = "radar2023";
                        locations_msg.header.stamp = ros::Time::now();
                        locations_msg.header.seq = 1;
                        this->pub_locations.publish(locations_msg);
                    }
                }
            }
            auto end_t = std::chrono::system_clock::now().time_since_epoch();

#ifdef Test
            char ch[255];
            sprintf(ch, "FPS %d", int(std::chrono::nanoseconds(1000000000).count() / (end_t - start_t).count()));
            std::string fps_str = ch;
            cv::putText(frameBag.frame, fps_str, {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 0}, 3);
            this->mapMapping->_plot_region_rect(this->show_region, frameBag.frame, this->K_0_Mat, this->C_0_Mat);
#endif

            sensor_msgs::ImagePtr image_msg;
            try
            {
                image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameBag.frame).toImageMsg();
                image_msg->header.frame_id = "radar2023";
                image_msg->header.stamp = ros::Time::now();
                image_msg->header.seq = 1;
                this->GUI_image_pub_.publish(image_msg);
            }
            catch (cv_bridge::Exception &e)
            {
                this->logger->error("cv_bridge exception: %s", e.what());
                this->logger->flush();
                continue;
            }
            this->logger->flush();

#ifdef ExperimentalOutput
            std::vector<string> msg;
            msg.emplace_back(to_string(pred.size()));
            msg.emplace_back(to_string(pred_size));
            msg.emplace_back(to_string(sumConfAverage(pred)));
            msg.emplace_back(to_string(pred_conf_average));
            msg.emplace_back(to_string((end_t - start_t).count()));
            this->myExpLog->input(msg);
#endif
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
        if (this->_if_record && this->myFrames.size() > 0 && !this->_recorder_block)
        {
            this->videoRecorder->write(this->myFrames.front());
            this->myFrames.pop();
        }
        else if (!this->_if_record)
        {
            sleep(1);
        }
    }
    this->videoRecorder->close();
    this->_if_record = false;
    this->nh->setParam("/radar2023/Recorder", false);
    this->logger->critical("VideoRecorderLoop Exit");
}

void Radar::spin()
{
    // TODO: 增加更多标志位来扩展流程控制
    this->init();
    assert(depthQueue && armorDetector);

#if !(defined UseOneLayerInfer)
#ifdef UsePointCloudSepTarget
    assert(movementDetector);
#else
    assert(carDetector);
#endif
#endif

    assert(cameraThread && myLocation && mapMapping && myUART && mySerial && videoRecorder);

#ifdef ExperimentalOutput
    assert(myExpLog);
#endif

    if (!this->_init_flag)
        return;
    std::string param_name;
    if (this->nh->searchParam("/radar2023/ExitProgram", param_name))
    {
        this->nh->getParam(param_name, this->ExitProgramSiginal);
    }
    else
    {
        ROS_WARN("Parameter ExitProgram not defined");
    }
    if (this->nh->searchParam("/radar2023/Recorder", param_name))
    {
        this->nh->getParam(param_name, this->VideoRecorderSiginal);
    }
    else
    {
        ROS_WARN("Parameter Recorder not defined");
    }

#ifdef ShowDepth
    if (this->nh->searchParam("/gui/CoverDepth", param_name))
    {
        this->nh->getParam(param_name, this->_if_coverDepth);
    }
    else
    {
        ROS_WARN("Parameter CoverDepth not defined");
    }
#endif

    if (this->ExitProgramSiginal == true)
    {
        this->stop();
        return;
    }
    this->_if_record = VideoRecorderSiginal;
    if (!this->mapMapping->_is_pass())
    {
        this->logger->info("Locate pick start ...Process");
        // TODO: Fix here
        Mat rvec, tvec;
        unique_lock<shared_timed_mutex> ulk(myMutex_cameraThread);
        try
        {
            if (!this->myLocation->locate_pick(this->cameraThread, this->ENEMY, rvec, tvec, this->K_0_Mat, this->C_0_Mat, this->E_0_Mat))
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
        this->mapMapping->push_T(rvec, tvec);
        this->logger->info("Locate pick Done");
    }
    if (!this->_thread_working && this->is_alive)
    {
        this->logger->info("Thread starting ...Process");
        this->_thread_working = true;
        if (!this->__RosSpinLoop_working)
        {
            this->__RosSpinLoop_working = true;
            this->rosSpinLoop = thread(std::bind(&Radar::RosSpinLoop, this));
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
    if (!this->mySerial->_is_open() && this->is_alive)
    {
        this->logger->info("Serial initing ...Process");
        this->mySerial->initSerial(this->SerialPortName, this->PASSWORD);
        this->logger->info("Serial initing ...Done");
    }
    if (!this->_Ser_working && this->mySerial->_is_open() && this->is_alive)
    {
        this->logger->info("SerThread initing ...Process");
        this->_Ser_working = true;
        this->serRead = thread(std::bind(&Radar::SerReadLoop, this));
        this->serWrite = thread(std::bind(&Radar::SerWriteLoop, this));
        this->logger->info("SerThread initing ...Done");
    }
    if (myFrames.size() > 0 && this->is_alive && !this->_if_record)
    {
        myFrames.pop();
    }
}

void Radar::stop()
{
    this->is_alive = false;
    this->logger->warn("Start Shutdown Process...");
    this->logger->flush();
    if (this->cameraThread->is_open())
        this->cameraThread->stop();
    if (this->_thread_working)
    {
        this->_thread_working = false;
        if (this->__RosSpinLoop_working)
        {
            this->__RosSpinLoop_working = false;
            this->rosSpinLoop.join();
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
    this->armorDetector->unInit();

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    this->carDetector->unInit();
#endif

#ifdef ExperimentalOutput
    this->myExpLog->uninit();
#endif

    this->logger->warn("Program Shutdown");
}

bool Radar::alive()
{
    return this->is_alive;
}