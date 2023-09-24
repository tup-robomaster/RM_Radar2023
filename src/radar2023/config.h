#define Test                                                //测试标志
#define TestWithVis                                         //显示可视化检测结果
// #define UsingVideo                                          //是否使用视频(！！！可能造成OOM, 请注意设置FRAME_DEPTH)

#define PASSWORD (char *)"momoko11"                         //用户密码，用于串口权限

#define SerialPortNAME (char *)"/dev/ttyUSB0"
#define lidarTopicName (char *)"/livox/lidar"

#define FRAME_DEPTH 500                                     //图像队列深度

#define CameraConfigPath (char *)"/home/ninefish/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Camera/params/Config_0.Config" // 相机配置文件名称
#define TestVideoPath (char *)"/home/ninefish/nine-fish/RM_Radar2023/resources/1.mp4"                    // DEMO视频路径

#define MaxPointsNum 10000 // 最大点云数量
#define ImageH 2064        // 图像高度
#define ImageW 3088        // 图像宽度
#define maxQueueSize 100   // 点云最大帧队列长度
#define LidarQueueSize 1   // 雷达消息队列

#define CAMERA_PARAM_PATH (char *)"/home/ninefish/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Camera/params/camera0.yaml" // 相机参数文件
#define VideoRecoderRath (char *)"/home/ninefish/nine-fish/RM_Radar2023/Record/"                              // 录制保存文件

#define OnnxMoudlePath (char *)"/home/ninefish/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/43best.onnx"
#define OnnxMoudlePath_c (char *)"/home/ninefish/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/best.onnx"
#define TensorRTEnginePath (char *)"/home/ninefish/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/model_trt.engine"     // Engine
#define TensorRTEnginePath_c (char *)"/home/ninefish/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/models/model_trt_c.engine" // Engine

#define MAXBO 3
#define ENEMY 1         // 0 RED 1 BLUE

#define Z_A true        // Z轴突变调整
#define L_P true        // 位置预测
#define Z_THRE 0.2      // Z轴突变阈值
#define Pre_Time 10     // 预测次数
#define Pre_radio 0.2f  // 预测速度比例
#define Real_Size_W 15. // 真实宽度
#define Real_Size_H 28. // 真实高度（长）
#define IoU_THRE 0.8f   // IoU预测阈值

#define LOGPATH (char *)"/home/ninefish/nine-fish/RM_Radar2023/src/radar2023/logs/" // log日志存储文件夹

/*---For old SepTarget method [实验性][已废弃][谨慎使用]---*/
//#define UsePointCloudSepTarget
#define MDHistorySize 200 //背景深度图帧队列大小，影响背景深度图密度
#define _blockSizeH 36 //栅格大小[建议取值能被图像大小整除]
#define _blockSizeW 36 //栅格大小
#define MTBoxRatio 0.1 //分割框扩大比例