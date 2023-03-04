// #define SpeedTest                                           //各环节速度测试（ms/FPS）
// #define ThreadSpeedTest                                     //线程速度测试（应关闭环节速度测试）
#define Test                                                //测试标志
// #define UsingVideo                                          //是否使用视频

#define USETRTAPI                                              //是否使用API构建模型

#define SerialPortNAME (char *)"/dev/ttyUSB0"
#define lidarTopicName (char *)"/livox/lidar"

#define CameraConfigPath (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Camera/params/Config_0.Config" // 相机配置文件名称
#define TestVideoPath (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/demo_resource/video0.mp4"                    // DEMO视频路径

#define MaxPointsNum 10000 // 最大点云数量
#define ImageH 2064        // 图像高度
#define ImageW 3088        // 图像宽度
#define maxQueueSize 30    // 点云最大帧队列长度
#define MDHistorySize 100  // 运动识别背景帧大小
#define _blockSizeH 32     // 栅格高
#define _blockSizeW 32     // 栅格宽
#define LidarQueueSize 1   // 雷达消息队列
#define OffsetRatio 3.5    // 标准差倍率(数值越大，感知敏感度越低，可减少噪点影响)
#define MTBoxRatio 1.1     // 运动目标框容错比例
#define K_size 3           // 核大小

#define CAMERA_PARAM_PATH (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Camera/params/camera0.yaml" // 相机参数文件
#define VideoRecoderRath (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/Recorder/"                              // 录制保存文件

#define USE_FP16 true
// #define USE_INT8 true
#define Is_p6 false
#define G_D 0.33 // n:0.33 s:0.33 m:0.67 l:1.0 x:1.33
#define G_W 0.50 // n:0.25 s:0.50 m:0.75 l:1.0 x:1.25
#define Is_p6_c false
#define G_D_c 0.33 // n:0.33 s:0.33 m:0.67 l:1.0 x:1.33
#define G_W_c 0.50 // n:0.25 s:0.50 m:0.75 l:1.0 x:1.25
#define OnnxMoudlePath (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/Moudles/armor.onnx"
#define OnnxMoudlePath_c (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/Moudles/armor.onnx"
#define TensorRTEnginePath (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/Moudles/best.engine"     // Engine
#define Yolov5wtsPath (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/Moudles/best.wts"             // wts
#define TensorRTEnginePath_c (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/Moudles/best.engine" // Engine
#define Yolov5wtsPath_c (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/RadarClass/Detectors/Moudles/best.wts"         // wts
#define TensorRTMaxBatchSize 30                                                                                                // 转换TRT最大BatchSize
#define TensorRTMaxBatchSize_c 30                                                                                               // 转换TRT最大BatchSize
#define INPUT_BLOB_NAME (char *)"data"                                                                                       // 输入名
#define OUTPUT_BLOB_NAME (char *)"prob"                                                                                     // 输出名
#define TRT_INPUT_H 256                                                                                                        // 输入尺寸_高
#define TRT_INPUT_W 256                                                                                                        // 输入尺寸_宽
#define TRT_INPUT_H_c 256                                                                                                      // 输入尺寸_高
#define TRT_INPUT_W_c 256                                                                                                      // 输入尺寸_宽
#define TRT_CLS_NUM 36                                                                                                         // 分类数量
#define TRT_CLS_NUM_c 36                                                                                                        // 分类数量
#define TRT_kOPT 10
#define TRT_kMAX TensorRTMaxBatchSize
#define TRT_kOPT_c 10
#define TRT_kMAX_c TensorRTMaxBatchSize_c
#define MAX_OUTPUT_BBOX_COUNT 1000
#define MAX_IMAGE_INPUT_SIZE_THRESH 3000 * 3000
#define TRT_OUTPUT_SIZE MAX_OUTPUT_BBOX_COUNT * 24UL / sizeof(float) + 1 // TRT_输出大小

#define MAXBO 3
#define ENEMY 1

#define Z_A true        // Z轴突变调整
#define L_P true        // 位置预测
#define Z_THRE 0.2      // Z轴突变阈值
#define Pre_Time 10     // 预测次数
#define Pre_radio 0.2f  // 预测速度比例
#define Real_Size_W 15. // 真实宽度
#define Real_Size_H 28. // 真实高度（长）
#define IoU_THRE 0.8f

#define LOGPATH (char *)"/home/nine-fish/RM_Radar2023/src/radar2023/logs/" // log日志存储文件夹