// #define SpeedTest                                           //各环节速度测试（ms/FPS）
// #define ThreadSpeedTest                                     //线程速度测试（应关闭环节速度测试）
// #define Test                                                //测试标志
// #define UsingVideo                                          //是否使用视频

#define SerialPortNAME      (char*)"/dev/ttyUSB0" 

#define CameraConfigPath    (char*)"Camera/Config_0.Config"        //相机配置文件名称
#define TestVideoPath       (char*)"demo_resource/video0.mp4"      //DEMO视频路径

#define MaxPointsNum        10000                           //最大点云数量
#define ImageH              2064                            //图像高度
#define ImageW              3088                            //图像宽度
#define maxQueueSize        30                              //点云最大帧队列长度
#define MDHistorySize       100                             //运动识别背景帧大小
#define _blockSizeH         32                              //栅格高
#define _blockSizeW         32                              //栅格宽
#define LidarQueueSize      1                               //雷达消息队列
#define OffsetRatio         3.5                             //标准差倍率(数值越大，感知敏感度越低，可减少噪点影响)
#define MTBoxRatio          1.1                             //运动目标框容错比例
#define K_size              3                               //核大小

#define CAMERA_PARAM_PATH   (char*)"/home/nine-fish/ws_Radar2023/src/radar2023/Camera/camera0.yaml"     //相机参数文件

#define TensorRTEnginePath  (char*)"/home/nine-fish/ws_Radar2023/src/radar2023/Moudles/yolov5.engine"   //Engine
#define Yolov5wtsPath       (char*)"/home/nine-fish/ws_Radar2023/src/radar2023/Moudles/yolov5.wts"      //wts
#define TensorRTMaxBatchSize 10                             //转换TRT最大BatchSize
#define INPUT_BLOB_NAME     (char*)"images"                 //输入名
#define OUTPUT_BLOB_NAME    (char*)"output0"                //输出名
#define TRT_INPUT_H         256                             //输入尺寸_高
#define TRT_INPUT_W         256                             //输入尺寸_宽
#define TRT_CLS_NUM         36                              //分类数量
#define TRT_ANCHOR_OUTPUT_NUM 25200                         //先验框数量
#define TensorRTBatchSize   1                               //TRT单次推理数量
#define MAX_OUTPUT_BBOX_COUNT   1000
#define MAX_IMAGE_INPUT_SIZE_THRESH 3088 * 2064
#define TRT_OUTPUT_SIZE     MAX_OUTPUT_BBOX_COUNT*24UL/sizeof(float) + 1 //TRT_输出大小

#define MAXBO               3
#define ENEMY               1

#define Z_A                 true                            //Z轴突变调整
#define L_P                 true                            //位置预测
#define Z_THRE              0.2                             //Z轴突变阈值
#define Pre_Time            10                              //预测次数
#define Pre_radio           0.2                             //预测速度比例            
#define Real_Size_W         15.                             //真实宽度
#define Real_Size_H         28.                             //真实高度（长）