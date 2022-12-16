#ifndef MY_TENSORRT_V7_H
#define MY_TENSORRT_V7_H

#include "../../Common/include/public.h"
#include <NvInferRuntime.h>
#include "./cuda_utils.h"
#include "./common.h"
#include "./preprocess.h"

/**
 * @brief Tensorrt
 * CPU/GPU预（后）处理的Tensorrt推理
 */
class MyTensorRT_v7
{
private:
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    void *buffers[2];
    int inputIndex;
    int outputIndex;
    uint8_t *img_host = nullptr;
    uint8_t *img_device = nullptr;

    int max_batchsize = 0;
    int input_H = 0;
    int input_W = 0;
    int cls_num = 0;

    float *output;
    
private:
    bool build_model(string wts_name, string engine_name, bool is_p6, float gd, float gw);
    void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, bool &is_p6, float &gd, float &gw, std::string &wts_name);
    ICudaEngine *build_engine_yolov7e6e(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name);
    ICudaEngine *build_engine_p6(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name);

public:
    MyTensorRT_v7();
    ~MyTensorRT_v7();

    bool initMyTensorRT_v7(char *tensorrtMoudlePath, char *onnxMoudlePath, bool is_p6, float gd, float gw, int max_batchsize, int input_H, int input_W, int cls_num);
    void unInitMyTensorRT_v7();
    vector<vector<Yolo::Detection>> doInference(vector<Mat> *input, int batchSize, float confidence_threshold = 0.25f, float nms_threshold = 0.45f);
};

#endif