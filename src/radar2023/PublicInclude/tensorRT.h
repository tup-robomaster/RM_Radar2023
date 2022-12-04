#ifndef MY_TENSORRT_H
#define MY_TENSORRT_H

#include "./public.h"
#include <NvInferRuntime.h>
#include "cuda_utils.h"
#include "common.h"
#include "preprocess.h"

/**
 * @brief Tensorrt
 * CPU/GPU预（后）处理的Tensorrt推理
 */
class MyTensorRT
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
    bool is_p6 = false;
    float gd = 0.67;    //n:0.33 s:0.33 m:0.67 l:1.0 x:1.33
    float gw = 0.75;    //n:0.25 s:0.50 m:0.75 l:1.0 x:1.25

private:
    bool build_model(string wts_name, string engine_name, bool is_p6, float gd, float gw);
    void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, bool &is_p6, float &gd, float &gw, std::string &wts_name);
    ICudaEngine *build_engine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name);
    ICudaEngine *build_engine_p6(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name);

public:
    MyTensorRT();
    ~MyTensorRT();

    void initMyTensorRT(char *tensorrtMoudlePath, char *onnxMoudlePath);
    void unInitMyTensorRT();
    vector<vector<Yolo::Detection>> doInference(vector<Mat> *input, int batchSize, float confidence_threshold = 0.25f, float nms_threshold = 0.45f);
};

#endif