#ifndef MY_TENSORRT_V5_H
#define MY_TENSORRT_V5_H

#include "../../Common/include/public.h"
#include <NvInferRuntime.h>
#include "./cuda_utils.h"
#include "./common.h"
#include "./preprocess.h"
#include <NvOnnxParser.h>
#include "logging.h"

/**
 * @brief Tensorrt
 * CPU/GPU预（后）处理的Tensorrt推理
 */
class MyTensorRT_v5
{
private:
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    float *buffers[2];
    int inputIndex;
    int outputIndex;
    uint8_t *img_host = nullptr;
    uint8_t *img_device = nullptr;

    int max_batchsize = 0;
    int input_H = 0;
    int input_W = 0;
    int cls_num = 0;

    float *output;

    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

private:
    bool build_model(string wts_name, string engine_name, bool is_p6, float gd, float gw);
    bool build_model(char *onnxMoudlePath, char *trtMoudleSavePath, bool fp16, int width, int height, int kopt, int kmax);
    void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, bool &is_p6, float &gd, float &gw, std::string &wts_name);
    ICudaEngine *build_engine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name);
    ICudaEngine *build_engine_p6(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name);

    void print_dims(const nvinfer1::Dims &dim);
    bool saveEngine(const ICudaEngine &engine, const std::string &fileName);

    bool buildAndSaveEngineFromOnnx(char *onnxMoudlePath, char *trtMoudleSavePath, bool fp16, int width, int height, int kopt, int kmax);

public:
    MyTensorRT_v5();
    ~MyTensorRT_v5();

    bool initMyTensorRT_v5(char* onnxMoudlePath,char *tensorrtMoudlePath, char *yolov5wts, bool is_p6, float gd, float gw, int max_batchsize, int input_H, int input_W, int cls_num, int fp16, int kopt, int kmax);
    void unInitMyTensorRT_v5();
    bool doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output, int batchSize);
    vector<vector<Yolo::Detection>> doInference(vector<Mat> *input, int batchSize, float confidence_threshold = 0.45f, float nms_threshold = 0.45f);
};

#endif