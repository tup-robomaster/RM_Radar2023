#include "../include/tensorRT_v5.h"

static const int OUTPUT_SIZE = MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;

static int get_width(int x, float gw, int divisor = 8)
{
    return int(ceil((x * gw) / divisor)) * divisor;
}

static int get_depth(int x, float gd)
{
    if (x == 1)
        return 1;
    int r = round(x * gd);
    if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0)
    {
        --r;
    }
    return std::max<int>(r, 1);
}

inline const char *severity_string(nvinfer1::ILogger::Severity t)
{
    switch (t)
    {
    case nvinfer1::ILogger::Severity::kINTERNAL_ERROR:
        return "internal_error";
    case nvinfer1::ILogger::Severity::kERROR:
        return "error";
    case nvinfer1::ILogger::Severity::kWARNING:
        return "warning";
    case nvinfer1::ILogger::Severity::kINFO:
        return "info";
    case nvinfer1::ILogger::Severity::kVERBOSE:
        return "verbose";
    default:
        return "unknown";
    }
}

MyTensorRT_v5::MyTensorRT_v5()
{
}

MyTensorRT_v5::~MyTensorRT_v5()
{
}

ICudaEngine *MyTensorRT_v5::build_engine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name)
{
    INetworkDefinition *network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, this->input_H, this->input_W});
    assert(data);
    std::map<std::string, Weights> weightMap = loadWeights(wts_name);
    /* ------ yolov5 backbone------ */
    auto conv0 = convBlock(network, weightMap, *data, get_width(64, gw), 6, 2, 1, "model.0");
    assert(conv0);
    auto conv1 = convBlock(network, weightMap, *conv0->getOutput(0), get_width(128, gw), 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), get_width(128, gw), get_width(128, gw), get_depth(3, gd), true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), get_width(256, gw), 3, 2, 1, "model.3");
    auto bottleneck_csp4 = C3(network, weightMap, *conv3->getOutput(0), get_width(256, gw), get_width(256, gw), get_depth(6, gd), true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_csp4->getOutput(0), get_width(512, gw), 3, 2, 1, "model.5");
    auto bottleneck_csp6 = C3(network, weightMap, *conv5->getOutput(0), get_width(512, gw), get_width(512, gw), get_depth(9, gd), true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_csp6->getOutput(0), get_width(1024, gw), 3, 2, 1, "model.7");
    auto bottleneck_csp8 = C3(network, weightMap, *conv7->getOutput(0), get_width(1024, gw), get_width(1024, gw), get_depth(3, gd), true, 1, 0.5, "model.8");
    auto spp9 = SPPF(network, weightMap, *bottleneck_csp8->getOutput(0), get_width(1024, gw), get_width(1024, gw), 5, "model.9");
    /* ------ yolov5 head ------ */
    auto conv10 = convBlock(network, weightMap, *spp9->getOutput(0), get_width(512, gw), 1, 1, 1, "model.10");

    auto upsample11 = network->addResize(*conv10->getOutput(0));
    assert(upsample11);
    upsample11->setResizeMode(ResizeMode::kNEAREST);
    upsample11->setOutputDimensions(bottleneck_csp6->getOutput(0)->getDimensions());

    ITensor *inputTensors12[] = {upsample11->getOutput(0), bottleneck_csp6->getOutput(0)};
    auto cat12 = network->addConcatenation(inputTensors12, 2);
    auto bottleneck_csp13 = C3(network, weightMap, *cat12->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.13");
    auto conv14 = convBlock(network, weightMap, *bottleneck_csp13->getOutput(0), get_width(256, gw), 1, 1, 1, "model.14");

    auto upsample15 = network->addResize(*conv14->getOutput(0));
    assert(upsample15);
    upsample15->setResizeMode(ResizeMode::kNEAREST);
    upsample15->setOutputDimensions(bottleneck_csp4->getOutput(0)->getDimensions());

    ITensor *inputTensors16[] = {upsample15->getOutput(0), bottleneck_csp4->getOutput(0)};
    auto cat16 = network->addConcatenation(inputTensors16, 2);

    auto bottleneck_csp17 = C3(network, weightMap, *cat16->getOutput(0), get_width(512, gw), get_width(256, gw), get_depth(3, gd), false, 1, 0.5, "model.17");

    /* ------ detect ------ */
    IConvolutionLayer *det0 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (this->cls_num + 5), DimsHW{1, 1}, weightMap["model.24.m.0.weight"], weightMap["model.24.m.0.bias"]);
    auto conv18 = convBlock(network, weightMap, *bottleneck_csp17->getOutput(0), get_width(256, gw), 3, 2, 1, "model.18");
    ITensor *inputTensors19[] = {conv18->getOutput(0), conv14->getOutput(0)};
    auto cat19 = network->addConcatenation(inputTensors19, 2);
    auto bottleneck_csp20 = C3(network, weightMap, *cat19->getOutput(0), get_width(512, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.20");
    IConvolutionLayer *det1 = network->addConvolutionNd(*bottleneck_csp20->getOutput(0), 3 * (this->cls_num + 5), DimsHW{1, 1}, weightMap["model.24.m.1.weight"], weightMap["model.24.m.1.bias"]);
    auto conv21 = convBlock(network, weightMap, *bottleneck_csp20->getOutput(0), get_width(512, gw), 3, 2, 1, "model.21");
    ITensor *inputTensors22[] = {conv21->getOutput(0), conv10->getOutput(0)};
    auto cat22 = network->addConcatenation(inputTensors22, 2);
    auto bottleneck_csp23 = C3(network, weightMap, *cat22->getOutput(0), get_width(1024, gw), get_width(1024, gw), get_depth(3, gd), false, 1, 0.5, "model.23");
    IConvolutionLayer *det2 = network->addConvolutionNd(*bottleneck_csp23->getOutput(0), 3 * (this->cls_num + 5), DimsHW{1, 1}, weightMap["model.24.m.2.weight"], weightMap["model.24.m.2.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, "model.24", std::vector<IConvolutionLayer *>{det0, det1, det2}, this->cls_num, this->input_H, this->input_W);
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));
    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    size_t memfree, total;
    cuMemGetInfo(&memfree, &total);
    config->setMaxWorkspaceSize(memfree);
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2 *calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif
    this->logger->info("Building engine, please wait for a while...");
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    this->logger->info("Build engine successfully");

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto &mem : weightMap)
    {
        free((void *)(mem.second.values));
    }

    return engine;
}

ICudaEngine *MyTensorRT_v5::build_engine_p6(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name)
{
    INetworkDefinition *network = builder->createNetworkV2(0U);
    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, this->input_H, this->input_W});
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights(wts_name);

    /* ------ yolov5 backbone------ */
    auto conv0 = convBlock(network, weightMap, *data, get_width(64, gw), 6, 2, 1, "model.0");
    auto conv1 = convBlock(network, weightMap, *conv0->getOutput(0), get_width(128, gw), 3, 2, 1, "model.1");
    auto c3_2 = C3(network, weightMap, *conv1->getOutput(0), get_width(128, gw), get_width(128, gw), get_depth(3, gd), true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *c3_2->getOutput(0), get_width(256, gw), 3, 2, 1, "model.3");
    auto c3_4 = C3(network, weightMap, *conv3->getOutput(0), get_width(256, gw), get_width(256, gw), get_depth(6, gd), true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *c3_4->getOutput(0), get_width(512, gw), 3, 2, 1, "model.5");
    auto c3_6 = C3(network, weightMap, *conv5->getOutput(0), get_width(512, gw), get_width(512, gw), get_depth(9, gd), true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *c3_6->getOutput(0), get_width(768, gw), 3, 2, 1, "model.7");
    auto c3_8 = C3(network, weightMap, *conv7->getOutput(0), get_width(768, gw), get_width(768, gw), get_depth(3, gd), true, 1, 0.5, "model.8");
    auto conv9 = convBlock(network, weightMap, *c3_8->getOutput(0), get_width(1024, gw), 3, 2, 1, "model.9");
    auto c3_10 = C3(network, weightMap, *conv9->getOutput(0), get_width(1024, gw), get_width(1024, gw), get_depth(3, gd), true, 1, 0.5, "model.10");
    auto sppf11 = SPPF(network, weightMap, *c3_10->getOutput(0), get_width(1024, gw), get_width(1024, gw), 5, "model.11");

    /* ------ yolov5 head ------ */
    auto conv12 = convBlock(network, weightMap, *sppf11->getOutput(0), get_width(768, gw), 1, 1, 1, "model.12");
    auto upsample13 = network->addResize(*conv12->getOutput(0));
    assert(upsample13);
    upsample13->setResizeMode(ResizeMode::kNEAREST);
    upsample13->setOutputDimensions(c3_8->getOutput(0)->getDimensions());
    ITensor *inputTensors14[] = {upsample13->getOutput(0), c3_8->getOutput(0)};
    auto cat14 = network->addConcatenation(inputTensors14, 2);
    auto c3_15 = C3(network, weightMap, *cat14->getOutput(0), get_width(1536, gw), get_width(768, gw), get_depth(3, gd), false, 1, 0.5, "model.15");

    auto conv16 = convBlock(network, weightMap, *c3_15->getOutput(0), get_width(512, gw), 1, 1, 1, "model.16");
    auto upsample17 = network->addResize(*conv16->getOutput(0));
    assert(upsample17);
    upsample17->setResizeMode(ResizeMode::kNEAREST);
    upsample17->setOutputDimensions(c3_6->getOutput(0)->getDimensions());
    ITensor *inputTensors18[] = {upsample17->getOutput(0), c3_6->getOutput(0)};
    auto cat18 = network->addConcatenation(inputTensors18, 2);
    auto c3_19 = C3(network, weightMap, *cat18->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.19");

    auto conv20 = convBlock(network, weightMap, *c3_19->getOutput(0), get_width(256, gw), 1, 1, 1, "model.20");
    auto upsample21 = network->addResize(*conv20->getOutput(0));
    assert(upsample21);
    upsample21->setResizeMode(ResizeMode::kNEAREST);
    upsample21->setOutputDimensions(c3_4->getOutput(0)->getDimensions());
    ITensor *inputTensors21[] = {upsample21->getOutput(0), c3_4->getOutput(0)};
    auto cat22 = network->addConcatenation(inputTensors21, 2);
    auto c3_23 = C3(network, weightMap, *cat22->getOutput(0), get_width(512, gw), get_width(256, gw), get_depth(3, gd), false, 1, 0.5, "model.23");

    auto conv24 = convBlock(network, weightMap, *c3_23->getOutput(0), get_width(256, gw), 3, 2, 1, "model.24");
    ITensor *inputTensors25[] = {conv24->getOutput(0), conv20->getOutput(0)};
    auto cat25 = network->addConcatenation(inputTensors25, 2);
    auto c3_26 = C3(network, weightMap, *cat25->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd), false, 1, 0.5, "model.26");

    auto conv27 = convBlock(network, weightMap, *c3_26->getOutput(0), get_width(512, gw), 3, 2, 1, "model.27");
    ITensor *inputTensors28[] = {conv27->getOutput(0), conv16->getOutput(0)};
    auto cat28 = network->addConcatenation(inputTensors28, 2);
    auto c3_29 = C3(network, weightMap, *cat28->getOutput(0), get_width(1536, gw), get_width(768, gw), get_depth(3, gd), false, 1, 0.5, "model.29");

    auto conv30 = convBlock(network, weightMap, *c3_29->getOutput(0), get_width(768, gw), 3, 2, 1, "model.30");
    ITensor *inputTensors31[] = {conv30->getOutput(0), conv12->getOutput(0)};
    auto cat31 = network->addConcatenation(inputTensors31, 2);
    auto c3_32 = C3(network, weightMap, *cat31->getOutput(0), get_width(2048, gw), get_width(1024, gw), get_depth(3, gd), false, 1, 0.5, "model.32");

    /* ------ detect ------ */
    IConvolutionLayer *det0 = network->addConvolutionNd(*c3_23->getOutput(0), 3 * (this->cls_num + 5), DimsHW{1, 1}, weightMap["model.33.m.0.weight"], weightMap["model.33.m.0.bias"]);
    IConvolutionLayer *det1 = network->addConvolutionNd(*c3_26->getOutput(0), 3 * (this->cls_num + 5), DimsHW{1, 1}, weightMap["model.33.m.1.weight"], weightMap["model.33.m.1.bias"]);
    IConvolutionLayer *det2 = network->addConvolutionNd(*c3_29->getOutput(0), 3 * (this->cls_num + 5), DimsHW{1, 1}, weightMap["model.33.m.2.weight"], weightMap["model.33.m.2.bias"]);
    IConvolutionLayer *det3 = network->addConvolutionNd(*c3_32->getOutput(0), 3 * (this->cls_num + 5), DimsHW{1, 1}, weightMap["model.33.m.3.weight"], weightMap["model.33.m.3.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, "model.33", std::vector<IConvolutionLayer *>{det0, det1, det2, det3}, this->cls_num, this->input_H, this->input_W);
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    size_t memfree, total;
    cuMemGetInfo(&memfree, &total);
    config->setMaxWorkspaceSize(memfree);
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2 *calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    this->logger->info("Building engine, please wait for a while...");
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    this->logger->info("Build engine successfully");

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto &mem : weightMap)
    {
        free((void *)(mem.second.values));
    }

    return engine;
}

void MyTensorRT_v5::print_dims(const nvinfer1::Dims &dim)
{
    for (int nIdxShape = 0; nIdxShape < dim.nbDims; ++nIdxShape)
    {

        printf("dim %d=%d\n", nIdxShape, dim.d[nIdxShape]);
    }
}

void MyTensorRT_v5::APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, bool &is_p6, float &gd, float &gw, std::string &wts_name)
{
    static sample::Logger gLogger;
    // Create builder
    IBuilder *builder = createInferBuilder(gLogger);
    IBuilderConfig *config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6)
    {
        this->logger->info("Build P6 Engine");
        engine = this->build_engine_p6(maxBatchSize, builder, config, nvinfer1::DataType::kFLOAT, gd, gw, wts_name);
    }
    else
    {
        this->logger->info("Build Default Engine");
        engine = this->build_engine(maxBatchSize, builder, config, nvinfer1::DataType::kFLOAT, gd, gw, wts_name);
    }
    assert(engine != nullptr);

    // Serialize the engine
    (*modelStream) = engine->serialize();

    // Close everything down
    engine->destroy();
    config->destroy();
    builder->destroy();
}

bool MyTensorRT_v5::build_model(string wts_name, string engine_name, bool is_p6, float gd, float gw)
{
    char engine_name_c[engine_name.length()];
    strcpy(engine_name_c, engine_name.data());

    if (access(engine_name_c, F_OK) == 0)
    {
        this->logger->info("TRT Moudle exists , Skip creation");
        return true;
    }
    if (!wts_name.empty())
    {
        char wts_name_c[wts_name.length()];
        strcpy(wts_name_c, wts_name.data());
        if (access(wts_name_c, F_OK) != 0)
        {
            this->logger->error("WTS doesn't exists , Stop creation");
            return false;
        }
        IHostMemory *modelStream{nullptr};
        APIToModel(this->max_batchsize, &modelStream, is_p6, gd, gw, wts_name);
        if (modelStream == nullptr)
            this->logger->error("Failed to build engine");
        assert(modelStream != nullptr);
        std::ofstream p(engine_name, std::ios::binary);
        if (!p)
        {
            this->logger->error("Could not open plan output file");
            return false;
        }
        p.write(reinterpret_cast<const char *>(modelStream->data()), modelStream->size());
        modelStream->destroy();
        return true;
    }
    return false;
}

bool MyTensorRT_v5::build_model(char *onnxMoudlePath, char *trtMoudleSavePath, bool fp16, int width, int height, int kopt, int kmax)
{
    if (access(trtMoudleSavePath, F_OK) == 0)
    {
        this->logger->info("TRT Moudle exists , Skip creation");
        return true;
    }
    if (access(onnxMoudlePath, F_OK) == 0)
    {
        this->buildAndSaveEngineFromOnnx(onnxMoudlePath, trtMoudleSavePath, fp16, width, height, kopt, kmax);
    }
    return false;
}

bool MyTensorRT_v5::initMyTensorRT_v5(char *onnxMoudlePath, char *tensorrtEngienPath, char *yolov5wts, bool is_p6, float gd, float gw, int max_batchsize, int input_H, int input_W, int cls_num, int fp16, int kopt, int kmax)
{
    this->max_batchsize = max_batchsize;
    this->input_H = input_H;
    this->input_W = input_W;
    this->cls_num = cls_num;
#ifdef USETRTAPI
    if (this->build_model(yolov5wts, tensorrtEngienPath, is_p6, gd, gw))
#else
    if (this->build_model(onnxMoudlePath, tensorrtEngienPath, fp16, input_W, input_H, kopt, kmax))
#endif
    {
        static sample::Logger gLogger;
        std::ifstream file(tensorrtEngienPath, std::ios::binary);
        if (!file.good())
        {
            this->logger->error("Read {} failed", tensorrtEngienPath);
            return false;
        }
        char *trtModelStream = nullptr;
        size_t size = 0;
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
        this->runtime = createInferRuntime(gLogger);
        assert(runtime != nullptr);
        this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        this->context = this->engine->createExecutionContext();
        assert(context != nullptr);
        delete[] trtModelStream;
        assert(this->engine->getNbBindings() == 2);
        this->inputIndex = this->engine->getBindingIndex(INPUT_BLOB_NAME);
        this->outputIndex = this->engine->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(this->inputIndex == 0);
        assert(this->outputIndex == 1);
        CUDA_CHECK(cudaMalloc((void**)&this->buffers[this->inputIndex], this->max_batchsize * 3 * this->input_H * this->input_W * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)&this->buffers[this->outputIndex], this->max_batchsize * OUTPUT_SIZE * sizeof(float)));
        CUDA_CHECK(cudaMallocHost((void **)&this->img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
        CUDA_CHECK(cudaMalloc((void **)&this->img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
        this->output = (float *)malloc(this->max_batchsize * OUTPUT_SIZE * sizeof(float));
    }
    else
    {
        this->logger->error("Failed to build the infer moudle !Please CHECK and Restart the Program.");
        return false;
    }
    return true;
}

void MyTensorRT_v5::unInitMyTensorRT_v5()
{
    CUDA_CHECK(cudaFree(img_device));
    CUDA_CHECK(cudaFreeHost(img_host));
    CUDA_CHECK(cudaFree(this->buffers[this->inputIndex]));
    CUDA_CHECK(cudaFree(this->buffers[this->outputIndex]));
    this->runtime->destroy();
}

bool MyTensorRT_v5::saveEngine(const ICudaEngine &engine, const std::string &fileName)
{
    std::ofstream engineFile(fileName, std::ios::binary);
    if (!engineFile)
    {
        this->logger->error("Cannot open engine file: {}", fileName);
        return false;
    }

    IHostMemory *serializedEngine = engine.serialize();
    if (serializedEngine == nullptr)
    {
        this->logger->error("Engine serialization failed");
        return false;
    }

    engineFile.write(static_cast<char *>(serializedEngine->data()), serializedEngine->size());
    return !engineFile.fail();
}

bool MyTensorRT_v5::buildAndSaveEngineFromOnnx(char *onnxMoudlePath, char *trtMoudleSavePath, bool fp16, int width, int height, int kopt, int kmax)
{
    static sample::Logger logger;
    IBuilder *pBuilder = createInferBuilder(logger);
    INetworkDefinition *pNetwork = pBuilder->createNetworkV2(1U << static_cast<int>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));

    nvinfer1::IBuilderConfig *config = pBuilder->createBuilderConfig();
    IOptimizationProfile *profile = pBuilder->createOptimizationProfile();

    profile->setDimensions("images", OptProfileSelector::kMIN, Dims4(1, 3, width, height));
    profile->setDimensions("images", OptProfileSelector::kOPT, Dims4(kopt, 3, width, height));
    profile->setDimensions("images", OptProfileSelector::kMAX, Dims4(kmax, 3, width, height));

    config->addOptimizationProfile(profile);

    auto parser = nvonnxparser::createParser(*pNetwork, logger.getTRTLogger());

    if (!parser->parseFromFile(onnxMoudlePath, static_cast<int>(logger.getReportableSeverity())))
    {

        this->logger->error("解析onnx模型失败");
        return false;
    }

    int maxBatchSize = kmax;

    config->setMaxWorkspaceSize(2 << 30);

    pBuilder->setMaxBatchSize(maxBatchSize);
    config->setFlag(BuilderFlag::kFP16);
    ICudaEngine *engine = pBuilder->buildEngineWithConfig(*pNetwork, *config);

    saveEngine(*engine, trtMoudleSavePath);
    nvinfer1::Dims dim = engine->getBindingDimensions(0);

    print_dims(dim);
    return true;
}

vector<vector<Yolo::Detection>> MyTensorRT_v5::doInference(vector<Mat> *input, int batchSize, float confidence_threshold, float nms_threshold)
{
    vector<vector<Yolo::Detection>> batch_res(batchSize);
    cudaStream_t stream = nullptr;
    CUDA_CHECK(cudaStreamCreate(&stream));
    float *buffer_idx = (float *)buffers[this->inputIndex];
    for (size_t b = 0; b < input->size(); ++b)
    {
        Mat img = input->at(b);
        if (img.empty())
            continue;
        size_t size_image = img.cols * img.rows * 3;
        size_t size_image_dst = this->input_H * this->input_W * 3;
        memcpy(img_host, img.data, size_image);
        CUDA_CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
        preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx, this->input_W, this->input_H, stream);
        buffer_idx += size_image_dst;
    }
    bool success = this->context->enqueue(batchSize, (void**)this->buffers, stream, nullptr);
    if (!success)
    {
        this->logger->error("DoInference failed");
        CUDA_CHECK(cudaStreamDestroy(stream));
        return {};
    }
    CUDA_CHECK(cudaMemcpyAsync(this->output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    CUDA_CHECK(cudaStreamDestroy(stream));
    for (int b = 0; b < int(input->size()); ++b)
    {
        auto &res = batch_res[b];
        nms(res, &this->output[b * OUTPUT_SIZE], confidence_threshold, nms_threshold);
    }
    return batch_res;
}

bool MyTensorRT_v5::doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output, int batchSize) {
    // infer on the batch asynchronously, and DMA output back to host
    bool success = context.enqueue(batchSize, buffers, stream, nullptr);
    if (!success)
    {
        return false;
    }
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
    return true;
}