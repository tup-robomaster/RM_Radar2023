#include "../include/ArmorDetector.h"

ArmorDetector::ArmorDetector(string engine_path, string onnx_path)
{
    assert(!engine_path.empty() && !onnx_path.empty());
    this->TensorRTEnginePath = engine_path;
    this->OnnxPath = onnx_path;
}

ArmorDetector::~ArmorDetector()
{
}

void ArmorDetector::accessModelTest()
{
    if (access(TensorRTEnginePath.c_str(), F_OK) != 0)
    {
#ifndef UseOneLayerInfer
        auto engine = this->armorTensorRT.createEngine(OnnxPath, 64, 640, 640, 0);
#else
        auto engine = this->armorTensorRT.createEngine(OnnxPath, 64, 1280, 1280, 0);
#endif
        this->armorTensorRT.saveEngineFile(engine, TensorRTEnginePath);
        delete engine;
    }
}

bool ArmorDetector::initModel()
{
    this->logger->info("ArmorDetector init Moudel");
    bool check = this->armorTensorRT.initModule(TensorRTEnginePath, 16, 12);
    this->logger->info("ArmorDetector Moudel inited");
    return check;
}

vector<bboxAndRect> ArmorDetector::infer(Mat &image, vector<DetectBox> &targets)
{
    vector<vector<TRTInferV1::DetectionObj>> results_pre;
    if (targets.size() == 0)
    {
#ifdef ExperimentalOutput
        this->detectedArmorNumThisFrame = 0;
        this->averageConfThisFrame = 0.0;
#endif
        return {};
    }
    vector<Mat> preProcessedImage = this->preProcess(image, targets);
    results_pre = this->armorTensorRT.doInference(preProcessedImage, 0.1, 0.25, 0.45);
    this->reBuildBoxs(results_pre, targets, preProcessedImage);
#ifdef ExperimentalOutput
    this->detectedArmorNumThisFrame = this->results.size();
    this->averageConfThisFrame = sumConfAverage(this->results);
#endif
    return this->results;
}

vector<Mat> ArmorDetector::preProcess(Mat &image, vector<DetectBox> &movingTargets)
{
    vector<Mat> output;
    for (vector<DetectBox>::iterator it = movingTargets.begin(); it != movingTargets.end();)
    {
        if (it->x2 - it->x1 == 0 || it->y2 - it->y1 == 0)
        {
            it = movingTargets.erase(it);
        }
        else
        {
            Rect target = Rect(it->x1, it->y1, it->x2 - it->x1, it->y2 - it->y1);
            makeRectSafe(target, image);
            output.emplace_back(image(target).clone());
            ++it;
        }
    }

    return output;
}

void ArmorDetector::reBuildBoxs(vector<vector<TRTInferV1::DetectionObj>> &armors, vector<DetectBox> &boxs, vector<Mat> &img)
{
    vector<bboxAndRect>().swap(this->results);
    if (armors.size() != boxs.size())
        return;
    for (size_t i = 0; i < boxs.size(); ++i)
    {
        for (auto &it : armors[i])
        {
            this->results.emplace_back(bboxAndRect{ArmorBoundingBox{true,
                                                                    (float)it.x1 + boxs[i].x1,
                                                                    (float)it.y1 + boxs[i].y1,
                                                                    abs((float)(it.x2 - it.x1)),
                                                                    abs((float)(it.y2 - it.y1)),
                                                                    (float)it.classId, it.confidence},
                                                   boxs[i]});
            this->logger->info("Arrmor: [x0] " + to_string(this->results.back().armor.x0) +
                               " [y0] " + to_string(this->results.back().armor.y0) +
                               " [w] " + to_string(this->results.back().armor.w) +
                               " [h] " + to_string(this->results.back().armor.h) +
                               " [cls] " + to_string(it.classId) +
                               " [conf] " + to_string(it.confidence));
        }
    }
}

void ArmorDetector::unInit()
{
    this->armorTensorRT.unInitModule();
}

#ifdef UseOneLayerInfer

vector<bboxAndRect> ArmorDetector::infer(Mat &image)
{
    vector<bboxAndRect>().swap(this->results);
    vector<vector<TRTInferV1::DetectionObj>> results_pre;
    vector<Mat> images = {image};
    results_pre = this->armorTensorRT.doInference(images, 0.1, 0.25, 0.45);
    if (results_pre.size() == 0)
    {
#ifdef ExperimentalOutput
        this->detectedArmorNumThisFrame = 0;
        this->averageConfThisFrame = 0.0;
#endif
        return {};
    }
    for (auto &it : results_pre[0])
    {
        this->results.emplace_back(bboxAndRect{ArmorBoundingBox{true,
                                                                (float)it.x1,
                                                                (float)it.y1,
                                                                abs((float)(it.x2 - it.x1)),
                                                                abs((float)(it.y2 - it.y1)),
                                                                (float)it.classId, it.confidence},
                                               DetectBox()});
        this->logger->info("Arrmor: [x0] " + to_string(this->results.back().armor.x0) +
                           " [y0] " + to_string(this->results.back().armor.y0) +
                           " [w] " + to_string(this->results.back().armor.w) +
                           " [h] " + to_string(this->results.back().armor.h) +
                           " [cls] " + to_string(it.classId) +
                           " [conf] " + to_string(it.confidence));
    }
#ifdef ExperimentalOutput
    this->detectedArmorNumThisFrame = this->results.size();
    this->averageConfThisFrame = sumConfAverage(this->results);
#endif
    return this->results;
}

#endif