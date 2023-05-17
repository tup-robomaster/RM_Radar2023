#include "../include/ArmorDetector.h"

ArmorDetector::ArmorDetector()
{
}

ArmorDetector::~ArmorDetector()
{
}

bool ArmorDetector::initModel()
{
    this->logger->info("ArmorDetector init Moudel");
    if (access(TensorRTEnginePath, F_OK) != 0)
    {
        auto engine = this->armorTensorRT.createEngine(OnnxMoudlePath, 64, 640, 640);
        this->armorTensorRT.saveEngineFile(engine, TensorRTEnginePath);
    }
    bool check = this->armorTensorRT.initMoudle(TensorRTEnginePath, 32, 36);
    this->logger->info("ArmorDetector Moudel inited");
    return check;
}

vector<bboxAndRect> ArmorDetector::infer(Mat &image, vector<Rect> &targets)
{
    vector<vector<TRTInferV1::DetectionObj>> results_pre;
    if (targets.size() == 0)
        return {};
    vector<Mat> preProcessedImage = this->preProcess(image, targets);
    results_pre = this->armorTensorRT.doInference(preProcessedImage, 0.6, 0.6, 0.3);
    this->reBuildBoxs(results_pre, targets, preProcessedImage);
    return this->results;
}

vector<Mat> ArmorDetector::preProcess(Mat &image, vector<Rect> &movingTargets)
{
    vector<Mat> output;
    for (vector<Rect>::iterator it = movingTargets.begin(); it != movingTargets.end();)
    {
        if (it->width == 0 || it->height == 0)
        {
            it = movingTargets.erase(it);
        }
        else
        {
            makeRectSafe(*it, image);
            output.emplace_back(image(*it));
            imshow("Test",image(*it));
            ++it;
        }
    }

    return output;
}

void ArmorDetector::reBuildBoxs(vector<vector<TRTInferV1::DetectionObj>> &armors, vector<Rect> &boxs, vector<Mat> &img)
{
    vector<bboxAndRect>().swap(this->results);
    if (armors.size() != boxs.size())
        return;
    for (size_t i = 0; i < boxs.size(); ++i)
    {
        for (auto &it : armors[i])
        {
#ifdef Test
            cout << "Arrmor:" << it.x1 << "|" << it.y1 << "|" << it.x2 << "|" << it.y2 << "cls:" << it.classId << "|conf:" << it.confidence << endl;
#endif
            this->results.emplace_back(bboxAndRect{ArmorBoundingBox{true, (float)it.x1 + boxs[i].x, (float)it.y1 + boxs[i].y, (float)(it.x2 - it.x1), (float)(it.y2 - it.y1), (float)it.classId, it.confidence}, boxs[i]});
        }
    }
}