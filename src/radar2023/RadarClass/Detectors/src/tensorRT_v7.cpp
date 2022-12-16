#include "../include/tensorRT_v7.h"

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

class TRTLogger : public nvinfer1::ILogger
{
public:
    virtual void log(Severity severity, nvinfer1::AsciiChar const *msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            if (severity == Severity::kWARNING)
                printf("\033[33m%s: %s\033[0m\n", severity_string(severity), msg);
            else if (severity == Severity::kERROR)
                printf("\031[33m%s: %s\033[0m\n", severity_string(severity), msg);
            else
                printf("%s: %s\n", severity_string(severity), msg);
        }
    }
};

MyTensorRT_v7::MyTensorRT_v7()
{
}

MyTensorRT_v7::~MyTensorRT_v7()
{
}

ICudaEngine *MyTensorRT_v7::build_engine_yolov7e6e(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name)
{
std::map<std::string, Weights> weightMap = loadWeights(wts_name);

    INetworkDefinition* network = builder->createNetworkV2(0U);
    ITensor* data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{ 3, this->input_H, this->input_W });
    assert(data);

    auto* conv0 = ReOrg(network, weightMap, *data, 3);


    IElementWiseLayer* conv1 = convBnSilu(network, weightMap, *conv0->getOutput(0), 80, 3, 1, 1, "model.1");
    auto conv2 = DownC(network, weightMap, *conv1->getOutput(0), 80, 160, "model.2");

    IElementWiseLayer* conv3 = convBnSilu(network, weightMap, *conv2->getOutput(0), 64, 1, 1, 0, "model.3");
    IElementWiseLayer* conv4 = convBnSilu(network, weightMap, *conv2->getOutput(0), 64, 1, 1, 0, "model.4");

    IElementWiseLayer* conv5 = convBnSilu(network, weightMap, *conv4->getOutput(0), 64, 3, 1, 1, "model.5");
    IElementWiseLayer* conv6 = convBnSilu(network, weightMap, *conv5->getOutput(0), 64, 3, 1, 1, "model.6");
    IElementWiseLayer* conv7 = convBnSilu(network, weightMap, *conv6->getOutput(0), 64, 3, 1, 1, "model.7");
    IElementWiseLayer* conv8 = convBnSilu(network, weightMap, *conv7->getOutput(0), 64, 3, 1, 1, "model.8");
    IElementWiseLayer* conv9 = convBnSilu(network, weightMap, *conv8->getOutput(0), 64, 3, 1, 1, "model.9");
    IElementWiseLayer* conv10 = convBnSilu(network, weightMap, *conv9->getOutput(0), 64, 3, 1, 1, "model.10");

    ITensor* input_tensor_11[] = { conv10->getOutput(0), conv8->getOutput(0),conv6->getOutput(0), conv4->getOutput(0),
        conv3->getOutput(0) };
    IConcatenationLayer* concat11 = network->addConcatenation(input_tensor_11, 5);

    IElementWiseLayer* conv12 = convBnSilu(network, weightMap, *concat11->getOutput(0), 160, 1, 1, 0, "model.12");


    IElementWiseLayer* conv13 = convBnSilu(network, weightMap, *conv2->getOutput(0), 64, 1, 1, 0, "model.13");
    IElementWiseLayer* conv14 = convBnSilu(network, weightMap, *conv2->getOutput(0), 64, 1, 1, 0, "model.14");

    IElementWiseLayer* conv15 = convBnSilu(network, weightMap, *conv14->getOutput(0), 64, 3, 1, 1, "model.15");
    IElementWiseLayer* conv16 = convBnSilu(network, weightMap, *conv15->getOutput(0), 64, 3, 1, 1, "model.16");
    IElementWiseLayer* conv17 = convBnSilu(network, weightMap, *conv16->getOutput(0), 64, 3, 1, 1, "model.17");
    IElementWiseLayer* conv18 = convBnSilu(network, weightMap, *conv17->getOutput(0), 64, 3, 1, 1, "model.18");
    IElementWiseLayer* conv19 = convBnSilu(network, weightMap, *conv18->getOutput(0), 64, 3, 1, 1, "model.19");
    IElementWiseLayer* conv20 = convBnSilu(network, weightMap, *conv19->getOutput(0), 64, 3, 1, 1, "model.20");
    ITensor* input_tensor_21[] = { conv20->getOutput(0), conv18->getOutput(0),conv16->getOutput(0), conv14->getOutput(0),
        conv13->getOutput(0) };
    IConcatenationLayer* concat21 = network->addConcatenation(input_tensor_21, 5);
    
    IElementWiseLayer* conv22 = convBnSilu(network, weightMap, *concat21->getOutput(0), 160, 1, 1, 0, "model.22");
    auto conv23 = network->addElementWise(*conv22->getOutput(0), *conv12->getOutput(0), ElementWiseOperation::kSUM);

    auto conv24 = DownC(network, weightMap, *conv23->getOutput(0), 160, 320, "model.24");
    IElementWiseLayer* conv25 = convBnSilu(network, weightMap, *conv24->getOutput(0), 128, 1, 1, 0, "model.25");
    IElementWiseLayer* conv26 = convBnSilu(network, weightMap, *conv24->getOutput(0), 128, 1, 1, 0, "model.26");

    IElementWiseLayer* conv27 = convBnSilu(network, weightMap, *conv26->getOutput(0), 128, 3, 1, 1, "model.27");
    IElementWiseLayer* conv28 = convBnSilu(network, weightMap, *conv27->getOutput(0), 128, 3, 1, 1, "model.28");
    IElementWiseLayer* conv29 = convBnSilu(network, weightMap, *conv28->getOutput(0), 128, 3, 1, 1, "model.29");
    IElementWiseLayer* conv30 = convBnSilu(network, weightMap, *conv29->getOutput(0), 128, 3, 1, 1, "model.30");
    IElementWiseLayer* conv31 = convBnSilu(network, weightMap, *conv30->getOutput(0), 128, 3, 1, 1, "model.31");
    IElementWiseLayer* conv32 = convBnSilu(network, weightMap, *conv31->getOutput(0), 128, 3, 1, 1, "model.32");

    ITensor* input_tensor_33[] = { conv32->getOutput(0), conv30->getOutput(0),conv28->getOutput(0), conv26->getOutput(0),
        conv25->getOutput(0)};
    IConcatenationLayer* concat33 = network->addConcatenation(input_tensor_33, 5);

    IElementWiseLayer* conv34 = convBnSilu(network, weightMap, *concat33->getOutput(0), 320, 1, 1, 0, "model.34");

    IElementWiseLayer* conv35 = convBnSilu(network, weightMap, *conv24->getOutput(0), 128, 1, 1, 0, "model.35");
    IElementWiseLayer* conv36 = convBnSilu(network, weightMap, *conv24->getOutput(0), 128, 1, 1, 0, "model.36");

    IElementWiseLayer* conv37 = convBnSilu(network, weightMap, *conv36->getOutput(0), 128, 3, 1, 1, "model.37");
    IElementWiseLayer* conv38 = convBnSilu(network, weightMap, *conv37->getOutput(0), 128, 3, 1, 1, "model.38");
    IElementWiseLayer* conv39 = convBnSilu(network, weightMap, *conv38->getOutput(0), 128, 3, 1, 1, "model.39");
    IElementWiseLayer* conv40 = convBnSilu(network, weightMap, *conv39->getOutput(0), 128, 3, 1, 1, "model.40");
    IElementWiseLayer* conv41 = convBnSilu(network, weightMap, *conv40->getOutput(0), 128, 3, 1, 1, "model.41");
    IElementWiseLayer* conv42 = convBnSilu(network, weightMap, *conv41->getOutput(0), 128, 3, 1, 1, "model.42");

    ITensor* input_tensor_43[] = { conv42->getOutput(0), conv40->getOutput(0),conv38->getOutput(0), conv36->getOutput(0),
        conv35->getOutput(0)};
    IConcatenationLayer* concat43 = network->addConcatenation(input_tensor_43, 5);
    IElementWiseLayer* conv44 = convBnSilu(network, weightMap, *concat43->getOutput(0), 320, 1, 1, 0, "model.44");

    auto conv45 = network->addElementWise(*conv44->getOutput(0), *conv34->getOutput(0), ElementWiseOperation::kSUM);

    auto conv46 = DownC(network, weightMap, *conv45->getOutput(0), 320, 640, "model.46");//=====


    IElementWiseLayer* conv47 = convBnSilu(network, weightMap, *conv46->getOutput(0), 256, 1, 1, 0, "model.47");
    IElementWiseLayer* conv48 = convBnSilu(network, weightMap, *conv46->getOutput(0), 256, 1, 1, 0, "model.48");

    IElementWiseLayer* conv49 = convBnSilu(network, weightMap, *conv48->getOutput(0), 256, 3, 1, 1, "model.49");
    IElementWiseLayer* conv50 = convBnSilu(network, weightMap, *conv49->getOutput(0), 256, 3, 1, 1, "model.50");
    IElementWiseLayer* conv51 = convBnSilu(network, weightMap, *conv50->getOutput(0), 256, 3, 1, 1, "model.51");
    IElementWiseLayer* conv52 = convBnSilu(network, weightMap, *conv51->getOutput(0), 256, 3, 1, 1, "model.52");
    IElementWiseLayer* conv53 = convBnSilu(network, weightMap, *conv52->getOutput(0), 256, 3, 1, 1, "model.53");
    IElementWiseLayer* conv54 = convBnSilu(network, weightMap, *conv53->getOutput(0), 256, 3, 1, 1, "model.54");
    
    ITensor* input_tensor_55[] = { conv54->getOutput(0), conv52->getOutput(0),conv50->getOutput(0), conv48->getOutput(0),
        conv47->getOutput(0) };
    IConcatenationLayer* concat55 = network->addConcatenation(input_tensor_55, 5);
    IElementWiseLayer* conv56 = convBnSilu(network, weightMap, *concat55->getOutput(0), 640, 1, 1, 0, "model.56");

    IElementWiseLayer* conv57 = convBnSilu(network, weightMap, *conv46->getOutput(0), 256, 1, 1, 0, "model.57");
    IElementWiseLayer* conv58 = convBnSilu(network, weightMap, *conv46->getOutput(0), 256, 1, 1, 0, "model.58");

    IElementWiseLayer* conv59 = convBnSilu(network, weightMap, *conv58->getOutput(0), 256, 3, 1, 1, "model.59");
    IElementWiseLayer* conv60 = convBnSilu(network, weightMap, *conv59->getOutput(0), 256, 3, 1, 1, "model.60");
    IElementWiseLayer* conv61 = convBnSilu(network, weightMap, *conv60->getOutput(0), 256, 3, 1, 1, "model.61");
    IElementWiseLayer* conv62 = convBnSilu(network, weightMap, *conv61->getOutput(0), 256, 3, 1, 1, "model.62");
    IElementWiseLayer* conv63 = convBnSilu(network, weightMap, *conv62->getOutput(0), 256, 3, 1, 1, "model.63");
    IElementWiseLayer* conv64 = convBnSilu(network, weightMap, *conv63->getOutput(0), 256, 3, 1, 1, "model.64");
    ITensor* input_tensor_65[] = { conv64->getOutput(0), conv62->getOutput(0),conv60->getOutput(0), conv58->getOutput(0),
        conv57->getOutput(0) };
    IConcatenationLayer* concat65 = network->addConcatenation(input_tensor_65, 5);
    IElementWiseLayer* conv66 = convBnSilu(network, weightMap, *concat65->getOutput(0), 640, 1, 1, 0, "model.66");
    auto conv67 = network->addElementWise(*conv66->getOutput(0), *conv56->getOutput(0), ElementWiseOperation::kSUM);

    auto conv68 = DownC(network, weightMap, *conv67->getOutput(0), 640, 960, "model.68");//=====

    IElementWiseLayer* conv69 = convBnSilu(network, weightMap, *conv68->getOutput(0), 384, 1, 1, 0, "model.69");
    IElementWiseLayer* conv70 = convBnSilu(network, weightMap, *conv68->getOutput(0), 384, 1, 1, 0, "model.70");

    IElementWiseLayer* conv71 = convBnSilu(network, weightMap, *conv70->getOutput(0), 384, 3, 1, 1, "model.71");
    IElementWiseLayer* conv72 = convBnSilu(network, weightMap, *conv71->getOutput(0), 384, 3, 1, 1, "model.72");
    IElementWiseLayer* conv73 = convBnSilu(network, weightMap, *conv72->getOutput(0), 384, 3, 1, 1, "model.73");
    IElementWiseLayer* conv74 = convBnSilu(network, weightMap, *conv73->getOutput(0), 384, 3, 1, 1, "model.74");
    IElementWiseLayer* conv75 = convBnSilu(network, weightMap, *conv74->getOutput(0), 384, 3, 1, 1, "model.75");
    IElementWiseLayer* conv76 = convBnSilu(network, weightMap, *conv75->getOutput(0), 384, 3, 1, 1, "model.76");
    ITensor* input_tensor_77[] = { conv76->getOutput(0), conv74->getOutput(0),conv72->getOutput(0), conv70->getOutput(0),
        conv69->getOutput(0) };
    IConcatenationLayer* concat77 = network->addConcatenation(input_tensor_77, 5);
    IElementWiseLayer* conv78 = convBnSilu(network, weightMap, *concat77->getOutput(0), 960, 1, 1, 0, "model.78");

    IElementWiseLayer* conv79 = convBnSilu(network, weightMap, *conv68->getOutput(0), 384, 1, 1, 0, "model.79");
    IElementWiseLayer* conv80 = convBnSilu(network, weightMap, *conv68->getOutput(0), 384, 1, 1, 0, "model.80");

    IElementWiseLayer* conv81 = convBnSilu(network, weightMap, *conv80->getOutput(0), 384, 3, 1, 1, "model.81");
    IElementWiseLayer* conv82 = convBnSilu(network, weightMap, *conv81->getOutput(0), 384, 3, 1, 1, "model.82");
    IElementWiseLayer* conv83 = convBnSilu(network, weightMap, *conv82->getOutput(0), 384, 3, 1, 1, "model.83");
    IElementWiseLayer* conv84 = convBnSilu(network, weightMap, *conv83->getOutput(0), 384, 3, 1, 1, "model.84");
    IElementWiseLayer* conv85 = convBnSilu(network, weightMap, *conv84->getOutput(0), 384, 3, 1, 1, "model.85");
    IElementWiseLayer* conv86 = convBnSilu(network, weightMap, *conv85->getOutput(0), 384, 3, 1, 1, "model.86");
    ITensor* input_tensor_87[] = { conv86->getOutput(0), conv84->getOutput(0),conv82->getOutput(0), conv80->getOutput(0),
        conv79->getOutput(0) };
    IConcatenationLayer* concat87 = network->addConcatenation(input_tensor_87, 5);
    IElementWiseLayer* conv88 = convBnSilu(network, weightMap, *concat87->getOutput(0), 960, 1, 1, 0, "model.88");
    auto conv89 = network->addElementWise(*conv88->getOutput(0), *conv78->getOutput(0), ElementWiseOperation::kSUM);


    auto conv90 = DownC(network, weightMap, *conv89->getOutput(0), 960, 1280, "model.90");

    IElementWiseLayer* conv91 = convBnSilu(network, weightMap, *conv90->getOutput(0), 512, 1, 1, 0, "model.91");
    IElementWiseLayer* conv92 = convBnSilu(network, weightMap, *conv90->getOutput(0), 512, 1, 1, 0, "model.92");

    IElementWiseLayer* conv93 = convBnSilu(network, weightMap, *conv92->getOutput(0), 512, 3, 1, 1, "model.93");
    IElementWiseLayer* conv94 = convBnSilu(network, weightMap, *conv93->getOutput(0), 512, 3, 1, 1, "model.94");
    IElementWiseLayer* conv95 = convBnSilu(network, weightMap, *conv94->getOutput(0), 512, 3, 1, 1, "model.95");
    IElementWiseLayer* conv96 = convBnSilu(network, weightMap, *conv95->getOutput(0), 512, 3, 1, 1, "model.96");
    IElementWiseLayer* conv97 = convBnSilu(network, weightMap, *conv96->getOutput(0), 512, 3, 1, 1, "model.97");
    IElementWiseLayer* conv98 = convBnSilu(network, weightMap, *conv97->getOutput(0), 512, 3, 1, 1, "model.98");
    ITensor* input_tensor_99[] = { conv98->getOutput(0), conv96->getOutput(0),conv94->getOutput(0), conv92->getOutput(0),
      conv91->getOutput(0) };
    IConcatenationLayer* concat99 = network->addConcatenation(input_tensor_99, 5);
    IElementWiseLayer* conv100 = convBnSilu(network, weightMap, *concat99->getOutput(0), 1280, 1, 1, 0, "model.100");
    
    IElementWiseLayer* conv101 = convBnSilu(network, weightMap, *conv90->getOutput(0), 512, 1, 1, 0, "model.101");
    IElementWiseLayer* conv102 = convBnSilu(network, weightMap, *conv90->getOutput(0), 512, 1, 1, 0, "model.102");

    IElementWiseLayer* conv103 = convBnSilu(network, weightMap, *conv102->getOutput(0), 512, 3, 1, 1, "model.103");
    IElementWiseLayer* conv104 = convBnSilu(network, weightMap, *conv103->getOutput(0), 512, 3, 1, 1, "model.104");
    IElementWiseLayer* conv105 = convBnSilu(network, weightMap, *conv104->getOutput(0), 512, 3, 1, 1, "model.105");
    IElementWiseLayer* conv106 = convBnSilu(network, weightMap, *conv105->getOutput(0), 512, 3, 1, 1, "model.106");
    IElementWiseLayer* conv107 = convBnSilu(network, weightMap, *conv106->getOutput(0), 512, 3, 1, 1, "model.107");
    IElementWiseLayer* conv108 = convBnSilu(network, weightMap, *conv107->getOutput(0), 512, 3, 1, 1, "model.108");
    ITensor* input_tensor_109[] = { conv108->getOutput(0), conv106->getOutput(0),conv104->getOutput(0), conv102->getOutput(0),
      conv101->getOutput(0) };
    IConcatenationLayer* concat109 = network->addConcatenation(input_tensor_109, 5);
    IElementWiseLayer* conv110 = convBnSilu(network, weightMap, *concat109->getOutput(0), 1280, 1, 1, 0, "model.110");
    auto conv111 = network->addElementWise(*conv110->getOutput(0), *conv100->getOutput(0), ElementWiseOperation::kSUM);
    //---------------------------yolov7e6e head---------------------------------
    auto conv112 = SPPCSPC(network, weightMap, *conv111->getOutput(0), 640, "model.112");
    IElementWiseLayer* conv113 = convBnSilu(network, weightMap, *conv112->getOutput(0), 480, 1, 1, 0, "model.113");


    float scale[] = { 1.0, 2.0, 2.0 };
    IResizeLayer* re114 = network->addResize(*conv113->getOutput(0));
    re114->setResizeMode(ResizeMode::kNEAREST);
    re114->setScales(scale, 3);

    IElementWiseLayer* conv115 = convBnSilu(network, weightMap, *conv89->getOutput(0), 480, 1, 1, 0, "model.115");
    ITensor* input_tensor_116[] = { conv115->getOutput(0), re114->getOutput(0) };
    IConcatenationLayer* concat116 = network->addConcatenation(input_tensor_116, 2);


    IElementWiseLayer* conv117 = convBnSilu(network, weightMap, *concat116->getOutput(0), 384, 1, 1, 0, "model.117");
    IElementWiseLayer* conv118 = convBnSilu(network, weightMap, *concat116->getOutput(0), 384, 1, 1, 0, "model.118");

    IElementWiseLayer* conv119 = convBnSilu(network, weightMap, *conv118->getOutput(0), 192, 3, 1, 1, "model.119");
    IElementWiseLayer* conv120 = convBnSilu(network, weightMap, *conv119->getOutput(0), 192, 3, 1, 1, "model.120");
    IElementWiseLayer* conv121 = convBnSilu(network, weightMap, *conv120->getOutput(0), 192, 3, 1, 1, "model.121");
    IElementWiseLayer* conv122 = convBnSilu(network, weightMap, *conv121->getOutput(0), 192, 3, 1, 1, "model.122");
    IElementWiseLayer* conv123 = convBnSilu(network, weightMap, *conv122->getOutput(0), 192, 3, 1, 1, "model.123");
    IElementWiseLayer* conv124 = convBnSilu(network, weightMap, *conv123->getOutput(0), 192, 3, 1, 1, "model.124");
    ITensor* input_tensor_125[] = { conv124->getOutput(0), conv123->getOutput(0),conv122->getOutput(0), conv121->getOutput(0),
        conv120->getOutput(0), conv119->getOutput(0), conv118->getOutput(0), conv117->getOutput(0) };
    IConcatenationLayer* concat125 = network->addConcatenation(input_tensor_125, 8);
    IElementWiseLayer* conv126 = convBnSilu(network, weightMap, *concat125->getOutput(0), 480, 1, 1, 0, "model.126");

    IElementWiseLayer* conv127 = convBnSilu(network, weightMap, *concat116->getOutput(0), 384, 1, 1, 0, "model.127");
    IElementWiseLayer* conv128 = convBnSilu(network, weightMap, *concat116->getOutput(0), 384, 1, 1, 0, "model.128");

    IElementWiseLayer* conv129 = convBnSilu(network, weightMap, *conv128->getOutput(0), 192, 3, 1, 1, "model.129");
    IElementWiseLayer* conv130 = convBnSilu(network, weightMap, *conv129->getOutput(0), 192, 3, 1, 1, "model.130");
    IElementWiseLayer* conv131 = convBnSilu(network, weightMap, *conv130->getOutput(0), 192, 3, 1, 1, "model.131");
    IElementWiseLayer* conv132 = convBnSilu(network, weightMap, *conv131->getOutput(0), 192, 3, 1, 1, "model.132");
    IElementWiseLayer* conv133 = convBnSilu(network, weightMap, *conv132->getOutput(0), 192, 3, 1, 1, "model.133");
    IElementWiseLayer* conv134 = convBnSilu(network, weightMap, *conv133->getOutput(0), 192, 3, 1, 1, "model.134");
    ITensor* input_tensor_135[] = { conv134->getOutput(0), conv133->getOutput(0),conv132->getOutput(0), conv131->getOutput(0),
        conv130->getOutput(0), conv129->getOutput(0), conv128->getOutput(0), conv127->getOutput(0) };
    IConcatenationLayer* concat135 = network->addConcatenation(input_tensor_135, 8);
    IElementWiseLayer* conv136 = convBnSilu(network, weightMap, *concat135->getOutput(0), 480, 1, 1, 0, "model.136");
    auto conv137 = network->addElementWise(*conv136->getOutput(0), *conv126->getOutput(0), ElementWiseOperation::kSUM);

    IElementWiseLayer* conv138 = convBnSilu(network, weightMap, *conv137->getOutput(0), 320, 1, 1, 0, "model.138");
    IResizeLayer* re139 = network->addResize(*conv138->getOutput(0));
    re139->setResizeMode(ResizeMode::kNEAREST);
    re139->setScales(scale, 3);
    IElementWiseLayer* conv140 = convBnSilu(network, weightMap, *conv67->getOutput(0), 320, 1, 1, 0, "model.140");
    ITensor* input_tensor_141[] = { conv140->getOutput(0), re139->getOutput(0) };
    IConcatenationLayer* concat141 = network->addConcatenation(input_tensor_141, 2);

    IElementWiseLayer* conv142 = convBnSilu(network, weightMap, *concat141->getOutput(0), 256, 1, 1, 0, "model.142");
    IElementWiseLayer* conv143 = convBnSilu(network, weightMap, *concat141->getOutput(0), 256, 1, 1, 0, "model.143");

    IElementWiseLayer* conv144 = convBnSilu(network, weightMap, *conv143->getOutput(0), 128, 3, 1, 1, "model.144");
    IElementWiseLayer* conv145 = convBnSilu(network, weightMap, *conv144->getOutput(0), 128, 3, 1, 1, "model.145");
    IElementWiseLayer* conv146 = convBnSilu(network, weightMap, *conv145->getOutput(0), 128, 3, 1, 1, "model.146");
    IElementWiseLayer* conv147 = convBnSilu(network, weightMap, *conv146->getOutput(0), 128, 3, 1, 1, "model.147");
    IElementWiseLayer* conv148 = convBnSilu(network, weightMap, *conv147->getOutput(0), 128, 3, 1, 1, "model.148");
    IElementWiseLayer* conv149 = convBnSilu(network, weightMap, *conv148->getOutput(0), 128, 3, 1, 1, "model.149");

    ITensor* input_tensor_150[] = { conv149->getOutput(0), conv148->getOutput(0),conv147->getOutput(0), conv146->getOutput(0),
        conv145->getOutput(0), conv144->getOutput(0), conv143->getOutput(0), conv142->getOutput(0) };
    IConcatenationLayer* concat150 = network->addConcatenation(input_tensor_150, 8);

    IElementWiseLayer* conv151 = convBnSilu(network, weightMap, *concat150->getOutput(0), 320, 1, 1, 0, "model.151");

    IElementWiseLayer* conv152 = convBnSilu(network, weightMap, *concat141->getOutput(0), 256, 1, 1, 0, "model.152");
    IElementWiseLayer* conv153 = convBnSilu(network, weightMap, *concat141->getOutput(0), 256, 1, 1, 0, "model.153");

    IElementWiseLayer* conv154 = convBnSilu(network, weightMap, *conv153->getOutput(0), 128, 3, 1, 1, "model.154");
    IElementWiseLayer* conv155 = convBnSilu(network, weightMap, *conv154->getOutput(0), 128, 3, 1, 1, "model.155");
    IElementWiseLayer* conv156 = convBnSilu(network, weightMap, *conv155->getOutput(0), 128, 3, 1, 1, "model.156");
    IElementWiseLayer* conv157 = convBnSilu(network, weightMap, *conv156->getOutput(0), 128, 3, 1, 1, "model.157");
    IElementWiseLayer* conv158 = convBnSilu(network, weightMap, *conv157->getOutput(0), 128, 3, 1, 1, "model.158");
    IElementWiseLayer* conv159 = convBnSilu(network, weightMap, *conv158->getOutput(0), 128, 3, 1, 1, "model.159");
    ITensor* input_tensor_160[] = { conv159->getOutput(0), conv158->getOutput(0),conv157->getOutput(0), conv156->getOutput(0),
        conv155->getOutput(0), conv154->getOutput(0), conv153->getOutput(0), conv152->getOutput(0) };
    IConcatenationLayer* concat160 = network->addConcatenation(input_tensor_160, 8);
    IElementWiseLayer* conv161 = convBnSilu(network, weightMap, *concat160->getOutput(0), 320, 1, 1, 0, "model.161");
    auto conv162 = network->addElementWise(*conv161->getOutput(0), *conv151->getOutput(0), ElementWiseOperation::kSUM);

    IElementWiseLayer* conv163 = convBnSilu(network, weightMap, *conv162->getOutput(0), 160, 1, 1, 0, "model.163");

    IResizeLayer* re164 = network->addResize(*conv163->getOutput(0));
    re164->setResizeMode(ResizeMode::kNEAREST);
    re164->setScales(scale, 3);

    IElementWiseLayer* conv165 = convBnSilu(network, weightMap, *conv45->getOutput(0), 160, 1, 1, 0, "model.165");
    ITensor* input_tensor_166[] = { conv165->getOutput(0), re164->getOutput(0) };
    IConcatenationLayer* concat166 = network->addConcatenation(input_tensor_166, 2);

    IElementWiseLayer* conv167 = convBnSilu(network, weightMap, *concat166->getOutput(0), 128, 1, 1, 0, "model.167");
    IElementWiseLayer* conv168 = convBnSilu(network, weightMap, *concat166->getOutput(0), 128, 1, 1, 0, "model.168");
    IElementWiseLayer* conv169 = convBnSilu(network, weightMap, *conv168->getOutput(0), 64, 3, 1, 1, "model.169");
    IElementWiseLayer* conv170 = convBnSilu(network, weightMap, *conv169->getOutput(0), 64, 3, 1, 1, "model.170");
    IElementWiseLayer* conv171 = convBnSilu(network, weightMap, *conv170->getOutput(0), 64, 3, 1, 1, "model.171");
    IElementWiseLayer* conv172 = convBnSilu(network, weightMap, *conv171->getOutput(0), 64, 3, 1, 1, "model.172");
    IElementWiseLayer* conv173 = convBnSilu(network, weightMap, *conv172->getOutput(0), 64, 3, 1, 1, "model.173");
    IElementWiseLayer* conv174 = convBnSilu(network, weightMap, *conv173->getOutput(0), 64, 3, 1, 1, "model.174");


    ITensor* input_tensor_175[] = { conv174->getOutput(0), conv173->getOutput(0),conv172->getOutput(0), conv171->getOutput(0),
       conv170->getOutput(0), conv169->getOutput(0), conv168->getOutput(0), conv167->getOutput(0) };
    IConcatenationLayer* concat175 = network->addConcatenation(input_tensor_175, 8); 
    IElementWiseLayer* conv176 = convBnSilu(network, weightMap, *concat175->getOutput(0), 160, 1, 1, 0, "model.176");
    IElementWiseLayer* conv177 = convBnSilu(network, weightMap, *concat166->getOutput(0), 128, 1, 1, 0, "model.177");
    IElementWiseLayer* conv178 = convBnSilu(network, weightMap, *concat166->getOutput(0), 128, 1, 1, 0, "model.178");
    
    IElementWiseLayer* conv179 = convBnSilu(network, weightMap, *conv178->getOutput(0), 64, 3, 1, 1, "model.179");
    IElementWiseLayer* conv180 = convBnSilu(network, weightMap, *conv179->getOutput(0), 64, 3, 1, 1, "model.180");
    IElementWiseLayer* conv181 = convBnSilu(network, weightMap, *conv180->getOutput(0), 64, 3, 1, 1, "model.181");
    IElementWiseLayer* conv182 = convBnSilu(network, weightMap, *conv181->getOutput(0), 64, 3, 1, 1, "model.182");
    IElementWiseLayer* conv183 = convBnSilu(network, weightMap, *conv182->getOutput(0), 64, 3, 1, 1, "model.183");
    IElementWiseLayer* conv184 = convBnSilu(network, weightMap, *conv183->getOutput(0), 64, 3, 1, 1, "model.184");
    ITensor* input_tensor_185[] = { conv184->getOutput(0), conv183->getOutput(0),conv182->getOutput(0), conv181->getOutput(0),
       conv180->getOutput(0), conv179->getOutput(0), conv178->getOutput(0), conv177->getOutput(0) };
    IConcatenationLayer* concat185 = network->addConcatenation(input_tensor_185, 8);
    IElementWiseLayer* conv186 = convBnSilu(network, weightMap, *concat185->getOutput(0), 160, 1, 1, 0, "model.186");
    auto conv187 = network->addElementWise(*conv186->getOutput(0), *conv176->getOutput(0), ElementWiseOperation::kSUM);

    auto conv188 = DownC(network, weightMap, *conv187->getOutput(0), 160, 320, "model.188");


    ITensor* input_tensor_189[] = { conv188->getOutput(0), conv162->getOutput(0) };
    IConcatenationLayer* concat189 = network->addConcatenation(input_tensor_189, 2);

    IElementWiseLayer* conv190 = convBnSilu(network, weightMap, *concat189->getOutput(0), 256, 1, 1, 0, "model.190");
    IElementWiseLayer* conv191 = convBnSilu(network, weightMap, *concat189->getOutput(0), 256, 1, 1, 0, "model.191");

    IElementWiseLayer* conv192 = convBnSilu(network, weightMap, *conv191->getOutput(0), 128, 3, 1, 1, "model.192");
    IElementWiseLayer* conv193 = convBnSilu(network, weightMap, *conv192->getOutput(0), 128, 3, 1, 1, "model.193");
    IElementWiseLayer* conv194 = convBnSilu(network, weightMap, *conv193->getOutput(0), 128, 3, 1, 1, "model.194");
    IElementWiseLayer* conv195 = convBnSilu(network, weightMap, *conv194->getOutput(0), 128, 3, 1, 1, "model.195");
    IElementWiseLayer* conv196 = convBnSilu(network, weightMap, *conv195->getOutput(0), 128, 3, 1, 1, "model.196");
    IElementWiseLayer* conv197 = convBnSilu(network, weightMap, *conv196->getOutput(0), 128, 3, 1, 1, "model.197");


    ITensor* input_tensor_198[] = { conv197->getOutput(0), conv196->getOutput(0),conv195->getOutput(0), conv194->getOutput(0),
       conv193->getOutput(0), conv192->getOutput(0), conv191->getOutput(0), conv190->getOutput(0) };
    IConcatenationLayer* concat198 = network->addConcatenation(input_tensor_198, 8);
    IElementWiseLayer* conv199 = convBnSilu(network, weightMap, *concat198->getOutput(0), 320, 1, 1, 0, "model.199");

    IElementWiseLayer* conv200 = convBnSilu(network, weightMap, *concat189->getOutput(0), 256, 1, 1, 0, "model.200");
    IElementWiseLayer* conv201 = convBnSilu(network, weightMap, *concat189->getOutput(0), 256, 1, 1, 0, "model.201");

    IElementWiseLayer* conv202 = convBnSilu(network, weightMap, *conv201->getOutput(0), 128, 3, 1, 1, "model.202");
    IElementWiseLayer* conv203 = convBnSilu(network, weightMap, *conv202->getOutput(0), 128, 3, 1, 1, "model.203");
    IElementWiseLayer* conv204 = convBnSilu(network, weightMap, *conv203->getOutput(0), 128, 3, 1, 1, "model.204");
    IElementWiseLayer* conv205 = convBnSilu(network, weightMap, *conv204->getOutput(0), 128, 3, 1, 1, "model.205");
    IElementWiseLayer* conv206 = convBnSilu(network, weightMap, *conv205->getOutput(0), 128, 3, 1, 1, "model.206");
    IElementWiseLayer* conv207 = convBnSilu(network, weightMap, *conv206->getOutput(0), 128, 3, 1, 1, "model.207");
    ITensor* input_tensor_208[] = { conv207->getOutput(0), conv206->getOutput(0),conv205->getOutput(0), conv204->getOutput(0),
      conv203->getOutput(0), conv202->getOutput(0), conv201->getOutput(0), conv200->getOutput(0) };
    IConcatenationLayer* concat208 = network->addConcatenation(input_tensor_208, 8);
    IElementWiseLayer* conv209 = convBnSilu(network, weightMap, *concat208->getOutput(0), 320, 1, 1, 0, "model.209");
    auto conv210 = network->addElementWise(*conv209->getOutput(0), *conv199->getOutput(0), ElementWiseOperation::kSUM);


    auto conv211 = DownC(network, weightMap, *conv210->getOutput(0), 320, 480, "model.211");
    ITensor* input_tensor_212[] = { conv211->getOutput(0), conv137->getOutput(0) };
    IConcatenationLayer* concat212 = network->addConcatenation(input_tensor_212, 2);

    IElementWiseLayer* conv213 = convBnSilu(network, weightMap, *concat212->getOutput(0), 384, 1, 1, 0, "model.213");
    IElementWiseLayer* conv214 = convBnSilu(network, weightMap, *concat212->getOutput(0), 384, 1, 1, 0, "model.214");

    IElementWiseLayer* conv215 = convBnSilu(network, weightMap, *conv214->getOutput(0), 192, 3, 1, 1, "model.215");
    IElementWiseLayer* conv216 = convBnSilu(network, weightMap, *conv215->getOutput(0), 192, 3, 1, 1, "model.216");
    IElementWiseLayer* conv217 = convBnSilu(network, weightMap, *conv216->getOutput(0), 192, 3, 1, 1, "model.217");
    IElementWiseLayer* conv218 = convBnSilu(network, weightMap, *conv217->getOutput(0), 192, 3, 1, 1, "model.218");
    IElementWiseLayer* conv219 = convBnSilu(network, weightMap, *conv218->getOutput(0), 192, 3, 1, 1, "model.219");
    IElementWiseLayer* conv220 = convBnSilu(network, weightMap, *conv219->getOutput(0), 192, 3, 1, 1, "model.220");

    ITensor* input_tensor_221[] = { conv220->getOutput(0), conv219->getOutput(0),conv218->getOutput(0), conv217->getOutput(0),
      conv216->getOutput(0), conv215->getOutput(0), conv214->getOutput(0), conv213->getOutput(0) };
    IConcatenationLayer* concat221 = network->addConcatenation(input_tensor_221, 8);
    IElementWiseLayer* conv222 = convBnSilu(network, weightMap, *concat221->getOutput(0), 480, 1, 1, 0, "model.222");

    IElementWiseLayer* conv223 = convBnSilu(network, weightMap, *concat212->getOutput(0), 384, 1, 1, 0, "model.223");
    IElementWiseLayer* conv224 = convBnSilu(network, weightMap, *concat212->getOutput(0), 384, 1, 1, 0, "model.224");

    IElementWiseLayer* conv225 = convBnSilu(network, weightMap, *conv224->getOutput(0), 192, 3, 1, 1, "model.225");
    IElementWiseLayer* conv226 = convBnSilu(network, weightMap, *conv225->getOutput(0), 192, 3, 1, 1, "model.226");
    IElementWiseLayer* conv227 = convBnSilu(network, weightMap, *conv226->getOutput(0), 192, 3, 1, 1, "model.227");
    IElementWiseLayer* conv228 = convBnSilu(network, weightMap, *conv227->getOutput(0), 192, 3, 1, 1, "model.228");
    IElementWiseLayer* conv229 = convBnSilu(network, weightMap, *conv228->getOutput(0), 192, 3, 1, 1, "model.229");
    IElementWiseLayer* conv230 = convBnSilu(network, weightMap, *conv229->getOutput(0), 192, 3, 1, 1, "model.230");
    ITensor* input_tensor_231[] = { conv230->getOutput(0), conv229->getOutput(0),conv228->getOutput(0), conv227->getOutput(0),
     conv226->getOutput(0), conv225->getOutput(0), conv224->getOutput(0), conv223->getOutput(0) };
    IConcatenationLayer* concat231 = network->addConcatenation(input_tensor_231, 8);
    IElementWiseLayer* conv232 = convBnSilu(network, weightMap, *concat231->getOutput(0), 480, 1, 1, 0, "model.232");

    auto conv233 = network->addElementWise(*conv232->getOutput(0), *conv222->getOutput(0), ElementWiseOperation::kSUM);


    auto conv234 = DownC(network, weightMap, *conv233->getOutput(0), 480, 640, "model.234");
    ITensor* input_tensor_235[] = { conv234->getOutput(0), conv112->getOutput(0) };
    IConcatenationLayer* concat235 = network->addConcatenation(input_tensor_235, 2);


    IElementWiseLayer* conv236 = convBnSilu(network, weightMap, *concat235->getOutput(0), 512, 1, 1, 0, "model.236");
    IElementWiseLayer* conv237 = convBnSilu(network, weightMap, *concat235->getOutput(0), 512, 1, 1, 0, "model.237");

    IElementWiseLayer* conv238 = convBnSilu(network, weightMap, *conv237->getOutput(0), 256, 3, 1, 1, "model.238");
    IElementWiseLayer* conv239 = convBnSilu(network, weightMap, *conv238->getOutput(0), 256, 3, 1, 1, "model.239");
    IElementWiseLayer* conv240 = convBnSilu(network, weightMap, *conv239->getOutput(0), 256, 3, 1, 1, "model.240");
    IElementWiseLayer* conv241 = convBnSilu(network, weightMap, *conv240->getOutput(0), 256, 3, 1, 1, "model.241");
    IElementWiseLayer* conv242 = convBnSilu(network, weightMap, *conv241->getOutput(0), 256, 3, 1, 1, "model.242");
    IElementWiseLayer* conv243 = convBnSilu(network, weightMap, *conv242->getOutput(0), 256, 3, 1, 1, "model.243");
  
    ITensor* input_tensor_244[] = { conv243->getOutput(0), conv242->getOutput(0),conv241->getOutput(0), conv240->getOutput(0),
     conv239->getOutput(0), conv238->getOutput(0), conv237->getOutput(0), conv236->getOutput(0) };
    IConcatenationLayer* concat244 = network->addConcatenation(input_tensor_244, 8);
    IElementWiseLayer* conv245 = convBnSilu(network, weightMap, *concat244->getOutput(0), 640, 1, 1, 0, "model.245");

    IElementWiseLayer* conv246 = convBnSilu(network, weightMap, *concat235->getOutput(0), 512, 1, 1, 0, "model.246");
    IElementWiseLayer* conv247 = convBnSilu(network, weightMap, *concat235->getOutput(0), 512, 1, 1, 0, "model.247");

    IElementWiseLayer* conv248 = convBnSilu(network, weightMap, *conv247->getOutput(0), 256, 3, 1, 1, "model.248");
    IElementWiseLayer* conv249 = convBnSilu(network, weightMap, *conv248->getOutput(0), 256, 3, 1, 1, "model.249");
    IElementWiseLayer* conv250 = convBnSilu(network, weightMap, *conv249->getOutput(0), 256, 3, 1, 1, "model.250");
    IElementWiseLayer* conv251 = convBnSilu(network, weightMap, *conv250->getOutput(0), 256, 3, 1, 1, "model.251");
    IElementWiseLayer* conv252 = convBnSilu(network, weightMap, *conv251->getOutput(0), 256, 3, 1, 1, "model.252");
    IElementWiseLayer* conv253 = convBnSilu(network, weightMap, *conv252->getOutput(0), 256, 3, 1, 1, "model.253");

    ITensor* input_tensor_254[] = { conv253->getOutput(0), conv252->getOutput(0),conv251->getOutput(0), conv250->getOutput(0),
    conv249->getOutput(0), conv248->getOutput(0), conv247->getOutput(0), conv246->getOutput(0) };
    IConcatenationLayer* concat254 = network->addConcatenation(input_tensor_254, 8);

    IElementWiseLayer* conv255= convBnSilu(network, weightMap, *concat254->getOutput(0), 640, 1, 1, 0, "model.255");
    auto conv256 = network->addElementWise(*conv255->getOutput(0), *conv245->getOutput(0), ElementWiseOperation::kSUM);

    IElementWiseLayer* conv257 = convBnSilu(network, weightMap, *conv187->getOutput(0), 320, 3, 1, 1, "model.257");
    IElementWiseLayer* conv258 = convBnSilu(network, weightMap, *conv210->getOutput(0), 640, 3, 1, 1, "model.258");
    IElementWiseLayer* conv259 = convBnSilu(network, weightMap, *conv233->getOutput(0), 960, 3, 1, 1, "model.259");
    IElementWiseLayer* conv260 = convBnSilu(network, weightMap, *conv256->getOutput(0), 1280, 3, 1, 1, "model.260");



    // out
    IConvolutionLayer* cv105_0 = network->addConvolutionNd(*conv257->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.261.m.0.weight"], weightMap["model.261.m.0.bias"]);
    assert(cv105_0);
    cv105_0->setName("cv105.0");
    IConvolutionLayer* cv105_1 = network->addConvolutionNd(*conv258->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.261.m.1.weight"], weightMap["model.261.m.1.bias"]);
    assert(cv105_1);
    cv105_1->setName("cv105.1");
    IConvolutionLayer* cv105_2 = network->addConvolutionNd(*conv259->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.261.m.2.weight"], weightMap["model.261.m.2.bias"]);
    assert(cv105_2);
    cv105_2->setName("cv105.2");
    IConvolutionLayer* cv105_3 = network->addConvolutionNd(*conv260->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.261.m.3.weight"], weightMap["model.261.m.3.bias"]);
    assert(cv105_3);
    cv105_3->setName("cv105.3");



    /*------------detect-----------*/
    auto yolo = addYoLoLayer(network, weightMap, "model.261", std::vector<IConvolutionLayer*>{cv105_0, cv105_1, cv105_2, cv105_3});
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));
    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    network->destroy();

    // Release host memory
    for (auto& mem : weightMap) {
        free((void*)(mem.second.values));
    }

    return engine;
}

ICudaEngine *MyTensorRT_v7::build_engine_p6(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, nvinfer1::DataType dt, float &gd, float &gw, std::string &wts_name)
{
    INetworkDefinition *network = builder->createNetworkV2(0U);
    // Create input tensor of shape {3, this->input_H, this->input_W} with name INPUT_BLOB_NAME
    ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, this->this->input_H, this->this->input_W});
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

    auto yolo = addYoLoLayer(network, weightMap, "model.33", std::vector<IConvolutionLayer *>{det0, det1, det2, det3}, this->cls_num, this->this->input_H, this->this->input_W);
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
    Int8EntropyCalibrator2 *calibrator = new Int8EntropyCalibrator2(1, this->input_W, this->input_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], Building engine, please wait for a while...\n");
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "Build engine successfully!\n");

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto &mem : weightMap)
    {
        free((void *)(mem.second.values));
    }

    return engine;
}

void MyTensorRT_v7::APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, bool &is_p6, float &gd, float &gw, std::string &wts_name)
{
    TRTLogger logger;
    // Create builder
    IBuilder *builder = createInferBuilder(logger);
    IBuilderConfig *config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Build P6 Engine\n");
        engine = this->build_engine_p6(maxBatchSize, builder, config, nvinfer1::DataType::kFLOAT, gd, gw, wts_name);
    }
    else
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], Build Default Engine\n");
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

bool MyTensorRT_v7::build_model(string wts_name, string engine_name, bool is_p6, float gd, float gw)
{
    char engine_name_c[engine_name.length()];
    strcpy(engine_name_c, engine_name.data());

    if (access(engine_name_c, F_OK) == 0)
    {
        fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
                   "[INFO], TRT Moudle exists , Skip creation\n");
        return true;
    }
    if (!wts_name.empty())
    {
        char wts_name_c[wts_name.length()];
        strcpy(wts_name_c, wts_name.data());
        if (access(wts_name_c, F_OK) != 0)
        {
            fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                       "[ERROR], WTS doesn't exists , Stop creation\n");
            return false;
        }
        IHostMemory *modelStream{nullptr};
        APIToModel(this->max_batchsize, &modelStream, is_p6, gd, gw, wts_name);
        if (modelStream == nullptr)
            fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                       "[ERROR], Failed to build engine !\n");
        assert(modelStream != nullptr);
        std::ofstream p(engine_name, std::ios::binary);
        if (!p)
        {
            fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                       "[ERROR], could not open plan output file\n");
            return false;
        }
        p.write(reinterpret_cast<const char *>(modelStream->data()), modelStream->size());
        modelStream->destroy();
        return true;
    }
    return false;
}

bool MyTensorRT_v7::initMyTensorRT_v7(char *tensorrtEngienPath, char *yolov5wts, bool is_p6, float gd, float gw, int max_batchsize, int this->input_H, int this->input_W, int cls_num)
{
    this->max_batchsize = max_batchsize;
    this->this->input_H = this->input_H;
    this->this->input_W = this->input_W;
    this->cls_num = cls_num;
    if (this->build_model(yolov5wts, tensorrtEngienPath, is_p6, gd, gw))
    {
        TRTLogger logger;
        std::ifstream file(tensorrtEngienPath, std::ios::binary);
        if (!file.good())
        {
            std::cerr << "read " << tensorrtEngienPath << " error!" << std::endl;
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
        this->runtime = createInferRuntime(logger);
        this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size, nullptr);
        this->context = this->engine->createExecutionContext();
        assert(this->engine->getNbBindings() == 2);
        this->inputIndex = this->engine->getBindingIndex(INPUT_BLOB_NAME);
        this->outputIndex = this->engine->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(this->inputIndex == 0);
        assert(this->outputIndex == 1);
        CUDA_CHECK(cudaMalloc(&this->buffers[this->inputIndex], this->max_batchsize * 3 * this->this->input_H * this->this->input_W * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&this->buffers[this->outputIndex], this->max_batchsize * TRT_OUTPUT_SIZE * sizeof(float)));
        CUDA_CHECK(cudaMallocHost((void **)&this->img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
        CUDA_CHECK(cudaMalloc((void **)&this->img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
        this->output = (float *)malloc(this->max_batchsize * TRT_OUTPUT_SIZE * sizeof(float));
    }
    else
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold | fmt::v9::bg(fmt::color::white),
                   "[ERROR], Failed to build the infer moudle !Please CHECK and Restart the Program.");
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   " TRT LOGIC BREAK.\n");
        return false;
    }
    return true;
}

void MyTensorRT_v7::unInitMyTensorRT_v7()
{
    CUDA_CHECK(cudaFree(this->buffers[this->inputIndex]));
    CUDA_CHECK(cudaFree(this->buffers[this->outputIndex]));
    this->runtime->destroy();
}

vector<vector<Yolo::Detection>> MyTensorRT_v7::doInference(vector<Mat> *input, int batchSize, float confidence_threshold, float nms_threshold)
{
    assert(this->context != nullptr);
    vector<vector<Yolo::Detection>> batch_res(batchSize);
    cudaStream_t stream = nullptr;
    CUDA_CHECK(cudaStreamCreate(&stream));
    float *buffer_idx = (float *)buffers[inputIndex];
    for (size_t b = 0; b < input->size(); b++)
    {
        Mat img = input->at(b);
        if (img.empty())
            continue;
        size_t size_image = img.cols * img.rows * 3;
        size_t size_image_dst = this->this->input_H * this->this->input_W * 3;
        memcpy(img_host, img.data, size_image);
        CUDA_CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
        preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx, this->this->input_W, this->this->input_H, stream);
        buffer_idx += size_image_dst;
    }
    // CUDA_CHECK(cudaMemcpyAsync(buffers[inputIndex], input,
    //                       batchSize * 3 * this->this->input_H * this->this->input_W * sizeof(float),
    //                       cudaMemcpyHostToDevice, stream));
    auto input_dims = this->engine->getBindingDimensions(0);
    input_dims.d[0] = batchSize;
    context->setBindingDimensions(0, input_dims);
    bool success = context->enqueueV2(buffers, stream, nullptr);
    if (!success)
    {
        fmt::print(fg(fmt::color::red) | fmt::emphasis::bold,
                   "[ERROR], doInference failed\n");
        CUDA_CHECK(cudaStreamDestroy(stream));
        return {};
    }
    CUDA_CHECK(cudaMemcpyAsync(this->output, buffers[outputIndex], batchSize * TRT_OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    CUDA_CHECK(cudaStreamDestroy(stream));
    for (int b = 0; b < batchSize; ++b)
    {
        auto &res = batch_res[b];
        nms(res, &this->output[b * TRT_OUTPUT_SIZE], confidence_threshold, nms_threshold);
    }
    return batch_res;
}