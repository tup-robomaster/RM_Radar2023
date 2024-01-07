#include "../include/ExpLog.h"

ExpLog::ExpLog()
{
}

ExpLog::~ExpLog()
{
}

void ExpLog::init(string outputDir)
{
    if (access(outputDir.c_str(), F_OK) == -1)
    {
        this->logger->error("[ERR] ExpOutputDir, non-existent");
    }
    else
    {
        char filename[1024];
        time_t currentTime = time(NULL);
        char chCurrentTime[256];
        strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d %H%M%S", localtime(&currentTime));
        strcat(chCurrentTime, ".csv");
        strcpy(filename, outputDir.c_str());
        int length = strlen(outputDir.c_str());
        if (outputDir[length - 1] != '/')
        {
            strcat(filename, "/");
        }
        strcat(filename, "ExpData");
        strcat(filename, chCurrentTime);
        this->oFile.open(filename, ios::out | ios::trunc);
        this->oFile << "识别数"
                    << ","
                    << "推理识别数"
                    << ","
                    << "平均置信度"
                    << ","
                    << "推理平均置信度"
                    << ","
                    << "耗时(ns)" << std::endl;
        this->oFile.flush();
    }
}

void ExpLog::input(vector<string> &msg)
{
    if (!this->oFile.is_open())
        return;
    for (const auto &it : msg)
    {
        this->oFile << it << ",";
    }
    this->oFile << std::endl;
    this->oFile.flush();
}

void ExpLog::uninit()
{
    this->oFile.flush();
    this->oFile.close();
}