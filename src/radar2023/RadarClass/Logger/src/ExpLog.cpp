#include "../include/ExpLog.h"

ExpLog::ExpLog()
{
    if (access(ExpOutputDir, 0) == -1)
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
        strcpy(filename, ExpOutputDir);
        int length = strlen(ExpOutputDir);
        string outputdir = ExpOutputDir;
        if (outputdir[length - 1] != '/')
        {
            strcat(filename, "/");
        }
        strcat(filename, "ExpData");
        strcat(filename, chCurrentTime);
        this->oFile.open(filename, ios::out | ios::trunc);
        this->oFile << "识别数"
                    << ","
                    << "平均置信度" << std::endl;
    }
}

ExpLog::~ExpLog()
{
}
