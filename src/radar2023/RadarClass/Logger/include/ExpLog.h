#ifndef _EXPLOG_H
#define _EXPLOG_H

#include "../../Common/include/public.h"
#include "Logger.h"

class ExpLog
{
private:
    ofstream oFile;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    ExpLog();
    ~ExpLog();
};

#endif