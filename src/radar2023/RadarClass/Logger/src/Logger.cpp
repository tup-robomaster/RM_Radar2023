#include "../include/Logger.h"

SpdLogger::SpdLogger()
{
}

SpdLogger::~SpdLogger()
{
}

void SpdLogger::registerLogger(char *logFile, char *loggerName)
{
    char filename[1024];
    time_t currentTime = time(NULL);
    char chCurrentTime[256];
    strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d %H%M%S", localtime(&currentTime));
    strcat(chCurrentTime, ".txt");
    strcpy(filename, logFile);
    strcat(filename, chCurrentTime);
    auto sink1 = std::make_shared<spdlog::sinks::stdout_sink_mt>();
    auto sink2 = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filename, 1024 * 1024 * 25, 100);
    std::vector<spdlog::sink_ptr> sinks = {sink1, sink2};
    logger = std::make_shared<spdlog::logger>(loggerName, sinks.begin(), sinks.end());
    sink1->set_pattern("[%Y-%m-%d %H:%M:%S.%e][%l]>>>%v");
    sink2->set_pattern("[%H:%M:%S.%e][%L]%v");
    logger->flush_on(spdlog::level::err);
    spdlog::register_logger(logger);
}

