#ifndef _LOGGER_H
#define _LOGGER_H

#include <cstdio>
#include <chrono>

#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_sinks.h"

using namespace spdlog;

class Logger
{
public:
    std::shared_ptr<spdlog::logger> logger;

public:
    Logger();
    ~Logger();
    void registerLogger(char *logFile, char *loggerName);
};

#endif