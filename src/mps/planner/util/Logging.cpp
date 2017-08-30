//
// Created by joshua on 8/30/17.
//
#include <mps/planner/util/Logging.h>

sim_env::LoggerPtr mps::planner::util::logging::logger_instance;

sim_env::LoggerPtr mps::planner::util::logging::getLogger() {
    if(!logger_instance) {
        resetLogger();
    }
    return logger_instance;
}

void mps::planner::util::logging::setLogger(sim_env::LoggerPtr logger) {
    logger_instance = logger;
}

void mps::planner::util::logging::resetLogger() {
    logger_instance = sim_env::DefaultLogger::getInstance();
}

void mps::planner::util::logging::logDebug(const std::string &msg, const std::string &prefix) {
    auto logger = getLogger();
    logger->logDebug(msg, prefix);
}

void mps::planner::util::logging::logWarn(const std::string &msg, const std::string &prefix) {
    auto logger = getLogger();
    logger->logWarn(msg, prefix);
}

void mps::planner::util::logging::logErr(const std::string &msg, const std::string &prefix) {
    auto logger = getLogger();
    logger->logErr(msg, prefix);
}

void mps::planner::util::logging::logInfo(const std::string &msg, const std::string &prefix) {
    auto logger = getLogger();
    logger->logInfo(msg, prefix);
}
