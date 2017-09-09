//
// Created by joshua on 8/30/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_LOGGING_H
#define MANIPULATION_PLANNING_SUITE_LOGGING_H

#include <sim_env/SimEnv.h>

//#define LOG_STREAM(X) (mps::planng::util::logging::getLogger()->getStream() << X)

namespace mps {
    namespace planner {
        namespace util {
            namespace logging {
                // Logger instance, do not use directly, but instead use functions below
                extern sim_env::LoggerPtr logger_instance;

                sim_env::LoggerPtr getLogger();
                void setLogger(sim_env::LoggerPtr logger);
                void resetLogger();
                void logDebug(const std::string& msg, const std::string& prefix);
                void logDebug(const boost::format& bf, const std::string& prefix);
                void logWarn(const std::string& msg, const std::string& prefix);
                void logWarn(const boost::format& bf, const std::string& prefix);
                void logErr(const std::string& msg, const std::string& prefix);
                void logErr(const boost::format& bf, const std::string& prefix);
                void logInfo(const std::string& msg, const std::string& prefix);
                void logInfo(const boost::format& bf, const std::string& prefix);
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_LOGGING_H
