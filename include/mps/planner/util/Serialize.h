//
// Created by joshua on 8/28/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_DATADUMPER_H
#define MANIPULATION_PLANNING_SUITE_DATADUMPER_H

#include <vector>
#include <tuple>
#include <Eigen/Core>
#include <ompl/base/State.h>
#include <ompl/control/Control.h>
#include <fstream>

namespace mps {
    namespace planner {
        namespace util {
            namespace serialize {
                /**
                 * A RealValueSerializable can be serialized as a sequence of
                 * comma separated real value numbers.
                 */
                class RealValueSerializable {
                public:
                    virtual ~RealValueSerializable() = 0;
                    /**
                     * Serialize this serializable as a sequence of comma
                     * separated real value numbers.
                     * @param ostream - stream to serialize to
                     * @return number of numbers
                     */
                    virtual void serializeInNumbers(std::ostream& ostream) const = 0;
                    /**
                     * Returns the number of numbers this serializable
                     * requires to be described.
                     * @return
                     */
//                    virtual unsigned int getNumNumbers() const = 0;
                    static Eigen::IOFormat eigen_format;
                };

                class OracleDataDumper {
                public:
                    typedef std::tuple<::ompl::base::State*, ::ompl::base::State*, ::ompl::control::Control*> DataTriplet;
                    OracleDataDumper();
                    ~OracleDataDumper();
                    void setFile(const std::string& file_name);
                    bool openFile();
                    void writeHeader(const std::string& some_text);
                    void saveData(::ompl::base::State* start,
                                ::ompl::base::State* result,
                                ::ompl::control::Control* control,
                                const std::string& add_info = std::string(""));
                    void saveData(const DataTriplet& data,
                                  const std::string& add_info = std::string(""));
                    void saveData(const std::vector<DataTriplet>& data,
                                  const std::string& add_info = std::string(""));
                    void closeFile();
                private:
                    bool _file_valid;
                    std::string _file_name;
                    std::ofstream _file_stream;
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_DATADUMPER_H
