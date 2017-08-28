//
// Created by joshua on 8/28/17.
//
#include <mps/planner/util/Serialize.h>
#include <fstream>
#include <sim_env/SimEnv.h>

using namespace mps::planner::util::serialize;

RealValueSerializable::~RealValueSerializable() {}

Eigen::IOFormat RealValueSerializable::eigen_format(Eigen::StreamPrecision, 0, ", ");

OracleDataDumper::OracleDataDumper() = default;

OracleDataDumper::~OracleDataDumper() = default;

void OracleDataDumper::setFile(const std::string &file_name) {
    _file_name = file_name;
    std::ifstream f(file_name.c_str());
    if (not f.good()) {
        std::stringstream ss;
        ss << "[mps::planner::util::serialize::OracleDataDumper::setFile] Could not access file ";
        ss << file_name;
        throw std::runtime_error(ss.str());
    }
}

void OracleDataDumper::saveData(::ompl::base::State *start, ::ompl::base::State *result,
                                ::ompl::control::Control *control) {
    DataTriplet triplet(start, result, control);
    saveData(triplet);
}

void OracleDataDumper::saveData(const DataTriplet& data) {
    std::vector<DataTriplet> a_vector;
    a_vector.push_back(data);
    saveData(a_vector);
}

void OracleDataDumper::saveData(const std::vector<DataTriplet>& data) {
    static const std::string log_prefix("[mps::planner::util::serialize::OracleDataDumper::saveFile]");
    std::ofstream f(_file_name.c_str(), std::ios_base::app);
    if (not f.good()) {
        std::stringstream ss;
        ss << log_prefix;
        ss << " Could not access file ";
        ss << _file_name;
        throw std::runtime_error(ss.str());
    }
    for (auto& data_item : data) {
        // cast start state
        auto* start_state = dynamic_cast<const RealValueSerializable*>(std::get<0>(data_item));
        if (!start_state) {
            std::string msg("Could not serialize start state. State type does not implement RealValueSerializable.");
            f.close();
            throw std::logic_error(log_prefix + msg);
        }

        // cast result state
        auto* result = dynamic_cast<const RealValueSerializable*>(std::get<1>(data_item));
        if (!result) {
            std::string msg("Could not serialize result state. State type does not implement RealValueSerializable.");
            f.close();
            throw std::logic_error(log_prefix + msg);
        }

        // cast control
        auto* control = dynamic_cast<const RealValueSerializable*>(std::get<2>(data_item));
        if (!control) {
            std::string msg("Could not serialize control. Control type does not implement RealValueSerializable.");
            f.close();
            throw std::logic_error(log_prefix + msg);
        }
        // now write data
        start_state->serializeInNumbers(f);
        f << ", ";
        result->serializeInNumbers(f);
        f << ", ";
        control->serializeInNumbers(f);
        f << "\n";
    }
    f.close();
    // done
}
