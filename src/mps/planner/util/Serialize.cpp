//
// Created by joshua on 8/28/17.
//
#include <mps/planner/util/Serialize.h>
#include <fstream>
#include <sim_env/SimEnv.h>
#include <string>

using namespace mps::planner::util::serialize;

RealValueSerializable::~RealValueSerializable() {}

Eigen::IOFormat RealValueSerializable::eigen_format(Eigen::StreamPrecision, 0, ", ");

void mps::planner::util::serialize::splitString(const std::string& text, std::vector<std::string>& splits, char delim) 
{
    splits.clear();
    std::stringstream ss(text);
    while (!ss.eof()) {
        std::string part_str;
        std::getline(ss, part_str, delim);
        splits.push_back(part_str);
    }
}

OracleDataDumper::OracleDataDumper() : _file_valid(false) {}

OracleDataDumper::~OracleDataDumper() = default;

void OracleDataDumper::setFile(const std::string &file_name) {
    _file_name = file_name;
    std::ifstream f(file_name.c_str());
    if (not f.good()) {
        std::ofstream of(file_name.c_str());
        of.close();
        f.open(file_name.c_str());
    }
    if (not f.good()) {
        std::stringstream ss;
        ss << "[mps::planner::util::serialize::OracleDataDumper::setFile] Could not access file ..";
        ss << file_name;
        throw std::runtime_error(ss.str());
    }
    _file_valid = true;
}

bool OracleDataDumper::openFile() {
    if(not _file_valid or _file_stream.is_open()) {
        return false;
    }
    _file_stream.open(_file_name.c_str(), std::ios_base::app);
    return true;
}

void OracleDataDumper::writeHeader(const std::string& some_text) {
    if (not _file_stream.good()) return;
    _file_stream << some_text << "\n";
}

void OracleDataDumper::saveData(::ompl::base::State *start, ::ompl::base::State *result,
                                ::ompl::control::Control *control,
                                const std::string& add_info) {
    DataTriplet triplet(start, result, control);
    saveData(triplet, add_info);
}

void OracleDataDumper::saveData(const DataTriplet& data, const std::string& add_info) {
    std::vector<DataTriplet> a_vector;
    a_vector.push_back(data);
    saveData(a_vector, add_info);
}

void OracleDataDumper::saveData(const std::vector<DataTriplet>& data, const std::string& add_info) {
    static const std::string log_prefix("[mps::planner::util::serialize::OracleDataDumper::saveFile]");
    if (not _file_stream.good()) {
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
            _file_stream.close();
            throw std::logic_error(log_prefix + msg);
        }

        // cast result state
        auto* result = dynamic_cast<const RealValueSerializable*>(std::get<1>(data_item));
        if (!result) {
            std::string msg("Could not serialize result state. State type does not implement RealValueSerializable.");
            _file_stream.close();
            throw std::logic_error(log_prefix + msg);
        }

        // cast control
        auto* control = dynamic_cast<const RealValueSerializable*>(std::get<2>(data_item));
        if (!control) {
            std::string msg("Could not serialize control. Control type does not implement RealValueSerializable.");
            _file_stream.close();
            throw std::logic_error(log_prefix + msg);
        }
        // now write data
        start_state->serializeInNumbers(_file_stream);
        _file_stream << ", ";
        result->serializeInNumbers(_file_stream);
        _file_stream << ", ";
        control->serializeInNumbers(_file_stream);
        if (add_info.empty()) {
            _file_stream << "\n";
        } else {
            _file_stream << ", " << add_info << "\n";
        }
    }
    // done
}

void OracleDataDumper::closeFile() {
    if (_file_stream.is_open()) {
        _file_stream.close();
    }
}
