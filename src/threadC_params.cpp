#include "threadC_params.h"

using Mat3 = std::array<double, 9>;
using Mat34 = std::array<double, 12>;

ThreadCParameters loadThreadCParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);

    ThreadCParameters params;

    // General
    params.F = config["F"].as<Mat3>();
    params.P_A = config["P_A"].as<Mat34>();
    params.P_B = config["P_B"].as<Mat34>();

    params.save_file = config["save_file"].as<bool>();
    
    return params;
}