#include "threadC_params.h"

using Mat3 = std::array<double, 9>;
using Mat34 = std::array<double, 12>;

ThreadCParameters loadThreadCParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);

    ThreadCParameters params;

    // General
    params.image_width = config["image_width"].as<int>();
    params.image_height = config["image_height"].as<int>();

    params.F = config["F"].as<Mat3>();
    params.P_A = config["P_A"].as<Mat34>();
    params.P_B = config["P_B"].as<Mat34>();

    params.K_A = config["K_A"].as<Mat3>();
    params.K_B = config["K_B"].as<Mat3>();

    params.D_A = config["D_A"].as<std::array<double, 5>>();
    params.D_B = config["D_B"].as<std::array<double, 5>>();

    params.save_file = config["save_file"].as<int>();
    
    return params;
}