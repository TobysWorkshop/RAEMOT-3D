#include "threadC_params.h"

Parameters loadThreadCParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);

    ThreadCParameters params;

    // General
    //params.width = config["width"].as<int>();
    
    return params;
}