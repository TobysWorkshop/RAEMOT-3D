#ifndef THREADC_PARAMS_H
#define THREADC_PARAMS_H
#include "yaml-cpp/yaml.h"
#include <string>

struct ThreadCParameters
{
  // int etc etc

};

Parameters loadThreadCParametersFromYAML(const std::string &yaml_file_path);

#endif // THREADC_PARAMS_H