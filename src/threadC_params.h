#ifndef THREADC_PARAMS_H
#define THREADC_PARAMS_H
#include "yaml-cpp/yaml.h"
#include <string>
#include <array>

using Mat3 = std::array<double, 9>;
using Mat34 = std::array<double, 12>;

struct ThreadCParameters
{
  Mat3 F{0, 0, 0,
         0, 0, 0, 
         0, 0, 0};

  Mat34 P_A{0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0};

  Mat34 P_B{0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0};
            
  bool save_file = true;

};

ThreadCParameters loadThreadCParametersFromYAML(const std::string &yaml_file_path);

#endif // THREADC_PARAMS_H