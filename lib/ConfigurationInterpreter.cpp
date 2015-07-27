/* -------------------------------------------------------------------------- *
 *              DoubleLoopControllers: ConfigurationInterpreter.cpp           *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Copyright 2015 Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * ---------------------------------------------------------------------------*/

#include <fstream>
#include <map>
using std::map;
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <string>
using std::string;

#include "ConfigurationInterpreter.h"

namespace patch{
  double to_double(const string numString){
    double val = 0;
    std::stringstream stm(numString);
    stm >> val;
    return val;
  }
}

//***************************************************
//    Public Constructors
//***************************************************

ConfigurationInterpreter::ConfigurationInterpreter(const char* filename, const char fieldSeparator) {
  using namespace std;
  fstream file(filename, ios::in);
  if (file){
    string line;
    while (getline(file, line)) {
      size_t sep = line.find_first_of(fieldSeparator);
      if (sep != string::npos) {
        string key   = trim(line.substr(0, sep));
        string value = trim(line.substr(sep + 1));
        if (!key.empty() && !value.empty())
          configItems_[key] = patch::to_double(value);
        else
          throw runtime_error("Error within configuration file.");
      }
      else
        throw runtime_error("Error within configuration file.");
    }
  }
  else
    throw runtime_error("Cannot open config file.");
}



//***************************************************
//    Public Accessor Functions
//***************************************************

void ConfigurationInterpreter::getMap(map<string, double>& newMap) {
  newMap = configItems_;
}


double ConfigurationInterpreter::getValue(const string& key) const {
  map<string, double>::const_iterator it = configItems_.find(key);
  if (it != configItems_.end())
    return it->second;
  else
    throw std::runtime_error("Cannot find config item.");
}


//***************************************************
//    Private Functions
//***************************************************

string ConfigurationInterpreter::trim(string str) {
  size_t pos = str.find_first_not_of(" \t\n");
  if (pos != string::npos)
    str.erase(0, pos);
  pos = str.find_last_not_of(" \t\n");
  if (pos != string::npos)
    str.erase(pos + 1);
  return str;
}
