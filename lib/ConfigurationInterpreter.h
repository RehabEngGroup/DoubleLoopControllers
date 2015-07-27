/* -------------------------------------------------------------------------- *
 *               DoubleLoopControllers: ConfigurationInterpreter.h            *
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

#ifndef ConfigurationInterpreter_h
#define ConfigurationInterpreter_h

#include <OpenSim/OpenSim.h>

#include <map>
#include <string>

class ConfigurationInterpreter
{
public:
  ConfigurationInterpreter(const char* filename, const char fieldSeparator = ':');
  double getValue(const std::string& key) const;
  void getMap(std::map<std::string, double>& newMap);
private:
  std::string trim(std::string str);
  std::map<std::string, double> configItems_;
};

#endif
