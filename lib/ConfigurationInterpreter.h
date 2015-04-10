// Authors: Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani


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
