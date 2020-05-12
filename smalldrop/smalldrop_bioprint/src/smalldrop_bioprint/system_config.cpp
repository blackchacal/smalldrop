// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file system_config.cpp
 * \brief Defines a class that manages the smalldrop system configurations.
 */

#include <fstream>
#include <sstream>

#include <smalldrop_bioprint/system_config.h>
#include <smalldrop_state/exceptions.h>

#include <ros/package.h>

// YAML-CPP Library
#include <yaml-cpp/yaml.h>

namespace smalldrop
{
namespace smalldrop_bioprint
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief SystemConfig::SystemConfig(ros::NodeHandle nh)
 */
SystemConfig::SystemConfig(ros::NodeHandle nh) : nh_(nh)
{
  config_filemap_ = {
    { "bioprint", "smalldrop_bioprint" },
    { "robot", "smalldrop_robot_arm" },
    { "teleop", "smalldrop_teleoperation" },
    { "vision", "smalldrop_vision" },
  };
}

/**
 * copybrief SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
 * int& param_val)
 */
bool SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
                                   int& param_val) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  if (nh_.hasParam(param_name))
  {
    nh_.getParam(param_name, param_val);
    return true;
  }
  return false;
}

/**
 * copybrief SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
 * double& param_val)
 */
bool SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
                                   double& param_val) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  if (nh_.hasParam(param_name))
  {
    nh_.getParam(param_name, param_val);
    return true;
  }
  return false;
}

/**
 * copybrief SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
 * std::string& param_val)
 */
bool SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
                                   std::string& param_val) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  if (nh_.hasParam(param_name))
  {
    nh_.getParam(param_name, param_val);
    return true;
  }
  return false;
}

/**
 * copybrief SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
 * std::vector<int>& param_val)
 */
bool SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
                                   std::vector<int>& param_val) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  if (nh_.hasParam(param_name))
  {
    nh_.getParam(param_name, param_val);
    return true;
  }
  return false;
}

/**
 * copybrief SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
 * std::vector<double>& param_val)
 */
bool SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
                                   std::vector<double>& param_val) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  if (nh_.hasParam(param_name))
  {
    nh_.getParam(param_name, param_val);
    return true;
  }
  return false;
}

/**
 * copybrief SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
 * std::vector<std::string>& param_val)
 */
bool SystemConfig::readConfigParam(const std::string module, const std::string section, const std::string param,
                                   std::vector<std::string>& param_val) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  if (nh_.hasParam(param_name))
  {
    nh_.getParam(param_name, param_val);
    return true;
  }
  return false;
}

/**
 * \copybrief SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string
 * param, const int value)
 */
void SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string param,
                                    const int value) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  // Store the param on the parameter server
  nh_.setParam(param_name, value);

  // Update the configuration file
  if (!writeConfigToFile(getConfigFilePath(module), module, section, param, std::to_string(value)))
    throw smalldrop_state::ConfigFileSaveException();
}

/**
 * \copybrief SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string
 * param, const double value)
 */
void SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string param,
                                    const double value) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  // Store the param on the parameter server
  nh_.setParam(param_name, value);

  // Update the configuration file
  if (!writeConfigToFile(getConfigFilePath(module), module, section, param, std::to_string(value)))
    throw smalldrop_state::ConfigFileSaveException();
}

/**
 * \copybrief SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string
 * param, const std::string value)
 */
void SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string param,
                                    const std::string value) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  // Store the param on the parameter server
  nh_.setParam(param_name, value);

  // Update the configuration file
  if (!writeConfigToFile(getConfigFilePath(module), module, section, param, value))
    throw smalldrop_state::ConfigFileSaveException();
}

/**
 * \copybrief SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string
 * param, const std::vector<int> value)
 */
void SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string param,
                                    const std::vector<int> value) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  // Store the param on the parameter server
  nh_.setParam(param_name, value);

  // Update the configuration file
  std::vector<std::string> new_value;
  for (size_t i = 0; i < value.size(); i++)
    new_value.push_back(std::to_string(value[i]));

  if (!writeConfigToFile(getConfigFilePath(module), module, section, param, new_value))
    throw smalldrop_state::ConfigFileSaveException();
}

/**
 * \copybrief SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string
 * param, const std::vector<double> value)
 */
void SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string param,
                                    const std::vector<double> value) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  // Store the param on the parameter server
  nh_.setParam(param_name, value);

  // Update the configuration file
  std::vector<std::string> new_value;
  for (size_t i = 0; i < value.size(); i++)
    new_value.push_back(std::to_string(value[i]));
  
  if (!writeConfigToFile(getConfigFilePath(module), module, section, param, new_value))
    throw smalldrop_state::ConfigFileSaveException();
}

/**
 * \copybrief SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string
 * param, const std::vector<std::string> value)
 */
void SystemConfig::writeConfigParam(const std::string module, const std::string section, const std::string param,
                                    const std::vector<std::string> value) const
{
  std::string param_name = buildConfigParamName(module, section, param);

  // Store the param on the parameter server
  nh_.setParam(param_name, value);

  // Update the configuration file
  if (!writeConfigToFile(getConfigFilePath(module), module, section, param, value))
    throw smalldrop_state::ConfigFileSaveException();
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief SystemConfig::getConfigFilePath()
 */
std::string SystemConfig::getConfigFilePath(const std::string module) const
{
  std::stringstream path;
  if (config_filemap_.find(module) != config_filemap_.end())
  {
    path << ros::package::getPath(config_filemap_.find(module)->second) << "/config/config.yaml";
    return path.str();
  }
  else
    return "";
}

/**
 * \copybrief SystemConfig::buildConfigParamName()
 */
std::string SystemConfig::buildConfigParamName(const std::string module, const std::string section,
                                               const std::string param) const
{
  std::stringstream param_name;
  param_name << "/smalldrop/" << module << "/" << section << "/" << param;

  return param_name.str();
}

/**
 * \copybrief SystemConfig::writeConfigToFile(const std::string filepath, const std::string module, const std::string
 * section, const std::string param, const std::string value)
 */
bool SystemConfig::writeConfigToFile(const std::string filepath, const std::string module, const std::string section,
                                     const std::string param, const std::string value) const
{
  try
  {
    YAML::Node config = YAML::LoadFile(filepath);

    // Parse tree to find the key and update value
    config["smalldrop"][module][section][param] = value;

    // Store value on file
    std::ofstream fout(filepath);
    fout << config;
    return true;
  }
  catch(const std::exception& e)
  {
    return false;
  }
}

/**
 * \copybrief SystemConfig::writeConfigToFile(const std::string filepath, const std::string module, const std::string section, const std::string param, 
 * const std::vector<std::string> value)
 */
bool SystemConfig::writeConfigToFile(const std::string filepath, const std::string module, const std::string section,
                                     const std::string param, const std::vector<std::string> value) const
{
  try
  {
    YAML::Node config = YAML::LoadFile(filepath);

    // Parse tree to find the key and update value
    config["smalldrop"][module][section][param] = value;

    // Store value on file
    std::ofstream fout(filepath);
    fout << config;
    return true;
  }
  catch(const std::exception& e)
  {
    return false;
  }
}

}  // namespace smalldrop_bioprint

}  // namespace smalldrop