// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file system_config.h
 * \brief Declares a class that manages the smalldrop system configurations.
 */

#ifndef _SMALLDROP_SYSTEM_CONFIG_H
#define _SMALLDROP_SYSTEM_CONFIG_H

#include <ros/ros.h>

namespace smalldrop
{
namespace smalldrop_bioprint
{
class SystemConfig
{
public:
  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn SystemConfig(ros::NodeHandle nh, std::string catkin_ws)
   * \brief Constructor
   */
  SystemConfig(ros::NodeHandle nh, std::string catkin_ws);

  ~SystemConfig()
  {
  }

  /**
   * \fn bool readConfigParam(const std::string module, const std::string section, const std::string param, int&
   * param_val) const \brief Reads a system configuration int value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   */
  bool readConfigParam(const std::string module, const std::string section, const std::string param,
                       int& param_val) const;

  /**
   * \fn bool readConfigParam(const std::string module, const std::string section, const std::string param, double&
   * param_val) const \brief Reads a system configuration double value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   */
  bool readConfigParam(const std::string module, const std::string section, const std::string param,
                       double& param_val) const;

  /**
   * \fn bool readConfigParam(const std::string module, const std::string section, const std::string param, std::string&
   * param_val) const \brief Reads a system configuration string value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   */
  bool readConfigParam(const std::string module, const std::string section, const std::string param,
                       std::string& param_val) const;

  /**
   * \fn bool readConfigParam(const std::string module, const std::string section, const std::string param,
   * std::vector<int>& param_val) const \brief Reads a system configuration int list.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   */
  bool readConfigParam(const std::string module, const std::string section, const std::string param,
                       std::vector<int>& param_val) const;

  /**
   * \fn bool readConfigParam(const std::string module, const std::string section, const std::string param,
   * std::vector<double>& param_val) const \brief Reads a system configuration double list.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   */
  bool readConfigParam(const std::string module, const std::string section, const std::string param,
                       std::vector<double>& param_val) const;

  /**
   * \fn bool readConfigParam(const std::string module, const std::string section, const std::string param,
   * std::vector<std::string>& param_val) const \brief Reads a system configuration string list.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   */
  bool readConfigParam(const std::string module, const std::string section, const std::string param,
                       std::vector<std::string>& param_val) const;

  /**
   * \fn void writeConfigParam(const std::string module, const std::string section, const std::string param, const int
   * value) const \brief Saves a system configuration int value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value Value to be stored.
   */
  void writeConfigParam(const std::string module, const std::string section, const std::string param,
                        const int value) const;

  /**
   * \fn void writeConfigParam(const std::string module, const std::string section, const std::string param, const
   * double value) const \brief Saves a system configuration double value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value Value to be stored.
   */
  void writeConfigParam(const std::string module, const std::string section, const std::string param,
                        const double value) const;

  /**
   * \fn void writeConfigParam(const std::string module, const std::string section, const std::string param, const
   * std::string value) const \brief Saves a system configuration string value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value Value to be stored.
   */
  void writeConfigParam(const std::string module, const std::string section, const std::string param,
                        const std::string value) const;

  /**
   * \fn void writeConfigParam(const std::string module, const std::string section, const std::string param, const
   * std::vector<int> value) const \brief Saves a system configuration list of integers value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value Value to be stored.
   */
  void writeConfigParam(const std::string module, const std::string section, const std::string param,
                        const std::vector<int> value) const;

  /**
   * \fn void writeConfigParam(const std::string module, const std::string section, const std::string param, const
   * std::vector<double> value) const \brief Saves a system configuration list of doubles value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value Value to be stored.
   */
  void writeConfigParam(const std::string module, const std::string section, const std::string param,
                        const std::vector<double> value) const;

  /**
   * \fn void writeConfigParam(const std::string module, const std::string section, const std::string param, const
   * std::vector<std::string> value) const \brief Saves a system configuration list of strings value.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value Value to be stored.
   */
  void writeConfigParam(const std::string module, const std::string section, const std::string param,
                        const std::vector<std::string> value) const;

private:
  /**
   * Class members
   *****************************************************************************************/

  ros::NodeHandle nh_; /** \var ROS node handle. */

  std::string ws_; /** \var Catkin workspace path. */

  std::map<std::string, std::string> config_filemap_; /** \var Stores the map between module names and associated
                                                         packages. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn std::string getConfigFilePath(const std::string module) const
   * \brief Returns the config file path based on the module selected.
   */
  std::string getConfigFilePath(const std::string module) const;

  /**
   * \fn std::string buildConfigParamName(const std::string module, const std::string section, const std::string param)
   * const \brief Builds the configuration parameter name.
   *
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   */
  std::string buildConfigParamName(const std::string module, const std::string section, const std::string param) const;

  /**
   * \fn bool writeConfigToFile(const std::string filepath, const std::string module, const std::string section, const
   * std::string param, const std::string value) const 
   * \brief Saves the new system configuration on the configuration file.
   *
   * \param filepath Path to the configuration file.
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value String value to be stored.
   */
  bool writeConfigToFile(const std::string filepath, const std::string module, const std::string section,
                         const std::string param, const std::string value) const;

  /**
   * \fn bool writeConfigToFile(const std::string filepath, const std::string module, const std::string section, const
   * std::string param, const std::vector<std::string> value) const 
   * \brief Saves the new system configuration on the configuration file.
   *
   * \param filepath Path to the configuration file.
   * \param module System configuration module. It is related to the package that manages the configuration file.
   * \param section System configuration section within a module.
   * \param param System configuration param within a section within a module.
   * \param value String list value to be stored.
   */
  bool writeConfigToFile(const std::string filepath, const std::string module, const std::string section,
                         const std::string param, const std::vector<std::string> value) const;
};

}  // namespace smalldrop_bioprint

}  // namespace smalldrop

#endif  // _SMALLDROP_SYSTEM_CONFIG_H