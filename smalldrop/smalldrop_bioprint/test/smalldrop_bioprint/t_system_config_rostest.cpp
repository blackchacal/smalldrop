// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_system_config.cpp
 * \brief Test file for SystemConfig class.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <smalldrop_bioprint/system_config.h>
#include <smalldrop_state/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sstream>

using namespace smalldrop::smalldrop_bioprint;
using namespace smalldrop::smalldrop_state;

class SystemConfigTest : public ::testing::Test
{
protected:
  std::string catkin_ws = "/home/rtonet/ROS/tese";
  ros::NodeHandle nh;

  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

public:
  std::string getConfigFilePath(const std::string ws, const std::string module) const
  {
    std::stringstream path;
    std::map<std::string, std::string> config_filemap = {
      { "bioprint", "smalldrop_bioprint" },
      { "robot", "smalldrop_robot_arm" },
      { "teleop", "smalldrop_teleoperation" },
    };

    path << ws << "/src/smalldrop/" << config_filemap.find(module)->second << "/config/test_config.yaml";
    return path.str();
  }
};

TEST_F(SystemConfigTest, readIntConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  int param_val;
  cfg.readConfigParam("module1", "section1", "param1", param_val);

  EXPECT_EQ(param_val, 10);
}

TEST_F(SystemConfigTest, readDoubleConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  double param_val;
  cfg.readConfigParam("module1", "section1", "param2", param_val);

  EXPECT_EQ(param_val, 5.3);
}

TEST_F(SystemConfigTest, readStringConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string param_val;
  cfg.readConfigParam("module1", "section1", "param3", param_val);

  EXPECT_EQ(param_val, "string");
}

TEST_F(SystemConfigTest, readIntListConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::vector<int> param_val;
  cfg.readConfigParam("module2", "section1", "param1", param_val);

  EXPECT_EQ(param_val.size(), 5);
  EXPECT_EQ(param_val[0], 1);
  EXPECT_EQ(param_val[1], 1);
  EXPECT_EQ(param_val[2], 2);
  EXPECT_EQ(param_val[3], 3);
  EXPECT_EQ(param_val[4], 5);
}

TEST_F(SystemConfigTest, readDoubleListConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::vector<double> param_val;
  cfg.readConfigParam("module1", "section2", "param1", param_val);

  EXPECT_EQ(param_val.size(), 7);
  EXPECT_EQ(param_val[0], 0);
  EXPECT_EQ(param_val[1], 0);
  EXPECT_EQ(param_val[2], 0);
  EXPECT_EQ(param_val[3], -1.571);
  EXPECT_EQ(param_val[4], 0);
  EXPECT_EQ(param_val[5], 1.571);
  EXPECT_EQ(param_val[6], 0.785);
}

TEST_F(SystemConfigTest, readStringListConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::vector<std::string> param_val;
  cfg.readConfigParam("module2", "section1", "param2", param_val);

  EXPECT_EQ(param_val.size(), 3);
  EXPECT_EQ(param_val[0], "str1");
  EXPECT_EQ(param_val[1], "str2");
  EXPECT_EQ(param_val[2], "str3");
}

TEST_F(SystemConfigTest, writeConfigParamWrongModuleThrowsException)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string param_val = "new_string";

  EXPECT_THROW(cfg.writeConfigParam("module1", "section1", "param3", param_val), ConfigFileSaveException);
}

TEST_F(SystemConfigTest, writeIntConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string module = "bioprint";
  std::string section = "section1";
  std::string param = "param3";
  int stored_val;
  int param_val = 12;

  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  YAML::Node config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  int file_val = config["smalldrop"][module][section][param].as<int>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = 2;
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<int>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = 1000;
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<int>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);
}

TEST_F(SystemConfigTest, writeDoubleConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string module = "bioprint";
  std::string section = "section1";
  std::string param = "param4";
  double stored_val;
  double param_val = 3.14;

  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  YAML::Node config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  double file_val = config["smalldrop"][module][section][param].as<double>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = 0.324;
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<double>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = 1.0023;
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<double>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);
}

TEST_F(SystemConfigTest, writeStringConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string module = "bioprint";
  std::string section = "section1";
  std::string param = "param5";
  std::string stored_val;
  std::string param_val = "new_string";

  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  YAML::Node config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  std::string file_val = config["smalldrop"][module][section][param].as<std::string>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = "hello world";
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::string>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = "bioprinting";
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::string>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);
}

TEST_F(SystemConfigTest, writeIntListConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string module = "bioprint";
  std::string section = "section1";
  std::string param = "param1";
  std::vector<int> stored_val;
  std::vector<int> param_val = { 12, 4, 7, 3, 15 };

  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  YAML::Node config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  std::vector<int> file_val = config["smalldrop"][module][section][param].as<std::vector<int>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = { 1, 1, 2, 3, 5, 8, 13 };
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::vector<int>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = { 10, 100, 1000, 10000, 100000 };
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::vector<int>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);
}

TEST_F(SystemConfigTest, writeDoubleListConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string module = "bioprint";
  std::string section = "section1";
  std::string param = "param6";
  std::vector<double> stored_val;
  std::vector<double> param_val = { 3.14, 12.4335, 44.1, 0.23 };

  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  YAML::Node config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  std::vector<double> file_val = config["smalldrop"][module][section][param].as<std::vector<double>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = { 0.1, 0.01, 0.001 };
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::vector<double>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = { 1.231, 2343.233, 1124.2, 1.23235 };
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::vector<double>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);
}

TEST_F(SystemConfigTest, writeStringListConfigParam)
{
  SystemConfig cfg(nh, catkin_ws);

  std::string module = "bioprint";
  std::string section = "section1";
  std::string param = "param2";
  std::vector<std::string> stored_val;
  std::vector<std::string> param_val = { "new_string_1", "new_string_2", "new_string_3" };

  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  YAML::Node config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  std::vector<std::string> file_val = config["smalldrop"][module][section][param].as<std::vector<std::string>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = { "back", "to", "the", "future" };
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::vector<std::string>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);

  param_val = { "what", "do", "you", "mean", "test?" };
  cfg.writeConfigParam(module, section, param, param_val);
  cfg.readConfigParam(module, section, param, stored_val);
  config = YAML::LoadFile(getConfigFilePath(catkin_ws, module));
  file_val = config["smalldrop"][module][section][param].as<std::vector<std::string>>();
  EXPECT_EQ(stored_val, param_val);
  EXPECT_EQ(file_val, param_val);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "t_system_config");
  return RUN_ALL_TESTS();
}
