// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file bioprinter.cpp
 * \brief Defines a class that controls the whole smalldrop system. The public API should allow
 * the system to be controlled by a GUI.
 */

#include <smalldrop_bioprint/bioprinter.h>

namespace smalldrop
{
namespace smalldrop_bioprint
{

/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief Bioprinter::Bioprinter(const bool simulation, const bool dev)
 */
Bioprinter::Bioprinter(const bool simulation, const bool development)
 : state_(STATE::OFF), operation_mode_(MODE::PRINT), is_sim_(simulation), is_dev_(development)
{
  state_topic_ = "/smalldrop/bioprint/state";

  setupPublishers();
  publishState();
}

/**
 * \copybrief Bioprinter::publishState()
 */
void Bioprinter::publishState()
{
  std::string state_str;
  switch (state_)
  {
  case STATE::INIT:
    state_str = "INIT"; 
    break;
  case STATE::IDLE:
    state_str = "IDLE"; 
    break;
  case STATE::PRINT:
    state_str = "PRINT"; 
    break;
  case STATE::PAUSE:
    state_str = "PAUSE"; 
    break;
  case STATE::ABORT:
    state_str = "ABORT"; 
    break;
  case STATE::REFILL:
    state_str = "REFILL"; 
    break;
  case STATE::CALIB_CAM:
    state_str = "CALIB_CAM"; 
    break;
  case STATE::CALIB_PHEAD:
    state_str = "CALIB_PHEAD"; 
    break;
  case STATE::ERROR:
    state_str = "ERROR"; 
    break;
  default:
    state_str = "OFF"; 
    break;
  }

  std_msgs::String msg;
  msg.data = state_str;
  state_pub_.publish(msg);
}

/**
 * \copybrief Bioprinter::isSimulation() const
 */
bool Bioprinter::isSimulation() const
{
  return is_sim_;
}

/**
 * \copybrief Bioprinter::isDevelopment()
 */
bool Bioprinter::isDevelopment() const
{
  return is_dev_;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief Bioprinter::setupPublishers()
 */
void Bioprinter::setupPublishers()
{
  state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 10);
}

}  // namespace smalldrop_bioprint

}  // namespace smalldrop