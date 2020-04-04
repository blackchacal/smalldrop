// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file bioprinter.cpp
 * \brief Main system node. It instantiates the Bioprinter class.
 */

#include <getopt.h>

#include <smalldrop_bioprint/system_state.h>
#include <smalldrop_bioprint/bioprinter.h>

using namespace smalldrop::smalldrop_bioprint;

// Global variables
bool is_sim = true;
bool is_dev = true;
bool calib_cam = false;
bool calib_phead = false;
std::string LOG_TAG = "smalldrop_bioprint";

// Function prototypes
bool processCmdArgs(int argc, char **argv);

/**
 * MAIN
 ************************************************************************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "bioprinter");

  if (!processCmdArgs(argc, argv))
    return 0;

  SystemState ss;
  Bioprinter bp(is_sim, is_dev);

  ros::Rate r(100); // 100 Hz
  while (ros::ok())
  {
    // Publish system state
    bp.publishState();

    // State machine
    switch (bp.getCurrentState())
    {
    case STATE::OFF:
      if (bp.getPrevState() == STATE::OFF) // We are initialising the system
        bp.setState(STATE::INIT);
      else // If previous state is not OFF it means we want to shutdown the system
        return 0;
      break;
    case STATE::INIT:
      bp.init(calib_cam, calib_phead, false);
      break;
    case STATE::IDLE:
      break;
    case STATE::PRINT:
      break;
    case STATE::MOVE:
      break;
    case STATE::PAUSE:
      break;
    case STATE::ABORT:
      break;
    case STATE::REFILL:
      break;
    case STATE::CALIB_CAM:
      break;
    case STATE::CALIB_PHEAD:
      break;
    case STATE::ERROR:
      break;
    default:
      break;
    }

    ros::spinOnce();
    r.sleep(); 
  }
  
  return 0;
}

/**
 * General Functions
 ************************************************************************************/

bool processCmdArgs(int argc, char **argv)
{
  // Process command-line arguments
  int opt;
  const char *const short_opts = ":hc:d:p:s:";
  const option long_opts[] = { 
    { "ccam", required_argument, nullptr, 'c' },
    { "dev", required_argument, nullptr, 'd' },
    { "cphead", required_argument, nullptr, 'p' },
    { "sim", required_argument, nullptr, 's' },
    { "help", no_argument, nullptr, 'h' },
    { nullptr, no_argument, nullptr, 0 } 
  };

  while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1)
  {
    switch (opt)
    {
      case 'c':
        calib_cam = static_cast<bool>(std::stoi(optarg));
        break;
      case 'd':
        is_dev = static_cast<bool>(std::stoi(optarg));
        break;
      case 'p':
        calib_phead = static_cast<bool>(std::stoi(optarg));
        break;
      case 's':
        is_sim = static_cast<bool>(std::stoi(optarg));
        break;
      case 'h':
      default:
        std::cout << "Help:" << std::endl;
        std::cout << "bioprinter --sim <0/1> --dev <0/1>" << std::endl;
        return false;
    }
  }
  return true;
}
