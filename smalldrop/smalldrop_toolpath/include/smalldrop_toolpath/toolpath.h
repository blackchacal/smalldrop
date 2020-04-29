// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file toolpath.h
 * \brief Declares ToolPath class for robot arm printing path planning.
 */

#ifndef _SMALLDROP_TOOL_PATH_H
#define _SMALLDROP_TOOL_PATH_H

#include <smalldrop_segmentation/i_wound_segmentation.h>
#include <smalldrop_segmentation/wound_segmentation_comanip.h>
#include <smalldrop_toolpath/path.h>

// Libraries
#include "opencv2/imgproc.hpp"

using namespace smalldrop::smalldrop_segmentation;

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \enum PRINT_ACTION
 * \brief Enumerate to describe the printing actions on each path pose.
 */
enum class PRINT_ACTION
{
  START,
  STOP,
  CONTINUE
};

/**
 * \enum IMAGE_AXIS
 * \brief Enum of the image axes
 */
enum class IMAGE_AXIS
{
  X,
  Y
};

/**
 * \typedef path_actions_t
 */
typedef std::vector<PRINT_ACTION> path_actions_t;

/**
 * \class ToolPath
 * \brief ToolPath printing path class.
 */
class ToolPath : public Path
{
public:
  virtual ~ToolPath(){};

  /**
   * \fn path_actions_t actions() const
   * \brief Returns the toolpath actions.
   */
  path_actions_t actions() const;

  /**
   * \fn points_t points() const
   * \brief Returns the toolpath points.
   */
  points_t points() const;

protected:
  /**
   * Class members
   *****************************************************************************************/

  path_actions_t actions_; /** \var Print actions for each path pose. */
  points_t points_;        /** \var List of path points. */

  // Robot workspace and segmentation image
  unsigned int image_width_;  /** \var Image width in px. */
  unsigned int image_height_; /** \var Image height in px. */
  double wsp_x_min_limit_;    /** \var Robot workspace coordinates minimum x limit. */
  double wsp_x_max_limit_;    /** \var Robot workspace coordinates maximum x limit. */
  double wsp_y_min_limit_;    /** \var Robot workspace coordinates minimum y limit. */
  double wsp_y_max_limit_;    /** \var Robot workspace coordinates maximum y limit. */

  /**
   * Class constructor & methods
   *****************************************************************************************/

  /**
   * \fn ToolPath(const img_wsp_calibration_t calibration_data)
   * \brief Constructor.
   *
   * \param calibration_data Data for image-workspace calibration.
   */
  ToolPath(const img_wsp_calibration_t calibration_data);

  /**
   * \fn std::vector<points_t> getGridLines(cv::Rect bounding_box, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X)
   * const \brief Obtains the points of the bounding box that form the grid parallel lines.
   *
   * \param bounding_box Bounding box of a contour.
   * \param offset Distance between grid lines.
   * \param axis The axis the is parallel to the grid lines.
   */
  std::vector<points_t> getGridLines(cv::Rect bounding_box, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X) const;

  /**
   * \fn void getComplexContours(const points_t contour, contours_t &complex_contours, cv::Mat &img) const
   * \brief Returns a more complex contour (more points) for proper grid lines intersection calculation.
   *
   * \param contour List of opencv points that form a wound contour.
   */
  void getComplexContours(const points_t contour, contours_t &complex_contours, cv::Mat &img) const;

  /**
   * \fn poses_t convPathPointToPose(const points_t path, const double pose_z) const
   * \brief Returns a wound filling as robot poses instead of opencv points.
   *
   * \param path List of opencv points that form a wound filling path.
   * \param pose_z Z axis coordinate for the robot path execution.
   */
  poses_t convPathPointToPose(const points_t path, const double pose_z) const;

  /**
   * \fn double convPx2Meter()
   * \brief Convert the pixel size to meters
   */
  std::vector<double> convPx2Meter() const;
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_TOOL_PATH_H