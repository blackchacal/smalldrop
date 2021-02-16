#include <ros/ros.h>
#include <ros/package.h>

#include <smalldrop_segmentation/wound_segmentation_camera_binarization.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// ROS messages
#include <visualization_msgs/Marker.h>
#include <smalldrop_msgs/SetPathType.h>

// Libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

#include <Eigen/Dense>

/**
 * Global variables
 *********************************************************************************************/

unsigned int img_width = 640;
unsigned int img_height = 480;
float wsp_x_min;
float wsp_x_max;
float wsp_y_min;
float wsp_y_max;
unsigned int contour_marker_id = 0;
const std::string rgb_info_topic = "/smalldrop/vision/camera/color/camera_info";
const std::string rgb_image_topic = "/smalldrop/vision/camera/color/image_raw";
// const std::string ir1_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
// const std::string ir1_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
// const std::string ir2_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
// const std::string ir2_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
const std::string depth_info_topic = "/smalldrop/vision/camera/depth/camera_info";
const std::string depth_image_topic = "/smalldrop/vision/camera/depth/image_raw";
const std::string rgb_pcloud_topic = "/smalldrop/vision/camera/depth/color/points";
sensor_msgs::Image rgb_image;
sensor_msgs::Image depth_image;
pcl::PointCloud<pcl::PointXYZ> pcloud;

/**
 * Function prototypes
 *********************************************************************************************/

void rgb_callback(const sensor_msgs::Image::ConstPtr &msg);
void depth_callback(const sensor_msgs::Image::ConstPtr &msg);
void pcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
cv::Mat cv_image(const sensor_msgs::Image rgb_img);
std::vector<std::vector<cv::Point>> segment_wound(const cv::Mat rgb_img);
void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform);

/**
 * Main
 *********************************************************************************************/

int main( int argc, char** argv )
{
  ros::init(argc, argv, "validate_wound_position");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/smalldrop/rviz/wound_segmentation_markers", 10);
  ros::Subscriber rgb_sub = nh.subscribe<sensor_msgs::Image>(rgb_image_topic, 10, rgb_callback);
  ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>(depth_image_topic, 10, depth_callback);
  ros::Subscriber pcloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(rgb_pcloud_topic, 10, pcloud_callback);

  // double distance = 0.702;
  // double distance = 0.549;
  // double distance = 0.688;
  // double wsp_w = 0.890 * distance; // 0.885 Depth FOV VGA (radians), from camera datasheet (page 35)
  // // double wsp_w = 0.838 * distance; 
  // double wsp_h = 0.75 * wsp_w;
  // wsp_x_min = -wsp_w/2;
  // wsp_x_max = wsp_w/2;
  // wsp_y_min = -wsp_h/2;
  // wsp_y_max = wsp_h/2;

  // ROS_INFO("Calibration data, w: %d, h: %d, xmin: %f, xmax: %f, ymin: %f, ymax: %f\n", img_width, img_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  tf::TransformListener tf_listener;
  Eigen::Matrix4d transform;
  transform.setIdentity();

  std::stringstream path;
  path << ros::package::getPath("smalldrop_rviz") << "/data/contour_position_data.dat";	
  std::string filepath = path.str();

  std::fstream fh;
  fh.open(filepath, std::fstream::out);

  ros::Rate r(100);
  while (ros::ok())
  {
    try
    {
      getTransformToBase(tf_listener, transform);

      Eigen::Affine3d t(transform);
      Eigen::Vector3d t_vect(t.translation());
      double distance = t_vect(2);
      double wsp_w = 0.890 * distance; // 0.885 Depth FOV VGA (radians), from camera datasheet (page 35)
      // double wsp_w = 0.838 * distance; 
      double wsp_h = 0.75 * wsp_w;
      wsp_x_min = -wsp_w/2;
      wsp_x_max = wsp_w/2;
      wsp_y_min = -wsp_h/2;
      wsp_y_max = wsp_h/2;

      ROS_INFO("Calibration data, w: %d, h: %d, xmin: %f, xmax: %f, ymin: %f, ymax: %f\n", img_width, img_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

      cv::Mat cv_rgb_img;
      std::vector<std::vector<cv::Point>> contours;

      cv_rgb_img = cv_image(rgb_image);
      if (cv_rgb_img.rows > 0)
        contours = segment_wound(cv_rgb_img);

      if (contours.size() > 0)
      {
        ROS_INFO("Number of Contours: %d", contours.size());
        for (size_t j = 0; j < contours[0].size(); j++)
        {
          int x, y;
          x = contours[0][j].x;
          y = contours[0][j].y;
          // The point (0, 0) image frame is (0.864088, 0.326087, 0.010044) is robot base frame.
          // The point (320, 240) image frame is (0.630245, 0.021934, 0.000165) is robot base frame.
          // x = 320;
          // y = 240;

          double xr = (wsp_x_max - wsp_x_min) / img_width * x + wsp_x_min;
          double yr = (wsp_y_max - wsp_y_min) / img_height * y + wsp_y_min;
          
          Eigen::Vector4d pt_cam_contour, pt_base_contour;
          pt_cam_contour << xr, yr, distance, 1;
          pt_base_contour = transform * pt_cam_contour;

          ROS_INFO("The %d th point (%d, %d) image frame is (%f, %f, %f) is robot base frame.\n", j, x, y, pt_base_contour(0), pt_base_contour(1), pt_base_contour(2));
          fh << "The " << j << "th point (" << x << ", " << y << ") image frame is (" << pt_base_contour(0) << ", " << pt_base_contour(1) << ", " << pt_base_contour(2) << ") is robot base frame.\n";

          // ROS_INFO("The error in position is x: %f, y: %f in meters\n", (t_vect(0)-pt_base_contour(0)), (t_vect(1)-pt_base_contour(1)));
        }
      }
      else
      {
        ROS_INFO("No contours.");
      }
      

      ros::spinOnce();
      r.sleep();
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Error: %s", e.what());
      fh.close();
      break;
    }
  }
  fh.close();
  
  return 0;
}

void rgb_callback(const sensor_msgs::Image::ConstPtr &msg)
{
  rgb_image = *msg;
}

void depth_callback(const sensor_msgs::Image::ConstPtr &msg)
{
  depth_image = *msg;
}

void pcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{
  pcloud = *msg;
}

cv::Mat cv_image(const sensor_msgs::Image rgb_img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    if (sensor_msgs::image_encodings::isColor(rgb_img.encoding))
      cv_ptr = cv_bridge::toCvCopy(rgb_img, sensor_msgs::image_encodings::BGR8);
    else if (rgb_img.encoding == "16UC1")
    {
      sensor_msgs::Image img;
      img.header = rgb_img.header;
      img.height = rgb_img.height;
      img.width = rgb_img.width;
      img.is_bigendian = rgb_img.is_bigendian;
      img.step = rgb_img.step;
      img.data = rgb_img.data;
      img.encoding = "mono16";

      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
      cv_ptr = cv_bridge::toCvCopy(rgb_img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    cv::Mat img;
    return img;
  }

  return cv_ptr->image;
}

std::vector<std::vector<cv::Point>> segment_wound(const cv::Mat rgb_img)
{
  try
  {
    cv::Mat rsize(img_height, img_width, CV_8UC3);
    rsize = cv::Scalar::all(0);
    cv::resize(rgb_img, rsize, rsize.size());

    // Set to grayscale
    cv::Mat grey;
    cv::cvtColor(rsize, grey, CV_BGR2GRAY);

    // Binarize image
    /*
      Threshold Type
      0: Binary
      1: Binary Inverted
      2: Threshold Truncated
      3: Threshold to Zero
      4: Threshold to Zero Inverted
    */
    cv::Mat bin;
    int threshold_value = 10;
    int threshold_type = cv::THRESH_BINARY_INV;// | cv::THRESH_OTSU;
    int const max_BINARY_value = 255;
    cv::threshold(grey, bin, threshold_value, max_BINARY_value, threshold_type);

    // Get contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    cv::drawContours(rsize, contours, -1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
    cv::imshow("Display window", rsize);                   // Show our image inside it.
    cv::waitKey(0);

    return contours;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Error creating segmentation image: %s", e.what());
    throw;
  }
}

void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform)
{
  tf::StampedTransform transformStamped;
  try {
    tf_listener.lookupTransform("panda_link0", "camera_depth_optical_frame", ros::Time(0), transformStamped);
    
    Eigen::Vector4d t_vector(transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1);
    transform.col(3) << transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1;
    transform.col(0) << transformStamped.getBasis().getColumn(0)[0], transformStamped.getBasis().getColumn(0)[1], transformStamped.getBasis().getColumn(0)[2], 0;
    transform.col(1) << transformStamped.getBasis().getColumn(1)[0], transformStamped.getBasis().getColumn(1)[1], transformStamped.getBasis().getColumn(1)[2], 0;
    transform.col(2) << transformStamped.getBasis().getColumn(2)[0], transformStamped.getBasis().getColumn(2)[1], transformStamped.getBasis().getColumn(2)[2], 0;

    // std::cout << transform << std::endl;
  } catch (tf2::TransformException &ex) {
    return;
  }
}