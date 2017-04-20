#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class MaskedImage
{
  ros::NodeHandle nh_;
  cv::Mat mask_image;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  MaskedImage() : it_(nh_)
  {
    image_sub_ = it_.subscribe("image_raw", 1, &MaskedImage::imageCb, this);
    image_pub_ = it_.advertise("masked_image", 1);

    ros::NodeHandle pnh("~");
    std::string mask_image_file = "masked.png";
    pnh.getParam("mask_image_file", mask_image_file);

    mask_image = cv::imread(mask_image_file);

    if(!mask_image.data)
    {
      ROS_ERROR("Could not open masked image (%s)", mask_image_file.c_str());
      return;
    }

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~MaskedImage()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat masked_image;
    cv_ptr->image.copyTo(masked_image, mask_image);

    masked_image.copyTo(cv_ptr->image);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "masked_image");
  MaskedImage mi;
  ros::spin();
  return 0;
}
