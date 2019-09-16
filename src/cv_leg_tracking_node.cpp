#include <cv_leg_tracking/cv_leg_tracking.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv,"cv_leg_tracking");
  ros::NodeHandle nh("~");
  std::shared_ptr<CvLegTracking> tracker;
  tracker.reset(new CvLegTracking(nh));
  ros::spin();
  return 0;
}
