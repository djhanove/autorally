#include "LTIMPC.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "LTIMPC");
  autorally_control::LTIMPC path_following_LTI_MPC("~");
  ros::spin();
}
