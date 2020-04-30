#include "LTVMPC.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LTVMPC");
  autorally_control::LTVMPC path_following_LTV_MPC("~");
  ros::spin();
}
