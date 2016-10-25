/**
 * \ref iosfl_node.cpp
 *
 *  \date 29/sep/2015
 *  \author Alessio Levratti
 *  \version 1.0
 *  \copyright GNU Public License.
 */

#include <iosfl/iosfl.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_feedback_linearization");
  iosfl* sf = new iosfl();

  while (ros::ok())
    ros::spinOnce();
}


