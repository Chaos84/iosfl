/**
 * \ref iosfl.hpp
 *
 *  \date 29/sep/2015
 *  \author Alessio Levratti
 *  \version 1.0
 *  \copyright GNU Public License.
 */

#ifndef IOSFL_HPP_
#define IOSFL_HPP_

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

/**
 * \ref SIGN returns the sign of a number. If the number is greater or equal to 0 this MACRO returns 1,
 * otherwise, it returns -1.
 */
#ifndef SIGN
#define SIGN(a) (((a)>=(0))?(1):(-1))
#endif //SIGN

/**
 * \brief This class implements the I/O State Feedback Linearization
 */
class iosfl
{
public:
  iosfl();
  virtual ~iosfl();
  static void satVel(geometry_msgs::Twist&, std::pair<double, double>);
private:
  void odometry_callback(const nav_msgs::OdometryConstPtr&);
  void state_feedback_linearization(const geometry_msgs::TwistConstPtr&);
  void satVel(geometry_msgs::Twist&);
  std::pair<double, unsigned short> whosMax(double a, double b);
  static std::pair<double, unsigned short> whosMax(double a, double b, std::pair<double, double>);
  /// Nodehandle
  ros::NodeHandle nh_;
  /// Speed publisher
  ros::Publisher speed_pub_;
  /// vx and vy subscriber
  ros::Subscriber speed_sub_;
  /// Axle of the robot
  double axle_;
  /// Lead length
  double b_;
  /// Maximum speed
  std::pair<double, double> vMax_;
  /// Velocity command to be published
  geometry_msgs::Twist cmd_vel_;
  /// Transform listener
  tf::TransformListener listener_;
};

#endif /* IOSFL_H_ */
