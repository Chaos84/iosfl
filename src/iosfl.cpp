/**
 * \ref iosfl.cpp
 *
 *  \date 29/set/2015
 *  \author Alessio Levratti
 *  \version 1.0
 *  \copyright GNU Public License.
 */

#include <iosfl/iosfl.h>

/**
 * Constructor
 */
iosfl::iosfl()
{
  // Create the private nodehandle to get private parameters from the ROS parameters server
  ros::NodeHandle pnh("~");
  pnh.param("iosfl_lead", b_, 0.1);
  pnh.param("axle", axle_, 0.33);
  double v;
  pnh.param("maximum_speed", v, .5);
  // Set the maximum speed
  vMax_.first = v;
  vMax_.second = 2 * v / axle_;

  // Set the publisher
  speed_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

  // Set the subscribers
  speed_sub_ = nh_.subscribe("speed", 1, &iosfl::state_feedback_linearization, this);
}

/**
 * Destructor
 */
iosfl::~iosfl()
{

}

/**
 * \brief Speed saturator
 *
 *This method saturates the linear and angular speeds for the robot
 * @param vel: the \ref geometry_msgs::Twist message to be saturated
 */
void iosfl::satVel(geometry_msgs::Twist& vel)
{
  double lin, rot;
  lin = vel.linear.x;
  rot = vel.angular.z;

  // Compute the maximum term between the linear and the angular speeds
  std::pair<double, unsigned short> res = this->whosMax(fabs(lin), fabs(rot));

  switch (res.second)
  {
    case 1:
      vel.linear.x = SIGN(lin) * vMax_.first;
      vel.angular.z = rot / res.first;
      break;
    case 2:
      vel.linear.x = lin / res.first;
      vel.angular.z = SIGN(rot) * vMax_.second;
      break;
    default:
      vel.linear.x = lin;
      vel.angular.z = rot;
      break;
  }
  return;
}

/**
 * \brief Speed saturator
 *
 *This method saturates the linear and angular speeds for the robot
 * @param vel: the \ref geometry_msgs::Twist message to be saturated
 * @param maxSpeed: an \ref std::pair<double, double> containing the maximum linear speed and maximum angular speed of the robot
 */
void iosfl::satVel(geometry_msgs::Twist& vel, std::pair<double, double> max_speed)
{
  double lin, rot;
  lin = vel.linear.x;
  rot = vel.angular.z;

  // Compute the maximum term between the linear and the angular speeds
  std::pair<double, unsigned short> res = whosMax(fabs(lin), fabs(rot), max_speed);

  switch (res.second)
  {
    case 1:
      vel.linear.x = SIGN(lin) * max_speed.first;
      vel.angular.z = rot / res.first;
      break;
    case 2:
      vel.linear.x = lin / res.first;
      vel.angular.z = SIGN(rot) * max_speed.second;
      break;
    default:
      vel.linear.x = lin;
      vel.angular.z = rot;
      break;
  }
  return;
}

/**
 * This method returns the maximum value between two parameters
 * and its index (1 or 2)
 * @param a: the first parameter
 * @param b: the second parameter
 * @return: a pair with the maximum value and its index (1 or 2). If the
 * two input parameters are equal, this method returns the pair <1, 3>
 */
std::pair<double, unsigned short> iosfl::whosMax(double a, double b)
{
  std::pair<double, unsigned short> res;

  double A = a / vMax_.first;
  double B = b / vMax_.second;

  if ((A > B) && (A > 1))
  {
    res.first = A;
    res.second = 1;
  }
  else if ((B > A) && (B > 1))
  {
    res.first = B;
    res.second = 2;
  }
  else
  {
    res.first = 1;
    res.second = 3;
  }
  return (res);
}

/**
 * This method returns the maximum value between two parameters
 * and its index (1 or 2)
 * @param a: the first parameter
 * @param b: the second parameter
 * @param maxSpeed: an \ref std::pair<double, double> containing the maximum linear speed and maximum angular speed of the robot
 * @return: an \ref std::pair<double, double> with the maximum value and its index (1 or 2). If the
 * two input parameters are equal, this method returns the pair <1, 3>
 */
std::pair<double, unsigned short> iosfl::whosMax(double a, double b, std::pair<double, double> maxSpeed)
{
  std::pair<double, unsigned short> res;

  double A = a / maxSpeed.first;
  double B = b / maxSpeed.second;

  if ((A > B) && (A > 1))
  {
    res.first = A;
    res.second = 1;
  }
  else if ((B > A) && (B > 1))
  {
    res.first = B;
    res.second = 2;
  }
  else
  {
    res.first = 1;
    res.second = 3;
  }
  return (res);
}

/**
 * This method computes the state feedback linearization
 * @param speed: desired linear speed to be converted into linear and angular speed
 */
void iosfl::state_feedback_linearization(const geometry_msgs::TwistConstPtr& speed)
{
  tf::StampedTransform tr;
  try
  {
    listener_.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(10));
    listener_.lookupTransform("map", "base_link", ros::Time(0), tr);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Something went wrong while trying to get the transformation from \"map\" to \"base_link\". Did you forgot to publish it?");
    ROS_ERROR("Exception: %s", ex.what());
    return;
  }

  float yaw = tf::getYaw(tr.getRotation());

  cmd_vel_.linear.x = cos(yaw) * speed->linear.x + sin(yaw) * speed->linear.y;
  cmd_vel_.angular.z = -1 / b_ * sin(yaw) * speed->linear.x + 1 / b_ * cos(yaw) * speed->linear.y;
  this->satVel(cmd_vel_);
  speed_pub_.publish(cmd_vel_);
}
