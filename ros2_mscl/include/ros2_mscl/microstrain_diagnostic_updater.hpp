#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/msg/diagnostic_updater.h"
#include "diagnostic_updater/msg/publisher.h"
#include "mscl_msgs/Status.h"
#include "include/microstrain_3dm.hpp"

#include <string>


namespace ros2_mscl
{
  class RosDiagnosticUpdater : private diagnostic_updater::Updater
  {
  public:
    RosDiagnosticUpdater();

    void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void statusCallback(const mscl_msgs::Status::ConstPtr& status);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;

    mscl_msgs::Status last_status_;
  };
}
